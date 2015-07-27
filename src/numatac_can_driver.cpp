/**
Software License Agreement (BSD)

\file      numatac_can_driver.cpp
\authors   Tony Baltovski <tbaltovski@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <string>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include "numatac_can_driver/numatac_can_driver.h"
#include "ros/ros.h"

namespace numatac_can_driver
{

NumaTacCANDriver::NumaTacCANDriver(std::string canbus_dev, uint8_t number_of_sensors):
  canbus_dev_(canbus_dev),
  is_connected_(false),
  number_of_sensors_(number_of_sensors)
{
  pac_.resize(number_of_sensors_);
  pdc_.resize(number_of_sensors_);
}

bool NumaTacCANDriver::connect()
{
  if ((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    ROS_ERROR("Error while opening socket");
    return false;
  }

  struct ifreq ifr;

  snprintf (ifr.ifr_name, sizeof(canbus_dev_.c_str()), "%s", canbus_dev_.c_str());

  if (ioctl(socket_, SIOCGIFINDEX, &ifr) < 0)
  {
    close(socket_);
    ROS_ERROR("Error while trying to device");
    return false;
  }

  struct sockaddr_can addr;
  addr.can_family  = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  ROS_DEBUG("%s at index %d", canbus_dev_.c_str(), ifr.ifr_ifindex);

  if (bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    ROS_ERROR("Error in socket bind");
    return false;
  }

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 1;  // microseconds

  setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  ROS_INFO("Opened Socket CAN on %s", canbus_dev_.c_str());
  is_connected_ = true;

  return is_connected_;
}


bool NumaTacCANDriver::isConnected()
{
  return is_connected_;
}


bool NumaTacCANDriver::getData()
{
  bt_info biotac;
  bt_data data;

  static int sample_count = 0;
  const int number_of_samples_in_batch = 1;

  // Read CAN frame
  struct can_frame frame;
  int bytes = read(socket_, &frame, sizeof(struct can_frame));

  if (bytes > 0)
  {
    data.index = sample_count;

    for (int i = 0; i < number_of_samples_in_batch; i++)
    {
      if (sample_count != 0)
      {
        if (i == 0)
        {
          data.batch_index++;
        }

        if ((i % (biotac.frame.frame_size)) == 0)
        {
          data.frame_index++;
        }

        data.time = 0.0;
      }
      else
      {
        data.batch_index = 1;
        data.frame_index = 1;
        data.time = 0;
      }

      data.channel_id = (frame.data[0] & 0x7E) >> 1;

      for (int j = 0; j < number_of_sensors_; j++)
      {
        // Combine two bytes of CAN message into a word (2 bytes) of data
        data.d[j].word = (frame.data[j * 2 + 1] >> 1) * 32 + (frame.data[j * 2 + 2] >> 3);

        // Checking the data parity
        if ((parity_values[frame.data[j * 2 + 1] >> 1] == frame.data[j * 2 + 1])
         && (parity_values[frame.data[j * 2 + 2] >> 1] == frame.data[j * 2 + 2]))
        {
          data.bt_parity[j] = PARITY_GOOD;
        }
        else
        {
          data.bt_parity[j] = PARITY_BAD;
        }
      }

      switch (data.channel_id)
      {
      case numatac_can_driver::PDC:
        //  Pressure DC
        for (int i = 0; i < number_of_sensors_; i++)
        {
          pdc_[i] =  data.d[i].word;
        }
        return true;
      case numatac_can_driver::PAC:
        //  Pressure AC
        for (int i = 0; i < number_of_sensors_; i++)
        {
          pac_[i] =  data.d[i].word;
        }
        return false;
      default:
        return true;
      }

      sample_count++;
    }
  }
  else
  {
    return true;
  }
}

int16_t NumaTacCANDriver::getPAC(uint8_t id)
{
  return pac_[id];
}
uint16_t NumaTacCANDriver::getPDC(uint8_t id)
{
  return pdc_[id];
}

}  // namespace numatac_can_driver
