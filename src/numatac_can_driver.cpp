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

bool NumaTacCANDriver::connect()
{

  if((socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    ROS_ERROR("Error while opening socket");
    return false;
  }

  struct ifreq ifr;

  strcpy(ifr.ifr_name, canbus_dev_.c_str());

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

  if(bind(socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    ROS_ERROR("Error in socket bind");
    return false;
  }

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 1; // microseconds

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
          data.batch_index ++;
        }

        if ((i % (biotac.frame.frame_size)) == 0)
        {
          data.frame_index ++;
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

      for (int j = 0; j < MAX_BIOTACS_FOR_JACO; j++)
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
      case PDC:
        //  Pressure DC
        pdc_[0] =  data.d[0].word;
        pdc_[1] =  data.d[1].word;
        pdc_[2] =  data.d[2].word;
        break;
      case PAC:
        //  Pressure AC
        pac_[0] =  data.d[0].word;
        pac_[1] =  data.d[1].word;
        pac_[2] =  data.d[2].word;
        break;
      }

      sample_count++;

    }
    return true;
  }
  else
  {
    return false;
  }

}

  uint16_t NumaTacCANDriver::getPAC(uint8_t id)
  {
    return pac_[id];
  }
  uint16_t NumaTacCANDriver::getPDC(uint8_t id)
  {
    return pdc_[id];
  }

}
