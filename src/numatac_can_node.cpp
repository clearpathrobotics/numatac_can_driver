/**
Software License Agreement (BSD)

\file      numatac_can_node.cpp
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
#include <vector>

#include "ros/ros.h"
#include "numatac_can_driver/numatac_can_driver.h"
#include "numatac_can_driver/HandPressure.h"

class NumaTacCANNode
{
  public:
    NumaTacCANNode(ros::NodeHandle& nh, ros::NodeHandle& pnh,
      numatac_can_driver::NumaTacCANDriver& driver, uint8_t number_of_sensors, bool tare) :
      driver_(driver),
      number_of_sensors_(number_of_sensors),
      tare_(tare)
    {

      pressure_msg_.fingers.resize(number_of_sensors_);
      pressure_pub_ = nh.advertise<numatac_can_driver::HandPressure>("hand_pressure", 100);

      tare_pac_.resize(number_of_sensors_);
      tare_pdc_.resize(number_of_sensors_);
    }

    bool connectIfNotConnected()
    {
      if (!driver_.isConnected())
      {
        if (!driver_.connect())
        {
          ROS_ERROR("Error connecting to CAN device. Retrying in 1 second.");
          return false;
        }
        else
        {
          ROS_INFO("Connection to CAN device successful.");
        }
      }
      return true;
    }

    void run()
    {

      ros::Rate rate(1000);

      while (ros::ok())
      {
        if (!connectIfNotConnected())
        {
          ros::Duration(1.0).sleep();
          continue;
        }

        while(driver_.getData())
        {
        }

        if(tare_)
        {
          for (int i = 0; i < number_of_sensors_; i++)
          {
            tare_pac_[i] = driver_.getPAC(i);
            tare_pdc_[i] = driver_.getPDC(i);
          }
          tare_ = false;
        }
        else
        {
          pressure_msg_.header.stamp = ros::Time::now();

          for (int i = 0; i < number_of_sensors_; i++)
          {
            pressure_msg_.fingers[i].fluid_pressure = (driver_.getPDC(i) - tare_pdc_[i]) * 12.94;
            pressure_msg_.fingers[i].dynamic_pressure = (driver_.getPAC(i) - tare_pac_[i]) * 0.13;
            pressure_msg_.fingers[i].finger_number = i + 1;
          }
          pressure_pub_.publish(pressure_msg_);
        }


        rate.sleep();
      }
    }

  private:
    uint8_t number_of_sensors_;
    bool tare_;
    std::vector<int16_t> tare_pac_;
    std::vector<uint16_t> tare_pdc_;
    numatac_can_driver::NumaTacCANDriver& driver_;
    numatac_can_driver::HandPressure pressure_msg_;
    ros::Publisher pressure_pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "numatac_can_driver");
  ros::NodeHandle nh, pnh("~");

  int number_of_sensors;
  std::string canbus_dev;
  bool tare;

  pnh.param<std::string>("canbus_dev", canbus_dev, "can0");
  pnh.param<int>("number_of_sensors", number_of_sensors, 3);
  pnh.param<bool>("tare", tare, true);

  numatac_can_driver::NumaTacCANDriver driver(canbus_dev, number_of_sensors);

  NumaTacCANNode node(nh, pnh, driver, number_of_sensors, tare);

  node.run();

}
