#include <string>

#include "ros/ros.h"
#include "numatac_can_driver/numatac_can_driver.h"
#include "numatac_can_driver/HandPressure.h"

class NumaTacCANNode
{
  public:
    NumaTacCANNode(ros::NodeHandle& nh, ros::NodeHandle& pnh, numatac_can_driver::NumaTacCANDriver& driver) :
      driver_(driver),
      number_of_sensors_(3)
    {
      pressure_pub_ = nh.advertise<numatac_can_driver::HandPressure>("hand_pressure", 100);

      pressure_msg_.fingers.resize(number_of_sensors_);
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

      ros::Rate rate(100);

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

        pressure_msg_.header.stamp = ros::Time::now();

        for (int i = 0; i < number_of_sensors_; i++)
        {
          pressure_msg_.fingers[i].fluid_pressure = (driver_.getPAC(i) - 0) * 12.94;
          pressure_msg_.fingers[i].dynamic_pressure = (driver_.getPDC(i) - 0) * 0.13;
          pressure_msg_.fingers[i].finger_number = i + 1;
        }
        pressure_pub_.publish(pressure_msg_);

        rate.sleep();
      }
    }

  private:
    uint8_t number_of_sensors_;
    numatac_can_driver::NumaTacCANDriver& driver_;
    numatac_can_driver::HandPressure pressure_msg_;
    ros::Publisher pressure_pub_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "numatac_can_driver");
  ros::NodeHandle nh, pnh("~");

  std::string canbus_dev = "can0";
  numatac_can_driver::NumaTacCANDriver driver(canbus_dev);

  NumaTacCANNode node(nh, pnh, driver);

  node.run();

}
