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
