#ifndef NUMATAC_CAN_DRIVER_H_
#define NUMATAC_CAN_DRIVER_H_

#include <stdint.h>
#include <string>
#include <vector>

#include "bt_can.h"


enum { PAC = 0,
       PDC
     };


namespace numatac_can_driver
{

class NumaTacCANDriver
{
public:
  NumaTacCANDriver(std::string canbus_dev):
    canbus_dev_(canbus_dev),
    is_connected_(false),
    number_of_sensors_(3)
  {
    pac_.resize(number_of_sensors_);
    pdc_.resize(number_of_sensors_);
  }

  bool connect();
  bool isConnected();
  bool getData();
  uint16_t getPAC(uint8_t id);
  uint16_t getPDC(uint8_t id);

private:
  int socket_;
  std::string canbus_dev_;  // CAN interface ID
  bool is_connected_;
  uint8_t number_of_sensors_;

  std::vector<uint16_t> pac_;
  std::vector<uint16_t> pdc_;
};

}  // numatac_can_driver

#endif  // NUMATAC_CAN_DRIVER_H_
