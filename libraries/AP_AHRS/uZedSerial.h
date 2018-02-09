#pragma once

#include <AP_SerialManager/AP_SerialManager.h>


class AP_uZedSerial
{
public:
  AP_uZedSerial(AP_SerialManager &serial_manager);
  static bool detect(AP_SerialManager &serial_manager);
  bool get_flag(int16_t &agc_flag);
  // bool send_telem();
}
