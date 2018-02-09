#pragma once

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <AP_SerialManager/AP_SerialManager.h>

class AP_InertialSensor_DMU11 : public AP_InertialSensor_Backend
{

public:
  // constructor
  AP_InertialSensor_DMU11(AP_InertialSensor &imu,
                          AP_SerialManager &serial_manager);
// static detection function
  static bool detect(AP_SerialManager &serial_manager);

  // update state
  // void update(void);

private:
  // bool AP_InertialSensor_DMU11::get_reading(Vector3f &gyro, Vector3f &accel);
  bool get_DMU11_data(void);
  AP_HAL::UARTDriver *uart = nullptr;

}
