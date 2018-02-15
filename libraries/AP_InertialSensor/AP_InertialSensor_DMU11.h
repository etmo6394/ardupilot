#pragma once

#include <AP_HAL/AP_HAL.h>

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

  // Start and register sensor instances
  void start(void) override;

  // update state
  // void update(void) override;

private:
  // bool get_reading(Vector3f &gyro, Vector3f &accel);
  bool get_DMU11_data(void);
  AP_HAL::UARTDriver *uart = nullptr;
  uint8_t _gyro_instance;
  uint8_t _accel_instance;
};
