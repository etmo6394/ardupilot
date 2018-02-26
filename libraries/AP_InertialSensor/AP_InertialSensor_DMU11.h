#pragma once

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"
#include <AP_SerialManager/AP_SerialManager.h>



class AP_InertialSensor_DMU11 : public AP_InertialSensor_Backend
{
public:
  // constructor
  AP_InertialSensor_DMU11(AP_InertialSensor &imu);

  // Static probe function
  static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu);

  // Start and register sensor instances
  void start(void) override;

  void accumulate(void) override;

  bool update(void) override;

  // update state
  // void update(void) override;

private:
  // bool get_reading(Vector3f &gyro, Vector3f &accel);
  AP_HAL::UARTDriver *uart = nullptr;
  uint8_t _gyro_instance;
  uint8_t _accel_instance;
  char linebuf[40];
  uint8_t linebuf_len = 0;
};
