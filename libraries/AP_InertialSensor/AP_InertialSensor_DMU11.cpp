/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
// spec sheet: https://www.siliconsensing.com/media/30801/dmu11-00-0100-132-rev-3.pdf


#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_DMU11.h"
//#include <AP_SerialManager/AP_SerialManager.h>
// #include <ctype.h>

// Declare external reference to HAL to gain access to namespace objects
extern const AP_HAL::HAL& hal;


//AP_InertialSensor_DMU11::AP_InertialSensor_DMU11(AP_InertialSensor &imu,
//                                                 AP_SerialManager &serial_manager) :
AP_InertialSensor_DMU11::AP_InertialSensor_DMU11(AP_InertialSensor &imu) :
    AP_InertialSensor_Backend(imu)
{
    AP_SerialManager &serial_manager = AP::serialmanager();

    hal.console->printf("Creating new dmu11 obj\n");
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_DMU11, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_DMU11, 0));
        hal.console->printf("Started sensor on uartE\n");
    }
}


/*
  SerialProtocol_DMU11 sets the serial baud to 115k, RX buffer to 40 bytes, and stop bits to 2
// // // bool AP_InertialSensor_DMU11::detect(AP_SerialManager &serial_manager)
// // // {
// // //     hal.console->printf("Detected DMU11 at 115k baud");
// // //     return serial_manager.find_serial(AP_SerialManager::SerialProtocol_DMU11, 0) != nullptr;
// // // }
*/




AP_InertialSensor_Backend *AP_InertialSensor_DMU11::probe(AP_InertialSensor &imu)
 //                                                         AP_SerialManager &serial_manager)
{
    AP_SerialManager &serial_manager = AP::serialmanager();
  // Return nullptr if no sensor is connected on uartE
  if (serial_manager.find_serial(AP_SerialManager::SerialProtocol_DMU11, 0) == nullptr){
    hal.console->printf("No detected sensor on uartE\n");
    return nullptr;
  }
  // Otherwise declare pointer to new object
  //AP_InertialSensor_DMU11 *sensor = new AP_InertialSensor_DMU11(imu,serial_manager);
  AP_InertialSensor_DMU11 *sensor = new AP_InertialSensor_DMU11(imu);
  hal.console->printf("Detected sensor on uartE\n");
  return sensor;
}





/*
  "Start" the sensor, register the DMU11 as one new gyro and one new accel
*/
void AP_InertialSensor_DMU11::start(void)
{
  // _gyro_instance = _imu.register_DMU11_gyro();
  // _accel_instance = _imu.register_DMU11_accel();
  _gyro_instance = _imu.register_gyro(0,0);
  hal.console->printf("Registered DMU11 gyro [%u]", _gyro_instance);
  _accel_instance = _imu.register_accel(0,0);
  hal.console->printf("Registered DMU11 accel [%u]", _accel_instance);

}

// read - return last value measured by sensor
// Vector3f stuff, refer to AP_InertialSensor.cpp lines ~1400-1600
// bool AP_InertialSensor_DMU11::get_reading(Vector3f &gyro, Vector3f &accel)
bool AP_InertialSensor_DMU11::get_DMU11_data(void)
{
    if (uart == nullptr) {
        return false;
    }
    uint16_t count = 0;
    int16_t nbytes = uart->available();
    while (nbytes-- > 0) {
      // read byte from buffer
      char c = uart->read();
      // immediately print to pixhawk console to verify data
      hal.console->printf("%c",c);

      count++;
    }



    if (count == 0) {
        return false;
    }
    // reading_cm = 100 * sum / count;
    return true;
}


/*
   Copy filtered data to frontend
*/
// bool AP_InertialSensor_DMU11::update(void);
// {
//   update_gyro(_gyro_instance);
//   update_accel(_accel_instance);
// }
bool AP_InertialSensor_DMU11::update(void)
{
  return false;
}
