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

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_DMU11.h"
#include <AP_SerialManager/AP_SerialManager.h>
// #include <ctype.h>

// Declare external reference to HAL to gain access to namespace objects
extern const AP_HAL::HAL& hal;

AP_InertialSensor_DMU11::AP_InertialSensor_DMU11(AP_InertialSensor &imu,
                                                 AP_SerialManager &serial_manager) :
    AP_InertialSensor_Backend(imu)
{
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_DMU11, 0);
    if (uart != nullptr) {
        uart->begin(serial_manager.find_baudrate(AP_SerialManager::SerialProtocol_DMU11, 0));
    }
}

/*
  SerialProtocol_DMU11 sets the serial baud to 115k, RX buffer to 40 bytes, and stop bits to 2
*/
bool AP_InertialSensor_DMU11::detect(AP_SerialManager &serial_manager)
{
    return serial_manager.find_serial(AP_SerialManager::SerialProtocol_DMU11, 0) != nullptr;
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
      hal.console->printf("%c",c)
      count++;
    }



    if (count == 0) {
        return false;
    }
    // reading_cm = 100 * sum / count;
    return true;
}

/*
   update the state of the sensor
*/
/*
void AP_InertialSensor_DMU11::update(void)
{
    if (get_reading(imu.gyro,imu.accel)) {  // Need to determine what inputs here
        // update range_valid state based on distance measured
        last_reading_ms = AP_HAL::millis();
        update_status();
    } else if (AP_HAL::millis() - last_reading_ms > 200) {
        // set_status(RangeFinder::RangeFinder_NoData);
    }
}
*/
