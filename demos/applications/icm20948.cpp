// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cmath>

#include <libhal-sensor/imu/icm20948.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

float compute_heading(float x, float y, float offset = 0.0)
{
  float angle = 360 - (atan2(y, x) * (180.0 / std::numbers::pi));
  angle += offset;  // Apply offset
  if (angle < 0) {
    angle += 360;
  } else if (angle >= 360) {
    angle -= 360;
  }
  return angle;
}

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& i2c = *p_map.i2c.value();

  hal::print(console, "icm Application Starting...\n\n");
  hal::delay(clock, 200ms);
  hal::sensor::icm20948 icm_device(i2c);

  hal::delay(clock, 200ms);
  icm_device.init_mag();
  hal::delay(clock, 100ms);

  icm_device.auto_offsets();

  while (true) {
    auto accel = icm_device.read_acceleration();
    hal::delay(clock, 10ms);

    auto gyro = icm_device.read_gyroscope();
    hal::delay(clock, 10ms);

    auto temp = icm_device.read_temperature();
    hal::delay(clock, 10ms);

    auto mag = icm_device.read_magnetometer();
    hal::delay(clock, 10ms);

    hal::print(console, "\n\n================Reading IMU================\n");

    hal::print<128>(console,
                    "\n\nG-Accel Values:    x = %fg, y = %fg, z = %fg",
                    accel.x,
                    accel.y,
                    accel.z);

    hal::print<128>(console,
                    "\n\nGyro Values:       x = %f,  y = %f,  z = %f",
                    gyro.x,
                    gyro.y,
                    gyro.z);

    hal::print<128>(console, "\n\nCurrent Temperature: %f°C", temp.temp);

    hal::print<128>(console,
                    "\n\nMagnetometer Values: x = %f, y = %f, z = %f",
                    mag.x,
                    mag.y,
                    mag.z);

    float heading = compute_heading(mag.x, mag.y, 0.0);
    hal::print<128>(console, "\n\nHeading: %f°", heading);
    hal::print(console, "\n\n===========================================\n");
  }
}