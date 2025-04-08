// Copyright 2024 - 2025 Khalil Estell and the libhal contributors
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

#include <libhal-sensor/imu/mpu6050.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& i2c = *p_map.i2c.value();

  hal::print(console, "MPU6050 Application Starting...\n");
  hal::sensor::mpu6050 mpu(i2c);

  while (true) {
    hal::print(
      console,
      "Setting acceleration max scale to 2g (2 earth gravities)... \n");

    mpu.configure_full_scale(hal::sensor::mpu6050::max_acceleration::g2);

    hal::print(console, "Reading acceleration... \n");
    auto acceleration = mpu.read();

    hal::print<128>(console,
                    "Scale: 2g:\tx = %fg, y = %fg, z = %fg \n",
                    acceleration.x,
                    acceleration.y,
                    acceleration.z);

    hal::delay(clock, 500ms);

    hal::print(
      console,
      "Setting acceleration max scale to 4g (4 earth gravities)... \n");

    mpu.configure_full_scale(hal::sensor::mpu6050::max_acceleration::g4);
    acceleration = mpu.read();
    hal::print<128>(console,
                    "Scale: 4g:\tx = %fg, y = %fg, z = %fg \n\n",
                    acceleration.x,
                    acceleration.y,
                    acceleration.z);
  }
}
