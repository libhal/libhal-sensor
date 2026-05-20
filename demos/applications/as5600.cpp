// Copyright 2026 Malia Labor and the libhal contributors
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

#include <libhal-sensor/as5600.hpp>
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

  hal::print(console, "AS5600 Application Starting...\n");
  hal::sensor::as5600 hall_sensor(i2c);

  if (hall_sensor.magnet_detected()) {
    hal::print(console, "Magnet detected\n");
  }
  auto const burn_count = hall_sensor.burn_count();
  auto const start_angle = hall_sensor.start_angle();
  auto const stop_angle = hall_sensor.stop_angle();
  auto const max_angle = hall_sensor.max_angle();
  auto const power_mode = hall_sensor.power_mode();
  auto const hysteresis = hall_sensor.hysteresis();
  bool watchdog_enabled = hall_sensor.watchdog_enabled();

  auto const agc = hall_sensor.auto_gain_control();
  auto const mag = hall_sensor.magnitude();

  hal::print<32>(console, "Burn count: %d \n", burn_count);
  hal::print<32>(console, "Start angle: %.2f \n", start_angle);
  hal::print<32>(console, "Stop angle: %.2f \n", stop_angle);
  hal::print<32>(console, "Max Angle: %.2f \n", max_angle);
  hal::print<32>(console, "Power Mode: %d \n", power_mode);
  hal::print<32>(console, "Hysteresis: %d \n", hysteresis);
  hal::print<32>(console, "WD: %d \n", watchdog_enabled);
  hal::print<32>(console, "AGC: %d \n", agc);
  hal::print<32>(console, "Magnitude: %d \n", mag);

  while (true) {
    if (hall_sensor.magnet_detected()) {
      auto raw_angle = hall_sensor.raw_angle();
      auto angle = hall_sensor.angle();

      hal::print<64>(console, "Raw Angle: %.2f    ", raw_angle);
      hal::print<64>(console, "Angle: %.2f \n", angle);
    }
    hal::delay(clock, 500ms);
  }
}
