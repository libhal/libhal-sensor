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

#include <libhal-sensor/multi/bmp180.hpp>
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

  hal::print(console, "BMP180 Application Starting...\n\n");
  hal::sensor::bmp180 bmp(i2c);

  hal::print(console, "Testing sensor reset...\n\n");
  bmp.reset();

  while (true) {
    hal::print(console, "Doing temperature readout...\n");
    auto temperature = bmp.temperature();
    hal::print<64>(console, "Temperature: %.2f°C \n\n", temperature);

    hal::print(console,
               "Doing pressure + temperature readout (10 samples)...\n");
    auto pressure = bmp.pressure(10);
    hal::print<64>(console,
                   "Pressure: %.2fPa, Temperature: %.2f°C \n\n",
                   pressure.pressure,
                   pressure.temperature);
    hal::delay(clock, 500ms);
  }
}
