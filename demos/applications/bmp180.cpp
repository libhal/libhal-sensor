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
    hal::sensor::bmp180 bmp(i2c, clock);

    hal::print(
        console,
        "Testing sensor reset...\n\n");
    bmp.reset();

    hal::print(
        console,
        "--------------------------------\n");

    int counter = 0;
    while (true) {
        hal::print(
        console,
        "Doing temperature readout...\n");

        auto temperature = bmp.temperature();
        hal::print<64>(console,
                    "Temperature: %f°C \n\n",
                    temperature);
        hal::print(
        console,
        "Doing pressure readout...\n");
        
        auto pressure = bmp.pressure(1) / 1000;
        hal::print<64>(console,
                    "Pressure: %fkPa \n\n",
                    pressure);

        hal::print<64>(
        console,
        "Updating oversampling setting to mode %i\n",
        counter);

        switch(counter) {
            case 0:
                bmp.update_oversampling_rate(hal::sensor::bmp180::oversampling_rate::ulta_low_power_mode_4500us);
                break;

            case 1:
                bmp.update_oversampling_rate(hal::sensor::bmp180::oversampling_rate::standard_mode_7500us);   
                break;

            case 2:
                bmp.update_oversampling_rate(hal::sensor::bmp180::oversampling_rate::high_resolution_mode_13500us);
                break;

            case 3:
                bmp.update_oversampling_rate(hal::sensor::bmp180::oversampling_rate::ultra_high_resolution_mode_25500us);
                break;

            default:
                bmp.update_oversampling_rate(hal::sensor::bmp180::oversampling_rate::ulta_low_power_mode_4500us);
                break;
        }
        counter = (counter == 3) ? 0 : counter + 1;

        hal::print(
        console,
        "--------------------------------\n");
    }
}
