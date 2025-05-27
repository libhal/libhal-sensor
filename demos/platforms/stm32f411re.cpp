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

#include <libhal-arm-mcu/dwt_counter.hpp>
#include <libhal-arm-mcu/startup.hpp>
#include <libhal-arm-mcu/stm32f411/clock.hpp>
#include <libhal-arm-mcu/stm32f411/constants.hpp>
#include <libhal-arm-mcu/stm32f411/i2c.hpp>
#include <libhal-arm-mcu/stm32f411/output_pin.hpp>
#include <libhal-arm-mcu/stm32f411/uart.hpp>
#include <libhal-arm-mcu/system_control.hpp>
#include <libhal/io_waiter.hpp>
#include <libhal/units.hpp>

#include <resource_list.hpp>

void initialize_platform(resource_list& p_resources)
{
  using namespace hal::literals;
  using st_peripheral = hal::stm32f411::peripheral;

  // Set the MCU to the maximum clock speed
  hal::stm32f411::maximum_speed_using_internal_oscillator();

  auto const cpu_frequency = hal::stm32f411::frequency(st_peripheral::cpu);
  static hal::cortex_m::dwt_counter steady_clock(cpu_frequency);

  static hal::stm32f411::i2c_manager<st_peripheral::i2c1> i2c1_manager;
  static auto i2c =
    i2c1_manager.acquire_i2c(hal::i2c::settings{ .clock_rate = 100_kHz });

  static hal::stm32f411::uart uart2(
    hal::port<2>, hal::buffer<128>, { .baud_rate = 115200 });

  static hal::stm32f411::output_pin led(st_peripheral::gpio_a, 5);

  p_resources.clock = &steady_clock;
  p_resources.console = &uart2;
  p_resources.i2c = &i2c;
  p_resources.status_led = &led;
}
