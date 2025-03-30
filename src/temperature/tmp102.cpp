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

#include <libhal-sensor/temperature/tmp102.hpp>

#include <array>
#include <span>

#include <libhal-util/as_bytes.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal/i2c.hpp>
#include <libhal/temperature_sensor.hpp>
#include <libhal/timeout.hpp>

namespace hal::tmp {

/// The address of the read-only register containing the temperature data.
static constexpr hal::byte temperature_register = 0x00;
/// The address of the register used to configure the device.
static constexpr hal::byte configuration_register = 0x01;

/// The command to enable one-shot shutdown mode.
static constexpr hal::byte one_shot_shutdown_mode = 0x81;

tmp102::tmp102(hal::i2c& p_i2c, hal::byte p_device_address)
  : m_i2c(&p_i2c)
  , m_address(p_device_address)
{
  // Attempt to sample the temperature to determine if the device is available
  // on the bus.
  [[maybe_unused]] auto value = tmp102::driver_read();
}

hal::celsius tmp102::driver_read()
{
  std::array<hal::byte, 1> payload{ temperature_register };
  constexpr float celsius_per_digit = 0.0625f;
  std::array<hal::byte, 2> buffer;

  one_shot_shutdown();

  hal::write_then_read(
    *m_i2c, m_address, payload, buffer, hal::never_timeout());

  // The temperature value is from bits [15:3], with the rest being zeros
  // The MSB is received first in the byte.
  int32_t const temperature_reading = (buffer[0] << 4) | (buffer[1] >> 4);

  return static_cast<float>(temperature_reading) * celsius_per_digit;
}

/// Sets the device to use one-shot shutdown mode. This allows power to be
/// conserved by putting the device in the shutdown state once a reading is
/// obtained.
void tmp102::one_shot_shutdown()
{
  hal::write(
    *m_i2c,
    m_address,
    std::array<hal::byte, 2>{ configuration_register, one_shot_shutdown_mode },
    hal::never_timeout());
}
}  // namespace hal::tmp
