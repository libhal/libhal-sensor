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

#include <libhal-sensor/imu/mpu6050.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/map.hpp>
#include <libhal/accelerometer.hpp>
#include <libhal/error.hpp>

namespace hal::sensor {

namespace {

/// The address of the read-only register containing the temperature data.
static constexpr hal::byte xyz_register = 0x3B;
/// The address of the register used to configure gravity scale the device.
static constexpr hal::byte configuration_register = 0x1C;
/// The address of the register used to initilize the device.
static constexpr hal::byte initalizing_register = 0x6B;
/// The command to enable one-shot shutdown mode.
static constexpr hal::byte who_am_i_register = 0x75;

void active_mode(hal::i2c& p_i2c, hal::byte p_address, bool p_is_active)
{
  constexpr auto sleep_mask = hal::bit_mask::from<6>();

  auto control =
    hal::write_then_read<1>(p_i2c,
                            p_address,
                            std::array{ hal::sensor::initalizing_register },
                            hal::never_timeout())[0];

  hal::bit_modify(control).insert<sleep_mask>(!p_is_active);

  hal::write(p_i2c,
             p_address,
             std::array{ hal::sensor::initalizing_register, control },
             hal::never_timeout());
}
}  // namespace

mpu6050::mpu6050(hal::i2c& p_i2c, hal::byte p_device_address)
  : m_i2c(&p_i2c)
  , m_address(p_device_address)
{
  constexpr std::array<hal::u8, 2> expected_ids = {
    0x68,  // real mpu6050 who_am_i value
    0x72,  // fake mpu6050 who_am_i value (but still works)
  };
  // Read out the identity register
  auto const device_id =
    hal::write_then_read<1>(*m_i2c,
                            m_address,
                            std::array{ hal::sensor::who_am_i_register },
                            hal::never_timeout())[0];

  bool id_verified = false;
  for (auto const& id : expected_ids) {
    if (device_id == id) {
      id_verified = true;
      break;
    }
  }

  if (not id_verified) {
    hal::safe_throw(hal::no_such_device(m_address, this));
  }

  power_on();
}

void mpu6050::configure_full_scale(max_acceleration p_gravity_code)
{
  constexpr auto scale_mask = hal::bit_mask::from<3, 4>();

  m_gscale = static_cast<hal::byte>(p_gravity_code);

  auto config = hal::write_then_read<1>(*m_i2c,
                                        m_address,
                                        std::array{ configuration_register },
                                        hal::never_timeout())[0];

  hal::bit_modify(config).insert<scale_mask>(m_gscale);

  hal::write(*m_i2c,
             m_address,
             std::array{ configuration_register, config },
             hal::never_timeout());
}

void mpu6050::power_on()
{
  return active_mode(*m_i2c, m_address, true);
}

void mpu6050::power_off()
{
  return active_mode(*m_i2c, m_address, false);
}

accelerometer::read_t mpu6050::driver_read()
{
  accelerometer::read_t acceleration;
  constexpr std::size_t bytes_per_axis = 2;
  constexpr std::size_t number_of_axis = 3;

  std::array<hal::byte, bytes_per_axis * number_of_axis> xyz_acceleration;
  hal::write_then_read(*m_i2c,
                       m_address,
                       std::array{ xyz_register },
                       xyz_acceleration,
                       hal::never_timeout());
  /**
   * First X-axis Byte (MSB first)
   * =========================================================================
   * Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0
   *  XD15 | XD14  |  XD13 |  XD12 |  XD11 |  XD10 |  XD9  |  XD8
   *
   * Final X-axis Byte (LSB)
   * =========================================================================
   * Bit 7 | Bit 6 | Bit 5 | Bit 4 | Bit 3 | Bit 2 | Bit 1 | Bit 0
   *   XD7 |   XD6 |   XD5 |   XD4 |   XD3 |   XD2 |   XD1 |   XD0
   *
   *
   * We simply shift and OR the bytes together to get them into a signed int
   * 16 value.
   */
  auto const x =
    static_cast<std::int16_t>(xyz_acceleration[0] << 8 | xyz_acceleration[1]);
  auto const y =
    static_cast<std::int16_t>(xyz_acceleration[2] << 8 | xyz_acceleration[3]);
  auto const z =
    static_cast<std::int16_t>(xyz_acceleration[4] << 8 | xyz_acceleration[5]);

  // Convert the 16 bit value into a floating point value m/S^2
  constexpr auto max =
    static_cast<float>(std::numeric_limits<std::int16_t>::max());
  constexpr auto min =
    static_cast<float>(std::numeric_limits<std::int16_t>::min());

  auto const output_limits =
    static_cast<float>(1 << (static_cast<int>(m_gscale) + 1));
  auto input_range = std::make_pair(max, min);
  auto output_range = std::make_pair(-output_limits, output_limits);

  acceleration.x = hal::map(x, input_range, output_range);
  acceleration.y = hal::map(y, input_range, output_range);
  acceleration.z = hal::map(z, input_range, output_range);

  return acceleration;
}

}  // namespace hal::sensor
