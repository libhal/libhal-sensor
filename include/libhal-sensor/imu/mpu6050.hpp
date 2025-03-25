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

#pragma once

#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/map.hpp>
#include <libhal/accelerometer.hpp>

namespace hal::sensor {
class mpu6050 : public hal::accelerometer
{
public:
  /// The device address when A0 is connected to GND.
  static constexpr hal::byte address_ground = 0b110'1000;
  /// The device address when A0 is connected to V+.
  static constexpr hal::byte address_voltage_high = 0b110'1001;

  enum class max_acceleration : hal::byte
  {
    /// 2x the average earth gravity of acceleration
    g2 = 0x00,
    /// 4x the average earth gravity of acceleration
    g4 = 0x01,
    /// 8x the average earth gravity of acceleration
    g8 = 0x02,
    /// 16x the average earth gravity of acceleration
    g16 = 0x03,
  };

  /**
   * @brief Construct an mpu6050 driver
   *
   * @param p_i2c - the driver for the i2c bus the MPU6050 is connected to
   * @param p_address - mpu6050 device address
   * @throws hal::no_such_device - when an invalid MPU6050 device is detected.
   * MPU6050 devices have a read-only ID register which allows a microcontroller
   * to determine what device it is connected to. This register will be read and
   * if it does not match the expected value, this exception is thrown.
   */
  explicit mpu6050(i2c& p_i2c, hal::byte p_address = address_ground);

  /**
   * @brief Changes the gravity scale that the MPU is reading. The larger the
   * scale, the less precise the reading.
   *
   * @param p_gravity_code - Scales in powers of 2 up to 16.
   */
  void configure_full_scale(max_acceleration p_gravity_code);

  /**
   * @brief Power on the device
   */
  void power_on();

  /**
   * @brief Power dow the device
   */
  void power_off();

private:
  accelerometer::read_t driver_read() override;

  /// The I2C peripheral used for communication with the device.
  hal::i2c* m_i2c;
  /// Gravity scale is the maximum absolute value of acceleration in units of
  /// earth's gravity (g) that the device will read.
  hal::byte m_gscale;
  /// The address used to communicate with the device.
  hal::byte m_address;
};

}  // namespace hal::sensor
