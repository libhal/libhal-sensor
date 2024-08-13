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

#pragma once

#include <libhal/i2c.hpp>
#include <libhal/temperature_sensor.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

namespace hal::tmp {
class tmp102 : public hal::temperature_sensor
{
public:
  /// The device address when A0 is connected to GND.
  static constexpr hal::byte address_ground = 0b100'1000;
  /// The device address when A0 is connected to V+.
  static constexpr hal::byte address_voltage_high = 0b100'1001;
  /// The device address when A0 is connected to SDA.
  static constexpr hal::byte address_sda = 0b100'1010;
  /// The device address when A0 is connected to SCL.
  static constexpr hal::byte address_scl = 0b100'1011;

  /**
   * @brief Construct a new tmp102 object
   *
   * Will probe the bus to see if the address provided is acknowledge by a
   * device.
   *
   * @param p_i2c - The I2C peripheral used for communication with the device.
   * @param p_device_address - The device address of the sensor. The address is
   * configured by physically modifying the connection of the P0 pin.
   * @throws hal::no_such_device - if no device responds on the bus when probed
   * by this constructor.
   */
  tmp102(hal::i2c& p_i2c, hal::byte p_device_address = address_ground);

private:
  hal::celsius driver_read() override;

  /// Sets the device to use one-shot shutdown mode. This allows power to be
  /// conserved by putting the device in the shutdown state once a reading is
  /// obtained.
  void one_shot_shutdown();

  /// The I2C peripheral used for communication with the device.
  hal::i2c* m_i2c;
  /// The configurable device address used for communication.
  hal::byte m_address;
};
}  // namespace hal::tmp
