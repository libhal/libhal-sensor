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

#pragma once

#include <libhal-util/bit.hpp>
#include <libhal/i2c.hpp>
#include <libhal/units.hpp>

namespace hal::sensor {
/**
 * @brief The AS5600 is a magnetic rotary position sensor with a high-resolution
 * 12-bit i2c output.
 *
 * This contactless system measures the absolute angle of a diametric magnetized
 * on-axis magnet. By default the output represents a range from 0 to 360
 * degrees. It is also possible to define a smaller range to the output by
 * programming a zero angle (start position) and a maximum angle (stop
 * position). The AS5600 is also equipped with a smart low power mode feature to
 * automatically reduce the power consumption.
 *
 */
class as5600
{
public:
  /**
   * @brief Power mode settings available to use when configuring.
   *
   */
  enum class power_mode_config
  {
    normal_mode,
    low_power_mode_1,
    low_power_mode_2,
    low_power_mode_3,
  };

  /**
   * @brief Hysteresis mode settings available to use when configuring.
   *
   */
  enum class hysteresis_config
  {
    off,
    lsb_1,
    lsb_2,
    lsb_3
  };

  /**
   * @brief Construct a new as5600 object
   *
   * @param p_i2c i2c bus of the device
   */
  as5600(hal::i2c& p_i2c);

  /**
   * @brief Get the start angle to use when using a narrower angular range.
   *
   * Used in combination with either a stop angle or the max angle of
   * the angular range. The angular range must be greater than 18
   * degrees.
   *
   * @return hal::degrees - Start angle (0 - 360 degree range).
   */
  hal::degrees start_angle();

  /**
   * @brief Get the stop angle to use when using a narrower angular range.
   *
   * The angular range must be greater than 18 degrees.
   *
   * @return hal::degrees - Stop angle (0 - 360 degree range).
   */
  hal::degrees stop_angle();

  /**
   * @brief Get the max angle to use when using a narrower angular range.
   *
   * The angular range must be greater than 18 degrees.
   *
   * @return hal::degrees - Max angle (0 - 360 degree range).
   */
  hal::degrees max_angle();

  /**
   * @brief Get the power mode setting for device. Normal operation mode is
   * provided with 3 low power modes.
   *
   * @return power_mode_config - Current power mode
   */
  power_mode_config power_mode();

  /**
   * @brief Get the hysteresis setting for device. 1 - 3 LSB hystersis settings
   * are available.
   *
   * @return hysteresis_config - Current hysteresis mode.
   */
  hysteresis_config hysteresis();

  /**
   * @brief Get the status of the watchdog timer
   *
   * @return true - Watchdog timer enabled
   * @return false - Watchdog timer disabled
   */
  bool watchdog_enabled();

  /**
   * @brief Set the start angle to use when using a narrower angular range.
   *
   * @param p_angle - Start angle (0 - 360 degree range)
   */
  void start_angle(hal::degrees p_angle);

  /**
   * @brief Set the stop angle to use when using a narrower angular range.
   *
   * @param p_angle - Stop angle (0 - 360 degree range)
   */
  void stop_angle(hal::degrees p_angle);

  /**
   * @brief Set the max angle to use when using a narrower angular range.
   *
   * @param p_angle - Max angle (0 - 360 degree range)
   */
  void max_angle(hal::degrees p_angle);

  /**
   * @brief Set the power mode setting for device.
   *
   * @param p_power_mode - Power mode to use.
   */
  void power_mode(power_mode_config p_power_mode);

  /**
   * @brief Set the hysteresis mode setting for the device
   *
   * @param p_hysteresis - Hysteresis mode to use.
   */
  void hysteresis(hysteresis_config p_hysteresis);

  /**
   * @brief Enable or disable the watchdog timer.
   *
   * @param p_enabled - Enabled status of watchdog timer
   */
  void watchdog(bool p_enabled);

  /**
   * @brief Get the unscaled and unmodified angle
   *
   * @return hal::degrees - Raw angle (0 - 360 degree range)
   */
  hal::degrees raw_angle();

  /**
   * @brief Get the scaled angle.
   *
   * Angle register has a 10-LSB hysteresis at the limit of the 360 degree range
   * to avoid discontinuity points or toggling of the output within one
   * rotation.
   *
   * @return hal::degrees
   */
  hal::degrees angle();

  /**
   * @brief Get the state of magnet detected status bit.
   *
   * @return true - Magnet is detected.
   * @return false - No magnet detected.
   */
  bool magnet_detected();

  /**
   * @brief Get the state of the magnet too strong status bit.
   *
   * Will be true if automatic gain control minimum gain overflows.
   *
   * @return true - Magnet too strong.
   * @return false - Magnet not too strong.
   */
  bool magnet_too_strong();

  /**
   * @brief Get the state of the magnet too weak status bit.
   *
   * Will be true if automatic gain control maximum gain overflows.
   *
   * @return true - Magnet too weak.
   * @return false - Magnet not too weak.
   */
  bool magnet_too_weak();

  /**
   * @brief Get the current gain of the automatic gain control.
   *
   * @return uint8_t - Current gain value.
   */
  uint8_t auto_gain_control();

  /**
   * @brief Get the magnitude value of the internal CORDIC
   *
   * @return uint16_t - Magnitude value
   */
  uint16_t magnitude();

private:
  hal::byte read_register(hal::byte p_register_address);

  hal::i2c* m_i2c;
  hal::byte m_address = 0x36;
};
}  // namespace hal::sensor
