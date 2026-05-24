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

#include <utility>

#include <libhal-util/bit.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal/i2c.hpp>
#include <libhal/pointers.hpp>
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
  enum class power_mode_config : u8
  {
    /// Normal mode with max sampling rate results in a current consumption
    /// of: 6.5 mA
    poll_max_6500uA = 0,
    /// Polling period = 5ms, current consumption of = 3.4 mA
    poll_5ms_3400uA = 0b01,
    /// Polling period = 20ms, current consumption of = 1.8 mA
    poll_20ms_1800uA = 0b10,
    /// Polling period = 100ms, current consumption of = 1.5 mA
    poll_100ms_1500uA = 0b11,
  };

  /**
   * @brief Hysteresis mode settings available to use when configuring. Check
   * data sheet for more information.
   *
   */
  enum class hysteresis_config : u8
  {
    off = 0,
    lsb_1 = 0b0100,
    lsb_2 = 0b1000,
    lsb_3 = 0b1100
  };

  /**
   * @brief Magnet status object.
   *
   */
  struct magnet
  {
    /**
     * @brief Magnet detected flag. Is true when magnet is detected.
     *
     */
    bool detected;
    /**
     * @brief Magnet too strong flag. Is true when magnetic field is too strong
     * (possibly too close to sensor).
     *
     */
    bool too_strong;
    /**
     * @brief Magnet too weak flag. Is true when magnetic field is too weak
     * (possibly too far from sensor).
     *
     */
    bool too_weak;
  };

  /**
   * @brief Construct a new as5600 object
   *
   * @param p_i2c i2c bus of the device
   */
  as5600(hal::strong_ptr<hal::i2c> const& p_i2c);

  /**
   * @brief Get the start angle to use when using a narrower angular range.
   *
   * Used in combination with either a stop angle or the angular range
   * register. The angular range must be greater than 18 degrees.
   *
   * @return hal::degrees - Start angle (0 - 360 degree range).
   */
  hal::degrees start_angle();

  /**
   * @brief Get the stop angle to use when using a narrower angular range.
   *
   * @return hal::degrees - Stop angle (0 - 360 degree range).
   */
  hal::degrees stop_angle();

  /**
   * @brief Get the angular range value.
   *
   * @return hal::degrees - Angular range (0 - 360 degree range).
   */
  hal::degrees angular_range();

  /**
   * @brief Get the power mode setting for device.
   *
   * @return power_mode_config - Current power mode
   */
  power_mode_config power_mode();

  /**
   * @brief Get the hysteresis setting for device.
   *
   * Check the AS5600 datasheet for more information about hysteresis.
   *
   * @return hysteresis_config - Current hysteresis mode.
   */
  hysteresis_config hysteresis();

  /**
   * @brief Get the status of the watchdog timer
   *
   * The watchdog timer allows saving power by switching into low power mode 3
   * (100 ms polling time) if the angle stays within the watchdog threshold of 4
   * LSB for at least one minute
   *
   * @return true - Watchdog timer enabled
   * @return false - Watchdog timer disabled
   */
  bool watchdog_enabled();

  /**
   * @brief Set the start raw angle to use when using a narrower angular range.
   * Any angle lower than this will be reported as a raw value of 0 until moved
   * back into tracking range.
   *
   * The angular range is split up between 4096 steps. Reducing the range will
   * increase the step resolution and must be greater than 18 degrees.
   *
   * @param p_angle - Start angle (0 - 360 degree range)
   */
  void start_angle(hal::degrees p_angle);

  /**
   * @brief Set the stop angle to use when using a narrower angular range.
   * Any angle higher than this will be reported as a raw value of 4095 until
   * moved back into tracking range.
   *
   * The angular range is split up between 4096 steps. Reducing the range will
   * increase the step resolution and must be greater than 18 degrees.
   *
   * @param p_angle - Stop angle (0 - 360 degree range)
   */
  void stop_angle(hal::degrees p_angle);

  /**
   * @brief Set the angular range instead of manually setting the stop angle
   *
   * stop angle = start angle + angular range
   *
   * @param p_angle - Angular range (0 - 360 degree range)
   */
  void angular_range(hal::degrees p_angle);

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
   * @brief Get the angle scaled to narrower angle range specified with start
   * position in combination with either stop position or angular range.
   *
   * Angle register has a hysteresis of 10-LSB at the limit of the 360 degree
   * range to avoid discontinuity points or toggling of the output within one
   * rotation. Check datasheet for more information.
   *
   * Explanation of hysteresis: the 10-LSB simply means that when the angle
   * crosses from 4095 (max 12-bit unsigned integer value) to 0, the sensor
   * waits until the angle represented by the value of 10 is reached, then the
   * angle value jumps from 4095 to 10. Rotating the angle back towards 0 will
   * decrease from 10 until 0 is reached. When the zero crossing is reached, the
   * value will stay 0 until the angle represented by 4085 is reached, then the
   * value will change.
   *
   * @return hal::degrees
   */
  hal::degrees angle();

  magnet magnet_status();

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
  template<usize Size>
  auto read_register(hal::byte p_register_address)
  {
    hal::write(
      *m_i2c, m_address, std::array<hal::byte, 1>{ p_register_address });
    return hal::read<Size>(*m_i2c, m_address);
  }

  void write_angle_to_register(hal::byte p_register, hal::degrees p_angle);

  hal::strong_ptr<hal::i2c> m_i2c;
  static constexpr hal::byte m_address = 0x36;

  std::pair<hal::degrees, hal::degrees> m_narrowed_range;
};
}  // namespace hal::sensor
