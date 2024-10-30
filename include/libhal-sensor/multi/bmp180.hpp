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

#include <cstdint>

#include <libhal/i2c.hpp>
#include <libhal/units.hpp>

namespace hal::sensor {

/**
 * @brief Driver for the BMP180 barometric pressure/temperature sensor
 */
class bmp180
{
public:
  /**
   * @brief Defines the oversampling rate with names containing the mode as
   * well as sample fetch times.
   */
  enum class oversampling_rate : hal::byte
  {
    /// 3 internal samples by sensor
    ulta_low_power_mode_4500us = 0x00,
    /// 5 internal samples by sensor
    standard_mode_7500us = 0x01,
    /// 9 internal samples by sensor
    high_resolution_mode_13500us = 0x02,
    /// 17 internal samples by sensor
    ultra_high_resolution_mode_25500us = 0x03,
  };

  /**
   * @brief Stores the temperature and pressure data that are both found when
   * calling pressure().
   */
  struct pressure_results
  {
    hal::celsius temperature;
    // Pressure is in units of Pascals
    float pressure;
  };

  /**
   * @brief Construct a bmp180 driver
   *
   * @param p_i2c - The driver for the SPI bus the BMP180 is connected to.
   * @param p_oversampling_setting - The specified oversampling setting. When
   * the sensor is requested for a single pressure sample, this setting
   * determines how many internal samples the sensor takes and averages before
   * returning the singular sample. The higher the oversampling rate, the less
   * RMS noise from the sensor. See documentation for `oversampling_rate`
   * enumeration class to know the internal samples taken for each mode.
   *
   * @throws hal::no_such_device - when an invalid BMP180 device is detected.
   * BMP180 devices have a read-only ID register which allows a microcontroller
   * to determine what device it is connected to. This register will be read and
   * if it does not match the expected value, this exception is thrown.
   * @throws hal::io_error - when something is wrong with the data
   * communication. The calibration data that the sensor uses needs to all have
   * non-boundary values, if any are 0x0000 or 0xFFFF then this is thrown.
   */
  explicit bmp180(hal::i2c& p_i2c,
                  oversampling_rate p_oversampling_setting =
                    oversampling_rate::ulta_low_power_mode_4500us);

  /**
   * @brief Reads the temperature
   *
   * @returns the temperature in celsius.
   */
  [[nodiscard]] hal::celsius temperature();

  /**
   * @brief Takes a pressure reading
   *
   * @param sample_amount - The amount of samples to take. This differs from the
   * oversampling setting because the oversampling setting describes how many
   * internal samples the sensor takes and averages before returning a single
   * pressure sample when requested. This setting takes x amount of those
   * returned singular samples, and then averages them for greater accuracy.
   * Since the sensor has a peak current of 650uA, and each singular sample
   * varies in both time and current consumption due to the oversampling
   * setting, this parameter is clamped to various maximum sample_amount's
   *
   * The maximum samples per oversampling rate setting is as follows:
   *
   *     - ulta_low_power_mode_4500us = 214 samples
   *     - standard_mode_7500us = 128 samples
   *     - high_resolution_mode_13500us = 72 samples
   *     - ultra_high_resolution_mode_25500us = 38 samples
   * @return pressure_results which contains the pressure and temperature
   */
  [[nodiscard]] pressure_results pressure(int sample_amount = 1);

  /**
   * @brief Sends a reset signal to the device to do a power-on sequence
   */
  void reset();

private:
  /**
   * @brief This stores the 11 words of calibration data that is unique to each
   * BMP180. This needs to be stored to be used for conversion from raw data to
   * usable data.
   */
  struct calibration_coefficients
  {
    // The names of the calibration values are taken directly from the
    // datasheet. The datasheet does not explain what these names mean, so we
    // keep the names as they are defined.
    std::int16_t ac1{}, ac2{}, ac3{};
    std::uint16_t ac4{}, ac5{}, ac6{};
    std::int16_t b1{}, b2{};
    std::int16_t mb{}, mc{}, md{};
  };
  /// The I2C peripheral used for communication with the device.
  hal::i2c* m_i2c;
  /// The address the device uses with i2c
  static constexpr hal::byte m_address = 0b111'0111;
  /// The calibration data thats unique to this sensor
  calibration_coefficients m_calibration_data;
  /// The oversampling rate for the sensors internal samples
  oversampling_rate m_oversampling_setting;
  /// The maximum amount of samples per second with the given oversampling rate
  int m_maximum_samples;
};

}  // namespace hal::sensor
