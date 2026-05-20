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

namespace hal::sensor {
class as5600
{
public:
  enum class power_mode_config
  {
    normal_mode,  // double check this
    low_power_mode_1,
    low_power_mode_2,
    low_power_mode_3,
  };

  enum class hysteresis_config
  {
    off,
    lsb_1,
    lsb_2,
    lsb_3
  };

  as5600(hal::i2c& p_i2c);

  uint8_t zmco();
  uint16_t zpos();
  uint16_t mpos();
  uint16_t max_angle();
  power_mode_config power_mode();
  hysteresis_config hysteresis();
  bool watchdog_enabled();

  void zpos(uint16_t p_position);
  void mpos(uint16_t p_position);
  void max_angle(uint16_t p_angle);
  void power_mode(power_mode_config p_power_mode);
  void hysteresis(hysteresis_config p_hysteresis);
  void watchdog(bool p_enabled);

  uint16_t raw_angle();
  uint16_t angle();

  bool magnet_detected();
  bool magnet_too_strong();
  bool magnet_too_weak();
  uint8_t auto_gain_control();
  uint16_t magnitude();

private:
  hal::byte read_register(hal::byte p_register_address);

  hal::i2c* m_i2c;
  hal::byte m_address = 0x36;
};
}  // namespace hal::sensor
