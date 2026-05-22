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

#include <algorithm>
#include <array>
#include <bitset>

#include <cstdint>
#include <libhal-sensor/as5600.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/map.hpp>

namespace hal::sensor {

constexpr auto as5600_raw_range = std::make_pair(0, 4095);
constexpr auto as5600_degree_range = std::make_pair(0.0f, 359.0f);

as5600::as5600(hal::strong_ptr<hal::i2c> const& p_i2c)
  : m_i2c(p_i2c)
{
  m_narrowed_range.first = start_angle();
  m_narrowed_range.second = stop_angle();
  if (m_narrowed_range.first == 0.0 && m_narrowed_range.second == 0) {
    m_narrowed_range.second = m_narrowed_range.first + angular_range();
    if (m_narrowed_range.second == 0) {
      m_narrowed_range.second = as5600_degree_range.second;
    }
  }
}

void as5600::write_angle_to_register(hal::byte p_register, hal::degrees p_angle)
{
  hal::degrees const clamped_angle =
    std::clamp(p_angle, as5600_degree_range.first, as5600_degree_range.second);
  uint16_t const position =
    hal::map(clamped_angle, as5600_degree_range, as5600_raw_range);

  hal::byte const hi_byte_base =
    as5600::read_register<1>(p_register)[0] & 0b11110000;
  hal::byte const low_byte = position;
  hal::byte const hi_byte = hi_byte_base | (position >> 8);
  hal::write(*m_i2c,
             m_address,
             std::array<hal::byte, 3>{ p_register, hi_byte, low_byte });
}

hal::degrees as5600::start_angle()
{
  auto register_bytes = as5600::read_register<2>(0x01);

  auto const low_byte = register_bytes[1];
  auto const hi_byte = (register_bytes[0] & 0b1111);
  uint16_t const combined = (hi_byte << 8) | low_byte;
  return hal::map(combined, as5600_raw_range, as5600_degree_range);
}

hal::degrees as5600::stop_angle()
{
  auto register_bytes = as5600::read_register<2>(0x03);

  auto const low_byte = register_bytes[1];
  auto const hi_byte = (register_bytes[0] & 0b1111);
  uint16_t const combined = (hi_byte << 8) | low_byte;
  return hal::map(combined, as5600_raw_range, as5600_degree_range);
}

hal::degrees as5600::angular_range()
{
  auto register_bytes = as5600::read_register<2>(0x05);

  auto const low_byte = register_bytes[1];
  auto const hi_byte = (register_bytes[0] & 0b1111);
  uint16_t const combined = (hi_byte << 8) | low_byte;
  return hal::map(combined, as5600_raw_range, as5600_degree_range);
}

as5600::power_mode_config as5600::power_mode()
{
  return static_cast<power_mode_config>(as5600::read_register<1>(0x08)[0] &
                                        0b11);
}

as5600::hysteresis_config as5600::hysteresis()
{
  return static_cast<hysteresis_config>(as5600::read_register<1>(0x08)[0] &
                                        0b1100);
}

bool as5600::watchdog_enabled()
{
  auto const conf_byte = as5600::read_register<1>(0x07)[0];
  std::bitset<8> conf_bits{ conf_byte };
  return conf_bits[5];
}

void as5600::start_angle(hal::degrees p_angle)
{
  auto const clamped_angle =
    std::clamp(p_angle, as5600_degree_range.first, as5600_degree_range.second);
  m_narrowed_range.first = clamped_angle;
  write_angle_to_register(0x01, clamped_angle);
}

void as5600::stop_angle(hal::degrees p_angle)
{
  auto const clamped_angle =
    std::clamp(p_angle, as5600_degree_range.first, as5600_degree_range.second);
  m_narrowed_range.second = clamped_angle;
  write_angle_to_register(0x03, clamped_angle);
}

void as5600::angular_range(hal::degrees p_angle)
{
  auto const clamped_angle =
    std::clamp(p_angle,
               as5600_degree_range.first,
               as5600_degree_range.second - m_narrowed_range.first);
  m_narrowed_range.second = m_narrowed_range.first + clamped_angle;
  write_angle_to_register(0x05, p_angle);
}

void as5600::power_mode(as5600::power_mode_config p_power_mode)
{
  hal::byte bits = std::to_underlying(p_power_mode);
  auto conf_byte = as5600::read_register<1>(0x08)[0];
  conf_byte = conf_byte & 0b11111100;
  conf_byte = conf_byte | bits;
  hal::write(*m_i2c, m_address, std::array<hal::byte, 2>{ 0x08, conf_byte });
}

void as5600::hysteresis(as5600::hysteresis_config p_hysteresis)
{
  hal::byte bits = std::to_underlying(p_hysteresis);
  auto conf_byte = as5600::read_register<1>(0x08)[0];
  conf_byte = conf_byte & 0b11110011;
  conf_byte = conf_byte | bits;
  hal::write(*m_i2c, m_address, std::array<hal::byte, 2>{ 0x08, conf_byte });
}

void as5600::watchdog(bool p_enabled)
{
  hal::byte conf_byte = as5600::read_register<1>(0x07)[0];
  std::bitset<8> conf_bits{ conf_byte };
  conf_bits.set(5, p_enabled);
  conf_byte = static_cast<uint8_t>(conf_bits.to_ulong());
  hal::write(*m_i2c, m_address, std::to_array<hal::byte>({ 0x07, conf_byte }));
}

hal::degrees as5600::raw_angle()
{
  auto register_bytes = as5600::read_register<2>(0x0C);

  auto const low_byte = register_bytes[1];
  auto const hi_byte = (register_bytes[0] & 0b1111);
  uint16_t const combined = (hi_byte << 8) | low_byte;
  return hal::map(combined, as5600_raw_range, as5600_degree_range);
}

hal::degrees as5600::angle()
{
  auto register_bytes = as5600::read_register<2>(0x0E);

  auto const low_byte = register_bytes[1];
  auto const hi_byte = (register_bytes[0] & 0b1111);
  uint16_t const combined = (hi_byte << 8) | low_byte;
  return hal::map(combined, as5600_raw_range, m_narrowed_range);
}

as5600::magnet as5600::magnet_status()
{
  auto const status_byte = as5600::read_register<1>(0x0B)[0];
  std::bitset<8> status_bits{ status_byte };

  as5600::magnet magnet;
  magnet.detected = status_bits[5];
  magnet.too_strong = status_bits[3];
  magnet.too_weak = status_bits[4];
  return magnet;
}

uint8_t as5600::auto_gain_control()
{
  return as5600::read_register<1>(0x1A)[0];
}

uint16_t as5600::magnitude()
{
  auto register_bytes = as5600::read_register<2>(0x1B);

  auto const low_byte = register_bytes[1];
  auto const hi_byte = (register_bytes[0] & 0b1111);

  return (hi_byte << 8) | low_byte;
}

}  // namespace hal::sensor
