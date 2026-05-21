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
#include <bitset>

#include <cstdint>
#include <libhal-sensor/as5600.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/map.hpp>

namespace hal::sensor {

as5600::as5600(hal::i2c& p_i2c)
  : m_i2c(&p_i2c)
{
}

hal::byte as5600::read_register(hal::byte p_register_address)
{
  hal::write(*m_i2c, m_address, std::array<hal::byte, 1>{ p_register_address });
  std::array<hal::byte, 1> response{};
  hal::read(*m_i2c, m_address, response);
  return response[0];
}

hal::degrees as5600::start_angle()
{
  auto const low_byte = read_register(0x02);
  auto const hi_byte = (read_register(0x01) & 0b1111) /*only take last 4 bits*/;
  uint16_t const combined = (hi_byte << 8) | low_byte;

  return hal::map(
    combined, std::make_pair(0, 4095), std::make_pair(0.0, 360.0));
}

hal::degrees as5600::stop_angle()
{
  auto const low_byte = read_register(0x04);
  auto const hi_byte = (read_register(0x03) & 0b1111) /*only take last 4 bits*/;
  uint16_t const combined = (hi_byte << 8) | low_byte;

  return hal::map(
    combined, std::make_pair(0, 4095), std::make_pair(0.0, 360.0));
}

hal::degrees as5600::max_angle()
{
  auto const low_byte = read_register(0x05);
  auto const hi_byte = (read_register(0x06) & 0b1111) /*only take last 4 bits*/;
  uint16_t const combined = (hi_byte << 8) | low_byte;

  return hal::map(
    combined, std::make_pair(0, 4095), std::make_pair(0.0, 360.0));
}

as5600::power_mode_config as5600::power_mode()
{
  auto conf_byte = read_register(0x08);
  conf_byte = conf_byte & 0b11;
  switch (conf_byte) {
    case 0b01:
      return power_mode_config::low_power_mode_1;
    case 0b10:
      return power_mode_config::low_power_mode_2;
    case 0b11:
      return power_mode_config::low_power_mode_3;
    default:
      return power_mode_config::normal_mode;
  }
}

as5600::hysteresis_config as5600::hysteresis()
{
  auto conf_byte = read_register(0x08);
  conf_byte = (conf_byte >> 2) & 0b11;
  switch (conf_byte) {
    case 0b01:
      return hysteresis_config::lsb_1;
    case 0b10:
      return hysteresis_config::lsb_2;
    case 0b11:
      return hysteresis_config::lsb_3;
    default:
      return hysteresis_config::off;
  }
}

bool as5600::watchdog_enabled()
{
  auto const conf_byte = read_register(0x07);
  std::bitset<8> conf_bits{ conf_byte };
  return conf_bits[5];
}

void as5600::start_angle(hal::degrees p_angle)
{
  hal::degrees const clamped_angle = std::clamp(p_angle, 0, 360);
  uint16_t const position = hal::map(
    clamped_angle, std::make_pair(0.0, 360.0), std::make_pair(0, 4095));

  hal::byte const hi_byte_base = read_register(0x01) & 0b11110000;
  hal::byte const low_byte = position;
  hal::byte const hi_byte = hi_byte_base | (position >> 8);
  hal::write(
    *m_i2c, m_address, std::array<hal::byte, 3>{ 0x01, hi_byte, low_byte });
}

void as5600::stop_angle(hal::degrees p_angle)
{
  hal::degrees const clamped_angle = std::clamp(p_angle, 0, 360);
  uint16_t const position = hal::map(
    clamped_angle, std::make_pair(0.0, 360.0), std::make_pair(0, 4095));

  hal::byte const hi_byte_base = read_register(0x03) & 0b11110000;
  hal::byte const low_byte = position;
  hal::byte const hi_byte = hi_byte_base | (position >> 8);
  hal::write(
    *m_i2c, m_address, std::array<hal::byte, 3>{ 0x03, hi_byte, low_byte });
}

void as5600::max_angle(hal::degrees p_angle)
{
  hal::degrees const clamped_angle = std::clamp(p_angle, 0, 360);
  uint16_t const position = hal::map(
    clamped_angle, std::make_pair(0.0, 360.0), std::make_pair(0, 4095));

  hal::byte const hi_byte_base = read_register(0x06) & 0b11110000;
  hal::byte const low_byte = position;
  hal::byte const hi_byte = hi_byte_base | (position >> 8);
  hal::write(
    *m_i2c, m_address, std::array<hal::byte, 3>{ 0x06, hi_byte, low_byte });
}

void as5600::power_mode(as5600::power_mode_config p_power_mode)
{
  hal::byte bits = 0b00;
  switch (p_power_mode) {
    case power_mode_config::low_power_mode_1:
      bits = 0b01;
      break;
    case power_mode_config::low_power_mode_2:
      bits = 0b10;
      break;
    case power_mode_config::low_power_mode_3:
      bits = 0b11;
      break;
    default:
      bits = 0b00;
  }

  auto conf_byte = read_register(0x08);
  conf_byte = conf_byte & 0b11111100;
  conf_byte = conf_byte | bits;
  hal::write(*m_i2c, m_address, std::array<hal::byte, 2>{ 0x08, conf_byte });
}

void as5600::hysteresis(as5600::hysteresis_config p_hysteresis)
{
  hal::byte bits = 0b00;
  switch (p_hysteresis) {
    case hysteresis_config::lsb_1:
      bits = 0b0100;
      break;
    case hysteresis_config::lsb_2:
      bits = 0b1000;
      break;
    case hysteresis_config::lsb_3:
      bits = 0b1100;
      break;
    default:
      bits = 0b0000;
  }

  auto conf_byte = read_register(0x08);
  conf_byte = conf_byte & 0b11110011;
  conf_byte = conf_byte | bits;
  hal::write(*m_i2c, m_address, std::array<hal::byte, 2>{ 0x08, conf_byte });
}

void as5600::watchdog(bool p_enabled)
{
  hal::byte conf_byte = read_register(0x07);
  std::bitset<8> conf_bits{ conf_byte };
  conf_bits.set(5, p_enabled);
  conf_byte = static_cast<uint8_t>(conf_bits.to_ulong());
  hal::write(*m_i2c, m_address, std::array<hal::byte, 2>{ 0x07, conf_byte });
}

hal::degrees as5600::raw_angle()
{
  auto const low_byte = read_register(0x0D);
  auto const hi_byte = (read_register(0x0C) & 0b1111) /*only take last 4 bits*/;
  uint16_t const combined = (hi_byte << 8) | low_byte;

  return hal::map(
    combined, std::make_pair(0, 4095), std::make_pair(0.0, 360.0));
}

hal::degrees as5600::angle()
{
  auto const low_byte = read_register(0x0F);
  auto const hi_byte = (read_register(0x0E) & 0b1111) /*only take last 4 bits*/;
  uint16_t const combined = (hi_byte << 8) | low_byte;

  return hal::map(
    combined, std::make_pair(0, 4095), std::make_pair(0.0, 360.0));
}

bool as5600::magnet_detected()
{
  auto const status_byte = read_register(0x0B);
  std::bitset<8> status_bits{ status_byte };
  return status_bits[5];
}

bool as5600::magnet_too_strong()
{
  auto const status_byte = read_register(0x0B);
  std::bitset<8> status_bits{ status_byte };
  return status_bits[3];
}

bool as5600::magnet_too_weak()
{
  auto const status_byte = read_register(0x0B);
  std::bitset<8> status_bits{ status_byte };
  return status_bits[4];
}

uint8_t as5600::auto_gain_control()
{
  return read_register(0x1A);
}

uint16_t as5600::magnitude()
{
  auto const low_byte = read_register(0x1C);
  auto const hi_byte = (read_register(0x1B) & 0b1111) /*only take last 4 bits*/;

  return (hi_byte << 8) | low_byte;
}

}  // namespace hal::sensor
