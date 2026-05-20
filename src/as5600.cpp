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

#include <bitset>

#include <libhal-sensor/as5600.hpp>
#include <libhal-util/i2c.hpp>

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

uint16_t as5600::get_raw_angle()
{
  auto const low_byte = read_register(0x0D);
  auto const hi_byte = (read_register(0x0C) & 0b1111) /*only take last 4 bits*/;

  return (hi_byte << 8) | low_byte;
}

uint16_t as5600::get_angle()
{
  auto const low_byte = read_register(0x0F);
  auto const hi_byte = (read_register(0x0E) & 0b1111) /*only take last 4 bits*/;

  return (hi_byte << 8) | low_byte;
}

uint16_t as5600::get_zpos()
{
  auto const low_byte = read_register(0x02);
  auto const hi_byte = (read_register(0x01) & 0b1111) /*only take last 4 bits*/;

  return (hi_byte << 8) | low_byte;
}

uint16_t as5600::get_mpos()
{
  auto const low_byte = read_register(0x04);
  auto const hi_byte = (read_register(0x03) & 0b1111) /*only take last 4 bits*/;

  return (hi_byte << 8) | low_byte;
}

uint16_t as5600::get_max_angle()
{
  auto const low_byte = read_register(0x05);
  auto const hi_byte = (read_register(0x06) & 0b1111) /*only take last 4 bits*/;

  return (hi_byte << 8) | low_byte;
}

void as5600::set_zpos(uint16_t p_position)
{
  hal::byte const hi_byte_base = read_register(0x01) & 0b11110000;
  hal::byte const low_byte = p_position;
  hal::byte const hi_byte = hi_byte_base | (p_position >> 8);
  hal::write(
    *m_i2c, m_address, std::array<hal::byte, 3>{ 0x01, hi_byte, low_byte });
}

void as5600::set_mpos(uint16_t p_position)
{
  hal::byte const hi_byte_base = read_register(0x03) & 0b11110000;
  hal::byte const low_byte = p_position;
  hal::byte const hi_byte = hi_byte_base | (p_position >> 8);
  hal::write(
    *m_i2c, m_address, std::array<hal::byte, 3>{ 0x03, hi_byte, low_byte });
}

void as5600::set_max_angle(uint16_t p_angle)
{
  hal::byte hi_byte_base = read_register(0x06) & 0b11110000;
  hal::byte low_byte = p_angle;
  hal::byte hi_byte = hi_byte_base | (p_angle >> 8);
  hal::write(
    *m_i2c, m_address, std::array<hal::byte, 3>{ 0x06, hi_byte, low_byte });
}

}  // namespace hal::sensor
