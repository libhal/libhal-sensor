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
#include <libhal/steady_clock.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

namespace hal::sensor {

class bmp180 
{
public:

enum class oversampling_rate : hal::byte
{
    ulta_low_power_mode_4500us = 0x00,
    standard_mode_7500us = 0x01,
    high_resolution_mode_13500us = 0x02,
    ultra_high_resolution_mode_25500us = 0x03,
};

explicit bmp180(hal::i2c& p_i2c, hal::steady_clock& p_clock, oversampling_rate p_oversampling_setting = oversampling_rate::ulta_low_power_mode_4500us);

void update_oversampling_rate(oversampling_rate p_oversampling_setting);

[[nodiscard]] hal::celsius temperature(/*timeout param*/);

[[nodiscard]] float pressure(int sample_amount = 1/*timeout param*/);

void reset(/*timeout param*/);

private:

// specify naming reason
struct calibration_coefficients {
    std::int16_t ac1{}, ac2{}, ac3{};
    std::uint16_t ac4{}, ac5{}, ac6{};
    std::int16_t b1{}, b2{};
    std::int16_t mb{}, mc{}, md{};
};

struct computation_variables {
    std::int32_t x1, x2, x3;
    std::int32_t b3, b5, b6;
    std::uint32_t b4, b7;
    std::int32_t temperature, pressure;
};

hal::i2c* m_i2c;

hal::steady_clock* m_clock;

oversampling_rate m_oversampling_setting;

calibration_coefficients m_calibration_data;

computation_variables m_computation_variables;

static constexpr hal::byte m_address = 0b111'0111;
};



} // namespace hal::sensor