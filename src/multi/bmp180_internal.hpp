#pragma once

#include <cstdint>

#include <libhal/units.hpp>

namespace hal::sensor::bmp180_internal {

// Struct to hold the computation variables. The names of the computation
// variables are taken directly from the datasheet. The datasheet does not
// explain what these names mean, so we keep the names as they are defined.
struct computation_variables
{
  std::int32_t x1, x2, x3;
  std::int32_t b3, b5, b6;
  std::uint32_t b4, b7;
  std::int32_t temperature, pressure;
};

// Compute the temperature in steps of 0.1C.
// These calculations are done the way they are without bit_modify as its
// following the exact algorithm for conversion given in figure 4 of the
// datasheet.
constexpr inline computation_variables calculate_true_temperature(
  std::int32_t p_uncompensated_temperature,
  std::uint16_t p_ac5,
  std::uint16_t p_ac6,
  std::int16_t p_mc,
  std::int16_t p_md)
{
  computation_variables variables;

  variables.x1 = ((p_uncompensated_temperature - p_ac6) * p_ac5) >> 15;
  variables.x2 = (p_mc << 11) / (variables.x1 + p_md);
  variables.b5 = variables.x1 + variables.x2;
  variables.temperature = (variables.b5 + 8) >> 4;

  return variables;
}

// Convert the raw pressure data into Pascals in steps of 1Pa.
// These calculations are done the way they are without bit_modify as its
// following the exact algorithm for conversion given in figure 4 of the
// datasheet.
constexpr inline computation_variables calculate_true_pressure(
  std::int32_t p_uncompensated_pressure,
  std::int16_t p_oversampling_setting,
  std::int32_t p_b5,
  std::int16_t p_ac1,
  std::int16_t p_ac2,
  std::int16_t p_ac3,
  std::uint16_t p_ac4,
  std::int16_t p_b1,
  std::int16_t p_b2)
{
  computation_variables variables;

  variables.b6 = p_b5 - 4000;
  variables.x1 = (p_b2 * ((variables.b6 * variables.b6) >> 12)) >> 11;
  variables.x2 = (p_ac2 * variables.b6) >> 11;
  variables.x3 = variables.x1 + variables.x2;
  variables.b3 = ((((static_cast<std::int32_t>(p_ac1)) * 4 + variables.x3)
                   << p_oversampling_setting) +
                  2) >>
                 2;
  variables.x1 = (p_ac3 * variables.b6) >> 13;
  variables.x2 = (p_b1 * ((variables.b6 * variables.b6) >> 12)) >> 16;
  variables.x3 = ((variables.x1 + variables.x2) + 2) >> 2;
  variables.b4 =
    (p_ac4 * static_cast<std::uint32_t>(variables.x3 + 32768)) >> 15;
  variables.b7 =
    (static_cast<std::uint32_t>(p_uncompensated_pressure) - variables.b3) *
    (50000 >> p_oversampling_setting);
  if (variables.b7 < 0x80000000) {
    variables.pressure = (variables.b7 * 2) / variables.b4;
  } else {
    variables.pressure = (variables.b7 / variables.b4) * 2;
  }
  variables.x1 = variables.pressure >> 8;
  variables.x1 *= variables.x1;
  variables.x1 = (variables.x1 * 3038) >> 16;
  variables.x2 = (-7357 * variables.pressure) >> 16;
  variables.pressure += (variables.x1 + variables.x2 + 3791) >> 4;

  return variables;
}

}  // namespace hal::sensor::bmp180_internal
