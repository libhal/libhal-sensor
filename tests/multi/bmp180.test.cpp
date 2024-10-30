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

#include <libhal-sensor/multi/bmp180.hpp>

#include "multi/bmp180_internal.hpp"

#include <boost/ut.hpp>

namespace hal::sensor {
boost::ut::suite test_bmp180 = []() {
  using namespace boost::ut;
  using namespace std::literals;

  "bmp180_internal::calculate_true_temperature()"_test = []() {
    // Setup
    std::int32_t uncompensated_temperature = 27898;
    std::uint16_t ac5 = 32757;
    std::uint16_t ac6 = 23153;
    std::int16_t mc = -8711;
    std::int16_t md = 2868;

    bmp180_internal::computation_variables variables_expected = { .x1 = 4743,
                                                                  .x2 = -2343,
                                                                  .x3 = 0,
                                                                  .b3 = 0,
                                                                  .b5 = 2400,
                                                                  .b6 = 0,
                                                                  .b4 = 0,
                                                                  .b7 = 0,
                                                                  .temperature =
                                                                    150,
                                                                  .pressure =
                                                                    0 };

    // Exercise
    auto variables_result = bmp180_internal::calculate_true_temperature(
      uncompensated_temperature, ac5, ac6, mc, md);

    // Verify
    expect(variables_result.x1 == variables_expected.x1);
    expect(variables_result.x2 == variables_expected.x2);
    expect(variables_result.b5 == variables_expected.b5);
    expect(variables_result.temperature == variables_expected.temperature);
  };

  "bmp180_internal::calculate_true_pressure()"_test = []() {
    // Setup
    std::int32_t uncompensated_pressure = 23843;
    std::int16_t oversampling_setting = 0;
    std::int32_t b5 = 2400;
    std::int16_t ac1 = 408;
    std::int16_t ac2 = -72;
    std::int16_t ac3 = -14383;
    std::uint16_t ac4 = 32741;
    std::int16_t b1 = 6190;
    std::int16_t b2 = 4;

    bmp180_internal::computation_variables variables_expected = {
      .x1 = 3454,
      .x2 = -7859,
      .x3 = 717,
      .b3 = 422,
      .b5 = 0,
      .b6 = -1600,
      .b4 = 33457,
      .b7 = 1171050000,
      .temperature = 0,
      .pressure = 69964
    };

    // Exercise
    auto variables_result =
      bmp180_internal::calculate_true_pressure(uncompensated_pressure,
                                               oversampling_setting,
                                               b5,
                                               ac1,
                                               ac2,
                                               ac3,
                                               ac4,
                                               b1,
                                               b2);

    // Verify
    expect(variables_result.x1 == variables_expected.x1);
    expect(variables_result.x2 == variables_expected.x2);
    expect(variables_result.x3 == variables_expected.x3);
    expect(variables_result.b3 == variables_expected.b3);
    expect(variables_result.b6 == variables_expected.b6);
    expect(variables_result.b4 == variables_expected.b4);
    expect(variables_result.b7 == variables_expected.b7);
    expect(variables_result.pressure == variables_expected.pressure);
  };
};
}  // namespace hal::sensor
