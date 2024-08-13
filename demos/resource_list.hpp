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

#include <optional>

#include <libhal/functional.hpp>
#include <libhal/i2c.hpp>
#include <libhal/output_pin.hpp>
#include <libhal/serial.hpp>
#include <libhal/steady_clock.hpp>

struct resource_list
{
  hal::callback<void()> reset;
  std::optional<hal::serial*> console = std::nullopt;
  std::optional<hal::steady_clock*> clock = std::nullopt;
  std::optional<hal::output_pin*> status_led = std::nullopt;
  std::optional<hal::i2c*> i2c = std::nullopt;
  // Add more driver interfaces here ...
};

// Application function is implemented by one of the .cpp files.
resource_list initialize_platform();
void application(resource_list& p_map);
