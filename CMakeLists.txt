# Copyright 2024 Khalil Estell
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
cmake_minimum_required(VERSION 3.15)

project(libhal-sensor LANGUAGES CXX)

libhal_test_and_make_library(
  LIBRARY_NAME libhal-sensor

  SOURCES
  src/imu/icm20948.cpp
  src/imu/mpu6050.cpp
  src/multi/bmp180.cpp
  src/multi/mpl3115a2.cpp
  src/temperature/tmp102.cpp

  TEST_SOURCES
  tests/imu/mpu6050.test.cpp
  tests/imu/mpu6050.test.cpp
  tests/temperature/tmp102.test.cpp
  tests/main.test.cpp
)
