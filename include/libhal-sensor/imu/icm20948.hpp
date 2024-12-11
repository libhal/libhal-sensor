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

#include <libhal/i2c.hpp>
#include <libhal/timeout.hpp>
#include <libhal/units.hpp>

namespace hal::sensor {

/**
 * @brief Driver for the icm20948 inertial measurement unit
 *
 */
class icm20948
{
public:
  enum class cycle : hal::byte
  {
    no_cycle = 0x00,
    gyro_cycle = 0x10,
    acc_cycle = 0x20,
    acc_gyro_cycle = 0x30,
    acc_gyro_i2c_mst_cycle = 0x70
  };

  enum class int_pin_pol : hal::byte
  {
    act_high,
    act_low
  };

  enum class int_type : hal::byte
  {
    fsync_int = 0x01,
    wom_int = 0x02,
    dmp_int = 0x04,
    data_ready_int = 0x08,
    fifo_ovf_int = 0x10,
    fifo_wm_int = 0x20
  };

  enum class fifo_type : hal::byte
  {
    fifo_acc = 0x10,
    fifo_gyr = 0x0E,
    fifo_acc_gyr = 0x1E
  };

  enum fifo_mode_choice : hal::byte
  {
    continuous,
    stop_when_full
  };

  enum gyro_range : hal::byte
  {
    gyro_range_250,
    gyro_range_500,
    gyro_range_1000,
    gyro_range_2000
  };

  enum digital_lowpass_filter : hal::byte
  {
    dlpf_0,
    dlpf_1,
    dlpf_2,
    dlpf_3,
    dlpf_4,
    dlpf_5,
    dlpf_6,
    dlpf_7,
    dlpf_off
  };

  enum gyro_avg_low_power : hal::byte
  {
    gyro_avg_1,
    gyro_avg_2,
    gyro_avg_4,
    gyro_avg_8,
    gyro_avg_16,
    gyro_avg_32,
    gyro_avg_64,
    gyro_avg_128
  };

  enum acc_range : hal::byte
  {
    acc_range_2g,
    acc_range_4g,
    acc_range_8g,
    acc_range_16g
  };

  enum acc_avg_low_power : hal::byte
  {
    acc_avg_4,
    acc_avg_8,
    acc_avg_16,
    acc_avg_32
  };

  enum wom_comp : hal::byte
  {
    wom_comp_disable,
    wom_comp_enable
  };

  enum ak09916_op_mode : hal::byte
  {
    ak09916_pwr_down = 0x00,
    ak09916_trigger_mode = 0x01,
    ak09916_cont_mode_10hz = 0x02,
    ak09916_cont_mode_20hz = 0x04,
    ak09916_cont_mode_50hz = 0x06,
    ak09916_cont_mode_100hz = 0x08
  };

  enum orientation : hal::byte
  {
    flat,
    flat_1,
    xy,
    xy_1,
    yx,
    yx_1
  };

  struct accel_read_t
  {
    float x;
    float y;
    float z;
  };

  struct gyro_read_t
  {
    float x;
    float y;
    float z;
  };

  struct mag_read_t
  {
    float x;
    float y;
    float z;
  };

  struct temp_read_t
  {
    float temp;
  };

  /*
   * Struct to define acceleration offsets. All units are in G's
   * This struct can be instantiated and passed as a
   *   parameter to icm20948::set_acceleration_offsets
   */
  struct acceleration_offset_t
  {
    float xmin;
    float xmax;
    float ymin;
    float ymax;
    float zmin;
    float zmax;
  };

  /*
   * Struct to define gyroscope offsets. All units are in degrees/second
   * This struct can be instantiated and passed as a
   *   parameter to icm20948::set_gyro_offsets
   */
  struct gyro_offset_t
  {
    float x_offset;
    float y_offset;
    float z_offset;
  };

  /**
   * @brief Read acceleration data from out_x_msb_r, out_x_lsb_r,
   *        out_y_msb_r, out_y_lsb_r, out_z_msb_r, out_z_lsb_r
   *        and perform acceleration conversion to g.
   */
  [[nodiscard]] accel_read_t read_acceleration();

  /**
   * @brief Read gyroscope data from out_x_msb_r, out_x_lsb_r,
   *        out_y_msb_r, out_y_lsb_r, out_z_msb_r, out_z_lsb_r
   *        and perform gyroscope conversion to rad/s.
   */
  [[nodiscard]] gyro_read_t read_gyroscope();

  /**
   * @brief Read magnetometer data from out_x_msb_r, out_x_lsb_r,
   *        out_y_msb_r, out_y_lsb_r, out_z_msb_r, out_z_lsb_r
   *        and perform magnetometer conversion to uT.
   */
  [[nodiscard]] mag_read_t read_magnetometer();

  /**
   * @brief Read pressure data from out_t_msb_r and out_t_lsb_r
   *        and perform temperature conversion to celsius.
   */
  [[nodiscard]] temp_read_t read_temperature();

  /**
   * @brief private constructor for icm20948 objects
   * @param p_i2c The I2C peripheral used for communication with the device.
   */
  icm20948(hal::i2c& p_i2c);

  void auto_offsets();

  /**
   * @brief Set default acceleration offsets
   *    All offset parameters are in 'g's
   * @param acc_offsets Acceleration offset struct.
   */
  void set_acceleration_offsets(acceleration_offset_t const& acc_offsets);

  /**
   * @brief Set default gyroscope offsets
   *    All offset parameters are in degrees/second
   * @param gyro_offsets Gyro offset struct.
   */
  void set_gyro_offsets(gyro_offset_t const& gyro_offsets);

  /**
   * @brief Read & return whoami register value
   */
  hal::byte whoami();

  void enable_acc(bool p_en_acc);
  void set_acc_range(acc_range p_acc_range);
  void set_acc_dlpf(digital_lowpass_filter p_dlpf);
  void set_acc_sample_rate_div(uint16_t p_acc_spl_rate_div);
  void enable_gyro(bool p_enable_gyro);
  [[deprecated("Use the API `enable_gyro()` with a full name.")]]
  void enable_gyr(bool p_enable_gyro)
  {
    enable_gyro(p_enable_gyro);
  }
  void set_gyro_range(gyro_range p_gyro_range);
  void set_gyro_dlpf(digital_lowpass_filter p_dlpf);
  void set_gyro_sample_rate_div(hal::byte p_gyro_spl_rate_div);
  void set_temp_dlpf(digital_lowpass_filter p_dlpf);

  /* Power, Sleep, Standby */
  void enable_cycle(cycle p_cycle);
  void enable_low_power(bool p_enable_low_power);
  void set_gyro_averg_cycle_mode(gyro_avg_low_power p_avg);
  void set_acc_averg_cycle_mode(acc_avg_low_power p_avg);
  void sleep(bool p_sleep);

  /* Magnetometer */
  void init_mag();
  void enable_bypass_mode();
  hal::byte mag_status1();
  hal::byte mag_status2();
  void reset_mag();
  hal::byte check_mag_mode();
  hal::byte whoami_ak09916_wia1_direct();
  hal::byte whoami_ak09916_wia2_direct();
  void set_mag_op_mode(ak09916_op_mode p_op_mode);
  void write_ak09916_register8(hal::byte p_reg, hal::byte p_val);

private:
  void set_clock_auto_select();
  void switch_bank(hal::byte p_new_bank);

  struct write8_param
  {
    hal::byte bank;
    hal::byte reg;
    hal::byte val;
  };

  struct write16_param
  {
    hal::byte bank;
    hal::byte reg;
    std::uint16_t val;
  };
  void write_register8(write8_param p_param);
  void write_register16(write16_param p_param);

  struct read_param
  {
    hal::byte bank;
    hal::byte reg;
  };

  [[nodiscard]] hal::byte read_register8(read_param p_param);
  [[nodiscard]] std::uint16_t read_register16(read_param p_param);
  void enable_mag_data_read(hal::byte p_reg, hal::byte p_bytes);

  void reset_icm20948();

  /* The I2C peripheral used for communication with the device. */
  hal::i2c* m_i2c;

  hal::byte m_current_bank;
  accel_read_t m_acc_offset_val;
  accel_read_t m_acc_corr_factor;
  gyro_read_t m_gyro_offset_val;
  hal::byte m_acc_range_factor;
  hal::byte m_gyro_range_factor;
  hal::byte m_reg_val;  // intermediate storage of register values
};
}  // namespace hal::sensor
