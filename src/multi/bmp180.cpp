#include <libhal-sensor/multi/bmp180.hpp>

#include <array>

#include <libhal-util/i2c.hpp>
#include <libhal-util/steady_clock.hpp>
#include <libhal/error.hpp>

using namespace std::literals;
namespace hal::sensor {
namespace {

auto device_id_register()
{
  return std::to_array<hal::byte>({ 0xD0 });
}

auto calibration_coefficients_register()
{
  return std::to_array<hal::byte>({ 0xAA });
}

auto control_measurement_register()
{
  return std::to_array<hal::byte>({ 0xF4 });
}

auto uncompensated_output_data_register()
{
  return std::to_array<hal::byte>({ 0xF6 });
}

auto soft_reset_register()
{
  return std::to_array<hal::byte>({ 0xE0 });
}

int conversion_status(hal::i2c* p_i2c, hal::byte p_address) {
  std::array<hal::byte, 1> buffer;
  hal::write_then_read(*p_i2c, p_address, control_measurement_register(), buffer, hal::never_timeout());
  hal::byte check_bit = ((buffer[0]) >> 5) & 1;
  if (check_bit == 0) return 1;
  else return 0;
}

}  // namespace

bmp180::bmp180(hal::i2c& p_i2c, hal::steady_clock& p_clock, oversampling_rate p_oversampling_setting)
  : m_i2c(&p_i2c)
  , m_clock(&p_clock)
  , m_oversampling_setting(p_oversampling_setting)
{
  static constexpr hal::byte expected_device_id = 0x55;

  auto device_id = hal::write_then_read<1>(*m_i2c, m_address, device_id_register(), hal::never_timeout())[0];

  if (device_id != expected_device_id) {
    hal::safe_throw(hal::no_such_device(m_address, this));
  }

  // create buffer to store returned calibration register data
  constexpr hal::byte calibration_registers_amount = 22;
  std::array<hal::byte, calibration_registers_amount> calibration_data_buffer;
  hal::write_then_read(*m_i2c, m_address, calibration_coefficients_register(), calibration_data_buffer, hal::never_timeout());

  // take the raw calibration data and convert it into the appropriate variables
  m_calibration_data.ac1 = (calibration_data_buffer[0] << 8) | calibration_data_buffer[1];
  m_calibration_data.ac2 = (calibration_data_buffer[2] << 8) | calibration_data_buffer[3];
  m_calibration_data.ac3 = (calibration_data_buffer[4] << 8) | calibration_data_buffer[5];
  m_calibration_data.ac4 = (calibration_data_buffer[6] << 8) | calibration_data_buffer[7];
  m_calibration_data.ac5 = (calibration_data_buffer[8] << 8) | calibration_data_buffer[9];
  m_calibration_data.ac6 = (calibration_data_buffer[10] << 8) | calibration_data_buffer[11];
  m_calibration_data.b1 = (calibration_data_buffer[12] << 8) | calibration_data_buffer[13];
  m_calibration_data.b2 = (calibration_data_buffer[14] << 8) | calibration_data_buffer[15];
  m_calibration_data.mb = (calibration_data_buffer[16] << 8) | calibration_data_buffer[17];
  m_calibration_data.mc = (calibration_data_buffer[18] << 8) | calibration_data_buffer[19];
  m_calibration_data.md = (calibration_data_buffer[20] << 8) | calibration_data_buffer[21];
  // verify data?
}

void bmp180::update_oversampling_rate(oversampling_rate p_oversampling_setting)
{
  m_oversampling_setting = p_oversampling_setting;
}

hal::celsius bmp180::temperature(/*timeout param*/)
{
  hal::byte control_setting = 0x2E;
  std::array<hal::byte, 2> sensor_config_buffer = {control_measurement_register()[0], control_setting};
  hal::write(*m_i2c, m_address, sensor_config_buffer, hal::never_timeout());

  while ( 1/*timeout param*/) {
    int conversion_finished = conversion_status(m_i2c, m_address);
    if (conversion_finished == 1) break;
    hal::delay(*m_clock, 1500us);
  }
  
  auto uncompensated_data_buffer = hal::write_then_read<2>(*m_i2c, m_address, uncompensated_output_data_register(), hal::never_timeout());
  std::int32_t uncompensated_temperature = (uncompensated_data_buffer[0] << 8) | uncompensated_data_buffer[1];

  // -----------------------------------------------------------------------------------
  // m_calibration_data.ac1 = 408;
  // m_calibration_data.ac2 = -72;
  // m_calibration_data.ac3 = -14383;
  // m_calibration_data.ac4 = 32741;
  // m_calibration_data.ac5 = 32757;
  // m_calibration_data.ac6 = 23153;
  // m_calibration_data.b1 = 6190;
  // m_calibration_data.b2 = 4;
  // m_calibration_data.mb = -32768;
  // m_calibration_data.mc = -8711;
  // m_calibration_data.md = 2868;
  // uncompensated_temperature = 27898;
  // -----------------------------------------------------------------------------------

  m_computation_variables.x1 = ((uncompensated_temperature - m_calibration_data.ac6) * m_calibration_data.ac5) >> 15;
  m_computation_variables.x2 = (m_calibration_data.mc << 11) / (m_computation_variables.x1 + m_calibration_data.md);
  m_computation_variables.b5 = m_computation_variables.x1 + m_computation_variables.x2;
  m_computation_variables.temperature = (m_computation_variables.b5 + 8) >> 4;

  hal::celsius temperature_in_celsius = static_cast<float>(m_computation_variables.temperature) / 10.0;

  return temperature_in_celsius;
}

float bmp180::pressure(int sample_amount/*timeout param*/)
{

  (void)bmp180::temperature();

  hal::byte control_setting = (static_cast<hal::byte>(m_oversampling_setting) << 6) | 0x34;
  std::array<hal::byte, 2> sensor_config_buffer = {control_measurement_register()[0], control_setting};

  hal::byte oversampling_setting = static_cast<hal::byte>(m_oversampling_setting);
  std::int32_t summed_uncompensated_pressures = 0;
  volatile int conversion_finished;

  for (int i = 0; i < sample_amount; i++) {
    conversion_finished = 0;
    hal::write(*m_i2c, m_address, sensor_config_buffer, hal::never_timeout());
    while ( 1 && conversion_finished == 0/*timeout param*/) {
      conversion_finished = conversion_status(m_i2c, m_address);
      hal::delay(*m_clock, 1500us);
    }

    auto uncompensated_data_buffer = hal::write_then_read<3>(*m_i2c, m_address, uncompensated_output_data_register(), hal::never_timeout());
    std::int32_t uncompensated_pressure = ((uncompensated_data_buffer[0] << 16) | (uncompensated_data_buffer[1] << 8) | uncompensated_data_buffer[2]) >> (8 - oversampling_setting);
    summed_uncompensated_pressures = summed_uncompensated_pressures + uncompensated_pressure;
  }
  summed_uncompensated_pressures /= sample_amount;

  // -----------------------------------------------------------------------------------
  // summed_uncompensated_pressures = 23843;
  // -----------------------------------------------------------------------------------

  // convert the raw pressure data into Pascals in steps of 1Pa
  m_computation_variables.b6 = m_computation_variables.b5 - 4000;
  m_computation_variables.x1 = (m_calibration_data.b2 * ((m_computation_variables.b6 * m_computation_variables.b6) >> 12)) >> 11;
  m_computation_variables.x2 = (m_calibration_data.ac2 * m_computation_variables.b6) >> 11;
  m_computation_variables.x3 = m_computation_variables.x1 + m_computation_variables.x2;
  m_computation_variables.b3 = (((static_cast<std::int32_t>(m_calibration_data.ac1) * 4 + m_computation_variables.x3) << oversampling_setting) + 2) >> 2;
  m_computation_variables.x1 = (m_calibration_data.ac3 * m_computation_variables.b6) >> 13;
  m_computation_variables.x2 = (m_calibration_data.b1 * ((m_computation_variables.b6 * m_computation_variables.b6) >> 12)) >> 16;
  m_computation_variables.x3 = ((m_computation_variables.x1 + m_computation_variables.x2) + 2) >> 2;
  m_computation_variables.b4 = (m_calibration_data.ac4 * static_cast<std::uint32_t>(m_computation_variables.x3 + 32768)) >> 15;
  m_computation_variables.b7 = (static_cast<std::uint32_t>(summed_uncompensated_pressures) - m_computation_variables.b3) * (50000 >> oversampling_setting);
  if (m_computation_variables.b7 < 0x80000000) {
      m_computation_variables.pressure = (m_computation_variables.b7 * 2) / m_computation_variables.b4; 
  } else {
      m_computation_variables.pressure = (m_computation_variables.b7 / m_computation_variables.b4) * 2;
  }
  m_computation_variables.x1 = m_computation_variables.pressure >> 8;
  m_computation_variables.x1 *= m_computation_variables.x1;
  m_computation_variables.x1 = (m_computation_variables.x1 * 3038) >> 16;
  m_computation_variables.x2 = (-7357 * m_computation_variables.pressure) >> 16;
  m_computation_variables.pressure += (m_computation_variables.x1 + m_computation_variables.x2 + 3791) >> 4;

  float pressure = static_cast<float>(m_computation_variables.pressure);

  return pressure;
}

void bmp180::reset(/*timeout param*/) {
  hal::byte reset_setting = 0xB6;
  std::array<hal::byte, 2> sensor_config_buffer = {soft_reset_register()[0], reset_setting};
  hal::write(*m_i2c, m_address, sensor_config_buffer, hal::never_timeout());
}

}  // namespace hal::sensor
