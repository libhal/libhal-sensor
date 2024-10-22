#include <libhal-sensor/multi/bmp180.hpp>

#include <array>

#include <libhal-util/i2c.hpp>
#include <libhal/error.hpp>

using namespace std::literals;
namespace hal::sensor {
namespace {

// functions for creating arrays containing the different register addresses in
// their payloads
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

// struct to hold the computation variables
struct computation_variables
{
  std::int32_t x1, x2, x3;
  std::int32_t b3, b5, b6;
  std::uint32_t b4, b7;
  std::int32_t temperature, pressure;
};

// struct to hold temperature helper function results
struct temperature_results
{
  hal::celsius temperature;
  std::int32_t b5;
};

// helper function to check if sensor has finished its data conversion
int conversion_status(hal::i2c* p_i2c, hal::byte p_address)
{
  std::array<hal::byte, 1> buffer;
  hal::write_then_read(*p_i2c,
                       p_address,
                       control_measurement_register(),
                       buffer,
                       hal::never_timeout());
  hal::byte check_bit = ((buffer[0]) >> 5) & 1;
  if (check_bit == 0)
    return 1;
  else
    return 0;
}

// helper function that computes and returns both the temperature result, as
// well as the b5 variable for pressure() to use
temperature_results compute_temperature_and_b5(hal::i2c* p_i2c,
                                               hal::byte p_address,
                                               std::uint16_t p_ac5,
                                               std::uint16_t p_ac6,
                                               std::int16_t p_mc,
                                               std::int16_t p_md)
{
  computation_variables variables;

  // tell sensor to start conversion for temperature
  hal::byte control_setting = 0x2E;
  std::array<hal::byte, 2> sensor_config_buffer = {
    control_measurement_register()[0], control_setting
  };
  hal::write(*p_i2c, p_address, sensor_config_buffer, hal::never_timeout());

  // loop to wait for conversion to complete
  int conversion_finished = 0;
  while (conversion_finished == 0) {
    conversion_finished = conversion_status(p_i2c, p_address);
  }

  // grab the raw data from the conversion and store it
  auto uncompensated_data_buffer =
    hal::write_then_read<2>(*p_i2c,
                            p_address,
                            uncompensated_output_data_register(),
                            hal::never_timeout());
  std::int32_t uncompensated_temperature =
    (uncompensated_data_buffer[0] << 8) | uncompensated_data_buffer[1];

  // compute the temperature in steps of 0.1C
  variables.x1 = ((uncompensated_temperature - p_ac6) * p_ac5) >> 15;
  variables.x2 = (p_mc << 11) / (variables.x1 + p_md);
  variables.b5 = variables.x1 + variables.x2;
  variables.temperature = (variables.b5 + 8) >> 4;

  hal::celsius temperature_in_celsius =
    static_cast<float>(variables.temperature) / 10.0;
  temperature_results temperature_data = { .temperature =
                                             temperature_in_celsius,
                                           .b5 = variables.b5 };
  return temperature_data;
}

}  // anonymous namespace

bmp180::bmp180(hal::i2c& p_i2c, oversampling_rate p_oversampling_setting)
  : m_i2c(&p_i2c)
  , m_oversampling_setting(p_oversampling_setting)
{
  static constexpr hal::byte expected_device_id = 0x55;

  // verify connection with the device
  auto device_id = hal::write_then_read<1>(
    *m_i2c, m_address, device_id_register(), hal::never_timeout())[0];

  if (device_id != expected_device_id) {
    hal::safe_throw(hal::no_such_device(m_address, this));
  }

  // determine the maximum amount of samples per second based on oversampling
  // setting
  switch (m_oversampling_setting) {
    case oversampling_rate::ulta_low_power_mode_4500us:
      m_maximum_samples = 214;
      break;
    case oversampling_rate::standard_mode_7500us:
      m_maximum_samples = 128;
      break;
    case oversampling_rate::high_resolution_mode_13500us:
      m_maximum_samples = 72;
      break;
    case oversampling_rate::ultra_high_resolution_mode_25500us:
      m_maximum_samples = 38;
      break;
    default:
      m_maximum_samples = 38;
      break;
  }

  // create buffer to store returned calibration register data
  constexpr hal::byte calibration_registers_amount = 22;
  std::array<hal::byte, calibration_registers_amount> calibration_data_buffer;
  hal::write_then_read(*m_i2c,
                       m_address,
                       calibration_coefficients_register(),
                       calibration_data_buffer,
                       hal::never_timeout());

  // check calibration register data for errors
  for (int i = 0; i < calibration_registers_amount; i += 2) {
    uint16_t verify_word =
      (calibration_data_buffer[i] << 8) | calibration_data_buffer[i + 1];
    if (verify_word == 0x0000 || verify_word == 0xFFFF) {
      hal::safe_throw(hal::io_error(this));
    }
  }

  // take the raw calibration data and convert it into the appropriate variables
  m_calibration_data.ac1 =
    (calibration_data_buffer[0] << 8) | calibration_data_buffer[1];
  m_calibration_data.ac2 =
    (calibration_data_buffer[2] << 8) | calibration_data_buffer[3];
  m_calibration_data.ac3 =
    (calibration_data_buffer[4] << 8) | calibration_data_buffer[5];
  m_calibration_data.ac4 =
    (calibration_data_buffer[6] << 8) | calibration_data_buffer[7];
  m_calibration_data.ac5 =
    (calibration_data_buffer[8] << 8) | calibration_data_buffer[9];
  m_calibration_data.ac6 =
    (calibration_data_buffer[10] << 8) | calibration_data_buffer[11];
  m_calibration_data.b1 =
    (calibration_data_buffer[12] << 8) | calibration_data_buffer[13];
  m_calibration_data.b2 =
    (calibration_data_buffer[14] << 8) | calibration_data_buffer[15];
  m_calibration_data.mb =
    (calibration_data_buffer[16] << 8) | calibration_data_buffer[17];
  m_calibration_data.mc =
    (calibration_data_buffer[18] << 8) | calibration_data_buffer[19];
  m_calibration_data.md =
    (calibration_data_buffer[20] << 8) | calibration_data_buffer[21];
}

hal::celsius bmp180::temperature()
{
  // run helper function which computes the temperature and b5
  auto temperature_data = compute_temperature_and_b5(m_i2c,
                                                     m_address,
                                                     m_calibration_data.ac5,
                                                     m_calibration_data.ac6,
                                                     m_calibration_data.mc,
                                                     m_calibration_data.md);

  // return just the temperature data
  return temperature_data.temperature;
}

bmp180::pressure_results bmp180::pressure(int sample_amount)
{
  // clamp sample amount to determined maximum
  sample_amount = std::clamp(sample_amount, 1, m_maximum_samples);

  computation_variables variables;
  // run helper function which computes the temperature and b5
  auto temperature_data = compute_temperature_and_b5(m_i2c,
                                                     m_address,
                                                     m_calibration_data.ac5,
                                                     m_calibration_data.ac6,
                                                     m_calibration_data.mc,
                                                     m_calibration_data.md);

  // tell sensor to start conversion for pressure
  hal::byte control_setting =
    (static_cast<hal::byte>(m_oversampling_setting) << 6) | 0x34;
  std::array<hal::byte, 2> sensor_config_buffer = {
    control_measurement_register()[0], control_setting
  };

  hal::byte oversampling_setting =
    static_cast<hal::byte>(m_oversampling_setting);
  std::int32_t summed_uncompensated_pressures = 0,
               average_uncompensated_pressure = 0;

  // loop to wait for conversion to complete
  int conversion_finished;
  for (int i = 0; i < sample_amount; i++) {
    conversion_finished = 0;
    hal::write(*m_i2c, m_address, sensor_config_buffer, hal::never_timeout());
    while (conversion_finished == 0) {
      conversion_finished = conversion_status(m_i2c, m_address);
    }

    // grab the raw data from the conversion and store it
    auto uncompensated_data_buffer =
      hal::write_then_read<3>(*m_i2c,
                              m_address,
                              uncompensated_output_data_register(),
                              hal::never_timeout());
    std::int32_t uncompensated_pressure =
      ((uncompensated_data_buffer[0] << 16) |
       (uncompensated_data_buffer[1] << 8) | uncompensated_data_buffer[2]) >>
      (8 - oversampling_setting);
    // accumulate the raw samples together to later find average
    summed_uncompensated_pressures += uncompensated_pressure;
  }
  average_uncompensated_pressure =
    summed_uncompensated_pressures / sample_amount;

  // convert the raw pressure data into Pascals in steps of 1Pa
  variables.b6 = temperature_data.b5 - 4000;
  variables.x1 =
    (m_calibration_data.b2 * ((variables.b6 * variables.b6) >> 12)) >> 11;
  variables.x2 = (m_calibration_data.ac2 * variables.b6) >> 11;
  variables.x3 = variables.x1 + variables.x2;
  variables.b3 =
    (((static_cast<std::int32_t>(m_calibration_data.ac1) * 4 + variables.x3)
      << oversampling_setting) + 2) >> 2;
  variables.x1 = (m_calibration_data.ac3 * variables.b6) >> 13;
  variables.x2 =
    (m_calibration_data.b1 * ((variables.b6 * variables.b6) >> 12)) >> 16;
  variables.x3 = ((variables.x1 + variables.x2) + 2) >> 2;
  variables.b4 = (m_calibration_data.ac4 *
                  static_cast<std::uint32_t>(variables.x3 + 32768)) >> 15;
  variables.b7 = (static_cast<std::uint32_t>(average_uncompensated_pressure) -
                  variables.b3) * (50000 >> oversampling_setting);
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

  float pressure = static_cast<float>(variables.pressure);
  pressure_results pressure_data = { .temperature =
                                       temperature_data.temperature,
                                     .pressure = pressure };
  return pressure_data;
}

void bmp180::reset()
{
  hal::byte reset_setting = 0xB6;
  std::array<hal::byte, 2> sensor_config_buffer = { soft_reset_register()[0],
                                                    reset_setting };
  hal::write(*m_i2c, m_address, sensor_config_buffer, hal::never_timeout());
}

}  // namespace hal::sensor
