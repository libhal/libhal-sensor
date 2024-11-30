#include <array>

#include <libhal-sensor/multi/bmp180.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal/error.hpp>

#include "bmp180_internal.hpp"

using namespace std::literals;
namespace hal::sensor {
namespace {

// Helper functions for creating arrays containing the different register
// addresses in their payloads.
constexpr auto device_id_register()
{
  return std::to_array<hal::byte>({ 0xD0 });
}

constexpr auto calibration_coefficients_register()
{
  return std::to_array<hal::byte>({ 0xAA });
}

constexpr auto control_measurement_register()
{
  return std::to_array<hal::byte>({ 0xF4 });
}

constexpr auto uncompensated_output_data_register()
{
  return std::to_array<hal::byte>({ 0xF6 });
}

// Helper function that creates an array containing the address and message for
// sensor reset.
constexpr auto soft_reset_config()
{
  constexpr hal::byte reset_address = 0xE0;
  constexpr hal::byte reset_setting = 0xB6;
  return std::to_array<hal::byte>({ reset_address, reset_setting });
}

// Helper function to spin until data conversion is complete.
void wait_for_conversion(hal::i2c* p_i2c, hal::byte p_address)
{
  bool conversion_busy = true;
  while (conversion_busy) {
    auto status = hal::write_then_read<1>(*p_i2c,
                                          p_address,
                                          control_measurement_register(),
                                          hal::never_timeout())[0];
    constexpr auto conversion_status_bit = hal::bit_mask::from(5);
    conversion_busy = hal::bit_extract<conversion_status_bit>(status);
  }
}

// Struct to hold temperature helper function results.
struct temperature_results
{
  hal::celsius temperature;
  std::int32_t b5;
};

// Helper function that computes and returns both the temperature result, as
// well as the b5 variable for pressure() to use.
temperature_results compute_temperature_and_b5(hal::i2c* p_i2c,
                                               hal::byte p_address,
                                               std::uint16_t p_ac5,
                                               std::uint16_t p_ac6,
                                               std::int16_t p_mc,
                                               std::int16_t p_md)
{
  // Tell sensor to start conversion for temperature.
  static constexpr hal::byte control_setting = 0x2E;
  std::array<hal::byte, 2> sensor_config_buffer = {
    control_measurement_register()[0], control_setting
  };
  hal::write(*p_i2c, p_address, sensor_config_buffer, hal::never_timeout());

  // Spins to wait for conversion to complete.
  wait_for_conversion(p_i2c, p_address);

  // Grab the raw data from the conversion and store it.
  auto uncompensated_data_buffer =
    hal::write_then_read<2>(*p_i2c,
                            p_address,
                            uncompensated_output_data_register(),
                            hal::never_timeout());
  // This bit manipulation for storing the returned uncompensated data comes
  // from figure 4 in the datasheet. It doesn't provide further explanation or
  // reasoning as to why its done this way, or why the datatype is int32_t.
  std::int32_t uncompensated_temperature =
    (uncompensated_data_buffer[0] << 8) + uncompensated_data_buffer[1];

  // Compute the temperature in steps of 0.1C.
  auto variables = bmp180_internal::calculate_true_temperature(
    uncompensated_temperature, p_ac5, p_ac6, p_mc, p_md);

  // Converting and encapsulating result data to return.
  hal::celsius temperature_in_celsius =
    static_cast<float>(variables.temperature) / 10.0f;
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
  // Verify connection with the device.
  static constexpr hal::byte expected_device_id = 0x55;
  auto device_id = hal::write_then_read<1>(
    *m_i2c, m_address, device_id_register(), hal::never_timeout())[0];

  if (device_id != expected_device_id) {
    hal::safe_throw(hal::no_such_device(m_address, this));
  }

  // Determine the maximum amount of samples per second based on oversampling
  // setting.
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
      [[fallthrough]];
    default:
      m_maximum_samples = 38;
      break;
  }

  // Create buffer to store returned calibration register data.
  constexpr hal::byte calibration_registers_amount = 22;
  std::array<hal::byte, calibration_registers_amount> calibration_data_buffer;
  hal::write_then_read(*m_i2c,
                       m_address,
                       calibration_coefficients_register(),
                       calibration_data_buffer,
                       hal::never_timeout());

  // Check calibration register data for errors.
  for (int i = 0; i < calibration_registers_amount; i += 2) {
    uint16_t verify_word =
      (calibration_data_buffer[i] << 8) | calibration_data_buffer[i + 1];
    if (verify_word == 0x0000 || verify_word == 0xFFFF) {
      hal::safe_throw(hal::io_error(this));
    }
  }

  // Take the raw calibration data and convert it into the appropriate
  // variables.
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
  // Run helper function which computes the temperature and b5.
  auto temperature_data = compute_temperature_and_b5(m_i2c,
                                                     m_address,
                                                     m_calibration_data.ac5,
                                                     m_calibration_data.ac6,
                                                     m_calibration_data.mc,
                                                     m_calibration_data.md);

  // Return just the temperature data.
  return temperature_data.temperature;
}

bmp180::pressure_results bmp180::pressure(int sample_amount)
{
  // Run helper function which computes the temperature and b5.
  auto temperature_data = compute_temperature_and_b5(m_i2c,
                                                     m_address,
                                                     m_calibration_data.ac5,
                                                     m_calibration_data.ac6,
                                                     m_calibration_data.mc,
                                                     m_calibration_data.md);

  // Prepare buffer to configure the sensor for pressure data gathering.
  std::int16_t oversampling_setting = hal::value(m_oversampling_setting);
  // Bits 0-4 is the value to set the sensor to pressure measurement.
  // Bit 5 is to tell the sensor to start a new conversion.
  // Bits 6-7 is the oversampling setting, which is set at construction.
  // Therefore 0x34 sets the first 6 bits, and the oversampling setting is OR'd
  // to set the 2 MSB's to complete the byte.
  hal::byte control_setting = (oversampling_setting << 6) | 0x34;
  std::array<hal::byte, 2> sensor_config_buffer = {
    control_measurement_register()[0], control_setting
  };

  // Clamp sample amount to determined maximum.
  sample_amount = std::clamp(sample_amount, 1, m_maximum_samples);

  // Loop that accumulates and averages sample_amount number of samples.
  std::int32_t summed_uncompensated_pressures = 0;
  for (int i = 0; i < sample_amount; i++) {
    hal::write(*m_i2c, m_address, sensor_config_buffer, hal::never_timeout());
    // Spins until conversion is complete.
    wait_for_conversion(m_i2c, m_address);

    // Grab the raw data from the conversion and store it.
    auto uncompensated_data_buffer =
      hal::write_then_read<3>(*m_i2c,
                              m_address,
                              uncompensated_output_data_register(),
                              hal::never_timeout());
    // This bit manipulation for storing the returned uncompensated data comes
    // from figure 4 in the datasheet. It doesn't provide further explanation or
    // reasoning as to why its done this way, or why the datatype is int32_t.
    std::int32_t uncompensated_pressure =
      ((uncompensated_data_buffer[0] << 16) +
       (uncompensated_data_buffer[1] << 8) + uncompensated_data_buffer[2]) >>
      (8 - oversampling_setting);
    // Accumulate the raw samples together to later find the average.
    summed_uncompensated_pressures += uncompensated_pressure;
  }
  std::int32_t average_uncompensated_pressure =
    summed_uncompensated_pressures / sample_amount;

  // Convert the raw pressure data into Pascals in steps of 1Pa.
  auto variables =
    bmp180_internal::calculate_true_pressure(average_uncompensated_pressure,
                                             oversampling_setting,
                                             temperature_data.b5,
                                             m_calibration_data.ac1,
                                             m_calibration_data.ac2,
                                             m_calibration_data.ac3,
                                             m_calibration_data.ac4,
                                             m_calibration_data.b1,
                                             m_calibration_data.b2);

  // Converting and encapsulating result data to return.
  float pressure = static_cast<float>(variables.pressure);
  pressure_results pressure_data = { .temperature =
                                       temperature_data.temperature,
                                     .pressure = pressure };
  return pressure_data;
}

void bmp180::reset()
{
  // Send reset message
  hal::write(*m_i2c, m_address, soft_reset_config(), hal::never_timeout());
}

}  // namespace hal::sensor
