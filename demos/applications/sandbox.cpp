#include <array>

#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

#include <resource_list.hpp>

void application(resource_list& p_map)
{
  using namespace std::chrono_literals;
  using namespace hal::literals;

  // grab resources
  auto& clock = *p_map.clock.value();
  auto& console = *p_map.console.value();
  auto& i2c = *p_map.i2c.value();

  // establish device address
  hal::byte device_address = 0b111'0111;

  // establish payload that contains register addresses
  std::array<hal::byte, 1> read_address_payload;

  // verify device is communicating properly
  std::array<hal::byte, 1> returned_device_id;
  read_address_payload[0] = 0xD0;
  hal::write_then_read(i2c, device_address, read_address_payload, returned_device_id, hal::never_timeout());
  hal::print<64>(console, "Device ID: %x\n", returned_device_id[0]);

  // create buffer to store returned calibration register data
  constexpr hal::byte calibration_registers = 22;
  std::array<hal::byte, calibration_registers> calibration_data_buffer;

  // read and store the calibration data from the calibration registers
  read_address_payload[0] = 0xAA;
  hal::write_then_read(i2c, device_address, read_address_payload, calibration_data_buffer, hal::never_timeout());

  // take the raw calibration data and convert it into the appropriate variables
  int16_t ac1 = (calibration_data_buffer[0] << 8) | calibration_data_buffer[1];
  int16_t ac2 = (calibration_data_buffer[2] << 8) | calibration_data_buffer[3];
  int16_t ac3 = (calibration_data_buffer[4] << 8) | calibration_data_buffer[5];
  uint16_t ac4 = (calibration_data_buffer[6] << 8) | calibration_data_buffer[7];
  uint16_t ac5 = (calibration_data_buffer[8] << 8) | calibration_data_buffer[9];
  uint16_t ac6 = (calibration_data_buffer[10] << 8) | calibration_data_buffer[11];
  int16_t b1 = (calibration_data_buffer[12] << 8) | calibration_data_buffer[13];
  int16_t b2 = (calibration_data_buffer[14] << 8) | calibration_data_buffer[15];
  int16_t mb = (calibration_data_buffer[16] << 8) | calibration_data_buffer[17];
  int16_t mc = (calibration_data_buffer[18] << 8) | calibration_data_buffer[19];
  int16_t md = (calibration_data_buffer[20] << 8) | calibration_data_buffer[21];

  // ----------------------------------------------------------------------------------
  // int16_t ac1 = 408;
  // int16_t ac2 = -72;
  // int16_t ac3 = -14383;
  // uint16_t ac4 = 32741;
  // uint16_t ac5 = 32757;
  // uint16_t ac6 = 23153;
  // int16_t b1 = 6190;
  // int16_t b2 = 4;
  // int16_t mb = -32768;
  // int16_t mc = -8711;
  // int16_t md = 2868;
  
  // int32_t ut = 27898;
  // int32_t up = 23843;
  // ----------------------------------------------------------------------------------

  // configure sampling mode
  uint8_t oversampling_setting = 0;

  // output the calibration data
  hal::print(console, "Calibration Data reads:\n");
  hal::print<256>(console, "ac1 = %hd, ac2 = %hd, ac3 = %hd, ac4 = %hu, ac5 = %hu, ac6 = %hu\n", ac1, ac2, ac3, ac4, ac5, ac6);
  hal::print<256>(console, "b1 = %hd, b2 = %hd, mb = %hd, mc = %hd, md = %hd\n\n", b1, b2, mb, mc, md);

  hal::print(console, "Application Starting...\n\n");
  while (true) {
    hal::print<64>(console, "\n------------------ Start of cycle ------------------\n");
    // tell sensor to start grabbing and outputting temperature sensor data
    std::array<hal::byte, 2> config_temp = {0xF4, 0x2E};
    hal::write(i2c, device_address, config_temp, hal::never_timeout());
    hal::delay(clock, 4500us);

    // read and store the temperature sensor data
    std::array<hal::byte, 2> UT_buffer;
    read_address_payload[0] = 0xF6;
    hal::write_then_read(i2c, device_address, read_address_payload, UT_buffer, hal::never_timeout());
    int32_t ut = (UT_buffer[0] << 8) | UT_buffer[1];
    hal::print<64>(console, "ut = %i\n", ut);
    
    // tell sensor to start grabbing and outputting pressure sensor data
    std::array<hal::byte, 2> config_pressure = {0xF4, 0x34};
    hal::write(i2c, device_address, config_pressure, hal::never_timeout());
    hal::delay(clock, 4500us);

    // read and store the pressure sensor data
    std::array<hal::byte, 3> UP_buffer;
    hal::write_then_read(i2c, device_address, read_address_payload, UP_buffer, hal::never_timeout());
    int32_t up = ((UP_buffer[0] << 16) | (UP_buffer[1] << 8) | UP_buffer[0]) >> (8 - oversampling_setting);
    hal::print<64>(console, "up = %i\n", up);

    // convert the raw temperature data into Celsius in steps of 0.1°C
    int32_t x1, x2, b5, t;

    x1 = ((ut - ac6) * ac5) >> 15;
    hal::print<64>(console, "x1 = %i\n", x1);
    x2 = (mc << 11) / (x1 + md);
    hal::print<64>(console, "x2 = %i\n", x2);
    b5 = x1 + x2;
    hal::print<64>(console, "b5 = %i\n", b5);
    t = (b5 + 8) >> 4;
    hal::print<64>(console, "t = %i\n", t);
    hal::delay(clock, 500ms);

    // convert the raw pressure data into Pascals in steps of 1Pa
    int32_t p, b6, x3, b3;
    uint32_t b4, b7;
    b6 = b5 - 4000;
    hal::print<64>(console, "b6 = %d\n", b6);
    x1 = (b2 * ((b6 * b6) >> 12)) >> 11;
    hal::print<64>(console, "x1 = %d\n", x1);
    x2 = (ac2 * b6) >> 11;
    hal::print<64>(console, "x2 = %d\n", x2);
    x3 = x1 + x2;
    hal::print<64>(console, "x3 = %d\n", x3);
    b3 = (((static_cast<int32_t>(ac1) * 4 + x3) << oversampling_setting) + 2) >> 2;
    hal::print<64>(console, "b3 = %d\n", b3);
    x1 = (ac3 * b6) >> 13;
    hal::print<64>(console, "x1 = %d\n", x1);
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
    hal::print<64>(console, "x2 = %d\n", x2);
    x3 = ((x1 + x2) + 2) >> 2;
    hal::print<64>(console, "x3 = %d\n", x3);
    b4 = (ac4 * static_cast<uint32_t>(x3 + 32768)) >> 15;
    hal::print<64>(console, "b4 = %u\n", b4);
    b7 = (static_cast<uint32_t>(up) - b3) * (50000 >> oversampling_setting);
    hal::print<64>(console, "b7 = %u\n", b7);
    if (b7 < 0x80000000) {
        p = (b7 * 2) / b4; 
    } else {
        p = (b7 / b4) * 2;
    }
    hal::print<64>(console, "p = %d\n", p);
    x1 = p >> 8;
    x1 *= x1;
    hal::print<64>(console, "x1 = %d\n", x1);
    x1 = (x1 * 3038) >> 16;
    hal::print<64>(console, "x1 = %d\n", x1);
    x2 = (-7357 * p) >> 16;
    hal::print<64>(console, "x2_P2 = %d\n", x2);
    p += (x1 + x2 + 3791) >> 4;
    hal::print<64>(console, "p = %d\n", p);
  }
}
