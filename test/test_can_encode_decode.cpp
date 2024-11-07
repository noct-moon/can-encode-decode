#include "../include/can_encode_decode.h"

#include <algorithm>
#include <bitset>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <vector>

template <typename T>
inline void print_binary(T v, const char *tail_msg = "") {
  uint64_t unsigned_v = 0;
  memcpy(&unsigned_v, &v, sizeof(T));
  std::cout << "print_binary: [" << std::bitset<64>(unsigned_v) << "]:(" << v << ") | " << tail_msg << std::endl;
}

inline void TEST_NEAR(double a, double b, double epsilon, uint64_t line) {
  if (fabs(a - b) > epsilon) {
    std::cout << "TEST_NEAR(" << line << ") failed: " << a << " != " << b << std::endl;
    exit(1);
  }
}

inline void TEST_EQ(uint64_t a, uint64_t b, uint64_t line) {
  if (a != b) {
    std::cout << "TEST_EQ(" << line << ") failed: " << a << " != " << b << std::endl;
    exit(1);
  }
}

#define TEST_EQ(a, b) TEST_EQ(a, b, __LINE__)
#define TEST_NEAR(a, b, epsilon) TEST_NEAR(a, b, epsilon, __LINE__)

int main(int argc, char *argv[]) {
  std::cout << "byte order intel, unsigned ----------------------------------------" << std::endl;
  // byte order intel, unsigned
  {
    Signal signal = {0, 12, 0.002132603, -2.0943, ByteOrder::Intel, DbcValueType::Signed};
    // signal minimum value
    double physical_value = -2.0943;
    print_binary<double>(physical_value, "unsigned min value");
    uint64_t signal_raw = PhysicalToRaw(physical_value, signal);
    TEST_EQ(signal_raw, 0);

    uint8_t can_data[8] = {0};
    Encode(can_data, physical_value, signal);
    TEST_EQ(can_data[0], 0);
    TEST_EQ(can_data[1], 0);

    double decoded_value = Decode(can_data, signal);
    TEST_NEAR(decoded_value, physical_value, std::numeric_limits<double>::epsilon());
    print_binary(decoded_value);

    // signal maximum value
    physical_value = 2.0943;
    print_binary<double>(physical_value, "unsigned max value");
    signal_raw = PhysicalToRaw(physical_value, signal);
    TEST_EQ(signal_raw, 0x07ac);
    // encode value
    memset(can_data, 0, sizeof(can_data));
    Encode(can_data, physical_value, signal);
    TEST_EQ(can_data[0], 0xac);
    TEST_EQ(can_data[1], 0x07);
    // decode value
    decoded_value = Decode(can_data, signal);
    TEST_NEAR(decoded_value, physical_value, 0.001);
    print_binary(decoded_value);

    // random value
    physical_value = -1.23456;
    print_binary<double>(physical_value, "random value");
    signal_raw = PhysicalToRaw(physical_value, signal);
    TEST_EQ(signal_raw, 0x0193);
    // encode value
    memset(can_data, 0, sizeof(can_data));
    Encode(can_data, physical_value, signal);
    TEST_EQ(can_data[0], 0x93);
    TEST_EQ(can_data[1], 0x01);
    // decode value
    decoded_value = Decode(can_data, signal);
    TEST_NEAR(decoded_value, physical_value, 0.001);

    print_binary(decoded_value);
  }

  std::cout << "\n" << "byte order intel, signed ----------------------------------------" << std::endl;
  {
    Signal signal = {0, 16, 0.1, 0, ByteOrder::Intel, DbcValueType::Signed};
    // signal minimum value
    double physical_value = -780;
    print_binary<double>(physical_value, "signed min value");
    // 0xffffffffffffe188
    uint64_t signal_raw = PhysicalToRaw(physical_value, signal);
    TEST_EQ(signal_raw & 0xffff, 0xe188);

    uint8_t can_data[8] = {0};
    Encode(can_data, physical_value, signal);
    TEST_EQ(can_data[0], 0x88);
    TEST_EQ(can_data[1], 0xe1);

    double decoded_value = Decode(can_data, signal);
    TEST_NEAR(decoded_value, physical_value, 1e-6);
    print_binary(decoded_value);

    // signal maximum value
    physical_value = 779.9;
    print_binary<double>(physical_value, "signed max value");
    signal_raw = PhysicalToRaw(physical_value, signal);
    TEST_EQ(signal_raw, 0x1e77);

    memset(can_data, 0, sizeof(can_data));
    Encode(can_data, physical_value, signal);
    TEST_EQ(can_data[0], 0x77);
    TEST_EQ(can_data[1], 0x1e);

    decoded_value = Decode(can_data, signal);
    TEST_NEAR(decoded_value, physical_value, 1e-6);
    print_binary(decoded_value);

    // random value
    physical_value = -123.456;
    print_binary<double>(physical_value, "random value");
    signal_raw = PhysicalToRaw(physical_value, signal);

    memset(can_data, 0, sizeof(can_data));
    Encode(can_data, physical_value, signal);
    TEST_EQ(can_data[0], 0x2d);
    TEST_EQ(can_data[1], 0xfb);

    decoded_value = Decode(can_data, signal);
    TEST_NEAR(decoded_value, physical_value, 0.1);
    print_binary(decoded_value);
  }

  std::cout << "\n" << "byte order intel, Float and Double ----------------------------------------" << std::endl;
  {
    Signal signal = {8, 32, 1.0, 0, ByteOrder::Intel, DbcValueType::IEEE_Float};
    double physical_value = 123.456f;
    print_binary<double>(physical_value, "float value");
    uint8_t can_data[64] = {0};
    Encode(can_data, physical_value, signal);

    TEST_EQ(can_data[0], 0x00);
    TEST_EQ(can_data[1], 0x79);
    TEST_EQ(can_data[2], 0xe9);
    TEST_EQ(can_data[3], 0xf6);
    TEST_EQ(can_data[4], 0x42);

    double decoded_value = Decode(can_data, signal);
    TEST_NEAR(decoded_value, physical_value, 0.001);
    print_binary(decoded_value);

    physical_value = -123.456;
    print_binary<double>(physical_value, "negtive float value");
    memset(can_data, 0, sizeof(can_data));
    Encode(can_data, physical_value, signal);

    TEST_EQ(can_data[1], 0x79);
    TEST_EQ(can_data[2], 0xe9);
    TEST_EQ(can_data[3], 0xf6);
    TEST_EQ(can_data[4], 0xc2);

    decoded_value = Decode(can_data, signal);
    TEST_NEAR(decoded_value, physical_value, 0.001);
    print_binary(decoded_value);
  }
  {
    Signal signal = {128, 64, 1.0, 0, ByteOrder::Intel, DbcValueType::IEEE_Double};

    double physical_value = 123.456;
    print_binary<double>(physical_value, "double value");
    uint8_t can_data[64] = {0};
    Encode(can_data, physical_value, signal);

    double decoded_value = Decode(can_data, signal);
    TEST_NEAR(decoded_value, physical_value, 0.001);
    print_binary<double>(decoded_value);

    physical_value = -123.456;
    print_binary<double>(physical_value, "negtive double value");
    memset(can_data, 0, sizeof(can_data));
    Encode(can_data, physical_value, signal);

    decoded_value = Decode(can_data, signal);
    TEST_NEAR(decoded_value, physical_value, 0.001);
    print_binary<double>(decoded_value);
  }

  std::cout << "\n" << "Decode byte order Motorola ----------------------------------------" << std::endl;
  {
    Signal unsigned_signal = {16, 24, 0.001, 0, ByteOrder::Motorola, DbcValueType::Unsigned};
    Signal signed_signal = {52, 20, 0.001, 0, ByteOrder::Motorola, DbcValueType::Signed};
    Signal float_signal = {88, 32, 1.0, 0, ByteOrder::Motorola, DbcValueType::IEEE_Float};
    Signal double_signal = {160, 64, 1.0, 0, ByteOrder::Motorola, DbcValueType::IEEE_Double};

    uint8_t can_data[64] = {0x12, 0xD6, 0x87, 0x00, 0xE1, 0xDC, 0x00, 0x00, 0xC7, 0xF1, 0x20, 0x65, 0x00,
                            0x40, 0xFE, 0x24, 0x0C, 0x9F, 0xBE, 0x76, 0xC9, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    uint64_t signal_raw = ExtractSignalFromRaw(can_data, unsigned_signal);
    print_binary(signal_raw, "unsigned raw value");

    double decoded_value = Decode(can_data, unsigned_signal);
    TEST_NEAR(decoded_value, 1234.567, 0.001);
    print_binary(decoded_value, "decode unsigned value");

    decoded_value = Decode(can_data, signed_signal);
    TEST_NEAR(decoded_value, -123.456, 0.001);
    print_binary(decoded_value, "decode signed value");

    decoded_value = Decode(can_data, float_signal);
    TEST_NEAR(decoded_value, -123456.789, 0.001);
    print_binary(decoded_value, "decode float value");

    decoded_value = Decode(can_data, double_signal);
    TEST_NEAR(decoded_value, 123456.789, 0.001);
    print_binary(decoded_value, "decode double value");
  }

  std::cout << "\n" << "Encode byte order Motorola ----------------------------------------" << std::endl;
  {
    Signal unsigned_signal = {16, 24, 0.001, 0, ByteOrder::Motorola, DbcValueType::Unsigned};
    Signal signed_signal = {52, 20, 0.001, 0, ByteOrder::Motorola, DbcValueType::Signed};
    Signal float_signal = {88, 32, 1.0, 0, ByteOrder::Motorola, DbcValueType::IEEE_Float};
    Signal double_signal = {160, 64, 1.0, 0, ByteOrder::Motorola, DbcValueType::IEEE_Double};

    double physical_unsigned_value = 1234.567;
    double physical_signed_value = -123.456;
    double physical_float_value = -123456.789;
    double physical_double_value = 123456.789;

    uint8_t can_data[64] = {0};
    Encode(can_data, physical_unsigned_value, unsigned_signal);
    Encode(can_data, physical_signed_value, signed_signal);
    Encode(can_data, physical_float_value, float_signal);
    Encode(can_data, physical_double_value, double_signal);

    const uint8_t expected[64] = {0x12, 0xD6, 0x87, 0x00, 0xE1, 0xDC, 0x00, 0x00, 0xC7, 0xF1, 0x20, 0x65, 0x00,
                                  0x40, 0xFE, 0x24, 0x0C, 0x9F, 0xBE, 0x76, 0xC9, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

    for (int i = 0; i < 64; ++i) {
      TEST_EQ(can_data[i], expected[i]);
    }

    double decoded_value = Decode(can_data, unsigned_signal);
    TEST_NEAR(decoded_value, physical_unsigned_value, 0.001);
    print_binary(decoded_value, "decode unsigned value");

    decoded_value = Decode(can_data, signed_signal);
    TEST_NEAR(decoded_value, physical_signed_value, 0.001);
    print_binary(decoded_value, "decode signed value");

    decoded_value = Decode(can_data, float_signal);
    TEST_NEAR(decoded_value, physical_float_value, 0.001);
    print_binary(decoded_value, "decode float value");

    decoded_value = Decode(can_data, double_signal);
    TEST_NEAR(decoded_value, physical_double_value, 0.001);
    print_binary(decoded_value, "decode double value");
  }

  return 0;
}
