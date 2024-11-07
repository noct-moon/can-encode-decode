#ifndef CAN_ENCODE_DECODE_H_
#define CAN_ENCODE_DECODE_H_

#include <cmath>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#define RAW_MASK(bit_length) (UINT64_MAX >> (8 * sizeof(uint64_t) - bit_length))

inline uint64_t ExtractSignalFromRaw(const uint8_t *can_data, uint16_t start_bit, uint16_t bit_length,
                                     bool is_motorola) {
  uint8_t start_byte = start_bit / 8;
  uint8_t bit_in_start_byte = start_bit % 8;

  uint64_t signal_raw = {0};
  if (is_motorola) {
    uint8_t end_byte = start_byte - (bit_in_start_byte + bit_length - 1) / 8U;
    for (int16_t i = end_byte; i < start_byte + 1; i++) {
      signal_raw += static_cast<uint64_t>(can_data[i]) << ((start_byte - i) * 8U);
    }
  } else {
    uint8_t end_byte = (start_bit + bit_length - 1) / 8U;
    for (int16_t i = end_byte; i >= start_byte; i--) {
      signal_raw += (static_cast<uint64_t>(can_data[i]) << (i - start_byte) * 8U);
    }
  }
  signal_raw >>= bit_in_start_byte;
  signal_raw &= RAW_MASK(bit_length);
  return signal_raw;
}

inline double RawToPhysical(uint64_t signal_raw, double factor, double offset, uint16_t bit_length, bool is_signed,
                            bool is_float) {
  double signal_physical = 0.0;
  if (is_float) {
    if (bit_length == sizeof(float) * 8) {
      float signal_float_value = 0.0f;
      uint32_t float_raw = static_cast<uint32_t>(signal_raw);
      std::memcpy(&signal_float_value, &float_raw, sizeof(float));
      signal_physical = static_cast<double>(signal_float_value);
    } else if (bit_length == sizeof(double) * 8) {
      std::memcpy(&signal_physical, &signal_raw, sizeof(double));
    } else {
      throw std::invalid_argument("Invalid bit length for float type");
    }
  } else {
    if (is_signed) {
      int64_t sign_mask = 1LL << (bit_length - 1);
      int64_t signed_signal_raw = (signal_raw ^ sign_mask) - sign_mask;
      signal_physical = static_cast<double>(signed_signal_raw);
    } else {
      signal_physical = static_cast<double>(signal_raw);
    }
  }
  signal_physical = std::fma(signal_physical, factor, offset);

  return signal_physical;
}

inline void ResetBits(uint8_t *can_data, uint16_t start_bit, uint16_t bit_length) {
  if (bit_length == 0) {
    return;
  }

  uint16_t start_byte = start_bit / 8;
  uint16_t end_bit = start_bit + bit_length - 1;
  uint16_t end_byte = end_bit / 8;

  uint8_t start_bit_offset = start_bit % 8;
  uint8_t end_bit_offset = end_bit % 8;

  // Handle the first byte
  if (start_byte == end_byte) {
    uint8_t mask = ((1 << (end_bit_offset - start_bit_offset + 1)) - 1) << start_bit_offset;
    can_data[start_byte] &= ~mask;
  } else {
    // Handle the first byte
    uint8_t start_mask = 0xFF << start_bit_offset;
    can_data[start_byte] &= ~start_mask;

    // Handle the bytes in between
    for (uint16_t i = start_byte + 1; i < end_byte; ++i) {
      can_data[i] = 0x00;
    }

    // Handle the last byte
    uint8_t end_mask = 0xFF >> (7 - end_bit_offset);
    can_data[end_byte] &= ~end_mask;
  }
}

inline uint64_t PhysicalToRaw(double physical_value, double factor, double offset, uint16_t bit_length, bool is_signed,
                              bool is_float) {
  physical_value = (physical_value - offset) / factor;
  uint64_t signal_raw = 0;

  if (is_float) {
    if (bit_length == sizeof(float) * 8) {
      float float_value = static_cast<float>(physical_value);
      memcpy(&signal_raw, &float_value, sizeof(float));
      signal_raw &= RAW_MASK(bit_length);
    } else if (bit_length == sizeof(double) * 8) {
      memcpy(&signal_raw, &physical_value, sizeof(double));
    } else {
      throw std::invalid_argument("Invalid bit length for float type");
    }
  } else {
    if (is_signed) {
      // drop the fractional part
      int64_t signed_signal_raw = static_cast<int64_t>(std::round(physical_value));
      signal_raw = static_cast<uint64_t>(signed_signal_raw);
    } else {
      signal_raw = static_cast<uint64_t>(std::round(physical_value));
    }
  }
  return signal_raw;
}

inline void RawToCanData(uint8_t *can_data, uint64_t signal_raw, uint16_t start_bit, uint16_t bit_length,
                         bool is_motorola) {
  uint8_t start_byte = start_bit / 8;
  uint8_t bit_in_start_byte = start_bit % 8;

  signal_raw &= RAW_MASK(bit_length);
  // Move the signal to the correct position
  signal_raw <<= bit_in_start_byte;

  if (is_motorola) {
    uint8_t end_byte = start_byte - (bit_in_start_byte + bit_length - 1) / 8U;
    for (int16_t i{start_byte}; i >= end_byte; i--) {
      can_data[i] |= signal_raw >> ((start_byte - i) * 8U);
    }
  } else {
    uint8_t end_byte = (start_bit + bit_length - 1) / 8U;
    for (int16_t i{start_byte}; i <= end_byte; i++) {
      can_data[i] |= signal_raw >> ((i - start_byte) * 8U);
    }
  }
}

inline double Decode(const uint8_t *can_data, uint16_t start_bit, uint16_t bit_length, double factor, double offset,
                     bool is_motorola, bool is_signed, bool is_float) {
  uint64_t signal_raw = ExtractSignalFromRaw(can_data, start_bit, bit_length, is_motorola);
  return RawToPhysical(signal_raw, factor, offset, bit_length, is_signed, is_float);
}

inline void Encode(uint8_t *can_data, double physical_value, uint16_t start_bit, uint16_t bit_length, double factor,
                   double offset, bool is_motorola, bool is_signed, bool is_float) {
  uint64_t signal_raw = PhysicalToRaw(physical_value, factor, offset, bit_length, is_signed, is_float);
  RawToCanData(can_data, signal_raw, start_bit, bit_length, is_motorola);
}

enum class ByteOrder { Motorola, Intel };

enum class DbcValueType { Signed, Unsigned, IEEE_Float, IEEE_Double };

struct Signal {
  uint16_t start_bit;
  uint16_t bit_length;
  double factor;
  double offset;
  ByteOrder byte_order;
  DbcValueType value_type;
};

inline uint64_t ExtractSignalFromRaw(const uint8_t *can_data, const Signal &signal) {
  return ExtractSignalFromRaw(can_data, signal.start_bit, signal.bit_length, signal.byte_order == ByteOrder::Motorola);
}

inline double RawToPhysical(uint64_t signal_raw, const Signal &signal) {
  return RawToPhysical(
      signal_raw, signal.factor, signal.offset, signal.bit_length, signal.value_type == DbcValueType::Signed,
      (signal.value_type == DbcValueType::IEEE_Float || signal.value_type == DbcValueType::IEEE_Double));
}

inline uint64_t PhysicalToRaw(double physical_value, const Signal &signal) {
  return PhysicalToRaw(
      physical_value, signal.factor, signal.offset, signal.bit_length, signal.value_type == DbcValueType::Signed,
      (signal.value_type == DbcValueType::IEEE_Float || signal.value_type == DbcValueType::IEEE_Double));
}

inline double Decode(const uint8_t *can_data, const Signal &signal) {
  return Decode(can_data, signal.start_bit, signal.bit_length, signal.factor, signal.offset,
                signal.byte_order == ByteOrder::Motorola, signal.value_type == DbcValueType::Signed,
                (signal.value_type == DbcValueType::IEEE_Float || signal.value_type == DbcValueType::IEEE_Double));
}

inline void Encode(uint8_t *can_data, double physical_value, const Signal &signal) {
  Encode(can_data, physical_value, signal.start_bit, signal.bit_length, signal.factor, signal.offset,
         signal.byte_order == ByteOrder::Motorola, signal.value_type == DbcValueType::Signed,
         (signal.value_type == DbcValueType::IEEE_Float || signal.value_type == DbcValueType::IEEE_Double));
}

#endif  // CAN_ENCODE_DECODE_H_
