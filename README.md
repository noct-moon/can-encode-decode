# can-encode-decode
A header only C++ functions for encode and decode can or canfd frames.

Support encoding and decoding for DBC’s Unsigned/Signed/IEEE Float/IEEE Double data types.

## Usage

```c++
// encode can/canfd frame data
uint8_t canfd_data[64];
// First method
Encode(canfd_data, -123.456, 0, 16, 0.1, false, true, false);
// Second method
Signal signal = {0, 16, 0.1, 0, ByteOrder::Intel, DbcValueType::Signed};
Encode(canfd_data, signal);

// decode can/canfd frame data
// First method
double value = Decode(canfd_data, 0, 16, 0.1, false, true, false);
// second method
value = Decode(canfd_data, signal);
```

## test

Compile and run test

```bash
cd test
g++ test_can_encode_decode.cpp -o test_can_encode_decode
./test_can_encode_decode
```
