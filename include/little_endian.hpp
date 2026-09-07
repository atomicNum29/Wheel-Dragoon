#pragma once

#include <stdint.h>

namespace little_endian
{
inline uint16_t read_u16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8u);
}

inline int16_t read_i16(const uint8_t *data)
{
    return static_cast<int16_t>(read_u16(data));
}

inline void write_u16(uint8_t *data, uint16_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFu);
    data[1] = static_cast<uint8_t>((value >> 8u) & 0xFFu);
}

inline void write_i16(uint8_t *data, int16_t value)
{
    // Convert before shifting so negative values are serialized as their
    // two's-complement bit pattern without implementation-defined shifts.
    write_u16(data, static_cast<uint16_t>(value));
}
}
