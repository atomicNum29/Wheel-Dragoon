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

inline uint32_t read_u32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8u) |
           (static_cast<uint32_t>(data[2]) << 16u) |
           (static_cast<uint32_t>(data[3]) << 24u);
}

inline int32_t read_i32(const uint8_t *data)
{
    return static_cast<int32_t>(read_u32(data));
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

inline void write_u32(uint8_t *data, uint32_t value)
{
    data[0] = static_cast<uint8_t>(value & 0xFFu);
    data[1] = static_cast<uint8_t>((value >> 8u) & 0xFFu);
    data[2] = static_cast<uint8_t>((value >> 16u) & 0xFFu);
    data[3] = static_cast<uint8_t>((value >> 24u) & 0xFFu);
}

inline void write_i32(uint8_t *data, int32_t value)
{
    write_u32(data, static_cast<uint32_t>(value));
}
}
