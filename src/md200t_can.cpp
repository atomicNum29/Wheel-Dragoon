#include "md200t_can.hpp"

#include "flexcan0.hpp"

// namespace
// {
constexpr uint8_t kMdrobotStandardCmdMid = 0x00u;
constexpr uint8_t kDlc = 8u;
constexpr uint32_t kTxTimeoutUs = 2000u;

constexpr uint8_t kPidPntVelCmd = 207u;
constexpr uint8_t kPidPntTqOff = 174u;

uint16_t make_standard_id(uint8_t driver_id)
{
    return static_cast<uint16_t>(((static_cast<uint16_t>(kMdrobotStandardCmdMid) & 0x07u) << 8) |
                                 static_cast<uint16_t>(driver_id));
}

void put_i16_le(uint8_t *data, uint8_t offset, int16_t value)
{
    const uint16_t raw = static_cast<uint16_t>(value);
    data[offset] = static_cast<uint8_t>(raw & 0xFFu);
    data[offset + 1u] = static_cast<uint8_t>((raw >> 8) & 0xFFu);
}

bool send_frame(const CanFrame &frame)
{
    return can_transmit(frame, kTxTimeoutUs);
}

// }

bool md200t_set_velocity(uint8_t driver_id, int16_t rpm1, int16_t rpm2)
{
    CanFrame frame = {};
    frame.id = make_standard_id(driver_id);
    frame.dlc = kDlc;
    frame.data[0] = kPidPntVelCmd;
    frame.data[1] = 1u;
    put_i16_le(frame.data, 2u, rpm1);
    frame.data[4] = 1u;
    put_i16_le(frame.data, 5u, rpm2);
    frame.data[7] = 0u;
    return send_frame(frame);
}

bool md200t_torque_off(uint8_t driver_id)
{
    CanFrame frame = {};
    frame.id = make_standard_id(driver_id);
    frame.dlc = kDlc;
    frame.data[0] = kPidPntTqOff;
    frame.data[1] = 1u;
    frame.data[2] = 1u;
    frame.data[3] = 0u;
    return send_frame(frame);
}
