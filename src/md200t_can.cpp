#include "md200t_can.hpp"

#include "flexcan0.hpp"
#include "little_endian.hpp"

namespace
{
constexpr uint8_t kDlc = 8u;
constexpr uint32_t kTxTimeoutUs = 2000u;

constexpr uint8_t kPidPntVelCmd = 207u;
constexpr uint8_t kPidPntTqOff = 174u;

uint16_t make_standard_id(uint8_t driver_id)
{
    // MDROBOT Standard command MID is zero, so the 11-bit command ID is the
    // driver ID itself. Responses use MID 7 and are decoded separately.
    return static_cast<uint16_t>(driver_id);
}
}

bool send_frame(const CanFrame &frame)
{
    return can_transmit(frame, kTxTimeoutUs);
}

bool md200t_set_velocity(uint8_t driver_id, int16_t rpm1, int16_t rpm2)
{
    CanFrame frame = {};
    frame.id = make_standard_id(driver_id);
    frame.dlc = kDlc;
    // PID 207 carries both channels. Byte 7 stays zero because no return data
    // is requested; each signed RPM is encoded little-endian independently.
    frame.data[0] = kPidPntVelCmd;
    frame.data[1] = 1u;
    little_endian::write_i16(&frame.data[2], rpm1);
    frame.data[4] = 1u;
    little_endian::write_i16(&frame.data[5], rpm2);
    frame.data[7] = 0u;
    return send_frame(frame);
}

bool md200t_torque_off(uint8_t driver_id)
{
    CanFrame frame = {};
    frame.id = make_standard_id(driver_id);
    frame.dlc = kDlc;
    // PID 174 enables TQ-OFF for both channels in one command.
    frame.data[0] = kPidPntTqOff;
    frame.data[1] = 1u;
    frame.data[2] = 1u;
    frame.data[3] = 0u;
    return send_frame(frame);
}

bool md200t_request_pid_data(uint8_t driver_id, uint8_t target_pid)
{
    CanFrame frame = {};
    frame.id = make_standard_id(driver_id);
    frame.dlc = kDlc;
    frame.data[0] = MD200T_PID_REQUEST_PID_DATA;
    frame.data[1] = target_pid;
    return send_frame(frame);
}

bool md200t_decode_response_id(uint16_t standard_id, uint8_t &driver_id)
{
    if (standard_id > 0x7FFu)
        return false;

    const uint8_t mid = static_cast<uint8_t>((standard_id >> 8u) & 0x07u);
    if (mid != MD200T_RESPONSE_MID)
        return false;

    driver_id = static_cast<uint8_t>(standard_id & 0xFFu);
    return true;
}
