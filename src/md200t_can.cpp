#include "md200t_can.hpp"

#include "flexcan0.hpp"
#include "little_endian.hpp"

namespace
{
constexpr uint8_t kDlc = CAN_PAYLOAD_SIZE;
constexpr uint32_t kTxTimeoutUs = 2000u;

constexpr uint8_t kPidPntVelCmd = 207u;
constexpr uint8_t kPidPntTqOff = 174u;
constexpr uint8_t kPidOffset = 0, kChannel1Enable = 1, kChannel1Rpm = 2;
constexpr uint8_t kChannel2Enable = 4, kChannel2Rpm = 5, kReturnData = 7;
constexpr uint8_t kTorqueOffChannel1 = 1, kTorqueOffChannel2 = 2, kTorqueOffReturnData = 3;
constexpr uint8_t kRequestedPid = 1, kEnabled = 1, kNoReturnData = 0;
constexpr uint8_t kResponseMidShift = 8, kResponseMidMask = 0x07, kDriverIdMask = 0xFF;

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
    frame.data[kPidOffset] = kPidPntVelCmd;
    frame.data[kChannel1Enable] = kEnabled;
    little_endian::write_i16(&frame.data[kChannel1Rpm], rpm1);
    frame.data[kChannel2Enable] = kEnabled;
    little_endian::write_i16(&frame.data[kChannel2Rpm], rpm2);
    frame.data[kReturnData] = kNoReturnData;
    return send_frame(frame);
}

bool md200t_torque_off(uint8_t driver_id)
{
    CanFrame frame = {};
    frame.id = make_standard_id(driver_id);
    frame.dlc = kDlc;
    // PID 174 enables TQ-OFF for both channels in one command.
    frame.data[kPidOffset] = kPidPntTqOff;
    frame.data[kTorqueOffChannel1] = kEnabled;
    frame.data[kTorqueOffChannel2] = kEnabled;
    frame.data[kTorqueOffReturnData] = kNoReturnData;
    return send_frame(frame);
}

bool md200t_request_pid_data(uint8_t driver_id, uint8_t target_pid)
{
    CanFrame frame = {};
    frame.id = make_standard_id(driver_id);
    frame.dlc = kDlc;
    frame.data[kPidOffset] = MD200T_PID_REQUEST_PID_DATA;
    frame.data[kRequestedPid] = target_pid;
    return send_frame(frame);
}

bool md200t_decode_response_id(uint16_t standard_id, uint8_t &driver_id)
{
    if (standard_id > CAN_STANDARD_ID_MAX)
        return false;

    const uint8_t mid = static_cast<uint8_t>((standard_id >> kResponseMidShift) & kResponseMidMask);
    if (mid != MD200T_RESPONSE_MID)
        return false;

    driver_id = static_cast<uint8_t>(standard_id & kDriverIdMask);
    return true;
}
