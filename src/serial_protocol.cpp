#include <Arduino.h>
#include "serial_protocol.hpp"
#include "control.hpp"
#include "can_control.hpp"
#include "can_debug.hpp"
#include "little_endian.hpp"

namespace protocol {
using namespace wire;
namespace {
constexpr uint32_t USB_SERIAL_BAUD = 115200;
typedef enum
{
    CMD_RX_WAIT_HEADER_0 = 0,
    CMD_RX_WAIT_HEADER_1 = 1,
    CMD_RX_WAIT_LENGTH = 2,
    CMD_RX_WAIT_TYPE = 3,
    CMD_RX_READ_REST = 4
} CommandRxState;
struct CommandFrameParser
{
    uint8_t packet[SERIAL_PACKET_MAX_SIZE];
    uint8_t index;
    CommandRxState state;
};
constexpr uint16_t SYSTEM_ERR_SERIAL_CHECKSUM_ERROR = (1u << 0);
constexpr uint16_t SYSTEM_ERR_COMMAND_TIMEOUT = (1u << 1);
constexpr uint16_t SYSTEM_ERR_CAN_TX_FAILURE = (1u << 2);
constexpr uint16_t SYSTEM_ERR_EMERGENCY_STOP_ACTIVE = (1u << 3);
constexpr uint16_t SYSTEM_ERR_BATTERY_LOW = (1u << 4);
constexpr uint16_t SYSTEM_ERR_SERIAL_FRAMING_ERROR = (1u << 5);
constexpr uint16_t SYSTEM_ERR_COMMAND_OUT_OF_RANGE = (1u << 6);
// System error bit 7 is reserved; no watchdog source is implemented.
constexpr uint16_t SYSTEM_ERR_DRIVER_STATUS_FAULT_PRESENT = (1u << 8);
constexpr uint16_t SYSTEM_ERR_DRIVER_RX_TIMEOUT = (1u << 9);
constexpr uint16_t SYSTEM_ERR_VOLTAGE_STALE = (1u << 10);
constexpr uint16_t SYSTEM_ERR_CAN_RX_OVERRUN = (1u << 11);

CommandFrameParser command_rx_parser = {{0}, 0, CMD_RX_WAIT_HEADER_0};
uint8_t system_status_tx_seq = 0;
} // namespace
void begin() { Serial.begin(USB_SERIAL_BAUD); }

static inline bool command_in_range(const CommandPacket &command)
{
    return command.v_milli_mps >= CMD_LINEAR_MILLI_MPS_MIN &&
           command.v_milli_mps <= CMD_LINEAR_MILLI_MPS_MAX &&
           command.w_milli_radps >= CMD_ANGULAR_MILLI_RADPS_MIN &&
           command.w_milli_radps <= CMD_ANGULAR_MILLI_RADPS_MAX;
}

static void command_parser_reset(CommandFrameParser &parser)
{
    parser.index = 0;
    parser.state = CMD_RX_WAIT_HEADER_0;
}

static void parse_complete_command_frame(const uint8_t *packet, unsigned long now)
{
    if (xor_checksum(packet, CMD_PACKET_SIZE - 1u) != packet[CMD_PACKET_SIZE - 1u])
    {
        control::report(control::Error::Checksum);
        return;
    }

    CommandPacket command = {
        little_endian::read_i16(&packet[wire::command::LINEAR]),
        little_endian::read_i16(&packet[wire::command::ANGULAR]),
        packet[wire::command::FLAGS]};

    if (!command_in_range(command))
    {
        control::report(control::Error::CommandRange);
        return;
    }

    control::enter_control_mode(now);
    control::accept_command(command, now);
}

static void parse_complete_reset_frame(const uint8_t *packet)
{
    if (xor_checksum(packet, RESET_PACKET_SIZE - 1u) != packet[RESET_PACKET_SIZE - 1u])
    {
        control::report(control::Error::Checksum);
        return;
    }

    // The two-byte magic prevents a valid but unintended short packet from
    // rebooting the controller. Sequence byte 4 is reserved for host tracking.
    if (packet[wire::reset::MAGIC_0] != RESET_MAGIC_0 || packet[wire::reset::MAGIC_1] != RESET_MAGIC_1)
    {
        control::report(control::Error::Framing);
        return;
    }

    control::reset_mcu();
}

// Known debug frame lengths are consumed even when their handlers are excluded.
// Never reinterpret an unsupported request payload as a nested control/reset frame.
static void parse_complete_serial_frame(const uint8_t *packet, unsigned long now)
{
    if (packet[LENGTH] == CMD_PACKET_LENGTH && packet[TYPE] == CMD_PACKET_TYPE)
    {
        parse_complete_command_frame(packet, now);
    }
    else if (packet[LENGTH] == RESET_PACKET_LENGTH && packet[TYPE] == RESET_PACKET_TYPE)
    {
        parse_complete_reset_frame(packet);
    }
    else if (packet[LENGTH] == CAN_BRIDGE_PACKET_LENGTH && packet[TYPE] == CAN_BRIDGE_PACKET_TYPE)
    {
#if FIRMWARE_CAN_DEBUG
        can_debug::handle_bridge_request(packet);
#else
        control::report(control::Error::Framing);
#endif
    }
    else if (packet[LENGTH] == CAN_BENCHMARK_PACKET_LENGTH && packet[TYPE] == CAN_BENCHMARK_PACKET_TYPE)
    {
#if FIRMWARE_CAN_DEBUG
        can_debug::handle_benchmark_request(packet);
#else
        control::report(control::Error::Framing);
#endif
    }
    else
    {
        control::report(control::Error::Framing);
    }
}

static void feed_command_parser(uint8_t byte_in, unsigned long now)
{
    switch (command_rx_parser.state)
    {
    case CMD_RX_WAIT_HEADER_0:
        if (byte_in == PKT_HEADER_0)
        {
            command_rx_parser.packet[HEADER_0] = byte_in;
            command_rx_parser.index = HEADER_1;
            command_rx_parser.state = CMD_RX_WAIT_HEADER_1;
        }
        else
        {
            control::report(control::Error::Framing);
        }
        break;

    case CMD_RX_WAIT_HEADER_1:
        if (byte_in == PKT_HEADER_1)
        {
            command_rx_parser.packet[HEADER_1] = byte_in;
            command_rx_parser.index = LENGTH;
            command_rx_parser.state = CMD_RX_WAIT_LENGTH;
        }
        else
        {
            control::report(control::Error::Framing);
            command_parser_reset(command_rx_parser);
        }
        break;

    case CMD_RX_WAIT_LENGTH:
        if (byte_in == CMD_PACKET_LENGTH ||
            byte_in == RESET_PACKET_LENGTH ||
            byte_in == CAN_BRIDGE_PACKET_LENGTH ||
            byte_in == CAN_BENCHMARK_PACKET_LENGTH)
        {
            command_rx_parser.packet[LENGTH] = byte_in;
            command_rx_parser.index = TYPE;
            command_rx_parser.state = CMD_RX_WAIT_TYPE;
        }
        else
        {
            control::report(control::Error::Framing);
            command_parser_reset(command_rx_parser);
        }
        break;

    case CMD_RX_WAIT_TYPE:
        if ((command_rx_parser.packet[LENGTH] == CMD_PACKET_LENGTH && byte_in == CMD_PACKET_TYPE) ||
            (command_rx_parser.packet[LENGTH] == RESET_PACKET_LENGTH && byte_in == RESET_PACKET_TYPE) ||
            (command_rx_parser.packet[LENGTH] == CAN_BRIDGE_PACKET_LENGTH && byte_in == CAN_BRIDGE_PACKET_TYPE) ||
            (command_rx_parser.packet[LENGTH] == CAN_BENCHMARK_PACKET_LENGTH && byte_in == CAN_BENCHMARK_PACKET_TYPE))
        {
            command_rx_parser.packet[TYPE] = byte_in;
            command_rx_parser.index = SEQUENCE;
            command_rx_parser.state = CMD_RX_READ_REST;
        }
        else
        {
            control::report(control::Error::Framing);
            command_parser_reset(command_rx_parser);
        }
        break;

    case CMD_RX_READ_REST:
        command_rx_parser.packet[command_rx_parser.index] = byte_in;
        command_rx_parser.index++;

        if (command_rx_parser.index >= FRAME_OVERHEAD + command_rx_parser.packet[LENGTH])
        {
            parse_complete_serial_frame(command_rx_parser.packet, now);
            command_parser_reset(command_rx_parser);
        }
        break;
    }
}

void receive_commands(unsigned long now)
{
    while (Serial.available() > 0)
    {
        int byte_in = Serial.read();
        if (byte_in >= 0)
            feed_command_parser((uint8_t)byte_in, now);
    }
}

uint8_t xor_checksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

static uint16_t build_system_error_bits(const FaultSnapshot &fault)
{
    uint16_t error = 0u;
    if (fault.serial_checksum_error)
        error |= SYSTEM_ERR_SERIAL_CHECKSUM_ERROR;
    if (fault.command_timeout)
        error |= SYSTEM_ERR_COMMAND_TIMEOUT;
    if (fault.can_tx_failure)
        error |= SYSTEM_ERR_CAN_TX_FAILURE;
    if (fault.emergency_stop_active)
        error |= SYSTEM_ERR_EMERGENCY_STOP_ACTIVE;
    if (fault.battery_low)
        error |= SYSTEM_ERR_BATTERY_LOW;
    if (fault.serial_framing_error)
        error |= SYSTEM_ERR_SERIAL_FRAMING_ERROR;
    if (fault.command_out_of_range)
        error |= SYSTEM_ERR_COMMAND_OUT_OF_RANGE;
    if (fault.driver_status_fault)
        error |= SYSTEM_ERR_DRIVER_STATUS_FAULT_PRESENT;
    if (fault.driver_rx_timeout)
        error |= SYSTEM_ERR_DRIVER_RX_TIMEOUT;
    if (fault.voltage_stale)
        error |= SYSTEM_ERR_VOLTAGE_STALE;
    if (fault.can_rx_overrun)
        error |= SYSTEM_ERR_CAN_RX_OVERRUN;
    return error;
}

static uint8_t telemetry_validity_bits(void)
{
    uint8_t validity = 0u;
    if (can_control::telemetry().lf.valid)
        validity |= wire::system::LF_VALID;
    if (can_control::telemetry().rf.valid)
        validity |= wire::system::RF_VALID;
    if (can_control::telemetry().lr.valid)
        validity |= wire::system::LR_VALID;
    if (can_control::telemetry().rr.valid)
        validity |= wire::system::RR_VALID;
    if (can_control::telemetry().driver_a.valid)
        validity |= wire::system::VOLTAGE_A_VALID;
    if (can_control::telemetry().driver_b.valid)
        validity |= wire::system::VOLTAGE_B_VALID;
    return validity;
}

static void put_motor_telemetry(uint8_t *packet, uint8_t offset, const MotorTelemetry &telemetry)
{
    packet[offset + wire::system::WHEEL_STATUS] = telemetry.status;
    little_endian::write_i16(&packet[offset + wire::system::WHEEL_RPM], telemetry.actual_rpm);
    little_endian::write_i32(&packet[offset + wire::system::WHEEL_ENCODER], telemetry.encoder_count);
    little_endian::write_u32(&packet[offset + wire::system::WHEEL_TIMESTAMP], telemetry.rx_timestamp_us);
}

static bool send_system_status(void)
{
    uint8_t packet[SYSTEM_STATUS_PACKET_SIZE] = {};
    const unsigned long now = millis();
    control::refresh_errors(now);
    // Serialize before consuming a report: a count of one is still reported.
    const FaultSnapshot fault = control::fault_snapshot(now);
    packet[HEADER_0] = PKT_HEADER_0;
    packet[HEADER_1] = PKT_HEADER_1;
    packet[LENGTH] = SYSTEM_STATUS_PACKET_LENGTH;
    packet[TYPE] = SYSTEM_STATUS_PACKET_TYPE;
    packet[wire::system::VERSION] = SYSTEM_STATUS_PACKET_VERSION;
    packet[wire::system::SEQUENCE] = system_status_tx_seq;
    packet[wire::system::STATE] = static_cast<uint8_t>(control::motor_state(fault));
    little_endian::write_u16(&packet[wire::system::ERROR], build_system_error_bits(fault));
    packet[wire::system::VALIDITY] = telemetry_validity_bits();
    little_endian::write_u16(&packet[wire::system::VOLTAGE_A], can_control::telemetry().driver_a.millivolts);
    little_endian::write_u16(&packet[wire::system::VOLTAGE_B], can_control::telemetry().driver_b.millivolts);
    put_motor_telemetry(packet, wire::system::LF, can_control::telemetry().lf);
    put_motor_telemetry(packet, wire::system::RF, can_control::telemetry().rf);
    put_motor_telemetry(packet, wire::system::LR, can_control::telemetry().lr);
    put_motor_telemetry(packet, wire::system::RR, can_control::telemetry().rr);

    if (Serial.availableForWrite() >= SYSTEM_STATUS_PACKET_SIZE)
    {
        little_endian::write_u32(&packet[wire::system::TIMESTAMP], micros());
        packet[SYSTEM_STATUS_PACKET_SIZE - 1u] = xor_checksum(packet, SYSTEM_STATUS_PACKET_SIZE - 1u);
        if (Serial.write(packet, SYSTEM_STATUS_PACKET_SIZE) != SYSTEM_STATUS_PACKET_SIZE)
            return false;
        system_status_tx_seq++;
        control::consume_error_reports();
        return true;
    }
    return false;
}

void send_status_if_ready(void)
{
    if (control::status().serial_mode != control::SERIAL_MODE_CONTROL || !can_control::status_pending())
        return;

    // Keep the TX pending flag set if the USB TX buffer is temporarily full. A
    // later loop sends the newest telemetry snapshot instead of blocking CAN.
    if (send_system_status())
        can_control::status_sent();
}

} // namespace protocol
