#include <Arduino.h>
#include "serial_protocol.hpp"
#include "control.hpp"
#include "can_control.hpp"
#include "can_debug.hpp"
#include "little_endian.hpp"
#include <string.h>

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
constexpr uint16_t SYSTEM_ERR_USB_TX_DROP = (1u << 12);
constexpr uint16_t SYSTEM_ERR_USB_TX_WRITE = (1u << 13);

CommandFrameParser command_rx_parser = {{0}, 0, CMD_RX_WAIT_HEADER_0};
constexpr uint32_t STATE_PERIOD_MS = 10;
constexpr uint8_t MAX_TX_SIZE = 63, TX_ATTEMPTS_PER_LOOP = 2;
static_assert(WHEEL_PACKET_SIZE <= MAX_TX_SIZE && VOLTAGE_PACKET_SIZE <= MAX_TX_SIZE &&
              STATE_PACKET_SIZE <= MAX_TX_SIZE, "Telemetry frame exceeds fixed TX storage");
static_assert(wire::system::SESSION + 1 + CHECKSUM_SIZE == STATE_PACKET_SIZE, "State layout size mismatch");
static_assert(CMD_PACKET_LENGTH + FRAME_OVERHEAD == CMD_PACKET_SIZE &&
              wire::command::ANGULAR + 2 + CHECKSUM_SIZE == CMD_PACKET_SIZE &&
              CMD_PACKET_SIZE <= SERIAL_PACKET_MAX_SIZE, "Command layout size mismatch");
static_assert(DRIVE_MODE_PIVOT <= wire::system::MODE_MASK &&
              static_cast<uint8_t>(vehicle::Transition::EMERGENCY_STOP) <= wire::system::MODE_MASK,
              "Mode/transition must fit their four-bit wire fields");
enum TxKind : uint8_t { WHEEL = 0, VOLTAGE = 1, STATE = 2, AUXILIARY = 3 };
struct TxFrame {
    uint8_t bytes[MAX_TX_SIZE] = {};
    uint8_t size = 0;
    uint32_t report_revision = 0, overrun_count = 0;
    uint32_t tx_drop_revision = 0, tx_write_revision = 0;
    uint16_t session = 0; // Internal completion guard; only the low byte is sent on the wire.
};
template <uint8_t Capacity> struct TxQueue {
    TxFrame frames[Capacity];
    uint8_t head = 0, count = 0;
    bool push(const TxFrame &frame) {
        const bool dropped = count == Capacity;
        if (dropped) { head = (head + 1) % Capacity; --count; }
        frames[(head + count) % Capacity] = frame;
        ++count;
        return dropped;
    }
    bool pop(TxFrame &frame) {
        if (!count) return false;
        frame = frames[head]; head = (head + 1) % Capacity; --count; return true;
    }
    bool clear() { const bool dropped = count != 0; count = head = 0; return dropped; }
};
TxQueue<16> wheel_queue;
TxQueue<4> voltage_queue;
TxQueue<1> state_queue, auxiliary_queue;
TxFrame active;
uint8_t active_offset = 0, active_kind = WHEEL, next_kind = WHEEL;
uint16_t wheel_seq[4] = {}, voltage_seq[2] = {}, state_seq = 0, telemetry_session = 0;
uint16_t tx_errors = 0;
// Internal revisions prevent a completed older snapshot clearing a newer event.
uint32_t tx_drop_revision = 0, tx_write_revision = 0;
uint32_t state_at_ms = 0;
bool state_clock_started = false;
} // namespace
void begin() { Serial.begin(USB_SERIAL_BAUD); }
void report_tx_drop() { tx_errors |= SYSTEM_ERR_USB_TX_DROP; ++tx_drop_revision; }

static inline bool command_in_range(const CommandPacket &command)
{
    const auto &motion = control::motion_output();
    if (!motion.valid_configuration()) return false;
    const float theta_limit = vehicle::rc_maximum_center_angle(motion.config.geometry);
    const bool pivot = (command.flags & CMD_FLAG_PIVOT) != 0;
    return isfinite(theta_limit) &&
           command.v_milli_mps >= CMD_LINEAR_MILLI_MPS_MIN &&
           command.v_milli_mps <= CMD_LINEAR_MILLI_MPS_MAX &&
           fabsf(command.theta_milli_rad * 0.001f) <= theta_limit &&
           command.w_milli_radps >= CMD_ANGULAR_MILLI_RADPS_MIN &&
           command.w_milli_radps <= CMD_ANGULAR_MILLI_RADPS_MAX &&
           (pivot ? command.v_milli_mps == 0 && command.theta_milli_rad == 0
                  : command.w_milli_radps == 0);
}

static void command_parser_reset(CommandFrameParser &parser)
{
    parser.index = 0;
    parser.state = CMD_RX_WAIT_HEADER_0;
}

void discard_pending_rx()
{
    command_parser_reset(command_rx_parser);
    const int buffered = Serial.available();
    for (int i = 0; i < buffered; ++i) Serial.read();
}

static void parse_complete_command_frame(const uint8_t *packet, unsigned long now)
{
    if (xor_checksum(packet, CMD_PACKET_SIZE - 1u) != packet[CMD_PACKET_SIZE - 1u])
    {
        control::report(control::Error::Checksum);
        return;
    }

    if (packet[wire::command::VERSION] != CMD_PACKET_VERSION ||
        (packet[wire::command::FLAGS] & ~CMD_FLAGS_MASK) != 0)
    {
        control::report(control::Error::Framing);
        return;
    }

    CommandPacket command = {
        little_endian::read_u16(&packet[wire::command::SEQUENCE]),
        little_endian::read_i16(&packet[wire::command::LINEAR]),
        little_endian::read_i16(&packet[wire::command::THETA]),
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
    else if (packet[LENGTH] == LEGACY_CMD_PACKET_LENGTH && packet[TYPE] == LEGACY_CMD_PACKET_TYPE)
    {
        // Consume the entire retired v/w frame; never grant drive authority or
        // reinterpret its payload as nested command/reset packets.
        control::report(control::Error::Framing);
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
        if (byte_in == CMD_PACKET_LENGTH || byte_in == LEGACY_CMD_PACKET_LENGTH ||
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
            (command_rx_parser.packet[LENGTH] == LEGACY_CMD_PACKET_LENGTH && byte_in == LEGACY_CMD_PACKET_TYPE) ||
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

static TxFrame frame_header(uint8_t type, uint8_t size, uint16_t sequence)
{
    TxFrame f;
    f.size = size;
    f.session = telemetry_session;
    f.bytes[HEADER_0] = PKT_HEADER_0;
    f.bytes[HEADER_1] = PKT_HEADER_1;
    f.bytes[LENGTH] = size - FRAME_OVERHEAD;
    f.bytes[TYPE] = type;
    f.bytes[4] = TELEMETRY_VERSION;
    little_endian::write_u16(f.bytes + 5, sequence);
    return f;
}
static void finish_frame(TxFrame &f) { f.bytes[f.size - 1] = xor_checksum(f.bytes, f.size - 1); }

void queue_wheel(uint8_t id, uint8_t status, int16_t rpm, int32_t encoder, uint32_t rx_us)
{
    if (id >= 4 || control::status().serial_mode != control::SERIAL_MODE_CONTROL) return;
    TxFrame f = frame_header(WHEEL_PACKET_TYPE, WHEEL_PACKET_SIZE, wheel_seq[id]++);
    f.bytes[7] = id;
    little_endian::write_u32(f.bytes + 8, rx_us);
    f.bytes[12] = status;
    little_endian::write_i16(f.bytes + 13, rpm);
    little_endian::write_i32(f.bytes + 15, encoder);
    finish_frame(f);
    if (wheel_queue.push(f)) report_tx_drop();
}
void queue_voltage(uint8_t id, uint16_t millivolts, uint32_t rx_us)
{
    if (id >= 2 || control::status().serial_mode != control::SERIAL_MODE_CONTROL) return;
    TxFrame f = frame_header(VOLTAGE_PACKET_TYPE, VOLTAGE_PACKET_SIZE, voltage_seq[id]++);
    f.bytes[7] = id;
    little_endian::write_u32(f.bytes + 8, rx_us);
    little_endian::write_u16(f.bytes + 12, millivolts);
    finish_frame(f);
    if (voltage_queue.push(f)) report_tx_drop();
}
static void discard_unstarted_active(bool report_loss = false)
{
    if (!active.size || active_offset) return; // Finish any already-started frame without interleaving.
    if (report_loss) report_tx_drop();
    active.size = 0;
}
void reset_telemetry_session()
{
    wheel_queue.clear();
    voltage_queue.clear();
    state_queue.clear();
    auxiliary_queue.clear();
    discard_unstarted_active();
    ++telemetry_session;
    state_clock_started = false;
    // Sequence counters and pending TX error flags persist through session resets.
}
bool queue_auxiliary(const uint8_t *packet, size_t size)
{
    if (size < 5 || size > MAX_TX_SIZE || auxiliary_queue.count) return false;
    TxFrame f;
    f.size = static_cast<uint8_t>(size);
    memcpy(f.bytes, packet, size);
    auxiliary_queue.push(f);
    return true;
}
void discard_debug_tx()
{
    if (auxiliary_queue.count && auxiliary_queue.frames[auxiliary_queue.head].bytes[TYPE] != 0x84)
        auxiliary_queue.clear();
    if (active_kind == AUXILIARY && active.bytes[TYPE] != 0x84) discard_unstarted_active();
}
static int16_t milli(float value)
{
    if (!isfinite(value)) return 0;
    return static_cast<int16_t>(lroundf(vehicle::clamp(value * 1000.0f, -32768.0f, 32767.0f)));
}
static void capture_state(uint32_t now_ms)
{
    const auto &live = control::status();
    const auto &motion = control::motion_output();
    const uint32_t observed_overruns = can_rx_overrun_count();
    const FaultSnapshot fault = control::fault_snapshot(now_ms);
    // Discard an unstarted older state snapshot too; a partial frame stays immutable.
    if (active_kind == STATE) discard_unstarted_active(true);
    if (state_queue.clear()) report_tx_drop();
    TxFrame f = frame_header(STATE_PACKET_TYPE, STATE_PACKET_SIZE, state_seq++);
    uint8_t *p = f.bytes;
    p[wire::system::STATE] = static_cast<uint8_t>(control::motor_state(fault));
    p[wire::system::MODE_MOTION] = (static_cast<uint8_t>(live.drive_mode) & wire::system::MODE_MASK) |
        ((static_cast<uint8_t>(motion.transition) & wire::system::MODE_MASK) << wire::system::TRANSITION_SHIFT);
    const bool request_valid = live.requested_motion_valid;
    // Same pre-TX drive gate used by the CAN command scheduler; not a TX ACK.
    const bool drive_allowed = live.motor_enabled && !live.driver_powerdown && !live.driver_starting &&
        !live.estop_active && !live.command_timeout && !control::has_driver_fault() && !control::reported(control::Error::BatteryLow);
    vehicle::RcCommand applied;
    const bool applied_valid = motion.applied_command(applied);
    uint16_t flags = (live.motor_enabled ? wire::system::ENABLED : 0) |
        (motion.ready ? wire::system::READY : 0) | (motion.feasible ? wire::system::FEASIBLE : 0) |
        (motion.limited ? wire::system::RPM_LIMITED : 0) | (request_valid ? wire::system::REQUEST_VALID : 0) |
        (live.estop_active ? wire::system::ESTOP_ACTIVE : 0) | (live.command_timeout ? wire::system::TIMEOUT_ACTIVE : 0) |
        (live.driver_powerdown ? wire::system::POWERDOWN : 0) | (live.driver_starting ? wire::system::DRIVER_STARTING : 0) |
        (drive_allowed ? wire::system::DRIVE_ALLOWED : 0) | (motion.calibrated() ? wire::system::STEERING_VALID : 0) |
        (applied_valid ? wire::system::APPLIED_VALID : 0) |
        (request_valid && live.requested_motion.pivot ? wire::system::REQUEST_PIVOT : 0);
    little_endian::write_u16(p + wire::system::FLAGS, flags);
    little_endian::write_u16(p + wire::system::ERROR, build_system_error_bits(fault) | tx_errors);
    p[wire::system::VALIDITY] = telemetry_validity_bits();
    little_endian::write_i16(p + wire::system::REQUEST_V, request_valid ? milli(live.requested_motion.linear_velocity) : 0);
    little_endian::write_i16(p + wire::system::REQUEST_THETA, request_valid ? milli(live.requested_motion.steering_angle) : 0);
    little_endian::write_i16(p + wire::system::REQUEST_W, request_valid ? milli(live.requested_motion.angular_velocity) : 0);
    little_endian::write_i16(p + wire::system::APPLIED_V, milli(applied.linear_velocity));
    little_endian::write_i16(p + wire::system::APPLIED_THETA, applied.pivot ? 0 : milli(applied.steering_angle));
    little_endian::write_i16(p + wire::system::APPLIED_W, applied.pivot ? milli(applied.angular_velocity) : 0);
    p[wire::system::APPLIED_MODE] = applied_valid ? static_cast<uint8_t>(applied.pivot ? DRIVE_MODE_PIVOT : DRIVE_MODE_MANUAL) : 0xFF;
    p[wire::system::SESSION] = static_cast<uint8_t>(telemetry_session);
    f.report_revision = control::error_report_revision();
    f.overrun_count = observed_overruns;
    f.tx_drop_revision = tx_drop_revision;
    f.tx_write_revision = tx_write_revision;
    finish_frame(f);
    state_queue.push(f);
}
static bool select_frame()
{
    for (uint8_t n = 0; n < 4; ++n) {
        const uint8_t kind = (next_kind + n) % 4;
        bool found = kind == WHEEL ? wheel_queue.pop(active) : kind == VOLTAGE ? voltage_queue.pop(active) :
                     kind == STATE ? state_queue.pop(active) : auxiliary_queue.pop(active);
        if (found) { active_kind = kind; next_kind = (kind + 1) % 4; active_offset = 0; return true; }
    }
    return false;
}
void service_tx()
{
    const uint32_t now_ms = millis();
    if (control::status().serial_mode == control::SERIAL_MODE_CONTROL) {
        if (!state_clock_started || uint32_t(now_ms - state_at_ms) >= STATE_PERIOD_MS) {
            if (state_clock_started) {
                const uint32_t missed = uint32_t(now_ms - state_at_ms) / STATE_PERIOD_MS - 1;
                if (missed) report_tx_drop();
                state_seq = static_cast<uint16_t>(state_seq + missed);
            }
            state_at_ms = now_ms;
            state_clock_started = true;
            capture_state(now_ms);
        }
    } else {
        wheel_queue.clear();
        voltage_queue.clear();
        state_queue.clear();
        if (active_kind != AUXILIARY) discard_unstarted_active();
        state_clock_started = false;
    }
    for (uint8_t attempt = 0; attempt < TX_ATTEMPTS_PER_LOOP; ++attempt) {
        if (!active.size && !select_frame()) return;
        const uint8_t remaining = active.size - active_offset;
        int room = Serial.availableForWrite();
        if (room > 0 && room < remaining) {
            Serial.send_now(); // Queue a partial USB buffer, without waiting for the host.
            room = Serial.availableForWrite();
        }
        if (room < remaining) return;
        const size_t written = Serial.write(active.bytes + active_offset, remaining);
        if (written != remaining) { tx_errors |= SYSTEM_ERR_USB_TX_WRITE; ++tx_write_revision; }
        // Teensy may return size_t(-1); never treat that as a successful/partial write.
        if (written > remaining || written == 0) return;
        active_offset += static_cast<uint8_t>(written);
        if (active_offset != active.size) return; // Preserve the tail before any other frame.
        Serial.send_now();
        if (active_kind == STATE && active.session == telemetry_session) {
            if (active.report_revision == control::error_report_revision() && active.overrun_count == can_rx_overrun_count())
                control::consume_error_reports();
            if (active.tx_drop_revision == tx_drop_revision) tx_errors &= ~SYSTEM_ERR_USB_TX_DROP;
            if (active.tx_write_revision == tx_write_revision) tx_errors &= ~SYSTEM_ERR_USB_TX_WRITE;
        }
        active.size = active_offset = 0;
    }
}

} // namespace protocol
