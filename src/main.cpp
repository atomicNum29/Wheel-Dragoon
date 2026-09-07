#include <Arduino.h>

#include "can_tx_schedule.hpp"
#include "flexcan0.hpp"
#include "little_endian.hpp"
#include "md200t_can.hpp"

#define _DEBUG 0

// car width and wheel radius
const float W = 0.485; // m
const float R = 0.13; // m

// Command packet protocol: AA 55 07 01 seq v_lo v_hi w_lo w_hi flags checksum
const uint8_t PKT_HEADER_0 = 0xAA;
const uint8_t PKT_HEADER_1 = 0x55;
const uint8_t CMD_PACKET_LENGTH = 7;
const uint8_t CMD_PACKET_TYPE = 0x01;
const uint8_t CMD_PACKET_SIZE = 11;
const uint8_t CAN_BRIDGE_PACKET_LENGTH = 15;
const uint8_t CAN_BRIDGE_PACKET_TYPE = 0x20;
const uint8_t CAN_BRIDGE_PACKET_SIZE = 19;
const uint8_t CAN_BRIDGE_RESPONSE_LENGTH = 14;
const uint8_t CAN_BRIDGE_RESPONSE_TYPE = 0xA0;
const uint8_t CAN_BRIDGE_RESPONSE_SIZE = 18;
const uint8_t SERIAL_PACKET_MAX_SIZE = CAN_BRIDGE_PACKET_SIZE;
const uint8_t STATUS_PACKET_LENGTH = 7;
const uint8_t STATUS_PACKET_TYPE = 0x81;
const uint8_t STATUS_PACKET_SIZE = 11;
const uint8_t SYSTEM_STATUS_PACKET_LENGTH = 39;
const uint8_t SYSTEM_STATUS_PACKET_TYPE = 0x82;
const uint8_t SYSTEM_STATUS_PACKET_VERSION = 1;
const uint8_t SYSTEM_STATUS_PACKET_SIZE = 43;
const int16_t CMD_LINEAR_MILLI_MPS_MIN = -2000;
const int16_t CMD_LINEAR_MILLI_MPS_MAX = 2000;
const int16_t CMD_ANGULAR_MILLI_RADPS_MIN = -5000;
const int16_t CMD_ANGULAR_MILLI_RADPS_MAX = 5000;
const float CMD_MILLI_UNIT_SCALE = 1000.0f;
const int32_t RC_PWM_CENTER_US = 1500;
const int32_t RC_PWM_DEADBAND_US = 50;
const uint8_t CMD_FLAG_ENABLE = 0x01;
const uint8_t CMD_FLAG_ESTOP = 0x02;
const uint8_t CAN_BRIDGE_STATUS_OK = 0;
const uint8_t CAN_BRIDGE_STATUS_TX_FAILED = 1;
const uint8_t CAN_BRIDGE_STATUS_RX_TIMEOUT = 2;
const uint8_t CAN_BRIDGE_STATUS_INVALID_REQUEST = 3;
const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long STATUS_PERIOD_MS = 500;      // 2 Hz
const unsigned long SYSTEM_STATUS_PERIOD_MS = 100; // 10 Hz
const uint16_t BATTERY_LOW_THRESHOLD_MV = 21000; // Configured low-voltage threshold for the 24 V system
const uint32_t MD200T_CAN_BITRATE = 250000;
const uint32_t CAN_BRIDGE_TX_TIMEOUT_US = 2000;
const uint32_t CAN_BRIDGE_MAX_RX_TIMEOUT_US = 100000;
const unsigned long MAIN_DATA_TIMEOUT_MS = 500;
const unsigned long VOLTAGE_TIMEOUT_MS = 3000;
const uint8_t CAN_RX_DRAIN_LIMIT = 8;

struct CommandPacket
{
    int16_t v_milli_mps;
    int16_t w_milli_radps;
    uint8_t flags;
};

typedef enum
{
    MOTOR_STATE_DISABLED = 0,
    MOTOR_STATE_ENABLED = 1,
    MOTOR_STATE_TIMEOUT_STOP = 2,
    MOTOR_STATE_ESTOP = 3,
    MOTOR_STATE_FAULT = 4,
    MOTOR_STATE_BOOTING = 5,
    MOTOR_STATE_CALIBRATION = 6
} MotorState;

typedef enum
{
    DRIVE_MODE_STOP = 0,
    DRIVE_MODE_MANUAL = 1,
    DRIVE_MODE_AUTO = 2
} DriveMode;

typedef enum
{
    SERIAL_MODE_CONTROL = 0,
    SERIAL_MODE_BRIDGE = 1
} SerialMode;

typedef enum
{
    CMD_RX_WAIT_HEADER_0 = 0,
    CMD_RX_WAIT_HEADER_1 = 1,
    CMD_RX_WAIT_LENGTH = 2,
    CMD_RX_WAIT_TYPE = 3,
    CMD_RX_READ_REST = 4
} CommandRxState;

#define MOTOR_ERR_CHECKSUM_ERROR (1u << 0)
#define MOTOR_ERR_COMMAND_TIMEOUT (1u << 1)
#define MOTOR_ERR_DRIVER_FAULT (1u << 2)
#define MOTOR_ERR_EMERGENCY_STOP_ACTIVE (1u << 3)
#define MOTOR_ERR_BATTERY_LOW (1u << 4)
#define MOTOR_ERR_SERIAL_FRAMING_ERROR (1u << 5)
#define MOTOR_ERR_COMMAND_OUT_OF_RANGE (1u << 6)
#define MOTOR_ERR_WATCHDOG_RESET_DETECTED (1u << 7)
#define MOTOR_ERR_OVER_CURRENT (1u << 8)
#define MOTOR_ERR_OVER_TEMPERATURE (1u << 9)
#define MOTOR_ERR_PARAMETER_ERROR (1u << 10)
// Legacy bits 8..10 remain reserved on the wire but have no independent MCU
// source. MD200T faults are reported by DRIVER_FAULT and v2 wheel.status.

#define SYSTEM_ERR_SERIAL_CHECKSUM_ERROR (1u << 0)
#define SYSTEM_ERR_COMMAND_TIMEOUT (1u << 1)
#define SYSTEM_ERR_CAN_TX_FAILURE (1u << 2)
#define SYSTEM_ERR_EMERGENCY_STOP_ACTIVE (1u << 3)
#define SYSTEM_ERR_BATTERY_LOW (1u << 4)
#define SYSTEM_ERR_SERIAL_FRAMING_ERROR (1u << 5)
#define SYSTEM_ERR_COMMAND_OUT_OF_RANGE (1u << 6)
#define SYSTEM_ERR_WATCHDOG_RESET_DETECTED (1u << 7)
#define SYSTEM_ERR_DRIVER_STATUS_FAULT_PRESENT (1u << 8)
#define SYSTEM_ERR_MAIN_DATA_STALE (1u << 9)
#define SYSTEM_ERR_VOLTAGE_STALE (1u << 10)
#define SYSTEM_ERR_CAN_RX_OVERRUN (1u << 11)

// remote control signal pins
const int w_speed_controller_pin = 0;
const int v_speed_controller_pin = 1;
const int mode_control_pin = 2;

volatile unsigned int v_pulseWidth = 0;
volatile unsigned int w_pulseWidth = 0;
volatile DriveMode mode_state = DRIVE_MODE_STOP;

struct WheelRpmCommand
{
    float lf_rpm;
    float lr_rpm;
    float rf_rpm;
    float rr_rpm;
};

struct Md200tDriverCommand
{
    float ch1_rpm;
    float ch2_rpm;
    bool enabled;
};

struct MotorTelemetry
{
    int16_t actual_rpm;
    uint16_t current_deci_amp;
    uint16_t controller_output;
    uint8_t status;
    unsigned long last_update_ms;
    bool valid; // Cleared when MAIN_DATA exceeds its freshness timeout.
};

struct DriverVoltageTelemetry
{
    uint16_t millivolts;
    unsigned long last_update_ms;
    bool valid; // Cleared when PID 143 exceeds its freshness timeout.
};

struct CanTxScheduler
{
    uint32_t cycle_start_us;
    uint8_t superframe_cycle;
    bool initialized;
    bool command_sent;
    bool poll_handled;
};

// One side-effect-free view of all live and latched fault sources. Legacy
// 0x81 and System Status v2 map this same view to their own wire bitfields.
struct FaultSnapshot
{
    bool serial_checksum_error;
    bool command_timeout;
    bool driver_fault;
    bool can_tx_failure;
    bool emergency_stop_active;
    bool battery_low;
    bool serial_framing_error;
    bool command_out_of_range;
    bool watchdog_reset;
    bool driver_status_fault;
    bool main_data_stale;
    bool voltage_stale;
    bool can_rx_overrun;
};

struct CommandFrameParser
{
    uint8_t packet[SERIAL_PACKET_MAX_SIZE];
    uint8_t index;
    CommandRxState state;
};

// RC linear velocity PWM pulse width decoder ISR.
void v_decodePWM();
// RC angular velocity PWM pulse width decoder ISR.
void w_decodePWM();
// RC mode select PWM decoder ISR.
void mode_decodePWM();
// Build the status packet error bitfield from current latched and live faults.
uint16_t motor_build_error_bits(void);
// Resolve the current high-level motor state with fault and safety priority.
MotorState motor_get_current_state(void);
// Return the lower MD200T input voltage when both driver readings are fresh.
uint16_t motor_read_battery_mv(void);
// Compute XOR checksum for ROS-MCU protocol packets.
uint8_t protocol_xor_checksum(const uint8_t *data, size_t len);
// Send one basic status packet to ROS over USB Serial.
void protocol_send_basic_status(void);
// Send the periodic basic status packet when the configured interval has elapsed.
void protocol_send_basic_status_if_due(void);
void protocol_send_system_status_if_due(void);
// Convert left/right skid-steer RPM targets into four wheel RPM targets.
void motor_set_left_right_rpm(float left_rpm, float right_rpm);
// Set every wheel target RPM to zero.
void motor_stop_all(void);
// Send the due automatic command pair or polling request without RX waiting.
void can_transmit_if_due(uint32_t now_us);
// Check whether command packet velocity fields are inside supported physical limits.
static bool command_in_range(const CommandPacket &command);
// Convert linear wheel speed in m/s to wheel RPM.
static float vel_mps_to_rpm(float v_mps);
// Convert floating-point target RPM to MD200T int16 command data.
static int16_t rpm_to_i16(float rpm);
// Return the signed RC PWM offset, or zero while inside the neutral deadband.
static int32_t rc_pwm_offset_with_deadband(unsigned int pulse_width_us);
// Convert packet milli-m/s linear velocity to m/s.
static float milli_mps_to_mps(int16_t milli_mps);
// Convert packet milli-rad/s angular velocity to rad/s.
static float milli_radps_to_radps(int16_t milli_radps);
// Reset command frame parser to wait for the next packet header.
static void command_parser_reset(CommandFrameParser &parser);
// Store a validated ROS command for use only while the RC switch selects AUTO.
static void apply_command_packet(const CommandPacket &command, unsigned long now);
// Parse a complete command frame buffer and apply it when valid.
static void parse_complete_command_frame(const uint8_t *packet, unsigned long now);
// Wrap one CAN frame or bridge status in the existing 0xA0 Serial packet.
static void send_can_bridge_response(uint8_t seq, uint8_t status, const CanFrame *frame);
// Parse and execute one USB Serial to CAN bridge request.
static void parse_complete_can_bridge_frame(const uint8_t *packet);
// Dispatch a complete USB Serial frame by packet type.
static void parse_complete_serial_frame(const uint8_t *packet, unsigned long now);
// Feed one Serial byte into the command frame state machine.
static void feed_command_parser(uint8_t byte_in, unsigned long now);
// Drain all currently buffered USB Serial bytes into the command parser.
static void serial_drain_command_packets(unsigned long now);
// Resolve the final output from one consistent RC mode/PWM snapshot.
static void update_effective_motor_command(DriveMode drive_mode,
                                           unsigned int v_pwm,
                                           unsigned int w_pwm,
                                           unsigned long now);
// Send both driver frames back-to-back from one command snapshot.
static void md200t_send_driver_commands(const Md200tDriverCommand &driver_a,
                                        const Md200tDriverCommand &driver_b);
static void reset_can_tx_scheduler(uint32_t now_us);
static void md200t_process_rx_frame(const CanFrame &frame, unsigned long now);
static void md200t_drain_rx(unsigned long now);
static void md200t_update_telemetry_health(unsigned long now);
static void initialize_control_session(unsigned long now);
static void enter_control_mode(unsigned long now);
static void enter_bridge_mode(void);
static void latch_safety_can_tx_failure(void);
static bool motor_has_driver_fault(void);
static FaultSnapshot capture_fault_snapshot(unsigned long now);
static uint16_t build_legacy_error_bits(const FaultSnapshot &fault);
static uint16_t build_system_error_bits(const FaultSnapshot &fault);
static MotorState resolve_motor_state(const FaultSnapshot &fault);

static bool booting = true;
static bool calibration_active = false;
static bool motor_enabled = false;
static bool estop_active = false;
static bool command_timeout = true;
// Motor/safety TX failures are sticky; diagnostic poll TX failures recover on
// the next successful poll and do not directly disable motor output.
static bool safety_can_tx_failure_latched = false;
static bool poll_can_tx_failure_active = false;
static bool driver_status_fault_latched = false;
static bool main_data_stale_latched = false;
static bool battery_low_latched = false;
static bool watchdog_reset_detected = false;
static bool checksum_error_latched = false;
static bool serial_framing_error_latched = false;
static bool command_out_of_range_latched = false;
static unsigned long last_valid_command_ms = 0;
static unsigned long last_status_tx_ms = 0;
static unsigned long last_system_status_tx_ms = 0;
static unsigned long control_mode_started_ms = 0;
static uint8_t status_tx_seq = 0;
static uint8_t system_status_tx_seq = 0;
static uint8_t bridge_tunnel_seq = 0;
static SerialMode serial_mode = SERIAL_MODE_CONTROL;

static WheelRpmCommand target_wheel_rpm_cmd = {0.0f, 0.0f, 0.0f, 0.0f};
static CommandPacket auto_command = {0, 0, 0u};
static CanTxScheduler can_tx_scheduler = {0u, 0u, false, false, false};
static CommandFrameParser command_rx_parser = {{0}, 0, CMD_RX_WAIT_HEADER_0};
static MotorTelemetry lf_telemetry = {0, 0u, 0u, 0u, 0u, false};
static MotorTelemetry rf_telemetry = {0, 0u, 0u, 0u, 0u, false};
static MotorTelemetry lr_telemetry = {0, 0u, 0u, 0u, 0u, false};
static MotorTelemetry rr_telemetry = {0, 0u, 0u, 0u, 0u, false};
static DriverVoltageTelemetry driver_a_voltage = {0u, 0u, false};
static DriverVoltageTelemetry driver_b_voltage = {0u, 0u, false};

void setup()
{
    Serial.begin(115200);

    pinMode(v_speed_controller_pin, INPUT);
    pinMode(w_speed_controller_pin, INPUT);
    pinMode(mode_control_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(v_speed_controller_pin), v_decodePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(w_speed_controller_pin), w_decodePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(mode_control_pin), mode_decodePWM, CHANGE);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);

    if (!can_begin(MD200T_CAN_BITRATE))
    {
        latch_safety_can_tx_failure();
        digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
        initialize_control_session(millis());
    }

    booting = false;
}

void loop()
{
    const unsigned long now = millis();
    // The ISR-owned values are sampled once so one loop iteration cannot mix
    // two RC modes or partially updated manual inputs.
    const DriveMode drive_mode = mode_state;
    const unsigned int v_pwm = v_pulseWidth;
    const unsigned int w_pwm = w_pulseWidth;

    serial_drain_command_packets(now);
    md200t_drain_rx(now);
    md200t_update_telemetry_health(now);
    update_effective_motor_command(drive_mode, v_pwm, w_pwm, now);
    can_transmit_if_due(micros());
    protocol_send_system_status_if_due();
    // Legacy 0x81 output is intentionally disabled; 0x82 is canonical.
    // protocol_send_basic_status_if_due();
}

static inline bool command_in_range(const CommandPacket &command)
{
    return command.v_milli_mps >= CMD_LINEAR_MILLI_MPS_MIN &&
           command.v_milli_mps <= CMD_LINEAR_MILLI_MPS_MAX &&
           command.w_milli_radps >= CMD_ANGULAR_MILLI_RADPS_MIN &&
           command.w_milli_radps <= CMD_ANGULAR_MILLI_RADPS_MAX;
}

// Helper: m/s -> wheel RPM
static inline float vel_mps_to_rpm(float v_mps)
{
    // wheel angular speed (rad/s) = v / R
    float omega = v_mps / (float)R;
    // RPM = omega * 60 / (2*pi)
    return omega * 60.0f / (2.0f * 3.1415926535f);
}

static inline int16_t rpm_to_i16(float rpm)
{
    if (rpm > 32767.0f)
        return 32767;
    if (rpm < -32768.0f)
        return -32768;

    return (int16_t)(rpm >= 0.0f ? rpm + 0.5f : rpm - 0.5f);
}

static inline int32_t rc_pwm_offset_with_deadband(unsigned int pulse_width_us)
{
    const int32_t offset_us = static_cast<int32_t>(pulse_width_us) - RC_PWM_CENTER_US;
    if (offset_us >= -RC_PWM_DEADBAND_US && offset_us <= RC_PWM_DEADBAND_US)
        return 0;
    return offset_us;
}

static inline float milli_mps_to_mps(int16_t milli_mps)
{
    return (float)milli_mps / CMD_MILLI_UNIT_SCALE;
}

static inline float milli_radps_to_radps(int16_t milli_radps)
{
    return (float)milli_radps / CMD_MILLI_UNIT_SCALE;
}

static void command_parser_reset(CommandFrameParser &parser)
{
    parser.index = 0;
    parser.state = CMD_RX_WAIT_HEADER_0;
}

static void apply_command_packet(const CommandPacket &command, unsigned long now)
{
    // Keep AUTO input separate from the effective output. STOP and MANUAL
    // therefore cannot leak a just-received ROS target into the CAN scheduler.
    auto_command = command;
    last_valid_command_ms = now;
}

static void update_effective_motor_command(DriveMode drive_mode,
                                           unsigned int v_pwm,
                                           unsigned int w_pwm,
                                           unsigned long now)
{
    if (serial_mode != SERIAL_MODE_CONTROL)
    {
        command_timeout = false;
        estop_active = false;
        motor_enabled = false;
        motor_stop_all();
        return;
    }

    if (drive_mode == DRIVE_MODE_STOP)
    {
        command_timeout = false;
        estop_active = false;
        motor_enabled = false;
        motor_stop_all();
        return;
    }

    if (drive_mode == DRIVE_MODE_MANUAL)
    {
        command_timeout = false;
        estop_active = false;
        motor_enabled = true;

        // Suppress receiver/joystick noise independently on both axes. Outside
        // the neutral band, preserve the existing scaling and direction signs.
        const float v_velocity = static_cast<float>(rc_pwm_offset_with_deadband(v_pwm)) / 250.0f;
        const float w_velocity = static_cast<float>(rc_pwm_offset_with_deadband(w_pwm)) / 100.0f;
        const float left_velocity = v_velocity - w_velocity * W / 2.0f;
        const float right_velocity = v_velocity + w_velocity * W / 2.0f;
        motor_set_left_right_rpm(vel_mps_to_rpm(left_velocity), vel_mps_to_rpm(right_velocity));
        return;
    }

    command_timeout = (last_valid_command_ms == 0u) || (now - last_valid_command_ms > COMMAND_TIMEOUT_MS);
    estop_active = (auto_command.flags & CMD_FLAG_ESTOP) != 0u;
    motor_enabled = ((auto_command.flags & CMD_FLAG_ENABLE) != 0u) && !estop_active && !command_timeout;

    if (!motor_enabled)
    {
        motor_stop_all();
        return;
    }

    const float v_velocity = milli_mps_to_mps(auto_command.v_milli_mps);
    const float w_velocity = milli_radps_to_radps(auto_command.w_milli_radps);
    const float left_velocity = v_velocity - w_velocity * W / 2.0f;
    const float right_velocity = v_velocity + w_velocity * W / 2.0f;
    motor_set_left_right_rpm(vel_mps_to_rpm(left_velocity), vel_mps_to_rpm(right_velocity));
}

static void parse_complete_command_frame(const uint8_t *packet, unsigned long now)
{
    if (protocol_xor_checksum(packet, CMD_PACKET_SIZE - 1u) != packet[CMD_PACKET_SIZE - 1u])
    {
        checksum_error_latched = true;
        return;
    }

    CommandPacket command = {
        little_endian::read_i16(&packet[5]),
        little_endian::read_i16(&packet[7]),
        packet[9]};

    if (!command_in_range(command))
    {
        command_out_of_range_latched = true;
        return;
    }

    enter_control_mode(now);
    apply_command_packet(command, now);
}

static void send_can_bridge_response(uint8_t seq, uint8_t status, const CanFrame *frame)
{
    uint8_t packet[CAN_BRIDGE_RESPONSE_SIZE];
    packet[0] = PKT_HEADER_0;
    packet[1] = PKT_HEADER_1;
    packet[2] = CAN_BRIDGE_RESPONSE_LENGTH;
    packet[3] = CAN_BRIDGE_RESPONSE_TYPE;
    packet[4] = seq;
    packet[5] = status;

    uint16_t id = 0u;
    uint8_t dlc = 0u;
    uint8_t data[8] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};
    if (frame != nullptr)
    {
        id = frame->id;
        dlc = frame->dlc;
        if (dlc > 8u)
            dlc = 8u;
        for (uint8_t i = 0u; i < dlc; ++i)
            data[i] = frame->data[i];
    }

    little_endian::write_u16(&packet[6], id);
    packet[8] = dlc;
    for (uint8_t i = 0u; i < 8u; ++i)
        packet[9u + i] = data[i];
    packet[17] = protocol_xor_checksum(packet, CAN_BRIDGE_RESPONSE_SIZE - 1u);

    if (Serial.availableForWrite() >= CAN_BRIDGE_RESPONSE_SIZE)
        Serial.write(packet, CAN_BRIDGE_RESPONSE_SIZE);
}

static void parse_complete_can_bridge_frame(const uint8_t *packet)
{
    const uint8_t seq = packet[4];
    if (protocol_xor_checksum(packet, CAN_BRIDGE_PACKET_SIZE - 1u) != packet[CAN_BRIDGE_PACKET_SIZE - 1u])
    {
        checksum_error_latched = true;
        send_can_bridge_response(seq, CAN_BRIDGE_STATUS_INVALID_REQUEST, nullptr);
        return;
    }

    CanFrame tx = {};
    tx.id = little_endian::read_u16(&packet[5]);
    tx.dlc = packet[7];
    if (tx.id > 0x7FFu || tx.dlc > 8u)
    {
        send_can_bridge_response(seq, CAN_BRIDGE_STATUS_INVALID_REQUEST, nullptr);
        return;
    }

    for (uint8_t i = 0u; i < 8u; ++i)
        tx.data[i] = packet[8u + i];

    enter_bridge_mode();
    bridge_tunnel_seq = seq;

    const uint16_t timeout_ms = little_endian::read_u16(&packet[16]);
    if (!can_transmit(tx, CAN_BRIDGE_TX_TIMEOUT_US))
    {
        latch_safety_can_tx_failure();
        send_can_bridge_response(seq, CAN_BRIDGE_STATUS_TX_FAILED, nullptr);
        return;
    }

    uint32_t timeout_us = static_cast<uint32_t>(timeout_ms) * 1000u;
    if (timeout_us > CAN_BRIDGE_MAX_RX_TIMEOUT_US)
        timeout_us = CAN_BRIDGE_MAX_RX_TIMEOUT_US;

    CanFrame rx = {};
    // Bridge mode is a transparent CAN monitor: the first frame received
    // within the request timeout is returned regardless of ID or PID.
    if (!can_receive(rx, timeout_us))
    {
        send_can_bridge_response(seq, CAN_BRIDGE_STATUS_RX_TIMEOUT, nullptr);
        return;
    }

    send_can_bridge_response(seq, CAN_BRIDGE_STATUS_OK, &rx);
}

static void parse_complete_serial_frame(const uint8_t *packet, unsigned long now)
{
    if (packet[2] == CMD_PACKET_LENGTH && packet[3] == CMD_PACKET_TYPE)
    {
        parse_complete_command_frame(packet, now);
    }
    else if (packet[2] == CAN_BRIDGE_PACKET_LENGTH && packet[3] == CAN_BRIDGE_PACKET_TYPE)
    {
        parse_complete_can_bridge_frame(packet);
    }
    else
    {
        serial_framing_error_latched = true;
    }
}

static void feed_command_parser(uint8_t byte_in, unsigned long now)
{
    switch (command_rx_parser.state)
    {
    case CMD_RX_WAIT_HEADER_0:
        if (byte_in == PKT_HEADER_0)
        {
            command_rx_parser.packet[0] = byte_in;
            command_rx_parser.index = 1;
            command_rx_parser.state = CMD_RX_WAIT_HEADER_1;
        }
        else
        {
            serial_framing_error_latched = true;
        }
        break;

    case CMD_RX_WAIT_HEADER_1:
        if (byte_in == PKT_HEADER_1)
        {
            command_rx_parser.packet[1] = byte_in;
            command_rx_parser.index = 2;
            command_rx_parser.state = CMD_RX_WAIT_LENGTH;
        }
        else
        {
            serial_framing_error_latched = true;
            command_parser_reset(command_rx_parser);
        }
        break;

    case CMD_RX_WAIT_LENGTH:
        if (byte_in == CMD_PACKET_LENGTH || byte_in == CAN_BRIDGE_PACKET_LENGTH)
        {
            command_rx_parser.packet[2] = byte_in;
            command_rx_parser.index = 3;
            command_rx_parser.state = CMD_RX_WAIT_TYPE;
        }
        else
        {
            serial_framing_error_latched = true;
            command_parser_reset(command_rx_parser);
        }
        break;

    case CMD_RX_WAIT_TYPE:
        if ((command_rx_parser.packet[2] == CMD_PACKET_LENGTH && byte_in == CMD_PACKET_TYPE) ||
            (command_rx_parser.packet[2] == CAN_BRIDGE_PACKET_LENGTH && byte_in == CAN_BRIDGE_PACKET_TYPE))
        {
            command_rx_parser.packet[3] = byte_in;
            command_rx_parser.index = 4;
            command_rx_parser.state = CMD_RX_READ_REST;
        }
        else
        {
            serial_framing_error_latched = true;
            command_parser_reset(command_rx_parser);
        }
        break;

    case CMD_RX_READ_REST:
        command_rx_parser.packet[command_rx_parser.index] = byte_in;
        command_rx_parser.index++;

        if (command_rx_parser.index >= 2u + 1u + command_rx_parser.packet[2] + 1u)
        {
            parse_complete_serial_frame(command_rx_parser.packet, now);
            command_parser_reset(command_rx_parser);
        }
        break;
    }
}

static void serial_drain_command_packets(unsigned long now)
{
    while (Serial.available() > 0)
    {
        int byte_in = Serial.read();
        if (byte_in >= 0)
            feed_command_parser((uint8_t)byte_in, now);
    }
}

static void reset_telemetry_validity(void)
{
    lf_telemetry = MotorTelemetry{0, 0u, 0u, 0u, 0u, false};
    rf_telemetry = MotorTelemetry{0, 0u, 0u, 0u, 0u, false};
    lr_telemetry = MotorTelemetry{0, 0u, 0u, 0u, 0u, false};
    rr_telemetry = MotorTelemetry{0, 0u, 0u, 0u, 0u, false};
    driver_a_voltage = DriverVoltageTelemetry{0u, 0u, false};
    driver_b_voltage = DriverVoltageTelemetry{0u, 0u, false};
}

static void initialize_control_session(unsigned long now)
{
    // A new control session must not reuse telemetry received before bridge
    // mode. Freshness grace periods start from this timestamp as well.
    reset_telemetry_validity();
    // A poll failure is a session-local, recoverable diagnostic. Do not carry
    // it across BRIDGE -> CONTROL before the new polling timeline starts.
    poll_can_tx_failure_active = false;
    control_mode_started_ms = now;
    reset_can_tx_scheduler(micros());
}

static void enter_control_mode(unsigned long now)
{
    if (serial_mode == SERIAL_MODE_CONTROL)
        return;

    serial_mode = SERIAL_MODE_CONTROL;
    initialize_control_session(now);
}

static void enter_bridge_mode(void)
{
    if (serial_mode == SERIAL_MODE_BRIDGE)
        return;

    serial_mode = SERIAL_MODE_BRIDGE;
    motor_enabled = false;
    motor_stop_all();

    // Automatic traffic is disabled before this back-to-back safety pair.
    const bool driver_a_ok = md200t_torque_off(MD200T_DRIVER_A_ID);
    const bool driver_b_ok = md200t_torque_off(MD200T_DRIVER_B_ID);
    if (!driver_a_ok || !driver_b_ok)
        latch_safety_can_tx_failure();
}

static MotorTelemetry *md200t_find_motor_telemetry(uint8_t driver_id, uint8_t pid)
{
    // Each controller drives a diagonal pair; keep this mapping explicit.
    if (driver_id == MD200T_DRIVER_A_ID)
        return pid == MD200T_PID_MAIN_DATA ? &lf_telemetry
                                          : (pid == MD200T_PID_MAIN_DATA2 ? &rr_telemetry : nullptr);
    if (driver_id == MD200T_DRIVER_B_ID)
        return pid == MD200T_PID_MAIN_DATA ? &rf_telemetry
                                          : (pid == MD200T_PID_MAIN_DATA2 ? &lr_telemetry : nullptr);
    return nullptr;
}

static void md200t_process_rx_frame(const CanFrame &frame, unsigned long now)
{
    uint8_t driver_id = 0u;
    if (!md200t_decode_response_id(frame.id, driver_id) || frame.dlc == 0u)
        return;

    const uint8_t pid = frame.data[0];
    MotorTelemetry *motor = md200t_find_motor_telemetry(driver_id, pid);
    if (motor != nullptr)
    {
        if (frame.dlc < 8u)
            return;

        motor->status = frame.data[1];
        motor->actual_rpm = little_endian::read_i16(&frame.data[2]);
        motor->current_deci_amp = little_endian::read_u16(&frame.data[4]);
        motor->controller_output = little_endian::read_u16(&frame.data[6]);
        motor->last_update_ms = now;
        motor->valid = true;
        if (motor->status != 0u)
            driver_status_fault_latched = true;
        return;
    }

    if (pid != MD200T_PID_VOLT_IN || frame.dlc < 3u)
        return;

    DriverVoltageTelemetry *voltage = nullptr;
    if (driver_id == MD200T_DRIVER_A_ID)
        voltage = &driver_a_voltage;
    else if (driver_id == MD200T_DRIVER_B_ID)
        voltage = &driver_b_voltage;
    else
        return;

    const int16_t raw_deci_volts = little_endian::read_i16(&frame.data[1]);
    // PID 143 is signed 0.1 V on the wire; only positive values that still
    // fit after conversion to uint16 millivolts are accepted.
    if (raw_deci_volts <= 0 || raw_deci_volts > 655)
        return;

    voltage->millivolts = static_cast<uint16_t>(static_cast<uint16_t>(raw_deci_volts) * 100u);
    voltage->last_update_ms = now;
    voltage->valid = true;
}

static void md200t_drain_rx(unsigned long now)
{
    CanFrame frame = {};
    for (uint8_t count = 0u; count < CAN_RX_DRAIN_LIMIT; ++count)
    {
        if (!can_receive(frame, 0u))
            break;

        if (serial_mode == SERIAL_MODE_BRIDGE)
        {
            // Forward every CAN frame without MDROBOT ID/PID filtering. The
            // latest 0x20 sequence identifies the active tunnel session.
            send_can_bridge_response(bridge_tunnel_seq, CAN_BRIDGE_STATUS_OK, &frame);
        }
        else
        {
            md200t_process_rx_frame(frame, now);
        }
    }
}

static bool telemetry_is_fresh(bool valid, unsigned long last_update_ms, unsigned long now, unsigned long timeout_ms)
{
    return valid && now - last_update_ms <= timeout_ms;
}

static void md200t_update_telemetry_health(unsigned long now)
{
    lf_telemetry.valid = telemetry_is_fresh(lf_telemetry.valid, lf_telemetry.last_update_ms, now, MAIN_DATA_TIMEOUT_MS);
    rf_telemetry.valid = telemetry_is_fresh(rf_telemetry.valid, rf_telemetry.last_update_ms, now, MAIN_DATA_TIMEOUT_MS);
    lr_telemetry.valid = telemetry_is_fresh(lr_telemetry.valid, lr_telemetry.last_update_ms, now, MAIN_DATA_TIMEOUT_MS);
    rr_telemetry.valid = telemetry_is_fresh(rr_telemetry.valid, rr_telemetry.last_update_ms, now, MAIN_DATA_TIMEOUT_MS);
    driver_a_voltage.valid = telemetry_is_fresh(driver_a_voltage.valid, driver_a_voltage.last_update_ms, now, VOLTAGE_TIMEOUT_MS);
    driver_b_voltage.valid = telemetry_is_fresh(driver_b_voltage.valid, driver_b_voltage.last_update_ms, now, VOLTAGE_TIMEOUT_MS);

    if (serial_mode == SERIAL_MODE_CONTROL && now - control_mode_started_ms >= MAIN_DATA_TIMEOUT_MS &&
        (!lf_telemetry.valid || !rf_telemetry.valid || !lr_telemetry.valid || !rr_telemetry.valid))
        main_data_stale_latched = true;

    if (driver_a_voltage.valid && driver_b_voltage.valid &&
        (driver_a_voltage.millivolts < BATTERY_LOW_THRESHOLD_MV || driver_b_voltage.millivolts < BATTERY_LOW_THRESHOLD_MV))
        battery_low_latched = true;

}

static void latch_safety_can_tx_failure(void)
{
    safety_can_tx_failure_latched = true;
}

static bool motor_has_driver_fault(void)
{
    // Recoverable diagnostic poll failures are intentionally excluded. A
    // sustained loss of poll responses is promoted by MAIN_DATA_STALE.
    return safety_can_tx_failure_latched || driver_status_fault_latched || main_data_stale_latched;
}

static FaultSnapshot capture_fault_snapshot(unsigned long now)
{
    FaultSnapshot fault = {};
    fault.serial_checksum_error = checksum_error_latched;
    fault.command_timeout = command_timeout;
    fault.driver_fault = motor_has_driver_fault();
    fault.can_tx_failure = safety_can_tx_failure_latched || poll_can_tx_failure_active;
    fault.emergency_stop_active = estop_active;
    fault.battery_low = battery_low_latched;
    fault.serial_framing_error = serial_framing_error_latched;
    fault.command_out_of_range = command_out_of_range_latched;
    fault.watchdog_reset = watchdog_reset_detected;
    fault.driver_status_fault = driver_status_fault_latched;
    fault.main_data_stale = main_data_stale_latched;
    fault.voltage_stale = serial_mode == SERIAL_MODE_CONTROL &&
                          now - control_mode_started_ms >= VOLTAGE_TIMEOUT_MS &&
                          (!driver_a_voltage.valid || !driver_b_voltage.valid);
    fault.can_rx_overrun = can_rx_overrun_detected();
    return fault;
}

uint8_t protocol_xor_checksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

uint16_t motor_read_battery_mv(void)
{
    if (!driver_a_voltage.valid || !driver_b_voltage.valid)
        return 0u;
    return driver_a_voltage.millivolts < driver_b_voltage.millivolts
               ? driver_a_voltage.millivolts
               : driver_b_voltage.millivolts;
}

uint16_t motor_build_error_bits(void)
{
    return build_legacy_error_bits(capture_fault_snapshot(millis()));
}

static uint16_t build_legacy_error_bits(const FaultSnapshot &fault)
{
    uint16_t error = 0u;

    if (fault.serial_checksum_error)
        error |= MOTOR_ERR_CHECKSUM_ERROR;
    if (fault.command_timeout)
        error |= MOTOR_ERR_COMMAND_TIMEOUT;
    if (fault.driver_fault)
        error |= MOTOR_ERR_DRIVER_FAULT;
    if (fault.emergency_stop_active)
        error |= MOTOR_ERR_EMERGENCY_STOP_ACTIVE;
    if (fault.battery_low)
        error |= MOTOR_ERR_BATTERY_LOW;
    if (fault.serial_framing_error)
        error |= MOTOR_ERR_SERIAL_FRAMING_ERROR;
    if (fault.command_out_of_range)
        error |= MOTOR_ERR_COMMAND_OUT_OF_RANGE;
    if (fault.watchdog_reset)
        error |= MOTOR_ERR_WATCHDOG_RESET_DETECTED;

    return error;
}

MotorState motor_get_current_state(void)
{
    return resolve_motor_state(capture_fault_snapshot(millis()));
}

static MotorState resolve_motor_state(const FaultSnapshot &fault)
{
    if (fault.driver_fault || fault.battery_low)
        return MOTOR_STATE_FAULT;
    if (fault.emergency_stop_active)
        return MOTOR_STATE_ESTOP;
    if (booting)
        return MOTOR_STATE_BOOTING;
    if (calibration_active)
        return MOTOR_STATE_CALIBRATION;
    if (fault.command_timeout)
        return MOTOR_STATE_TIMEOUT_STOP;
    if (!motor_enabled)
        return MOTOR_STATE_DISABLED;
    return MOTOR_STATE_ENABLED;
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
    if (fault.watchdog_reset)
        error |= SYSTEM_ERR_WATCHDOG_RESET_DETECTED;
    if (fault.driver_status_fault)
        error |= SYSTEM_ERR_DRIVER_STATUS_FAULT_PRESENT;
    if (fault.main_data_stale)
        error |= SYSTEM_ERR_MAIN_DATA_STALE;
    if (fault.voltage_stale)
        error |= SYSTEM_ERR_VOLTAGE_STALE;
    if (fault.can_rx_overrun)
        error |= SYSTEM_ERR_CAN_RX_OVERRUN;
    return error;
}

static uint8_t telemetry_validity_bits(void)
{
    uint8_t validity = 0u;
    if (lf_telemetry.valid)
        validity |= 1u << 0;
    if (rf_telemetry.valid)
        validity |= 1u << 1;
    if (lr_telemetry.valid)
        validity |= 1u << 2;
    if (rr_telemetry.valid)
        validity |= 1u << 3;
    if (driver_a_voltage.valid)
        validity |= 1u << 4;
    if (driver_b_voltage.valid)
        validity |= 1u << 5;
    return validity;
}

static void put_motor_telemetry(uint8_t *packet, uint8_t offset, const MotorTelemetry &telemetry)
{
    packet[offset] = telemetry.status;
    little_endian::write_i16(&packet[offset + 1u], telemetry.actual_rpm);
    little_endian::write_u16(&packet[offset + 3u], telemetry.current_deci_amp);
    little_endian::write_u16(&packet[offset + 5u], telemetry.controller_output);
}

static void protocol_send_system_status(void)
{
    uint8_t packet[SYSTEM_STATUS_PACKET_SIZE] = {};
    const unsigned long now = millis();
    // State and error fields must describe the same instant even when an
    // event latch is cleared after this packet is accepted by USB Serial.
    const FaultSnapshot fault = capture_fault_snapshot(now);
    packet[0] = PKT_HEADER_0;
    packet[1] = PKT_HEADER_1;
    packet[2] = SYSTEM_STATUS_PACKET_LENGTH;
    packet[3] = SYSTEM_STATUS_PACKET_TYPE;
    packet[4] = SYSTEM_STATUS_PACKET_VERSION;
    packet[5] = system_status_tx_seq;
    packet[6] = static_cast<uint8_t>(resolve_motor_state(fault));
    little_endian::write_u16(&packet[7], build_system_error_bits(fault));
    packet[9] = telemetry_validity_bits();
    little_endian::write_u16(&packet[10], driver_a_voltage.millivolts);
    little_endian::write_u16(&packet[12], driver_b_voltage.millivolts);
    put_motor_telemetry(packet, 14u, lf_telemetry);
    put_motor_telemetry(packet, 21u, rf_telemetry);
    put_motor_telemetry(packet, 28u, lr_telemetry);
    put_motor_telemetry(packet, 35u, rr_telemetry);
    packet[42] = protocol_xor_checksum(packet, SYSTEM_STATUS_PACKET_SIZE - 1u);

    if (Serial.availableForWrite() >= SYSTEM_STATUS_PACKET_SIZE)
    {
        Serial.write(packet, SYSTEM_STATUS_PACKET_SIZE);
        system_status_tx_seq++;
        checksum_error_latched = false;
        serial_framing_error_latched = false;
        command_out_of_range_latched = false;
    }
}

void protocol_send_basic_status(void)
{
    uint8_t packet[STATUS_PACKET_SIZE];
    const FaultSnapshot fault = capture_fault_snapshot(millis());
    const uint16_t error = build_legacy_error_bits(fault);
    const uint16_t battery_mv = motor_read_battery_mv();

    packet[0] = PKT_HEADER_0;
    packet[1] = PKT_HEADER_1;
    packet[2] = STATUS_PACKET_LENGTH;
    packet[3] = STATUS_PACKET_TYPE;
    // Basic Status Packet uses its own MCU TX sequence counter.
    packet[4] = status_tx_seq;
    packet[5] = static_cast<uint8_t>(resolve_motor_state(fault));
    little_endian::write_u16(&packet[6], error);
    little_endian::write_u16(&packet[8], battery_mv);
    packet[10] = protocol_xor_checksum(packet, STATUS_PACKET_SIZE - 1);

    if (Serial.availableForWrite() >= STATUS_PACKET_SIZE)
    {
        Serial.write(packet, STATUS_PACKET_SIZE);
        status_tx_seq++;
    }
}

void protocol_send_basic_status_if_due(void)
{
    if (serial_mode != SERIAL_MODE_CONTROL)
        return;

    unsigned long now = millis();
    if (now - last_status_tx_ms >= STATUS_PERIOD_MS)
    {
        last_status_tx_ms = now;
#if _DEBUG
        Serial.print("DEBUG: ");
        Serial.print("State: ");
        Serial.print(motor_get_current_state());
        Serial.print(", Error: ");
        Serial.print(motor_build_error_bits(), HEX);
        Serial.print(", Battery: ");
        Serial.print(motor_read_battery_mv());
        Serial.print(", Last Command: ");
        Serial.print(last_valid_command_ms);
        Serial.print(", Command Timeout: ");
        Serial.print(command_timeout);
        Serial.print(", Estop: ");
        Serial.print(estop_active);
        Serial.print(", Motor Enabled: ");
        Serial.print(motor_enabled);
        Serial.print(", v_pulseWidth: ");
        Serial.print(v_pulseWidth);
        Serial.print(", w_pulseWidth: ");
        Serial.print(w_pulseWidth);
        Serial.print(", Mode: ");
        Serial.println(mode_state);
#else
        protocol_send_basic_status();
#endif
    }
}

void protocol_send_system_status_if_due(void)
{
    if (serial_mode != SERIAL_MODE_CONTROL)
        return;

    const unsigned long now = millis();
    if (now - last_system_status_tx_ms < SYSTEM_STATUS_PERIOD_MS)
        return;

    last_system_status_tx_ms = now;
    protocol_send_system_status();
}

void motor_set_left_right_rpm(float left_rpm, float right_rpm)
{
    target_wheel_rpm_cmd.lf_rpm = left_rpm;
    target_wheel_rpm_cmd.lr_rpm = left_rpm;
    target_wheel_rpm_cmd.rf_rpm = right_rpm;
    target_wheel_rpm_cmd.rr_rpm = right_rpm;
}

void motor_stop_all(void)
{
    motor_set_left_right_rpm(0.0f, 0.0f);
}

static void md200t_send_driver_commands(const Md200tDriverCommand &driver_a, const Md200tDriverCommand &driver_b)
{
    bool driver_a_ok = false;
    if (driver_a.enabled)
    {
        driver_a_ok = md200t_set_velocity(MD200T_DRIVER_A_ID,
                                          rpm_to_i16(driver_a.ch1_rpm),
                                          rpm_to_i16(driver_a.ch2_rpm));
    }
    else
    {
        driver_a_ok = md200t_torque_off(MD200T_DRIVER_A_ID);
    }

    // Always attempt Driver B immediately after Driver A. Both frames were
    // built from the same command snapshot, and one failure must not suppress
    // the other driver's update.
    bool driver_b_ok = false;
    if (driver_b.enabled)
    {
        driver_b_ok = md200t_set_velocity(MD200T_DRIVER_B_ID,
                                          rpm_to_i16(driver_b.ch1_rpm),
                                          rpm_to_i16(driver_b.ch2_rpm));
    }
    else
    {
        driver_b_ok = md200t_torque_off(MD200T_DRIVER_B_ID);
    }

    if (!driver_a_ok || !driver_b_ok)
        latch_safety_can_tx_failure();
    digitalWrite(LED_BUILTIN, motor_has_driver_fault() || battery_low_latched);
}

static void reset_can_tx_scheduler(uint32_t now_us)
{
    can_tx_scheduler.cycle_start_us = now_us;
    can_tx_scheduler.superframe_cycle = 0u;
    can_tx_scheduler.initialized = true;
    can_tx_scheduler.command_sent = false;
    can_tx_scheduler.poll_handled = false;
}

void can_transmit_if_due(uint32_t now_us)
{
    if (serial_mode != SERIAL_MODE_CONTROL || !can_tx_scheduler.initialized)
        return;

    // Advance directly to the current cycle. Missed cycles are not replayed,
    // which prevents a delayed main loop from producing a CAN burst.
    const uint32_t elapsed_us = static_cast<uint32_t>(now_us - can_tx_scheduler.cycle_start_us);
    if (elapsed_us >= CAN_TX_COMMAND_PERIOD_US)
    {
        const uint32_t skipped_cycles = elapsed_us / CAN_TX_COMMAND_PERIOD_US;
        can_tx_scheduler.cycle_start_us += skipped_cycles * CAN_TX_COMMAND_PERIOD_US;
        can_tx_scheduler.superframe_cycle = static_cast<uint8_t>(
            (can_tx_scheduler.superframe_cycle + skipped_cycles % CAN_TX_SUPERFRAME_CYCLES) %
            CAN_TX_SUPERFRAME_CYCLES);
        can_tx_scheduler.command_sent = false;
        can_tx_scheduler.poll_handled = false;
    }

    const uint32_t cycle_phase_us = static_cast<uint32_t>(now_us - can_tx_scheduler.cycle_start_us);
    if (!can_tx_scheduler.command_sent)
    {
        can_tx_scheduler.command_sent = true;

        // Fault and safety gates are evaluated once for both drivers so their
        // back-to-back frames carry one coherent command snapshot.
        const bool command_enabled = motor_enabled && !estop_active && !command_timeout &&
                                     !motor_has_driver_fault() && !battery_low_latched;
        const WheelRpmCommand target = command_enabled
                                           ? target_wheel_rpm_cmd
                                           : WheelRpmCommand{0.0f, 0.0f, 0.0f, 0.0f};

        const Md200tDriverCommand driver_a = {
            target.lf_rpm, // MD200T A MOT1 = LF
            target.rr_rpm, // MD200T A MOT2 = RR
            command_enabled};

        const Md200tDriverCommand driver_b = {
            target.rf_rpm, // MD200T B MOT1 = RF
            target.lr_rpm, // MD200T B MOT2 = LR
            command_enabled};

        md200t_send_driver_commands(driver_a, driver_b);

        // If the pair occupied the +5 ms phase, discard this cycle's poll
        // instead of appending it immediately after the motor commands.
        const uint32_t completed_phase_us = static_cast<uint32_t>(micros() - can_tx_scheduler.cycle_start_us);
        if (cycle_phase_us >= CAN_TX_POLL_PHASE_US || completed_phase_us >= CAN_TX_POLL_PHASE_US)
            can_tx_scheduler.poll_handled = true;
        return;
    }

    if (can_tx_scheduler.poll_handled || cycle_phase_us < CAN_TX_POLL_PHASE_US)
        return;

    can_tx_scheduler.poll_handled = true;
    ScheduledPollRequest request = {};
    if (!scheduled_poll_for_cycle(can_tx_scheduler.superframe_cycle, request))
        return;

    // PID 4 only starts the request. Responses are decoded later by the normal
    // non-blocking RX drain, so this scheduler never waits for telemetry.
    poll_can_tx_failure_active = !md200t_request_pid_data(request.driver_id, request.pid);
}

void v_decodePWM()
{
    static unsigned long prevTime = 0;
    static unsigned long dataA[10] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    static unsigned int idx = 0;
    static unsigned long sum = 15000;

    if ((prevTime != 0) && !digitalRead(v_speed_controller_pin))
    {
        unsigned long tmp = micros() - prevTime;
        if (tmp > 2000)
            return;
        sum += tmp - dataA[idx];
        dataA[idx] = tmp;
        idx = (idx + 1) % 10;
        v_pulseWidth = sum / 10;
        prevTime = 0;
    }
    else
    {
        prevTime = micros();
    }
}

void w_decodePWM()
{
    static unsigned long prevTime = 0;
    static unsigned long dataA[10] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    static unsigned int idx = 0;
    static unsigned long sum = 15000;

    if ((prevTime != 0) && !digitalRead(w_speed_controller_pin))
    {
        unsigned long tmp = micros() - prevTime;
        if (tmp > 2000)
            return;
        sum += tmp - dataA[idx];
        dataA[idx] = tmp;
        idx = (idx + 1) % 10;
        w_pulseWidth = sum / 10;
        prevTime = 0;
    }
    else
    {
        prevTime = micros();
    }
}

void mode_decodePWM()
{
    static unsigned long prevTime = 0;
    static unsigned long dataA[10] = {1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
    static unsigned int idx = 0;
    static unsigned long sum = 15000;

    if ((prevTime != 0) && !digitalRead(mode_control_pin))
    {
        unsigned long tmp = micros() - prevTime;
        if (tmp > 2000)
            return;
        sum += tmp - dataA[idx];
        dataA[idx] = tmp;
        idx = (idx + 1) % 10;
        if (sum / 10 < 1300)
            mode_state = DRIVE_MODE_STOP;
        else if (sum / 10 < 1700)
            mode_state = DRIVE_MODE_MANUAL;
        else
            mode_state = DRIVE_MODE_AUTO;
        prevTime = 0;
    }
    else
    {
        prevTime = micros();
    }
}
