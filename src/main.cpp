#include <Arduino.h>

#include "flexcan0.hpp"
#include "md200t_can.hpp"

#define _DEBUG 1

// car width and wheel radius
const float W = 0.42; // m
const float R = 0.13; // m

// Command packet protocol: AA 55 07 01 seq v_lo v_hi w_lo w_hi flags checksum
const uint8_t PKT_HEADER_0 = 0xAA;
const uint8_t PKT_HEADER_1 = 0x55;
const uint8_t CMD_PACKET_LENGTH = 7;
const uint8_t CMD_PACKET_TYPE = 0x01;
const uint8_t CMD_PACKET_SIZE = 11;
const uint8_t STATUS_PACKET_LENGTH = 7;
const uint8_t STATUS_PACKET_TYPE = 0x81;
const uint8_t STATUS_PACKET_SIZE = 11;
const int16_t CMD_LINEAR_MILLI_MPS_MIN = -2000;
const int16_t CMD_LINEAR_MILLI_MPS_MAX = 2000;
const int16_t CMD_ANGULAR_MILLI_RADPS_MIN = -5000;
const int16_t CMD_ANGULAR_MILLI_RADPS_MAX = 5000;
const float CMD_MILLI_UNIT_SCALE = 1000.0f;
const uint8_t CMD_FLAG_ENABLE = 0x01;
const uint8_t CMD_FLAG_ESTOP = 0x02;
const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long STATUS_PERIOD_MS = 500;      // 2 Hz
const uint16_t BATTERY_LOW_THRESHOLD_MV = 21000; // 24 V battery placeholder threshold
const uint32_t MD200T_CAN_BITRATE = 250000;

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

struct CommandFrameParser
{
    uint8_t packet[CMD_PACKET_SIZE];
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
// Read battery voltage in millivolts; returns 0 until ADC sensing is implemented.
uint16_t motor_read_battery_mv(void);
// Compute XOR checksum for ROS-MCU protocol packets.
uint8_t protocol_xor_checksum(const uint8_t *data, size_t len);
// Send one basic status packet to ROS over USB Serial.
void protocol_send_basic_status(void);
// Send the periodic basic status packet when the 20 Hz interval has elapsed.
void protocol_send_basic_status_if_due(void);
// Convert left/right skid-steer RPM targets into four wheel RPM targets.
void motor_set_left_right_rpm(float left_rpm, float right_rpm);
// Set every wheel target RPM to zero.
void motor_stop_all(void);
// Periodically forward the current wheel targets to the MD200T command layer.
void motor_send_target_if_due(void);
// Check whether command packet velocity fields are inside supported physical limits.
static bool command_in_range(const CommandPacket &command);
// Convert linear wheel speed in m/s to wheel RPM.
static float vel_mps_to_rpm(float v_mps);
// Convert floating-point target RPM to MD200T int16 command data.
static int16_t rpm_to_i16(float rpm);
// Decode a little-endian signed 16-bit integer from packet bytes.
static int16_t read_i16_le(const unsigned char *data);
// Compute XOR checksum for a command frame candidate.
static uint8_t xor_checksum(const unsigned char *data, uint8_t len);
// Convert packet milli-m/s linear velocity to m/s.
static float milli_mps_to_mps(int16_t milli_mps);
// Convert packet milli-rad/s angular velocity to rad/s.
static float milli_radps_to_radps(int16_t milli_radps);
// Reset command frame parser to wait for the next packet header.
static void command_parser_reset(CommandFrameParser &parser);
// Apply a validated ROS command packet to MCU drive state and target RPM.
static void apply_command_packet(const CommandPacket &command, unsigned long now);
// Parse a complete command frame buffer and apply it when valid.
static void parse_complete_command_frame(const uint8_t *packet, unsigned long now);
// Feed one Serial byte into the command frame state machine.
static void feed_command_parser(uint8_t byte_in, unsigned long now);
// Drain all currently buffered USB Serial bytes into the command parser.
static void serial_drain_command_packets(unsigned long now);
// Send prepared per-driver commands to MD200T devices over CAN once protocol details are set.
static void md200t_send_driver_commands(const Md200tDriverCommand &driver_a, const Md200tDriverCommand &driver_b);

static bool booting = true;
static bool calibration_active = false;
static bool motor_enabled = false;
static bool estop_active = false;
static bool command_timeout = true;
static bool driver_fault = false;
static bool over_current = false;
static bool over_temperature = false;
static bool parameter_error = false;
static bool watchdog_reset_detected = false;
static bool checksum_error_latched = false;
static bool serial_framing_error_latched = false;
static bool command_out_of_range_latched = false;
static unsigned long last_valid_command_ms = 0;
static unsigned long last_status_tx_ms = 0;
static unsigned long last_md200t_command_tx_ms = 0;
static uint8_t status_tx_seq = 0;

const unsigned long MD200T_COMMAND_PERIOD_MS = 10;

static WheelRpmCommand target_wheel_rpm_cmd = {0.0f, 0.0f, 0.0f, 0.0f};
static CommandFrameParser command_rx_parser = {{0}, 0, CMD_RX_WAIT_HEADER_0};

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
        driver_fault = true;
        digitalWrite(LED_BUILTIN, HIGH);
    }

    booting = false;
}

void loop()
{
    unsigned long now = millis();

    if (mode_state == DRIVE_MODE_AUTO)
    {
        command_timeout = (last_valid_command_ms == 0) || (now - last_valid_command_ms > COMMAND_TIMEOUT_MS);
        if (command_timeout)
            motor_enabled = false;
    }
    else
    {
        command_timeout = false;
        estop_active = false;
        motor_enabled = (mode_state == DRIVE_MODE_MANUAL);
    }

    if (mode_state == DRIVE_MODE_STOP)
    {
        // Stop mode
        motor_stop_all();
    }
    else if (mode_state == DRIVE_MODE_MANUAL)
    {
        float v_velocity = v_pulseWidth;
        float w_velocity = w_pulseWidth;

        v_velocity -= 1500; // 양수면 전진
        w_velocity -= 1500; // 양수면 좌선회 (우측 바퀴가 +)

        v_velocity /= 250; // -500~500 범위를 -2~2 범위로 줄임. (m/s)
        w_velocity /= 100; // -500~500 범위를 -5~5 범위로 줄임. (rad/s)

        float left_velocity = 0;
        float right_velocity = 0;

        left_velocity = v_velocity - w_velocity * W / 2;
        right_velocity = v_velocity + w_velocity * W / 2;

        // --- Target RPM from v/w command ---
        float left_rpm_ref = vel_mps_to_rpm((float)left_velocity);
        float right_rpm_ref = vel_mps_to_rpm((float)right_velocity);

        motor_set_left_right_rpm(left_rpm_ref, right_rpm_ref);
    }
    else if (mode_state == DRIVE_MODE_AUTO)
    {
        serial_drain_command_packets(now);

        if (command_timeout)
        {
            motor_stop_all();
        }
    }

    motor_send_target_if_due();
    protocol_send_basic_status_if_due();
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

static inline int16_t read_i16_le(const unsigned char *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static inline uint8_t xor_checksum(const unsigned char *data, uint8_t len)
{
    uint8_t checksum = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
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
    last_valid_command_ms = now;
    command_timeout = false;
    estop_active = (command.flags & CMD_FLAG_ESTOP) != 0;
    motor_enabled = ((command.flags & CMD_FLAG_ENABLE) != 0) && !estop_active;

    float v_velocity = milli_mps_to_mps(command.v_milli_mps);
    float w_velocity = milli_radps_to_radps(command.w_milli_radps);

    if (!motor_enabled)
    {
        v_velocity = 0.0f;
        w_velocity = 0.0f;
    }

    float left_velocity = v_velocity - w_velocity * W / 2;
    float right_velocity = v_velocity + w_velocity * W / 2;

    float left_rpm_ref = vel_mps_to_rpm(left_velocity);
    float right_rpm_ref = vel_mps_to_rpm(right_velocity);

    motor_set_left_right_rpm(left_rpm_ref, right_rpm_ref);
}

static void parse_complete_command_frame(const uint8_t *packet, unsigned long now)
{
    if (xor_checksum(packet, CMD_PACKET_SIZE - 1) != packet[CMD_PACKET_SIZE - 1])
    {
        checksum_error_latched = true;
        return;
    }

    CommandPacket command = {
        read_i16_le(&packet[5]),
        read_i16_le(&packet[7]),
        packet[9]};

    if (!command_in_range(command))
    {
        command_out_of_range_latched = true;
        return;
    }

    apply_command_packet(command, now);
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
        if (byte_in == CMD_PACKET_LENGTH)
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
        if (byte_in == CMD_PACKET_TYPE)
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

        if (command_rx_parser.index >= CMD_PACKET_SIZE)
        {
            parse_complete_command_frame(command_rx_parser.packet, now);
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
    // TODO: Add ADC-based battery voltage measurement. Return 0 while unsupported.
    return 0;
}

uint16_t motor_build_error_bits(void)
{
    uint16_t error = 0;

    if (checksum_error_latched)
        error |= MOTOR_ERR_CHECKSUM_ERROR;
    if (command_timeout)
        error |= MOTOR_ERR_COMMAND_TIMEOUT;
    if (driver_fault)
        error |= MOTOR_ERR_DRIVER_FAULT;
    if (estop_active)
        error |= MOTOR_ERR_EMERGENCY_STOP_ACTIVE;

    uint16_t battery_mv = motor_read_battery_mv();
    if (battery_mv > 0 && battery_mv < BATTERY_LOW_THRESHOLD_MV)
        error |= MOTOR_ERR_BATTERY_LOW;

    if (serial_framing_error_latched)
        error |= MOTOR_ERR_SERIAL_FRAMING_ERROR;
    if (command_out_of_range_latched)
        error |= MOTOR_ERR_COMMAND_OUT_OF_RANGE;
    if (watchdog_reset_detected)
        error |= MOTOR_ERR_WATCHDOG_RESET_DETECTED;
    if (over_current)
        error |= MOTOR_ERR_OVER_CURRENT;
    if (over_temperature)
        error |= MOTOR_ERR_OVER_TEMPERATURE;
    if (parameter_error)
        error |= MOTOR_ERR_PARAMETER_ERROR;

    return error;
}

MotorState motor_get_current_state(void)
{
    if (driver_fault || over_current || over_temperature || parameter_error)
        return MOTOR_STATE_FAULT;
    if (estop_active)
        return MOTOR_STATE_ESTOP;
    if (booting)
        return MOTOR_STATE_BOOTING;
    if (calibration_active)
        return MOTOR_STATE_CALIBRATION;
    if (command_timeout)
        return MOTOR_STATE_TIMEOUT_STOP;
    if (!motor_enabled)
        return MOTOR_STATE_DISABLED;
    return MOTOR_STATE_ENABLED;
}

void protocol_send_basic_status(void)
{
    uint8_t packet[STATUS_PACKET_SIZE];
    uint16_t error = motor_build_error_bits();
    uint16_t battery_mv = motor_read_battery_mv();

    packet[0] = PKT_HEADER_0;
    packet[1] = PKT_HEADER_1;
    packet[2] = STATUS_PACKET_LENGTH;
    packet[3] = STATUS_PACKET_TYPE;
    // Basic Status Packet uses its own MCU TX sequence counter.
    packet[4] = status_tx_seq;
    packet[5] = (uint8_t)motor_get_current_state();
    packet[6] = (uint8_t)(error & 0xFF);
    packet[7] = (uint8_t)((error >> 8) & 0xFF);
    packet[8] = (uint8_t)(battery_mv & 0xFF);
    packet[9] = (uint8_t)((battery_mv >> 8) & 0xFF);
    packet[10] = protocol_xor_checksum(packet, STATUS_PACKET_SIZE - 1);

    if (Serial.availableForWrite() >= STATUS_PACKET_SIZE)
    {
        Serial.write(packet, STATUS_PACKET_SIZE);
        status_tx_seq++;
        checksum_error_latched = false;
        serial_framing_error_latched = false;
        command_out_of_range_latched = false;
    }
}

void protocol_send_basic_status_if_due(void)
{
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
    bool ok = true;

    if (driver_a.enabled)
    {
        ok = md200t_set_motor1_rpm(MD200T_DRIVER_A_ID, rpm_to_i16(driver_a.ch1_rpm)) && ok;
        ok = md200t_set_motor2_rpm(MD200T_DRIVER_A_ID, rpm_to_i16(driver_a.ch2_rpm)) && ok;
    }
    else
    {
        ok = md200t_torque_off(MD200T_DRIVER_A_ID) && ok;
    }

    if (driver_b.enabled)
    {
        ok = md200t_set_motor1_rpm(MD200T_DRIVER_B_ID, rpm_to_i16(driver_b.ch1_rpm)) && ok;
        ok = md200t_set_motor2_rpm(MD200T_DRIVER_B_ID, rpm_to_i16(driver_b.ch2_rpm)) && ok;
    }
    else
    {
        ok = md200t_torque_off(MD200T_DRIVER_B_ID) && ok;
    }

    driver_fault = !ok;
    digitalWrite(LED_BUILTIN, !ok);
}

void motor_send_target_if_due(void)
{
    unsigned long now = millis();
    if (now - last_md200t_command_tx_ms < MD200T_COMMAND_PERIOD_MS)
        return;

    last_md200t_command_tx_ms = now;

    bool command_enabled = motor_enabled && !estop_active && !command_timeout && !driver_fault && !over_current && !over_temperature && !parameter_error;
    WheelRpmCommand target = command_enabled ? target_wheel_rpm_cmd : WheelRpmCommand{0.0f, 0.0f, 0.0f, 0.0f};

    Md200tDriverCommand driver_a = {
        target.lf_rpm, // MD200T A CH1 = LF
        target.rr_rpm, // MD200T A CH2 = RR
        command_enabled};

    Md200tDriverCommand driver_b = {
        target.rf_rpm, // MD200T B CH1 = RF
        target.lr_rpm, // MD200T B CH2 = LR
        command_enabled};

    md200t_send_driver_commands(driver_a, driver_b);
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
