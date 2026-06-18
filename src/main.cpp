#include <Arduino.h>

// car width and wheel radius
const double W = 0.42; // m
const double R = 0.14; // m

// Command packet protocol: AA 55 07 01 seq v_lo v_hi w_lo w_hi flags checksum
const uint8_t PKT_HEADER_0 = 0xAA;
const uint8_t PKT_HEADER_1 = 0x55;
const uint8_t CMD_PACKET_LENGTH = 7;
const uint8_t CMD_PACKET_TYPE = 0x01;
const uint8_t CMD_PACKET_SIZE = 11;
const uint8_t STATUS_PACKET_LENGTH = 7;
const uint8_t STATUS_PACKET_TYPE = 0x81;
const uint8_t STATUS_PACKET_SIZE = 11;
const uint8_t RX_QUEUE_SIZE = 64;
const int16_t CMD_MIN = -1000;
const int16_t CMD_MAX = 1000;
const float CMD_NORMALIZATION_SCALE = 1000.0f;
const float MAX_LINEAR_MPS = 2.0f;
const float MAX_ANGULAR_RADPS = 5.0f;
const uint8_t CMD_FLAG_ENABLE = 0x01;
const uint8_t CMD_FLAG_ESTOP = 0x02;
const unsigned long COMMAND_TIMEOUT_MS = 500;
const unsigned long STATUS_PERIOD_MS = 50;       // 20 Hz
const uint16_t BATTERY_LOW_THRESHOLD_MV = 21000; // 24 V battery placeholder threshold

struct CommandPacket
{
    int16_t v_cmd;
    int16_t w_cmd;
    uint8_t flags;
};

struct ByteQueue
{
    unsigned char data[RX_QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
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

void v_decodePWM();
void w_decodePWM();
void mode_decodePWM();
uint16_t motor_build_error_bits(void);
MotorState motor_get_current_state(void);
uint16_t motor_read_battery_mv(void);
uint8_t protocol_xor_checksum(const uint8_t *data, size_t len);
void protocol_send_basic_status(void);
void protocol_send_basic_status_if_due(void);

// motor speed control pins
const int lf_speed_control_pin = 3;
const int lr_speed_control_pin = 4;
const int rr_speed_control_pin = 5;
const int rf_speed_control_pin = 6;

volatile unsigned int led_state = 0;

#define CONTROL_DT_MS 10

void control_tick();

// motor direction control pins
const int left_dir_control_pin = 14;
const int right_dir_control_pin = 15;

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
static uint8_t status_tx_seq = 0;

static inline bool command_in_range(int16_t cmd)
{
    return cmd >= CMD_MIN && cmd <= CMD_MAX;
}

// Helper: m/s -> wheel RPM
static inline float vel_mps_to_rpm(float v_mps)
{
    // wheel angular speed (rad/s) = v / R
    float omega = v_mps / (float)R;
    // RPM = omega * 60 / (2*pi)
    return omega * 60.0f / (2.0f * 3.1415926535f);
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

static inline float normalized_linear_to_mps(int16_t cmd)
{
    // ROS maps max linear.x to +/-1000. Keep the scale equal to manual mode.
    return ((float)cmd / CMD_NORMALIZATION_SCALE) * MAX_LINEAR_MPS;
}

static inline float normalized_angular_to_radps(int16_t cmd)
{
    // ROS maps max angular.z to +/-1000. Keep the scale equal to manual mode.
    return ((float)cmd / CMD_NORMALIZATION_SCALE) * MAX_ANGULAR_RADPS;
}

static inline void queue_push(ByteQueue &queue, unsigned char value)
{
    if (queue.count >= RX_QUEUE_SIZE)
    {
        queue.tail = (queue.tail + 1) % RX_QUEUE_SIZE;
        queue.count--;
    }

    queue.data[queue.head] = value;
    queue.head = (queue.head + 1) % RX_QUEUE_SIZE;
    queue.count++;
}

static inline unsigned char queue_peek(const ByteQueue &queue, uint8_t offset)
{
    return queue.data[(queue.tail + offset) % RX_QUEUE_SIZE];
}

static inline void queue_pop(ByteQueue &queue)
{
    if (queue.count == 0)
        return;

    queue.tail = (queue.tail + 1) % RX_QUEUE_SIZE;
    queue.count--;
}

static void read_serial_byte_to_queue(ByteQueue &queue)
{
    if (Serial.available() <= 0)
        return;

    int byte_in = Serial.read();
    if (byte_in >= 0)
        queue_push(queue, (unsigned char)byte_in);
}

static bool try_parse_command_packet(ByteQueue &queue, CommandPacket &command)
{
    if (queue.count < 2)
        return false;

    if (queue_peek(queue, 0) != PKT_HEADER_0)
    {
        serial_framing_error_latched = true;
        queue_pop(queue);
        return false;
    }

    if (queue_peek(queue, 1) != PKT_HEADER_1)
    {
        serial_framing_error_latched = true;
        queue_pop(queue);
        return false;
    }

    if (queue.count < 4)
        return false;

    if (queue_peek(queue, 2) != CMD_PACKET_LENGTH || queue_peek(queue, 3) != CMD_PACKET_TYPE)
    {
        serial_framing_error_latched = true;
        queue_pop(queue);
        return false;
    }

    if (queue.count < CMD_PACKET_SIZE)
        return false;

    unsigned char packet[CMD_PACKET_SIZE];
    for (uint8_t i = 0; i < CMD_PACKET_SIZE; i++)
    {
        packet[i] = queue_peek(queue, i);
    }

    if (xor_checksum(packet, CMD_PACKET_SIZE - 1) != packet[CMD_PACKET_SIZE - 1])
    {
        checksum_error_latched = true;
        queue_pop(queue);
        return false;
    }

    command.v_cmd = read_i16_le(&packet[5]);
    command.w_cmd = read_i16_le(&packet[7]);
    command.flags = packet[9];

    for (uint8_t i = 0; i < CMD_PACKET_SIZE; i++)
    {
        queue_pop(queue);
    }

    if (!command_in_range(command.v_cmd) || !command_in_range(command.w_cmd))
    {
        command_out_of_range_latched = true;
        return false;
    }

    return true;
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
        protocol_send_basic_status();
    }
}

// --- Fixed-rate 10ms control scheduling (Teensy IntervalTimer) ---
IntervalTimer controlTimer;

// Target wheel RPM references computed in loop() and consumed in control ISR
volatile float left_rpm_ref_cmd = 0.0f;
volatile float right_rpm_ref_cmd = 0.0f;

void setup()
{
    Serial.begin(115200);

    pinMode(v_speed_controller_pin, INPUT);
    pinMode(w_speed_controller_pin, INPUT);
    pinMode(mode_control_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(v_speed_controller_pin), v_decodePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(w_speed_controller_pin), w_decodePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(mode_control_pin), mode_decodePWM, CHANGE);

    pinMode(lf_speed_control_pin, OUTPUT);
    pinMode(lr_speed_control_pin, OUTPUT);
    pinMode(rr_speed_control_pin, OUTPUT);
    pinMode(rf_speed_control_pin, OUTPUT);

    analogWriteFrequency(lf_speed_control_pin, 10000); // 10 kHz PWM
    analogWriteFrequency(lr_speed_control_pin, 10000);
    analogWriteFrequency(rr_speed_control_pin, 10000);
    analogWriteFrequency(rf_speed_control_pin, 10000);

    analogWriteResolution(10); // 10 bit resolution (0-1023)

    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(left_dir_control_pin, OUTPUT);
    pinMode(right_dir_control_pin, OUTPUT);

    // Start fixed-rate 10ms control loop
    controlTimer.begin(control_tick, CONTROL_DT_MS * 1000); // microseconds
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
        noInterrupts();
        left_rpm_ref_cmd = 0.0f;
        right_rpm_ref_cmd = 0.0f;
        interrupts();
    }
    else if (mode_state == DRIVE_MODE_MANUAL)
    {
        double v_velocity = v_pulseWidth;
        double w_velocity = w_pulseWidth;

        v_velocity -= 1500; // 양수면 전진
        w_velocity -= 1500; // 양수면 좌선회 (우측 바퀴가 +)

        v_velocity /= 250; // -500~500 범위를 -2~2 범위로 줄임. (m/s)
        w_velocity /= 100; // -500~500 범위를 -5~5 범위로 줄임. (rad/s)

        double left_velocity = 0;
        double right_velocity = 0;

        left_velocity = v_velocity - w_velocity * W / 2;
        right_velocity = v_velocity + w_velocity * W / 2;

        // --- Target RPM from v/w command ---
        float left_rpm_ref = vel_mps_to_rpm((float)left_velocity);
        float right_rpm_ref = vel_mps_to_rpm((float)right_velocity);

        noInterrupts();
        left_rpm_ref_cmd = left_rpm_ref;
        right_rpm_ref_cmd = right_rpm_ref;
        interrupts();

        // // Debug (optional)
        // Serial.print(">L_rpm_ref:");
        // Serial.println(left_rpm_ref);
        // Serial.print(">R_rpm_ref:");
        // Serial.println(right_rpm_ref);
        // Serial.print(">lf_RPS:");
        // Serial.println((float)lf_sum_1s / (float)WHEEL_COUNTS_PER_REV);
        // Serial.print(">lr_RPS:");
        // Serial.println((float)lr_sum_1s / (float)WHEEL_COUNTS_PER_REV);
        // Serial.print(">rf_RPS:");
        // Serial.println((float)rf_sum_1s / (float)WHEEL_COUNTS_PER_REV);
        // Serial.print(">rr_RPS:");
        // Serial.println((float)rr_sum_1s / (float)WHEEL_COUNTS_PER_REV);
        // // Serial.print(">lf_sum_1s:");
        // // Serial.println(lf_sum_1s);
        // // Serial.print(">lr_sum_1s:");
        // // Serial.println(lr_sum_1s);
        // // Serial.print(">rr_sum_1s:");
        // // Serial.println(rr_sum_1s);
        // // Serial.print(">rf_sum_1s:");
        // // Serial.println(rf_sum_1s);
        // delay(10);
    }
    else if (mode_state == DRIVE_MODE_AUTO)
    {
        // Auto mode: receive normalized ROS command packet from UART
        static ByteQueue rx_queue = {{0}, 0, 0, 0};
        CommandPacket command;

        read_serial_byte_to_queue(rx_queue);

        if (try_parse_command_packet(rx_queue, command))
        {
            last_valid_command_ms = now;
            command_timeout = false;
            estop_active = (command.flags & CMD_FLAG_ESTOP) != 0;
            motor_enabled = ((command.flags & CMD_FLAG_ENABLE) != 0) && !estop_active;

            float v_velocity = normalized_linear_to_mps(command.v_cmd);
            float w_velocity = normalized_angular_to_radps(command.w_cmd);

            if (!motor_enabled)
            {
                v_velocity = 0.0f;
                w_velocity = 0.0f;
            }

            double left_velocity = v_velocity - w_velocity * W / 2;
            double right_velocity = v_velocity + w_velocity * W / 2;

            // --- Target RPM from v/w command ---
            float left_rpm_ref = vel_mps_to_rpm((float)left_velocity);
            float right_rpm_ref = vel_mps_to_rpm((float)right_velocity);

            noInterrupts();
            left_rpm_ref_cmd = left_rpm_ref;
            right_rpm_ref_cmd = right_rpm_ref;
            interrupts();
        }

        if (command_timeout)
        {
            noInterrupts();
            left_rpm_ref_cmd = 0.0f;
            right_rpm_ref_cmd = 0.0f;
            interrupts();
        }
    }

    protocol_send_basic_status_if_due();
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

void control_tick()
{
    float left_rpm_ref, right_rpm_ref;

    noInterrupts();
    left_rpm_ref = left_rpm_ref_cmd;
    right_rpm_ref = right_rpm_ref_cmd;
    interrupts();

    // Direction from target sign; control magnitude is PI output
    bool left_fwd = (left_rpm_ref >= 0.0f);
    bool right_fwd = (right_rpm_ref >= 0.0f);

    float left_ref_abs = fabsf(left_rpm_ref);
    float right_ref_abs = fabsf(right_rpm_ref);

    // Low-speed deadband to reduce hunting
    if (left_ref_abs < 1.0f)
        left_ref_abs = 0.0f;
    if (right_ref_abs < 1.0f)
        right_ref_abs = 0.0f;

    int lf_u = constrain(left_ref_abs * 5, 0, 1023); // Open-loop for initial testing
    int lr_u = constrain(left_ref_abs * 5, 0, 1023);
    int rr_u = constrain(right_ref_abs * 5, 0, 1023);
    int rf_u = constrain(right_ref_abs * 5, 0, 1023);

    // Apply direction pins
    digitalWrite(left_dir_control_pin, left_fwd ? HIGH : LOW);
    digitalWrite(right_dir_control_pin, right_fwd ? LOW : HIGH);

    // Apply PWM
    analogWrite(lf_speed_control_pin, (int)lf_u);
    analogWrite(lr_speed_control_pin, (int)lr_u);
    analogWrite(rr_speed_control_pin, (int)rr_u);
    analogWrite(rf_speed_control_pin, (int)rf_u);
}
