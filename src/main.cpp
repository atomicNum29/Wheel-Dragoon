#include <Arduino.h>

// car width and wheel radius
const double W = 0.42; // m
const double R = 0.14; // m

// remote control signal pins
const int w_speed_controller_pin = 0;
const int v_speed_controller_pin = 1;

volatile unsigned int v_pulseWidth = 0;
volatile unsigned int w_pulseWidth = 0;

void v_decodePWM();
void w_decodePWM();

// motor speed control pins
const int lf_speed_control_pin = 3;
const int lr_speed_control_pin = 4;
const int rr_speed_control_pin = 5;
const int rf_speed_control_pin = 6;

volatile unsigned int led_state = 0;

// motor control feedback pins, 60 CPR encoders
#define WHEEL_COUNTS_PER_REV 60
#define WHEEL_DT_MS 10
#define WHEEL_WINDOW_MS 1000
#define WHEEL_BINS (WHEEL_WINDOW_MS / WHEEL_DT_MS)

const int lf_wheel_pulse_pin = 9;
const int lr_wheel_pulse_pin = 10;
const int rr_wheel_pulse_pin = 11;
const int rf_wheel_pulse_pin = 12;

// Per-10ms bin counters (incremented by encoder ISR)
volatile uint16_t lf_bin_count = 0;
volatile uint16_t lr_bin_count = 0;
volatile uint16_t rr_bin_count = 0;
volatile uint16_t rf_bin_count = 0;

// Ring buffers of per-bin counts and 1-second sliding sums
volatile uint16_t lf_bins[WHEEL_BINS] = {0};
volatile uint16_t lr_bins[WHEEL_BINS] = {0};
volatile uint16_t rr_bins[WHEEL_BINS] = {0};
volatile uint16_t rf_bins[WHEEL_BINS] = {0};

volatile uint32_t lf_sum_1s = 0;
volatile uint32_t lr_sum_1s = 0;
volatile uint32_t rr_sum_1s = 0;
volatile uint32_t rf_sum_1s = 0;

volatile uint16_t wheel_bin_idx = 0;

void lf_wheel_pulse_ISR();
void lr_wheel_pulse_ISR();
void rr_wheel_pulse_ISR();
void rf_wheel_pulse_ISR();
void wheel_bins_update();
void control_tick();

// motor direction control pins
const int left_dir_control_pin = 14;
const int right_dir_control_pin = 15;

// --- Simple PI speed controller (per side) ---
struct PIController
{
    float kp;
    float ki;
    float integral;
    float out_min;
    float out_max;
    float i_min;
    float i_max;
};

static inline float pi_update(PIController &pi, float error, float dt_s)
{
    // Integrator with clamp (basic anti-windup)
    pi.integral += error * dt_s;
    pi.integral = constrain(pi.integral, pi.i_min, pi.i_max);

    float u = pi.kp * error + pi.ki * pi.integral;
    return constrain(u, pi.out_min, pi.out_max);
}

// Tunables (start conservative)
PIController leftPI = {.kp = 8.0f, .ki = 2.0f, .integral = 0.0f, .out_min = 0.0f, .out_max = 1023.0f, .i_min = -300.0f, .i_max = 300.0f};
PIController rightPI = {.kp = 8.0f, .ki = 2.0f, .integral = 0.0f, .out_min = 0.0f, .out_max = 1023.0f, .i_min = -300.0f, .i_max = 300.0f};

// 10ms control loop
static constexpr float CTRL_DT_S = (float)WHEEL_DT_MS / 1000.0f;

// Helper: m/s -> wheel RPM
static inline float vel_mps_to_rpm(float v_mps)
{
    // wheel angular speed (rad/s) = v / R
    float omega = v_mps / (float)R;
    // RPM = omega * 60 / (2*pi)
    return omega * 60.0f / (2.0f * 3.1415926535f);
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
    attachInterrupt(digitalPinToInterrupt(v_speed_controller_pin), v_decodePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(w_speed_controller_pin), w_decodePWM, CHANGE);

    pinMode(lf_speed_control_pin, OUTPUT);
    pinMode(lr_speed_control_pin, OUTPUT);
    pinMode(rr_speed_control_pin, OUTPUT);
    pinMode(rf_speed_control_pin, OUTPUT);

    analogWriteFrequency(lf_speed_control_pin, 10000); // 10 kHz PWM
    analogWriteFrequency(lr_speed_control_pin, 10000);
    analogWriteFrequency(rr_speed_control_pin, 10000);
    analogWriteFrequency(rf_speed_control_pin, 10000);

    analogWriteResolution(10); // 10 bit resolution (0-1023)

    attachInterrupt(digitalPinToInterrupt(lf_wheel_pulse_pin), lf_wheel_pulse_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(lr_wheel_pulse_pin), lr_wheel_pulse_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rr_wheel_pulse_pin), rr_wheel_pulse_ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(rf_wheel_pulse_pin), rf_wheel_pulse_ISR, CHANGE);

    pinMode(LED_BUILTIN, OUTPUT);

    pinMode(left_dir_control_pin, OUTPUT);
    pinMode(right_dir_control_pin, OUTPUT);

    // Start fixed-rate 10ms control loop
    controlTimer.begin(control_tick, WHEEL_DT_MS * 1000); // microseconds
}

void loop()
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
    // Serial.print(">L_rpm_meas:");
    // Serial.println(left_rpm_meas);
    // Serial.print(">R_rpm_ref:");
    // Serial.println(right_rpm_ref);
    // Serial.print(">R_rpm_meas:");
    // Serial.println(right_rpm_meas);
    // delay(10);
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

void lf_wheel_pulse_ISR()
{
    lf_bin_count++;
    digitalWrite(LED_BUILTIN, led_state ^= 1);
}

void lr_wheel_pulse_ISR()
{
    lr_bin_count++;
    digitalWrite(LED_BUILTIN, led_state ^= 1);
}

void rr_wheel_pulse_ISR()
{
    rr_bin_count++;
    digitalWrite(LED_BUILTIN, led_state ^= 1);
}

void rf_wheel_pulse_ISR()
{
    rf_bin_count++;
    digitalWrite(LED_BUILTIN, led_state ^= 1);
}

void wheel_bins_update()
{
    // Snapshot current bin counts atomically, then clear them
    uint16_t lf_c, lr_c, rf_c, rr_c;
    noInterrupts();
    lf_c = lf_bin_count;
    lf_bin_count = 0;
    lr_c = lr_bin_count;
    lr_bin_count = 0;
    rf_c = rf_bin_count;
    rf_bin_count = 0;
    rr_c = rr_bin_count;
    rr_bin_count = 0;
    interrupts();

    // Advance ring index
    wheel_bin_idx = (wheel_bin_idx + 1) % WHEEL_BINS;

    // Update sums: sum = sum - old_bin + new_bin
    lf_sum_1s = lf_sum_1s - lf_bins[wheel_bin_idx] + lf_c;
    lf_bins[wheel_bin_idx] = lf_c;

    lr_sum_1s = lr_sum_1s - lr_bins[wheel_bin_idx] + lr_c;
    lr_bins[wheel_bin_idx] = lr_c;

    rf_sum_1s = rf_sum_1s - rf_bins[wheel_bin_idx] + rf_c;
    rf_bins[wheel_bin_idx] = rf_c;

    rr_sum_1s = rr_sum_1s - rr_bins[wheel_bin_idx] + rr_c;
    rr_bins[wheel_bin_idx] = rr_c;
}

void control_tick()
{
    // 1) Update 1-second sliding window sums (pure counter math)
    wheel_bins_update();

    // 2) Snapshot measurements (1-second pulses) and references atomically
    uint32_t lf1, lr1, rf1, rr1;
    float left_rpm_ref, right_rpm_ref;

    noInterrupts();
    lf1 = lf_sum_1s;
    lr1 = lr_sum_1s;
    rf1 = rf_sum_1s;
    rr1 = rr_sum_1s;
    left_rpm_ref = left_rpm_ref_cmd;
    right_rpm_ref = right_rpm_ref_cmd;
    interrupts();

    // pulses/sec -> RPM = (pulses_per_sec / CPR) * 60
    float lf_rpm = (float)lf1 * 60.0f / (float)WHEEL_COUNTS_PER_REV;
    float lr_rpm = (float)lr1 * 60.0f / (float)WHEEL_COUNTS_PER_REV;
    float rf_rpm = (float)rf1 * 60.0f / (float)WHEEL_COUNTS_PER_REV;
    float rr_rpm = (float)rr1 * 60.0f / (float)WHEEL_COUNTS_PER_REV;

    // Side RPM (average of two wheels)
    float left_rpm_meas = 0.5f * (lf_rpm + lr_rpm);
    float right_rpm_meas = 0.5f * (rf_rpm + rr_rpm);

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

    float left_meas_abs = fabsf(left_rpm_meas);
    float right_meas_abs = fabsf(right_rpm_meas);

    // PI update at fixed 10ms
    float left_err = left_ref_abs - left_meas_abs;
    float right_err = right_ref_abs - right_meas_abs;

    float left_u = pi_update(leftPI, left_err, CTRL_DT_S);
    float right_u = pi_update(rightPI, right_err, CTRL_DT_S);

    // Apply direction pins
    digitalWrite(left_dir_control_pin, left_fwd ? HIGH : LOW);
    digitalWrite(right_dir_control_pin, right_fwd ? LOW : HIGH);

    // Apply PWM
    analogWrite(lf_speed_control_pin, (int)left_u);
    analogWrite(lr_speed_control_pin, (int)left_u);
    analogWrite(rf_speed_control_pin, (int)right_u);
    analogWrite(rr_speed_control_pin, (int)right_u);
}
