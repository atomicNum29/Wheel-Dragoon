#include <Arduino.h>

// car width and wheel radius
const double W = 0.42;      // m
const double R = 0.139 / 2; // m

// remote control signal pins
const int w_speed_controller_pin = 0;
const int v_speed_controller_pin = 1;
const int mode_control_pin = 2;

volatile unsigned int v_pulseWidth = 0;
volatile unsigned int w_pulseWidth = 0;
volatile unsigned int mode_state = 0; // 0: stop, 1: manual, 2: auto(UART)

void v_decodePWM();
void w_decodePWM();
void mode_decodePWM();

// motor speed control pins
const int lf_speed_control_pin = 3;
const int lr_speed_control_pin = 4;
const int rr_speed_control_pin = 5;
const int rf_speed_control_pin = 6;

volatile unsigned int led_state = 0;

// motor control feedback pins, 60 CPR encoders
#define WHEEL_COUNTS_PER_REV 60
#define WHEEL_DT_MS 10
// Use a shorter sliding window for faster feedback (reduces ~1s oscillation)
#define WHEEL_WINDOW_MS 200
#define WHEEL_BINS (WHEEL_WINDOW_MS / WHEEL_DT_MS)

// Low-speed estimator (pulse-to-pulse timing) settings
#define DT_AVG_N 4
#define DT_TIMEOUT_US 500000UL // if no pulse for 0.5s => treat as 0 speed
#define HYBRID_MIN_PULSES 5    // if >= this many pulses in window, prefer window estimator

const int lf_wheel_pulse_pin = 9;
const int lr_wheel_pulse_pin = 10;
const int rr_wheel_pulse_pin = 11;
const int rf_wheel_pulse_pin = 12;

// Per-10ms bin counters (incremented by encoder ISR)
volatile uint16_t lf_bin_count = 0;
volatile uint16_t lr_bin_count = 0;
volatile uint16_t rr_bin_count = 0;
volatile uint16_t rf_bin_count = 0;

// Ring buffers of per-bin counts and sliding sums over WHEEL_WINDOW_MS
volatile uint16_t lf_bins[WHEEL_BINS] = {0};
volatile uint16_t lr_bins[WHEEL_BINS] = {0};
volatile uint16_t rr_bins[WHEEL_BINS] = {0};
volatile uint16_t rf_bins[WHEEL_BINS] = {0};
volatile uint8_t wheel_bin_idx = 0;

volatile uint32_t lf_sum_win = 0;
volatile uint32_t lr_sum_win = 0;
volatile uint32_t rr_sum_win = 0;
volatile uint32_t rf_sum_win = 0;

// Pulse-to-pulse timing buffers for low-speed estimation
volatile uint32_t lf_last_pulse_us = 0;
volatile uint32_t lr_last_pulse_us = 0;
volatile uint32_t rr_last_pulse_us = 0;
volatile uint32_t rf_last_pulse_us = 0;

volatile uint32_t lf_dt_buf[DT_AVG_N] = {0};
volatile uint32_t lr_dt_buf[DT_AVG_N] = {0};
volatile uint32_t rr_dt_buf[DT_AVG_N] = {0};
volatile uint32_t rf_dt_buf[DT_AVG_N] = {0};

volatile uint32_t lf_dt_sum = 0;
volatile uint32_t lr_dt_sum = 0;
volatile uint32_t rr_dt_sum = 0;
volatile uint32_t rf_dt_sum = 0;

volatile uint8_t lf_dt_idx = 0;
volatile uint8_t lr_dt_idx = 0;
volatile uint8_t rr_dt_idx = 0;
volatile uint8_t rf_dt_idx = 0;

volatile uint8_t lf_dt_valid = 0;
volatile uint8_t lr_dt_valid = 0;
volatile uint8_t rr_dt_valid = 0;
volatile uint8_t rf_dt_valid = 0;

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
PIController lfPI = {.kp = 8.0f, .ki = 0.0f, .integral = 0.0f, .out_min = 0.0f, .out_max = 1023.0f, .i_min = -300.0f, .i_max = 300.0f};
PIController lrPI = {.kp = 8.0f, .ki = 0.0f, .integral = 0.0f, .out_min = 0.0f, .out_max = 1023.0f, .i_min = -300.0f, .i_max = 300.0f};
PIController rrPI = {.kp = 8.0f, .ki = 0.0f, .integral = 0.0f, .out_min = 0.0f, .out_max = 1023.0f, .i_min = -300.0f, .i_max = 300.0f};
PIController rfPI = {.kp = 8.0f, .ki = 0.0f, .integral = 0.0f, .out_min = 0.0f, .out_max = 1023.0f, .i_min = -300.0f, .i_max = 300.0f};

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
    if (mode_state == 0)
    {
        // Stop mode
        noInterrupts();
        left_rpm_ref_cmd = 0.0f;
        right_rpm_ref_cmd = 0.0f;
        interrupts();
    }
    else if (mode_state == 1)
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
        // Serial.print(">lf_RPM:");
        // Serial.println((float)lf_sum_win * 60.0f / (float)WHEEL_COUNTS_PER_REV);
        // Serial.print(">lr_RPM:");
        // Serial.println((float)lr_sum_win * 60.0f / (float)WHEEL_COUNTS_PER_REV);
        // Serial.print(">rf_RPM:");
        // Serial.println((float)rf_sum_win * 60.0f / (float)WHEEL_COUNTS_PER_REV);
        // Serial.print(">rr_RPM:");
        // Serial.println((float)rr_sum_win * 60.0f / (float)WHEEL_COUNTS_PER_REV);
        // // Serial.print(">lf_sum_win:");
        // // Serial.println(lf_sum_win);
        // // Serial.print(">lr_sum_win:");
        // // Serial.println(lr_sum_win);
        // // Serial.print(">rr_sum_win:");
        // // Serial.println(rr_sum_win);
        // // Serial.print(">rf_sum_win:");
        // // Serial.println(rf_sum_win);
        // delay(10);
    }
    else
    {
        static unsigned int message_complete = 0;
        static unsigned char rx_buffer[32];
        static unsigned int rx_index = 0;
        static float v_cmd = 0.0f;
        static float w_cmd = 0.0f;

        // Auto mode: (not implemented) receive v/w from UART
        if (Serial.available() > 0)
        {
            // Packet: 0xAA 0x55 | float v (4) | float w (4) | 0x55 0xAA
            unsigned char byte_in = Serial.read();
            rx_buffer[rx_index++] = byte_in;
            if (rx_index >= 2)
            {
                // Check header
                if ((rx_buffer[0] != 0xAA) || (rx_buffer[1] != 0x55))
                {
                    // Shift buffer left by one
                    for (unsigned int i = 1; i < rx_index; i++)
                    {
                        rx_buffer[i - 1] = rx_buffer[i];
                    }
                    rx_index--;
                }
            }
            if (rx_index >= 12)
            {
                // Check footer
                if ((rx_buffer[10] == 0x55) && (rx_buffer[11] == 0xAA))
                {
                    // Extract v/w floats
                    memcpy(&v_cmd, &rx_buffer[2], sizeof(float));
                    memcpy(&w_cmd, &rx_buffer[6], sizeof(float));
                    message_complete = 1;
                    Serial.print(">ACK");
                }
                // Reset for next message
                rx_index = 0;
            }
        }
        if (message_complete == 1)
        {
            double left_velocity = v_cmd - w_cmd * W / 2;
            double right_velocity = v_cmd + w_cmd * W / 2;

            // --- Target RPM from v/w command ---
            float left_rpm_ref = vel_mps_to_rpm((float)left_velocity);
            float right_rpm_ref = vel_mps_to_rpm((float)right_velocity);

            noInterrupts();
            left_rpm_ref_cmd = left_rpm_ref;
            right_rpm_ref_cmd = right_rpm_ref;
            interrupts();

            message_complete = 0;
            rx_index = 0;
        }
    }
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
            mode_state = 0; // stop
        else if (sum / 10 < 1700)
            mode_state = 1; // manual
        else
            mode_state = 2; // auto(UART)
        prevTime = 0;
    }
    else
    {
        prevTime = micros();
    }
}

void lf_wheel_pulse_ISR()
{
    uint32_t now = micros();
    if (lf_last_pulse_us != 0)
    {
        uint32_t dt = now - lf_last_pulse_us;
        // rolling average over DT_AVG_N samples
        lf_dt_sum -= lf_dt_buf[lf_dt_idx];
        lf_dt_buf[lf_dt_idx] = dt;
        lf_dt_sum += dt;
        lf_dt_idx = (uint8_t)((lf_dt_idx + 1) % DT_AVG_N);
        if (lf_dt_valid < DT_AVG_N)
            lf_dt_valid++;
    }
    lf_last_pulse_us = now;
    lf_bin_count++;
}

void lr_wheel_pulse_ISR()
{
    uint32_t now = micros();
    if (lr_last_pulse_us != 0)
    {
        uint32_t dt = now - lr_last_pulse_us;
        lr_dt_sum -= lr_dt_buf[lr_dt_idx];
        lr_dt_buf[lr_dt_idx] = dt;
        lr_dt_sum += dt;
        lr_dt_idx = (uint8_t)((lr_dt_idx + 1) % DT_AVG_N);
        if (lr_dt_valid < DT_AVG_N)
            lr_dt_valid++;
    }
    lr_last_pulse_us = now;
    lr_bin_count++;
}

void rr_wheel_pulse_ISR()
{
    uint32_t now = micros();
    if (rr_last_pulse_us != 0)
    {
        uint32_t dt = now - rr_last_pulse_us;
        rr_dt_sum -= rr_dt_buf[rr_dt_idx];
        rr_dt_buf[rr_dt_idx] = dt;
        rr_dt_sum += dt;
        rr_dt_idx = (uint8_t)((rr_dt_idx + 1) % DT_AVG_N);
        if (rr_dt_valid < DT_AVG_N)
            rr_dt_valid++;
    }
    rr_last_pulse_us = now;
    rr_bin_count++;
}

void rf_wheel_pulse_ISR()
{
    uint32_t now = micros();
    if (rf_last_pulse_us != 0)
    {
        uint32_t dt = now - rf_last_pulse_us;
        rf_dt_sum -= rf_dt_buf[rf_dt_idx];
        rf_dt_buf[rf_dt_idx] = dt;
        rf_dt_sum += dt;
        rf_dt_idx = (uint8_t)((rf_dt_idx + 1) % DT_AVG_N);
        if (rf_dt_valid < DT_AVG_N)
            rf_dt_valid++;
    }
    rf_last_pulse_us = now;
    rf_bin_count++;
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
    lf_sum_win = lf_sum_win - lf_bins[wheel_bin_idx] + lf_c;
    lf_bins[wheel_bin_idx] = lf_c;

    lr_sum_win = lr_sum_win - lr_bins[wheel_bin_idx] + lr_c;
    lr_bins[wheel_bin_idx] = lr_c;

    rf_sum_win = rf_sum_win - rf_bins[wheel_bin_idx] + rf_c;
    rf_bins[wheel_bin_idx] = rf_c;

    rr_sum_win = rr_sum_win - rr_bins[wheel_bin_idx] + rr_c;
    rr_bins[wheel_bin_idx] = rr_c;
}

void control_tick()
{
    // 1) Update 1-second sliding window sums (pure counter math)
    wheel_bins_update();

    // 2) Snapshot measurements (window counts + pulse timing) and references atomically
    uint32_t lf_cnt, lr_cnt, rf_cnt, rr_cnt;
    float left_rpm_ref, right_rpm_ref;

    uint32_t lf_dt_sum_l, lr_dt_sum_l, rf_dt_sum_l, rr_dt_sum_l;
    uint8_t lf_dt_valid_l, lr_dt_valid_l, rf_dt_valid_l, rr_dt_valid_l;
    uint32_t lf_last_us_l, lr_last_us_l, rf_last_us_l, rr_last_us_l;

    noInterrupts();
    lf_cnt = lf_sum_win;
    lr_cnt = lr_sum_win;
    rf_cnt = rf_sum_win;
    rr_cnt = rr_sum_win;
    left_rpm_ref = left_rpm_ref_cmd;
    right_rpm_ref = right_rpm_ref_cmd;

    lf_dt_sum_l = lf_dt_sum;
    lr_dt_sum_l = lr_dt_sum;
    rf_dt_sum_l = rf_dt_sum;
    rr_dt_sum_l = rr_dt_sum;

    lf_dt_valid_l = lf_dt_valid;
    lr_dt_valid_l = lr_dt_valid;
    rf_dt_valid_l = rf_dt_valid;
    rr_dt_valid_l = rr_dt_valid;

    lf_last_us_l = lf_last_pulse_us;
    lr_last_us_l = lr_last_pulse_us;
    rf_last_us_l = rf_last_pulse_us;
    rr_last_us_l = rr_last_pulse_us;
    interrupts();

    const float window_s = (float)WHEEL_WINDOW_MS / 1000.0f;

    // Window-based RPM estimate: (counts in window / window_s) / CPR * 60
    float lf_rpm_win = ((float)lf_cnt / window_s) * 60.0f / (float)WHEEL_COUNTS_PER_REV;
    float lr_rpm_win = ((float)lr_cnt / window_s) * 60.0f / (float)WHEEL_COUNTS_PER_REV;
    float rf_rpm_win = ((float)rf_cnt / window_s) * 60.0f / (float)WHEEL_COUNTS_PER_REV;
    float rr_rpm_win = ((float)rr_cnt / window_s) * 60.0f / (float)WHEEL_COUNTS_PER_REV;

    // Pulse-to-pulse (dt) based RPM estimate for low speed
    uint32_t now_us = micros();

    auto rpm_from_dt = [&](uint32_t dt_sum_us, uint8_t valid, uint32_t last_us) -> float
    {
        if (valid == 0)
            return 0.0f;
        if (last_us == 0)
            return 0.0f;
        if ((now_us - last_us) > DT_TIMEOUT_US)
            return 0.0f;
        float avg_dt = (float)dt_sum_us / (float)valid; // us per pulse
        if (avg_dt <= 1.0f)
            return 0.0f;
        float pulses_per_s = 1000000.0f / avg_dt;
        return pulses_per_s * 60.0f / (float)WHEEL_COUNTS_PER_REV;
    };

    float lf_rpm_dt = rpm_from_dt(lf_dt_sum_l, lf_dt_valid_l, lf_last_us_l);
    float lr_rpm_dt = rpm_from_dt(lr_dt_sum_l, lr_dt_valid_l, lr_last_us_l);
    float rf_rpm_dt = rpm_from_dt(rf_dt_sum_l, rf_dt_valid_l, rf_last_us_l);
    float rr_rpm_dt = rpm_from_dt(rr_dt_sum_l, rr_dt_valid_l, rr_last_us_l);

    // Hybrid: if enough pulses in the window, prefer window estimate; otherwise use dt estimate
    float lf_rpm_meas = (lf_cnt >= HYBRID_MIN_PULSES) ? lf_rpm_win : lf_rpm_dt;
    float lr_rpm_meas = (lr_cnt >= HYBRID_MIN_PULSES) ? lr_rpm_win : lr_rpm_dt;
    float rf_rpm_meas = (rf_cnt >= HYBRID_MIN_PULSES) ? rf_rpm_win : rf_rpm_dt;
    float rr_rpm_meas = (rr_cnt >= HYBRID_MIN_PULSES) ? rr_rpm_win : rr_rpm_dt;

    // Debug (optional)
    Serial.print(">lf_RPM:");
    Serial.println(lf_rpm_meas);
    Serial.print(">lr_RPM:");
    Serial.println(lr_rpm_meas);
    Serial.print(">rf_RPM:");
    Serial.println(rf_rpm_meas);
    Serial.print(">rr_RPM:");
    Serial.println(rr_rpm_meas);
    Serial.print(">L_rpm_ref:");
    Serial.println(left_rpm_ref);
    Serial.print(">R_rpm_ref:");
    Serial.println(right_rpm_ref);

    // Direction from target sign; (still per-side direction pins)
    bool left_fwd = (left_rpm_ref >= 0.0f);
    bool right_fwd = (right_rpm_ref >= 0.0f);

    // Per-wheel RPM references (currently same within a side)
    float lf_ref = left_rpm_ref;
    float lr_ref = left_rpm_ref;
    float rf_ref = right_rpm_ref;
    float rr_ref = right_rpm_ref;

    float lf_ref_abs = fabsf(lf_ref);
    float lr_ref_abs = fabsf(lr_ref);
    float rf_ref_abs = fabsf(rf_ref);
    float rr_ref_abs = fabsf(rr_ref);

    // Low-speed deadband to reduce hunting
    if (lf_ref_abs < 1.0f)
        lf_ref_abs = 0.0f;
    if (lr_ref_abs < 1.0f)
        lr_ref_abs = 0.0f;
    if (rf_ref_abs < 1.0f)
        rf_ref_abs = 0.0f;
    if (rr_ref_abs < 1.0f)
        rr_ref_abs = 0.0f;

    float lf_meas_abs = fabsf(lf_rpm_meas);
    float lr_meas_abs = fabsf(lr_rpm_meas);
    float rf_meas_abs = fabsf(rf_rpm_meas);
    float rr_meas_abs = fabsf(rr_rpm_meas);

    // PI update at fixed 10ms (per-wheel, fully separated)
    float lf_err = lf_ref_abs - lf_meas_abs;
    float lr_err = lr_ref_abs - lr_meas_abs;
    float rf_err = rf_ref_abs - rf_meas_abs;
    float rr_err = rr_ref_abs - rr_meas_abs;

    float lf_u = pi_update(lfPI, lf_err, CTRL_DT_S);
    float lr_u = pi_update(lrPI, lr_err, CTRL_DT_S);
    float rf_u = pi_update(rfPI, rf_err, CTRL_DT_S);
    float rr_u = pi_update(rrPI, rr_err, CTRL_DT_S);

    // Apply direction pins
    digitalWrite(left_dir_control_pin, left_fwd ? HIGH : LOW);
    digitalWrite(right_dir_control_pin, right_fwd ? LOW : HIGH);

    // Apply PWM
    analogWrite(lf_speed_control_pin, (int)lf_u);
    analogWrite(lr_speed_control_pin, (int)lr_u);
    analogWrite(rf_speed_control_pin, (int)rf_u);
    analogWrite(rr_speed_control_pin, (int)rr_u);
}
