#include <Arduino.h>
#include "rc_io.hpp"

namespace rc_io {
namespace {
constexpr int VELOCITY_PIN = 1, STEERING_PIN = 0, MODE_PIN = 2;
constexpr int FILTER_SAMPLES = 10;
constexpr long MODE_INITIAL_US = 1200;
constexpr long MODE_VALID_MIN_US = 800, MODE_VALID_MAX_US = 2200;
constexpr long MODE_POWERDOWN_END_US = 1100, MODE_STOP_END_US = 1300;
constexpr long MODE_MANUAL_END_US = 1600, MODE_PIVOT_END_US = 1800;
volatile unsigned int v_pulseWidth = 0, w_pulseWidth = 0;
volatile DriveMode mode_state = DRIVE_MODE_STOP;

// Each channel keeps its own history. Initial samples and integer division match
// the existing receiver filter; changing a validity rule is a separate policy edit.
class PulseFilter {
public:
    explicit PulseFilter(long initial_us) : sum(initial_us * FILTER_SAMPLES) {
        for (long &sample : samples) sample = initial_us;
    }
    long push(long pulse_us) {
        sum += pulse_us - samples[index];
        samples[index] = pulse_us;
        index = (index + 1) % FILTER_SAMPLES;
        return sum / FILTER_SAMPLES;
    }
private:
    long samples[FILTER_SAMPLES];
    long sum;
    unsigned int index = 0;
};
PulseFilter velocity_filter(PWM_CENTER_US), steering_filter(PWM_CENTER_US);
PulseFilter mode_filter(MODE_INITIAL_US);

void v_decodePWM()
{
    static unsigned long prevTime = 0;

    if ((prevTime != 0) && !digitalRead(VELOCITY_PIN))
    {
        long tmp = micros() - prevTime;
        if (tmp > static_cast<long>(PWM_MAX_US))
            return;
        v_pulseWidth = velocity_filter.push(tmp);
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

    if ((prevTime != 0) && !digitalRead(STEERING_PIN))
    {
        long tmp = micros() - prevTime;
        if (tmp > static_cast<long>(PWM_MAX_US))
            return;
        w_pulseWidth = steering_filter.push(tmp);
        prevTime = 0;
    }
    else
    {
        prevTime = micros();
    }
}

void mode_decodePWM()
{
    static uint32_t rise_time_us = 0;

    // Each rising edge starts a fresh measurement.
    if (digitalRead(MODE_PIN))
    {
        rise_time_us = micros();
        return;
    }

    // Here, the falling edge completes a measurement. Ignore any pulse widths outside the expected range.
    const long pulse_width_us = (long)micros() - rise_time_us;
    if (pulse_width_us < MODE_VALID_MIN_US || pulse_width_us > MODE_VALID_MAX_US)
        return;

    const long mode_pulseWidth = mode_filter.push(pulse_width_us);
    if (mode_pulseWidth < MODE_POWERDOWN_END_US)
        mode_state = DRIVE_MODE_POWERDOWN;
    else if (mode_pulseWidth < MODE_STOP_END_US)
        mode_state = DRIVE_MODE_STOP;
    else if (mode_pulseWidth < MODE_MANUAL_END_US)
        mode_state = DRIVE_MODE_MANUAL;
    else if (mode_pulseWidth < MODE_PIVOT_END_US)
        mode_state = DRIVE_MODE_PIVOT;
    else
        mode_state = DRIVE_MODE_AUTO;
}

} // namespace

void initialize_inputs()
{
    pinMode(VELOCITY_PIN, INPUT);
    pinMode(STEERING_PIN, INPUT);
    pinMode(MODE_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(VELOCITY_PIN), v_decodePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(STEERING_PIN), w_decodePWM, CHANGE);
    attachInterrupt(digitalPinToInterrupt(MODE_PIN), mode_decodePWM, CHANGE);
}
DriveMode read_mode() { return mode_state; }
PwmInput read_pwm() { return {v_pulseWidth, w_pulseWidth}; }

void initialize_steering(const vehicle::MotionController &motion)
{
    if (!motion.calibrated()) return;
    analogWriteResolution(vehicle::servo_resolution_bits);
    for (const auto &calibration : motion.calibration) {
        analogWriteFrequency(calibration.pin, vehicle::servo_frequency_hz);
        analogWrite(calibration.pin, vehicle::pwm_duty(vehicle::pulse_for_angle(calibration, 0.0f)));
    }
}
int32_t offset_with_deadband(unsigned int pulse_width_us)
{
    if (pulse_width_us < PWM_MIN_US) pulse_width_us = PWM_MIN_US;
    if (pulse_width_us > PWM_MAX_US) pulse_width_us = PWM_MAX_US;
    int32_t offset_us = static_cast<int32_t>(pulse_width_us) - PWM_CENTER_US;
    if (offset_us >= -PWM_DEADBAND_US && offset_us <= PWM_DEADBAND_US)
        return 0;
    offset_us = (offset_us > 0) ? (offset_us - PWM_DEADBAND_US) : (offset_us + PWM_DEADBAND_US);
    return offset_us;
}

void write_steering(const vehicle::MotionController &motion)
{
    if (!motion.calibrated()) return;
    for (int i = 0; i < vehicle::STEERING_COUNT; ++i) {
        const auto &c = motion.calibration[i];
        const int pulse_us = vehicle::pulse_for_angle(c, motion.angle[i]);
        analogWrite(c.pin, vehicle::pwm_duty(pulse_us));
    }
}

} // namespace rc_io
