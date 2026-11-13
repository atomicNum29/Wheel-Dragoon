#include <Arduino.h>
#include "control.hpp"
#include "rc_io.hpp"
#include "can_control.hpp"
#include "serial_protocol.hpp"
#include "can_debug.hpp"
#include "timing_diagnostics.hpp"

namespace {
constexpr uint32_t STARTUP_IO_DELAY_MS = 100;
}

void setup()
{
    // STOP on boot: power contacts precede RC interrupt attachment.
    control::initialize_power();
    protocol::begin();
    timing::begin();
    rc_io::initialize_steering(control::motion_output());
    control::initialize_motion();
    rc_io::initialize_inputs();
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    delay(STARTUP_IO_DELAY_MS);
    control::initialize_session(millis());
    can_control::begin();
    control::finish_boot();
}

void loop()
{
    timing::begin_loop();
    const unsigned long now = millis();
    const DriveMode drive_mode = rc_io::read_mode();
    timing::measure(timing::POWER, [&]() { control::update_power(drive_mode, now); });
    const rc_io::PwmInput pwm = rc_io::read_pwm();

    timing::measure(timing::SERIAL_RX, [&]() { protocol::receive_commands(now); });
#if FIRMWARE_CAN_DEBUG
    timing::measure(timing::BENCH_TX_PRE, [&]() { can_debug::flush_result(); });
#endif
    timing::measure(timing::CAN_RX, [&]() { can_control::drain_rx(now); });
#if FIRMWARE_CAN_DEBUG
    timing::measure(timing::BENCH_UPDATE, [&]() { can_debug::update(micros()); });
    timing::measure(timing::BENCH_TX_POST, [&]() { can_debug::flush_result(); });
#endif
    timing::measure(timing::HEALTH, [&]() { can_control::update_health(now); });
    timing::measure(timing::MOTION, [&]() {
        control::update_drive(drive_mode, pwm.velocity_us, pwm.steering_us, now);
    });
    timing::measure(timing::ERRORS, [&]() { control::refresh_errors(now); });
    timing::measure(timing::CAN_SCHEDULE, [&]() { can_control::transmit_if_due(micros()); });
    timing::measure(timing::SYSTEM_TX, [&]() { protocol::send_status_if_ready(); });
    rc_io::write_steering(control::motion_output());
#if FIRMWARE_TIMING_DIAGNOSTICS
    timing::measure(timing::DIAGNOSTIC_TX, [&]() { timing::publish_if_due(); });
#endif
    timing::end_loop();
}
