#include <Arduino.h>
#include <kinetis.h>
#include <stdint.h>
#include "control.hpp"
#include "rc_io.hpp"
#include "can_control.hpp"
#include "can_debug.hpp"
#include "serial_protocol.hpp"
#include "md200t_can.hpp"
#include "error_report.hpp"

namespace control {
namespace {
constexpr unsigned long COMMAND_TIMEOUT_MS = 500;
constexpr uint32_t DRIVER_STARTUP_DELAY_MS = 1000;
constexpr uint32_t SYSTEM_RESET_REQUEST = 0x05FA0004u; // AIRCR VECTKEY | SYSRESETREQ
constexpr float RPM_ROUNDING_OFFSET = 0.5f;
constexpr int DRIVER_A_RELAY_PIN = 5, DRIVER_B_RELAY_PIN = 6;
Status live;
uint32_t driver_power_on_ms = 0;
unsigned long last_valid_command_ms = 0;
CommandPacket auto_command = {};
int reports[static_cast<uint8_t>(Error::Count)] = {};
vehicle::MotionController motion(vehicle::MotionConfig{}, vehicle::left_servo, vehicle::right_servo);
} // namespace

static void motor_stop_all();
static void update_vehicle_output(vehicle::RcCommand command, unsigned long now);
const Status &status() { return live; }
const vehicle::MotionController &motion_output() { return motion; }
void report(Error error) { reports[static_cast<uint8_t>(error)] = ERROR_REPORT_REPEAT_COUNT; }
bool reported(Error error) { return reports[static_cast<uint8_t>(error)] > 0; }
void report_safety_tx_failure() { report(Error::SafetyCanTx); }

void initialize_power()
{
    pinMode(DRIVER_A_RELAY_PIN, OUTPUT);
    pinMode(DRIVER_B_RELAY_PIN, OUTPUT);
    digitalWrite(DRIVER_A_RELAY_PIN, HIGH);
    digitalWrite(DRIVER_B_RELAY_PIN, HIGH);
}
void finish_boot() { live.booting = false; }
void initialize_motion() { motion.update({0, 0}, millis(), false); }

void initialize_session(unsigned long now)
{
    for (int &remaining : reports) remaining = 0;
    can_clear_rx_overrun_reports();
    live.control_mode_started_ms = now;
    can_control::reset_session(micros());
}

void update_power(DriveMode drive_mode, unsigned long now)
{
    const bool powerdown = drive_mode == DRIVE_MODE_POWERDOWN;
    if (powerdown != live.driver_powerdown) {
        live.driver_powerdown = powerdown;
        live.driver_starting = !powerdown;
        driver_power_on_ms = static_cast<uint32_t>(now);
        can_debug::cancel();
        live.serial_mode = SERIAL_MODE_CONTROL;
        initialize_session(now);
    }
    const int relay_level = live.driver_powerdown ? LOW : HIGH;
    digitalWrite(DRIVER_A_RELAY_PIN, relay_level);
    digitalWrite(DRIVER_B_RELAY_PIN, relay_level);
    if (live.driver_starting && static_cast<uint32_t>(now - driver_power_on_ms) >= DRIVER_STARTUP_DELAY_MS) {
        live.driver_starting = false;
        initialize_session(now);
    }
}

void consume_error_reports()
{
    for (int &remaining : reports) if (remaining > 0) --remaining;
    can_rx_overrun_report_sent();
}

int16_t rpm_to_i16(float rpm)
{
    if (!isfinite(rpm)) return 0;
    const float limit = fminf(motion.config.rpm_limit, vehicle::mdh100_maximum_rpm);
    if (!isfinite(limit) || limit <= 0) return 0;
    rpm = vehicle::clamp(rpm, -limit, limit);
    if (rpm > static_cast<float>(INT16_MAX))
        return INT16_MAX;
    if (rpm < static_cast<float>(INT16_MIN))
        return INT16_MIN;

    return static_cast<int16_t>(rpm >= 0.0f ? rpm + RPM_ROUNDING_OFFSET : rpm - RPM_ROUNDING_OFFSET);
}

void accept_command(const CommandPacket &command, unsigned long now)
{
    // Preserve reception/status bookkeeping without granting host drive authority.
    auto_command = command;
    last_valid_command_ms = now;
}

void update_drive(DriveMode drive_mode,
                  unsigned int v_pwm, unsigned int w_pwm, unsigned long now)
{
    if (live.serial_mode != SERIAL_MODE_CONTROL)
    {
        live.command_timeout = false;
        live.estop_active = false;
        live.motor_enabled = false;
        motor_stop_all();
        return;
    }

    if (live.driver_starting || drive_mode == DRIVE_MODE_STOP || drive_mode == DRIVE_MODE_POWERDOWN)
    {
        live.command_timeout = false;
        live.estop_active = false;
        live.motor_enabled = false;
        motor_stop_all();
        return;
    }

    if (drive_mode == DRIVE_MODE_MANUAL || drive_mode == DRIVE_MODE_PIVOT)
    {
        live.command_timeout = false;
        live.estop_active = false;
        live.motor_enabled = true;

        if (drive_mode == DRIVE_MODE_PIVOT)
        {
            const float angular_velocity = static_cast<float>(rc_io::offset_with_deadband(w_pwm)) /
                rc_io::PWM_ACTIVE_SPAN_US * vehicle::rc_pivot_maximum_angular_velocity;
            update_vehicle_output({0.0f, 0.0f, true, angular_velocity}, now);
            return;
        }

        // Clamp receiver range, remove the inclusive 1450..1550 us deadband,
        // and map each remaining 450 us continuously to the full steering range.
        // Keep the previous throttle scale: endpoints request +/-1.8 m/s before RPM limiting.
        const float v_velocity = static_cast<float>(rc_io::offset_with_deadband(v_pwm)) / rc_io::THROTTLE_US_PER_MPS;
        const float steering_input = static_cast<float>(rc_io::offset_with_deadband(w_pwm)) / rc_io::PWM_ACTIVE_SPAN_US;
        const float inner_angle = steering_input * vehicle::rc_maximum_inner_angle;
        const float curvature = vehicle::curvature_from_inner_angle(inner_angle, motion.config.geometry);
        update_vehicle_output({v_velocity, curvature}, now);
        return;
    }

    // RC-only firmware: AUTO packets remain parseable, but never enable drive.
    live.command_timeout = (last_valid_command_ms == 0u) || (now - last_valid_command_ms > COMMAND_TIMEOUT_MS);
    live.estop_active = (auto_command.flags & protocol::wire::CMD_FLAG_ESTOP) != 0u;
    live.motor_enabled = false;
    motor_stop_all();
}

void reset_mcu(void)
{
    // Stop periodic traffic before attempting the final TQ-OFF pair.
    // TX waits are bounded; a failure does not defer the software reset.
    can_debug::cancel();
    live.serial_mode = SERIAL_MODE_RESETTING;
    live.motor_enabled = false;
    live.estop_active = false;
    live.command_timeout = false;
    motor_stop_all();

    if (!live.driver_powerdown && !live.driver_starting)
        can_control::send_torque_off();

    // Teensy 3.2 / Cortex-M4 system reset request. Disable interrupts after
    // both CAN transmissions so no ISR can run between the request and reset.
    __disable_irq();
    SCB_AIRCR = SYSTEM_RESET_REQUEST;
    while (true)
    {
    }
}

void enter_control_mode(unsigned long now)
{
    can_debug::cancel();
    if (live.serial_mode == SERIAL_MODE_CONTROL)
        return;

    live.serial_mode = SERIAL_MODE_CONTROL;
    initialize_session(now);
}

bool has_driver_fault(void)
{
    // Voltage-request failures are diagnostic-only. Safety errors prevent
    // output while their ten-report countdown remains nonzero.
    return reports[static_cast<uint8_t>(Error::SafetyCanTx)] > 0 ||
           reports[static_cast<uint8_t>(Error::DriverStatus)] > 0 ||
           reports[static_cast<uint8_t>(Error::DriverTimeout)] > 0;
}

static bool voltage_telemetry_is_stale(unsigned long now)
{
    return !live.driver_powerdown && !live.driver_starting && live.serial_mode == SERIAL_MODE_CONTROL &&
           now - live.control_mode_started_ms >= can_control::VOLTAGE_TIMEOUT_MS &&
           (!can_control::telemetry().driver_a.valid || !can_control::telemetry().driver_b.valid);
}

void refresh_errors(unsigned long now)
{
    if (live.command_timeout)
        report(Error::CommandTimeout);
    if (live.estop_active)
        report(Error::Estop);
    if (voltage_telemetry_is_stale(now))
        report(Error::VoltageStale);
}

FaultSnapshot fault_snapshot(unsigned long now)
{
    FaultSnapshot fault = {};
    fault.serial_checksum_error = reports[static_cast<uint8_t>(Error::Checksum)] > 0;
    fault.command_timeout = live.command_timeout || reports[static_cast<uint8_t>(Error::CommandTimeout)] > 0;
    fault.command_timeout_active = live.command_timeout;
    fault.driver_fault = has_driver_fault();
    fault.can_tx_failure = reports[static_cast<uint8_t>(Error::SafetyCanTx)] > 0 ||
                           reports[static_cast<uint8_t>(Error::VoltageCanTx)] > 0;
    fault.emergency_stop_active = live.estop_active || reports[static_cast<uint8_t>(Error::Estop)] > 0;
    fault.emergency_stop_input_active = live.estop_active;
    fault.battery_low = reports[static_cast<uint8_t>(Error::BatteryLow)] > 0;
    fault.serial_framing_error = reports[static_cast<uint8_t>(Error::Framing)] > 0;
    fault.command_out_of_range = reports[static_cast<uint8_t>(Error::CommandRange)] > 0;
    fault.driver_status_fault = reports[static_cast<uint8_t>(Error::DriverStatus)] > 0;
    fault.driver_rx_timeout = reports[static_cast<uint8_t>(Error::DriverTimeout)] > 0;
    fault.voltage_stale = reports[static_cast<uint8_t>(Error::VoltageStale)] > 0 || voltage_telemetry_is_stale(now);
    fault.can_rx_overrun = can_rx_overrun_detected();
    return fault;
}

MotorState motor_state(const FaultSnapshot &fault)
{
    if (live.driver_powerdown)
        return MOTOR_STATE_DRIVER_POWERDOWN;
    if (fault.driver_fault || fault.battery_low)
        return MOTOR_STATE_FAULT;
    if (fault.emergency_stop_input_active)
        return MOTOR_STATE_ESTOP;
    if (live.booting || live.driver_starting)
        return MOTOR_STATE_BOOTING;
    if (fault.command_timeout_active)
        return MOTOR_STATE_TIMEOUT_STOP;
    if (!live.motor_enabled)
        return MOTOR_STATE_DISABLED;
    return MOTOR_STATE_ENABLED;
}

static void motor_stop_all(void)
{
    motion.update({0,0}, millis(), false);
    live.target_wheel_rpm_cmd = {};
}

static void update_vehicle_output(vehicle::RcCommand command, unsigned long now)
{
    const bool allowed = !has_driver_fault() && !reports[static_cast<uint8_t>(Error::BatteryLow)];
    motion.update(command, static_cast<uint32_t>(now), allowed);
    live.motor_enabled = live.motor_enabled && motion.ready && allowed;
    live.target_wheel_rpm_cmd = {motion.rpm[vehicle::LF], motion.rpm[vehicle::LR], motion.rpm[vehicle::RF], motion.rpm[vehicle::RR]};
}

#if FIRMWARE_CAN_DEBUG
void enter_bridge_mode(void)
{
    if (live.serial_mode == SERIAL_MODE_BRIDGE)
        return;

    live.serial_mode = SERIAL_MODE_BRIDGE;
    live.motor_enabled = false;
    motor_stop_all();

    if (live.driver_powerdown || live.driver_starting)
        return;

    // Automatic traffic is disabled before this back-to-back safety pair.
    const bool driver_a_ok = md200t_torque_off(MD200T_DRIVER_A_ID);
    const bool driver_b_ok = md200t_torque_off(MD200T_DRIVER_B_ID);
    if (!driver_a_ok || !driver_b_ok)
        report_safety_tx_failure();
}

#endif
} // namespace control
