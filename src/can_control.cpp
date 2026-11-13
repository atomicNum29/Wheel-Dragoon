#include <Arduino.h>
#include "can_control.hpp"
#include "can_tx_schedule.hpp"
#include "control.hpp"
#include "can_debug.hpp"
#include "timing_diagnostics.hpp"
#include "little_endian.hpp"

namespace can_control {
namespace {
constexpr uint32_t MD200T_CAN_BITRATE = 500000;
constexpr uint16_t BATTERY_LOW_THRESHOLD_MV = 21000;
constexpr uint16_t MILLIVOLTS_PER_DECI_V = 100;
constexpr uint16_t MAX_VOLTAGE_DECI_V = UINT16_MAX / MILLIVOLTS_PER_DECI_V;
constexpr uint8_t CAN_RX_DRAIN_LIMIT = CAN_RX_QUEUE_CAPACITY;
#if FIRMWARE_TIMING_DIAGNOSTICS
constexpr uint32_t DIAGNOSTIC_TX_PHASE_US = 7000;
#endif
namespace monitor_wire {
constexpr uint8_t PID = 0, STATUS_OR_VOLTAGE = 1, RPM = 2, ENCODER = 4;
constexpr uint8_t VOLTAGE_FRAME_SIZE = 3;
}
struct Md200tDriverCommand
{
    float ch1_rpm;
    float ch2_rpm;
    bool enabled;
};
struct CanTxScheduler
{
    uint32_t cycle_start_us;
    uint32_t driver_a_tx_us;
    uint32_t driver_b_tx_us;
    uint8_t superframe_cycle;
    bool initialized;
    bool driver_a_command_sent;
    bool driver_a_response_checked;
    bool driver_b_command_sent;
    bool driver_b_response_checked;
    bool voltage_poll_handled;
    Md200tDriverCommand pending_driver_b_command;
    uint8_t expected_wheel_mask;
    uint8_t received_wheel_mask;
    bool round_status_tx_requested;
};

Telemetry received = {};
CanTxScheduler can_tx_scheduler = {};
bool system_status_tx_pending = false;
} // namespace

const Telemetry &telemetry() { return received; }
bool status_pending() { return system_status_tx_pending; }
void status_sent() { system_status_tx_pending = false; }
void begin()
{
    if (!can_begin(MD200T_CAN_BITRATE)) {
        control::report_safety_tx_failure();
        digitalWrite(LED_BUILTIN, HIGH);
    }
}
void drain_rx(unsigned long now)
{
    timing::observe_rx_poll();
    process_queued_rx(now);
}
void send_torque_off()
{
    // Immediate safety pair, outside the periodic response-window schedule.
    const bool a_ok = md200t_torque_off(MD200T_DRIVER_A_ID);
    const bool b_ok = md200t_torque_off(MD200T_DRIVER_B_ID);
    if (!a_ok || !b_ok) control::report_safety_tx_failure();
    digitalWrite(LED_BUILTIN, control::has_driver_fault() || control::reported(control::Error::BatteryLow));
}
static MotorTelemetry *md200t_find_motor_telemetry(uint8_t driver_id,
                                                   uint8_t pid,
                                                   uint8_t &wheel_mask)
{
    // Each controller drives a diagonal pair; keep this mapping explicit.
    if (driver_id == MD200T_DRIVER_A_ID)
    {
        if (pid == MD200T_PID_MONITOR)
        {
            wheel_mask = CAN_TX_WHEEL_LF_MASK;
            return &received.lf;
        }
        if (pid == MD200T_PID_MONITOR2)
        {
            wheel_mask = CAN_TX_WHEEL_RR_MASK;
            return &received.rr;
        }
    }
    if (driver_id == MD200T_DRIVER_B_ID)
    {
        if (pid == MD200T_PID_MONITOR)
        {
            wheel_mask = CAN_TX_WHEEL_RF_MASK;
            return &received.rf;
        }
        if (pid == MD200T_PID_MONITOR2)
        {
            wheel_mask = CAN_TX_WHEEL_LR_MASK;
            return &received.lr;
        }
    }
    return nullptr;
}

static bool md200t_response_window_open(uint8_t driver_id, uint32_t rx_timestamp_us)
{
    if (driver_id == MD200T_DRIVER_A_ID)
    {
        return can_tx_scheduler.driver_a_command_sent &&
               !can_tx_scheduler.driver_a_response_checked &&
               !can_tx_response_window_expired(can_tx_scheduler.driver_a_tx_us,
                                               rx_timestamp_us);
    }
    if (driver_id == MD200T_DRIVER_B_ID)
    {
        return can_tx_scheduler.driver_b_command_sent &&
               !can_tx_scheduler.driver_b_response_checked &&
               !can_tx_response_window_expired(can_tx_scheduler.driver_b_tx_us,
                                               rx_timestamp_us);
    }
    return false;
}

static void md200t_process_rx_frame(const CanFrame &frame,
                                    unsigned long now,
                                    uint32_t rx_timestamp_us)
{
    uint8_t driver_id = 0u;
    if (!md200t_decode_response_id(frame.id, driver_id) || frame.dlc == 0u)
        return;

    const uint8_t pid = frame.data[monitor_wire::PID];
    uint8_t wheel_mask = 0u;
    MotorTelemetry *motor = md200t_find_motor_telemetry(driver_id, pid, wheel_mask);
    if (motor != nullptr)
    {
#if FIRMWARE_TIMING_DIAGNOSTICS
        const timing::WheelSlot slot = driver_id == MD200T_DRIVER_A_ID
            ? (pid == MD200T_PID_MONITOR2 ? timing::A_MONITOR2 : timing::A_MONITOR)
            : (pid == MD200T_PID_MONITOR2 ? timing::B_MONITOR2 : timing::B_MONITOR);
        const bool sent = driver_id == MD200T_DRIVER_A_ID
            ? can_tx_scheduler.driver_a_command_sent : can_tx_scheduler.driver_b_command_sent;
        const uint32_t tx_us = driver_id == MD200T_DRIVER_A_ID
            ? can_tx_scheduler.driver_a_tx_us : can_tx_scheduler.driver_b_tx_us;
        timing::observe_wheel(slot, frame.dlc, rx_timestamp_us, sent, tx_us,
                              md200t_response_window_open(driver_id, rx_timestamp_us));
#endif
        // Only a complete response received inside the 3 ms window belonging
        // to this driver's latest command can refresh wheel telemetry.
        if (frame.dlc < CAN_PAYLOAD_SIZE || !md200t_response_window_open(driver_id, rx_timestamp_us))
            return;

        motor->status = frame.data[monitor_wire::STATUS_OR_VOLTAGE];
        motor->actual_rpm = little_endian::read_i16(&frame.data[monitor_wire::RPM]);
        motor->encoder_count = little_endian::read_i32(&frame.data[monitor_wire::ENCODER]);
        motor->rx_timestamp_us = rx_timestamp_us;
        motor->last_update_ms = now;
        motor->valid = true;
        if (motor->status != 0u)
            control::report(control::Error::DriverStatus);

        // With user-configured PID149=7, PID207/174 returns PID196/201. Count those
        // four command-triggered frames in the current 10 ms command round and
        // request combined USB status transmission as soon as all wheels are present.
        if (control::status().serial_mode == control::SERIAL_MODE_CONTROL &&
            can_tx_scheduler.initialized &&
            (can_tx_scheduler.expected_wheel_mask & wheel_mask) != 0u)
        {
            can_tx_scheduler.received_wheel_mask |= wheel_mask;
            if (can_tx_response_pair_complete(CAN_TX_DRIVER_A_WHEELS_MASK,
                                              can_tx_scheduler.received_wheel_mask))
                can_tx_scheduler.driver_a_response_checked = true;
            if (can_tx_response_pair_complete(CAN_TX_DRIVER_B_WHEELS_MASK,
                                              can_tx_scheduler.received_wheel_mask))
                can_tx_scheduler.driver_b_response_checked = true;
            if (!can_tx_scheduler.round_status_tx_requested &&
                can_tx_wheel_round_complete(can_tx_scheduler.expected_wheel_mask,
                                            can_tx_scheduler.received_wheel_mask))
            {
                can_tx_scheduler.round_status_tx_requested = true;
                system_status_tx_pending = true;
            }
        }
        return;
    }

    if (pid != MD200T_PID_VOLT_IN || frame.dlc < monitor_wire::VOLTAGE_FRAME_SIZE)
        return;

    DriverVoltageTelemetry *voltage = nullptr;
    if (driver_id == MD200T_DRIVER_A_ID)
        voltage = &received.driver_a;
    else if (driver_id == MD200T_DRIVER_B_ID)
        voltage = &received.driver_b;
    else
        return;

    const int16_t raw_deci_volts = little_endian::read_i16(&frame.data[monitor_wire::STATUS_OR_VOLTAGE]);
    // PID 143 is signed 0.1 V on the wire; only positive values that still
    // fit after conversion to uint16 millivolts are accepted.
    if (raw_deci_volts <= 0 || raw_deci_volts > MAX_VOLTAGE_DECI_V)
        return;

    voltage->millivolts = static_cast<uint16_t>(static_cast<uint16_t>(raw_deci_volts) * MILLIVOLTS_PER_DECI_V);
    voltage->last_update_ms = now;
    voltage->valid = true;
}

void process_queued_rx(unsigned long now)
{
    CanFrame frame = {};
    uint32_t rx_timestamp_us = 0;
    for (uint8_t count = 0u; count < CAN_RX_DRAIN_LIMIT; ++count)
    {
        if (!can_receive_timed(frame, rx_timestamp_us, 0u))
            break;

        if (control::status().driver_powerdown || control::status().driver_starting)
            continue;

#if FIRMWARE_CAN_DEBUG
        if (control::status().serial_mode == control::SERIAL_MODE_BRIDGE) {
            can_debug::consume_rx(frame, rx_timestamp_us);
            continue;
        }
#endif
        md200t_process_rx_frame(frame, now, rx_timestamp_us);
    }
}

static bool telemetry_is_fresh(bool valid, unsigned long last_update_ms, unsigned long now, unsigned long timeout_ms)
{
    return valid && now - last_update_ms <= timeout_ms;
}

void update_health(unsigned long now)
{
    if (control::status().driver_powerdown || control::status().driver_starting)
        return;

    received.driver_a.valid = telemetry_is_fresh(received.driver_a.valid, received.driver_a.last_update_ms, now, VOLTAGE_TIMEOUT_MS);
    received.driver_b.valid = telemetry_is_fresh(received.driver_b.valid, received.driver_b.last_update_ms, now, VOLTAGE_TIMEOUT_MS);

    if (received.driver_a.valid && received.driver_b.valid &&
        (received.driver_a.millivolts < BATTERY_LOW_THRESHOLD_MV || received.driver_b.millivolts < BATTERY_LOW_THRESHOLD_MV))
        control::report(control::Error::BatteryLow);
}

static bool md200t_send_driver_command(uint8_t driver_id,
                                       const Md200tDriverCommand &command)
{
    if (command.enabled)
        return md200t_set_velocity(driver_id,
                                   control::rpm_to_i16(command.ch1_rpm),
                                   control::rpm_to_i16(command.ch2_rpm));
    return md200t_torque_off(driver_id);
}

static void invalidate_wheel_telemetry(uint8_t wheel_mask)
{
    if ((wheel_mask & CAN_TX_WHEEL_LF_MASK) != 0u)
        received.lf.valid = false;
    if ((wheel_mask & CAN_TX_WHEEL_RF_MASK) != 0u)
        received.rf.valid = false;
    if ((wheel_mask & CAN_TX_WHEEL_LR_MASK) != 0u)
        received.lr.valid = false;
    if ((wheel_mask & CAN_TX_WHEEL_RR_MASK) != 0u)
        received.rr.valid = false;
}

static void close_driver_response_window(uint8_t driver_id)
{
    // RX can arrive during other work after the loop's earlier drain.
    // Process already queued responses using ISR timestamps before declaring loss.
    process_queued_rx(millis());
    const uint8_t expected_mask = driver_id == MD200T_DRIVER_A_ID
                                      ? CAN_TX_DRIVER_A_WHEELS_MASK
                                      : CAN_TX_DRIVER_B_WHEELS_MASK;
    const uint8_t missing_mask = static_cast<uint8_t>(
        expected_mask & static_cast<uint8_t>(~can_tx_scheduler.received_wheel_mask));
    if (missing_mask != 0u)
    {
#if FIRMWARE_TIMING_DIAGNOSTICS
        const timing::DriverSlot driver_index = driver_id == MD200T_DRIVER_A_ID ? timing::DRIVER_A : timing::DRIVER_B;
        const uint32_t tx_us = driver_index == timing::DRIVER_A ? can_tx_scheduler.driver_a_tx_us : can_tx_scheduler.driver_b_tx_us;
        timing::observe_timeout(driver_index, missing_mask, tx_us);
#endif
        invalidate_wheel_telemetry(missing_mask);
        control::report(control::Error::DriverTimeout);
    }

    if (driver_id == MD200T_DRIVER_A_ID)
        can_tx_scheduler.driver_a_response_checked = true;
    else
        can_tx_scheduler.driver_b_response_checked = true;

    digitalWrite(LED_BUILTIN, control::has_driver_fault() || control::reported(control::Error::BatteryLow));
}

static void reset_can_tx_scheduler(uint32_t now_us)
{
    can_tx_scheduler = {};
    can_tx_scheduler.cycle_start_us = now_us;
    can_tx_scheduler.superframe_cycle = 0u;
    can_tx_scheduler.initialized = true;
}

void transmit_if_due(uint32_t now_us)
{
    if (control::status().serial_mode != control::SERIAL_MODE_CONTROL || !can_tx_scheduler.initialized)
        return;

    if (control::status().driver_powerdown || control::status().driver_starting)
    {
        // Keep USB status at 100 Hz without opening CAN response windows.
        if (static_cast<uint32_t>(now_us - can_tx_scheduler.cycle_start_us) >= CAN_TX_COMMAND_PERIOD_US)
        {
            can_tx_scheduler.cycle_start_us = now_us;
            system_status_tx_pending = true;
        }
        return;
    }

    // Advance directly to the current cycle. A round that did not collect all
    // four responses still requests one fallback status so diagnostics remain
    // visible, but missed CAN events are never replayed across cycle boundaries.
    const uint32_t elapsed_us = static_cast<uint32_t>(now_us - can_tx_scheduler.cycle_start_us);
    if (elapsed_us >= CAN_TX_COMMAND_PERIOD_US)
    {
        // If a command was sent late, do not start the next driver's command
        // until its full 3 ms response window has elapsed.
        if (can_tx_scheduler.driver_a_command_sent &&
            !can_tx_scheduler.driver_a_response_checked)
        {
            if (!can_tx_response_window_expired(can_tx_scheduler.driver_a_tx_us, now_us))
                return;
            close_driver_response_window(MD200T_DRIVER_A_ID);
        }
        if (can_tx_scheduler.driver_b_command_sent &&
            !can_tx_scheduler.driver_b_response_checked)
        {
            if (!can_tx_response_window_expired(can_tx_scheduler.driver_b_tx_us, now_us))
                return;
            close_driver_response_window(MD200T_DRIVER_B_ID);
        }

        if (!can_tx_scheduler.round_status_tx_requested)
            system_status_tx_pending = true;

        const uint32_t skipped_cycles = elapsed_us / CAN_TX_COMMAND_PERIOD_US;
        can_tx_scheduler.cycle_start_us += skipped_cycles * CAN_TX_COMMAND_PERIOD_US;
        can_tx_scheduler.superframe_cycle = static_cast<uint8_t>(
            (can_tx_scheduler.superframe_cycle + skipped_cycles % CAN_TX_SUPERFRAME_CYCLES) %
            CAN_TX_SUPERFRAME_CYCLES);
        can_tx_scheduler.driver_a_command_sent = false;
        can_tx_scheduler.driver_a_response_checked = false;
        can_tx_scheduler.driver_b_command_sent = false;
        can_tx_scheduler.driver_b_response_checked = false;
        can_tx_scheduler.voltage_poll_handled = false;
        can_tx_scheduler.expected_wheel_mask = 0u;
        can_tx_scheduler.received_wheel_mask = 0u;
        can_tx_scheduler.round_status_tx_requested = false;
    }

    const uint32_t cycle_phase_us = static_cast<uint32_t>(now_us - can_tx_scheduler.cycle_start_us);
    if (!can_tx_scheduler.driver_a_command_sent)
    {
        // Both driver commands use this one coherent target snapshot even
        // though Driver B is intentionally transmitted 3 ms later.
        const bool command_enabled = control::status().motor_enabled && !control::status().estop_active && !control::status().command_timeout &&
                                     !control::has_driver_fault() && !control::reported(control::Error::BatteryLow);
        const WheelRpmCommand target = command_enabled
                                           ? control::status().target_wheel_rpm_cmd
                                           : WheelRpmCommand{0.0f, 0.0f, 0.0f, 0.0f};

        const Md200tDriverCommand driver_a = {
            target.lf_rpm, // MD200T A MOT1 = LF
            target.rr_rpm, // MD200T A MOT2 = RR
            command_enabled};

        can_tx_scheduler.pending_driver_b_command = Md200tDriverCommand{
            target.rf_rpm, // MD200T B MOT1 = RF
            target.lr_rpm, // MD200T B MOT2 = LR
            command_enabled};

        can_tx_scheduler.driver_a_command_sent = true;
        can_tx_scheduler.driver_a_tx_us = micros();
        can_tx_scheduler.expected_wheel_mask = CAN_TX_DRIVER_A_WHEELS_MASK;
        if (!md200t_send_driver_command(MD200T_DRIVER_A_ID, driver_a))
            control::report_safety_tx_failure();
        digitalWrite(LED_BUILTIN, control::has_driver_fault() || control::reported(control::Error::BatteryLow));
        return;
    }

    if (!can_tx_scheduler.driver_b_command_sent)
    {
        if (cycle_phase_us < CAN_TX_DRIVER_B_PHASE_US)
            return;
        // Preserve the full A-to-B idle interval even when both A responses
        // arrive early or Driver A itself was sent late in the cycle.
        if (!can_tx_response_window_expired(can_tx_scheduler.driver_a_tx_us, now_us))
            return;
        if (!can_tx_scheduler.driver_a_response_checked)
        {
            close_driver_response_window(MD200T_DRIVER_A_ID);
        }

        // A response timeout or any other safety fault detected during the
        // first window changes Driver B's scheduled frame to TQ-OFF.
        if (!control::status().motor_enabled || control::has_driver_fault() || control::reported(control::Error::BatteryLow) || control::status().estop_active || control::status().command_timeout)
            can_tx_scheduler.pending_driver_b_command.enabled = false;

        can_tx_scheduler.driver_b_command_sent = true;
        can_tx_scheduler.driver_b_tx_us = micros();
        can_tx_scheduler.expected_wheel_mask |= CAN_TX_DRIVER_B_WHEELS_MASK;
        if (!md200t_send_driver_command(MD200T_DRIVER_B_ID,
                                        can_tx_scheduler.pending_driver_b_command))
            control::report_safety_tx_failure();
        digitalWrite(LED_BUILTIN, control::has_driver_fault() || control::reported(control::Error::BatteryLow));
        return;
    }

    // Keep all later automatic TX outside Driver B's complete 3 ms window,
    // even when both responses arrive before the deadline.
    if (!can_tx_response_window_expired(can_tx_scheduler.driver_b_tx_us, now_us))
        return;
    if (!can_tx_scheduler.driver_b_response_checked)
        close_driver_response_window(MD200T_DRIVER_B_ID);

    // A complete pair requests 0x82 transmission from RX immediately. A partial/empty
    // pair requests one fallback with timeout bits here at the deadline.
    if (!can_tx_scheduler.round_status_tx_requested)
    {
        can_tx_scheduler.round_status_tx_requested = true;
        system_status_tx_pending = true;
    }

    if (!can_tx_scheduler.voltage_poll_handled)
    {
        ScheduledVoltageRequest voltage_request = {};
        if (!scheduled_voltage_poll(can_tx_scheduler.superframe_cycle, voltage_request))
        {
            can_tx_scheduler.voltage_poll_handled = true;
        }
        else
        {
            if (cycle_phase_us < voltage_request.phase_us)
                return;

            can_tx_scheduler.voltage_poll_handled = true;
            if (!md200t_request_pid_data(voltage_request.driver_id, voltage_request.pid))
                control::report(control::Error::VoltageCanTx);
            return;
        }
    }
}

void reset_session(uint32_t now_us)
{
    received = {};
    system_status_tx_pending = false;
    reset_can_tx_scheduler(now_us);
}
#if FIRMWARE_TIMING_DIAGNOSTICS
bool diagnostic_output_ready(uint32_t now_us)
{
    if (control::status().serial_mode != control::SERIAL_MODE_CONTROL || !can_tx_scheduler.initialized)
        return false;
    if (control::status().driver_powerdown || control::status().driver_starting) return true;
    const uint32_t phase = now_us - can_tx_scheduler.cycle_start_us;
    return phase >= DIAGNOSTIC_TX_PHASE_US && phase < CAN_TX_COMMAND_PERIOD_US &&
        can_tx_scheduler.driver_a_response_checked && can_tx_scheduler.driver_b_response_checked &&
        can_tx_scheduler.voltage_poll_handled;
}
#endif
} // namespace can_control
