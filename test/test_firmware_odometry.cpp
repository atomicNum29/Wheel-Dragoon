#include <cassert>
#include <deque>
#include <limits>
#include <vector>

#include "../src/main.cpp"

uint64_t fake_clock_us = 0u;
uint32_t SCB_AIRCR = 0u;
FakeSerial Serial;

struct TimedFrame
{
    CanFrame frame;
    uint64_t time;
};
static std::deque<TimedFrame> rx_frames;
static std::vector<CanFrame> tx_frames;
static int fake_overrun_reports_remaining = 0;

bool can_begin(uint32_t) { return true; }
bool can_is_initialized() { return true; }
bool can_rx_overrun_detected() { return fake_overrun_reports_remaining > 0; }
void can_rx_overrun_report_sent()
{
    if (fake_overrun_reports_remaining > 0)
        --fake_overrun_reports_remaining;
}
bool can_transmit(const CanFrame &frame, uint32_t)
{
    tx_frames.push_back(frame);
    return true;
}
bool can_receive(CanFrame &frame, uint32_t)
{
    if (rx_frames.empty())
        return false;
    frame = rx_frames.front().frame;
    fake_clock_us = rx_frames.front().time;
    rx_frames.pop_front();
    return true;
}

static CanFrame monitor(uint8_t driver, uint8_t pid, int16_t rpm, int32_t count,
                        uint8_t status = 0u)
{
    CanFrame frame = {};
    frame.id = 0x700u | driver;
    frame.dlc = 8u;
    frame.data[0] = pid;
    frame.data[1] = status;
    little_endian::write_i16(frame.data + 2, rpm);
    little_endian::write_i32(frame.data + 4, count);
    return frame;
}

static void reset_test()
{
    fake_clock_us = 10000u;
    command_timeout = false;
    estop_active = false;
    motor_enabled = false;
    booting = false;
    calibration_active = false;
    control_mode_started_ms = 10u;
    command_timeout_reports_remaining = 0;
    estop_reports_remaining = 0;
    voltage_stale_reports_remaining = 0;
    voltage_request_can_tx_failure_reports_remaining = 0;
    watchdog_reset_reports_remaining = 0;
    fake_overrun_reports_remaining = 0;
    Serial = {};
    serial_mode = SERIAL_MODE_CONTROL;
    system_status_pending = false;
    system_status_tx_seq = 0u;
    driver_status_fault_reports_remaining = 0;
    driver_rx_timeout_reports_remaining = 0;
    safety_can_tx_failure_reports_remaining = 0;
    battery_low_reports_remaining = 0;
    checksum_error_reports_remaining = 0;
    serial_framing_error_reports_remaining = 0;
    command_out_of_range_reports_remaining = 0;
    reset_telemetry_validity();
    reset_can_tx_scheduler(10000u);
    can_tx_scheduler.driver_a_command_sent = true;
    can_tx_scheduler.driver_a_tx_us = 10000u;
    can_tx_scheduler.driver_b_command_sent = true;
    can_tx_scheduler.driver_b_tx_us = 13000u;
    can_tx_scheduler.expected_wheel_mask = CAN_TX_ALL_WHEELS_MASK;
    rx_frames.clear();
    tx_frames.clear();
}

static void receive_round()
{
    rx_frames.push_back({monitor(1, 196, 100, 123456), 10700u});
    rx_frames.push_back({monitor(1, 201, 80, 654321), 11020u});
    rx_frames.push_back({monitor(2, 196, -100, -123456), 13700u});
    rx_frames.push_back({monitor(2, 201, -80, -654321), 14100u});
    md200t_drain_rx(10u);
}

static void test_receive_and_timestamp()
{
    reset_test();
    receive_round();
    assert(lf_telemetry.actual_rpm == 100 && lf_telemetry.encoder_count == 123456);
    assert(rf_telemetry.actual_rpm == -100 && rf_telemetry.encoder_count == -123456);
    assert(lr_telemetry.actual_rpm == -80 && lr_telemetry.encoder_count == -654321);
    assert(rr_telemetry.actual_rpm == 80 && rr_telemetry.encoder_count == 654321);
    assert(lf_telemetry.rx_timestamp_us == 10700u);
    assert(rr_telemetry.rx_timestamp_us == 11020u);
    assert(rf_telemetry.rx_timestamp_us == 13700u);
    assert(lr_telemetry.rx_timestamp_us == 14100u);
    assert(telemetry_validity_bits() == 0x0Fu);
    assert(system_status_pending && can_tx_scheduler.driver_a_response_checked &&
           can_tx_scheduler.driver_b_response_checked);
}

static void test_invalid_duplicate_and_missing_response()
{
    reset_test();
    CanFrame frame = monitor(1, 196, -32768, std::numeric_limits<int32_t>::min(), 0x20u);
    CanFrame short_frame = frame;
    short_frame.dlc = 7u;
    md200t_process_rx_frame(short_frame, 10u, 10700u);
    md200t_process_rx_frame(monitor(1, 193, 7, 7), 10u, 10700u);
    md200t_process_rx_frame(monitor(3, 196, 7, 7), 10u, 10700u);
    md200t_process_rx_frame(frame, 10u, 13000u); // Deadline is exclusive.
    assert(!lf_telemetry.valid && !driver_status_fault_reports_remaining);
    md200t_process_rx_frame(frame, 10u, 10700u);
    assert(lf_telemetry.status == 0x20u && driver_status_fault_reports_remaining);
    assert(lf_telemetry.encoder_count == std::numeric_limits<int32_t>::min());
    md200t_process_rx_frame(monitor(1, 196, 7, 7), 10u, 10800u);
    assert(lf_telemetry.actual_rpm == 7 && lf_telemetry.rx_timestamp_us == 10800u);
    close_driver_response_window(1);
    assert(driver_rx_timeout_reports_remaining && lf_telemetry.valid && !rr_telemetry.valid);
    const MotorTelemetry saved = lf_telemetry;
    invalidate_wheel_telemetry(CAN_TX_WHEEL_LF_MASK);
    assert(!lf_telemetry.valid && lf_telemetry.encoder_count == saved.encoder_count &&
           lf_telemetry.rx_timestamp_us == saved.rx_timestamp_us);
}

static void test_clock_wrap_and_session_reset()
{
    reset_test();
    fake_clock_us = 0x100000100ULL;
    can_tx_scheduler.driver_a_tx_us = 0xFFFFFE00u;
    rx_frames.push_back({monitor(1, 196, 32767, std::numeric_limits<int32_t>::max()),
                         fake_clock_us});
    md200t_drain_rx(0u);
    assert(lf_telemetry.rx_timestamp_us == 0x100u);
    assert(lf_telemetry.encoder_count == std::numeric_limits<int32_t>::max());
    Serial.space = 64;
    assert(protocol_send_system_status());
    assert(little_endian::read_u32(Serial.output.data() + 14) == 0x100u);
    assert(little_endian::read_u32(Serial.output.data() + 25) == 0x100u);
    initialize_control_session(1u);
    assert(!lf_telemetry.valid && lf_telemetry.rx_timestamp_us == 0u);
    assert(micros() == 0x100u); // Session change doesn't alter the raw timer.
}

static void test_wire_snapshot_and_usb_stream()
{
    reset_test();
    receive_round();
    fake_clock_us = 14500u;
    checksum_error_reports_remaining = ERROR_REPORT_REPEAT_COUNT;
    serial_framing_error_reports_remaining = ERROR_REPORT_REPEAT_COUNT;
    command_out_of_range_reports_remaining = ERROR_REPORT_REPEAT_COUNT;
    assert(!protocol_send_system_status());
    assert(Serial.output.empty() && system_status_tx_seq == 0u);
    assert(checksum_error_reports_remaining && serial_framing_error_reports_remaining && command_out_of_range_reports_remaining);
    Serial.space = 62;
    assert(!protocol_send_system_status());
    assert(Serial.output.empty() && checksum_error_reports_remaining);
    Serial.space = 63;
    assert(protocol_send_system_status());
    assert(Serial.output.size() == 63u);
    assert(Serial.write_requests == std::vector<size_t>{63u});
    assert(system_status_tx_seq == 1u && checksum_error_reports_remaining == 9 &&
           serial_framing_error_reports_remaining == 9 && command_out_of_range_reports_remaining == 9);
    const uint8_t *p = Serial.output.data();
    assert(p[0] == 0xAA && p[1] == 0x55 && p[2] == 59u && p[3] == 0x82 && p[4] == 2u);
    assert(p[5] == 0u && p[9] == 0x0Fu);
    assert(little_endian::read_u32(p + 14) == 14500u);
    assert(little_endian::read_i16(p + 19) == 100);
    assert(little_endian::read_i32(p + 21) == 123456);
    assert(little_endian::read_u32(p + 25) == 10700u);
    assert(little_endian::read_i32(p + 32) == -123456);
    assert(little_endian::read_u32(p + 36) == 13700u);
    assert(little_endian::read_i32(p + 43) == -654321);
    assert(little_endian::read_u32(p + 47) == 14100u);
    assert(little_endian::read_i32(p + 54) == 654321);
    assert(little_endian::read_u32(p + 58) == 11020u);
    assert(p[62] == protocol_xor_checksum(p, 62u));
}

static void test_response_protection_unchanged()
{
    reset_test();
    reset_can_tx_scheduler(10000u);
    can_transmit_if_due(10000u);
    assert(tx_frames.size() == 1u && tx_frames[0].id == 1u);
    rx_frames.push_back({monitor(1, 196, 0, 0), 10700u});
    rx_frames.push_back({monitor(1, 201, 0, 0), 11020u});
    md200t_drain_rx(11u);
    can_transmit_if_due(12999u);
    assert(tx_frames.size() == 1u);
    fake_clock_us = 13000u;
    can_transmit_if_due(13000u);
    assert(tx_frames.size() == 2u && tx_frames[1].id == 2u);
    for (const CanFrame &frame : tx_frames)
        assert(frame.data[0] != 149u);
}

static void test_snapshot_timestamp_before_serial_write()
{
    reset_test();
    receive_round();
    fake_clock_us = 14500u;
    Serial.space = 64;
    Serial.write_time_us = 800u;
    send_can_bridge_response(7, 0, nullptr); // Existing direct bridge sender.
    assert(fake_clock_us == 15300u && Serial.output.size() == 18u);
    assert(!protocol_send_system_status()); // Only 46 bytes remain.
    assert(system_status_tx_seq == 0u && Serial.output.size() == 18u);
    fake_clock_us = 20000u;
    Serial.space = 64;
    assert(protocol_send_system_status());
    assert(fake_clock_us == 20800u);
    assert(Serial.output.size() == 18u + 63u);
    const uint8_t *packet = Serial.output.data() + 18u;
    assert(little_endian::read_u32(packet + 14) == 20000u);
    assert(little_endian::read_u32(packet + 25) == 10700u);
    assert(packet[62] == protocol_xor_checksum(packet, 62u));
    assert((Serial.write_requests == std::vector<size_t>{18u, 63u}));
}

static uint16_t send_status_error_bits()
{
    Serial.space = 64;
    Serial.output.clear();
    assert(protocol_send_system_status());
    assert(Serial.output.size() == 63u);
    return little_endian::read_u16(Serial.output.data() + 7);
}

static void test_ten_reports_for_all_errors()
{
    reset_test();
    int *const counters[] = {
        &checksum_error_reports_remaining, &command_timeout_reports_remaining,
        &safety_can_tx_failure_reports_remaining, &estop_reports_remaining,
        &battery_low_reports_remaining, &serial_framing_error_reports_remaining,
        &command_out_of_range_reports_remaining, &watchdog_reset_reports_remaining,
        &driver_status_fault_reports_remaining, &driver_rx_timeout_reports_remaining,
        &voltage_stale_reports_remaining, &fake_overrun_reports_remaining,
        &voltage_request_can_tx_failure_reports_remaining,
    };
    for (int *counter : counters)
        *counter = ERROR_REPORT_REPEAT_COUNT;
    // Reading status and building an unsent packet must not consume reports.
    motor_build_error_bits();
    capture_fault_snapshot(millis());
    assert(!protocol_send_system_status());
    for (int *counter : counters)
        assert(*counter == 10);
    // A write can fail even after the USB space check (e.g. disconnect).
    Serial.space = 63;
    Serial.reject_write = true;
    assert(!protocol_send_system_status());
    assert(Serial.output.empty() && system_status_tx_seq == 0u);
    for (int *counter : counters)
        assert(*counter == 10);
    Serial.reject_write = false;
    for (int packet = 0; packet < 10; ++packet)
    {
        assert(send_status_error_bits() == 0x0FFFu);
        assert(Serial.output[6] == MOTOR_STATE_FAULT);
        for (int *counter : counters)
            assert(*counter == 9 - packet);
    }
    assert(!motor_has_driver_fault());
    for (int packet = 0; packet < 3; ++packet)
    {
        assert(send_status_error_bits() == 0u);
        for (int *counter : counters)
            assert(*counter == 0); // No underflow or resurrection.
    }
}

static void test_retrigger_and_independent_countdowns()
{
    reset_test();
    uint8_t bad_command[CMD_PACKET_SIZE] = {0xAA, 0x55, 7, 1};
    parse_complete_command_frame(bad_command, millis()); // Bad XOR checksum.
    assert(checksum_error_reports_remaining == 10);
    for (int i = 0; i < 4; ++i)
        assert(send_status_error_bits() == SYSTEM_ERR_SERIAL_CHECKSUM_ERROR);
    assert(checksum_error_reports_remaining == 6);
    parse_complete_command_frame(bad_command, millis());
    assert(checksum_error_reports_remaining == 10);
    serial_framing_error_reports_remaining = 3;
    for (int i = 0; i < 10; ++i)
    {
        const uint16_t expected = SYSTEM_ERR_SERIAL_CHECKSUM_ERROR |
                                 (i < 3 ? SYSTEM_ERR_SERIAL_FRAMING_ERROR : 0u);
        assert(send_status_error_bits() == expected);
    }
    assert(send_status_error_bits() == 0u);
    latch_safety_can_tx_failure();
    initialize_control_session(millis());
    assert(safety_can_tx_failure_reports_remaining == 10);
    // A new control session must not clear an unreported diagnostic either.
    voltage_request_can_tx_failure_reports_remaining = 7;
    initialize_control_session(millis());
    assert(voltage_request_can_tx_failure_reports_remaining == 7);
}

static void test_live_inputs_reload_without_delaying_input_release()
{
    reset_test();
    command_timeout = true;
    estop_active = true;
    const uint16_t bits = SYSTEM_ERR_COMMAND_TIMEOUT | SYSTEM_ERR_EMERGENCY_STOP_ACTIVE;
    for (int i = 0; i < 15; ++i)
    {
        assert(send_status_error_bits() == bits);
        assert(command_timeout_reports_remaining == 9 && estop_reports_remaining == 9);
        assert(Serial.output[6] == MOTOR_STATE_ESTOP);
    }
    command_timeout = false;
    estop_active = false;
    for (int i = 0; i < 9; ++i)
    {
        assert(send_status_error_bits() == bits);
        assert(Serial.output[6] == MOTOR_STATE_DISABLED);
    }
    assert(send_status_error_bits() == 0u);
}

static void test_persistent_battery_and_stale_voltage_reload()
{
    reset_test();
    driver_a_voltage = DriverVoltageTelemetry{20000u, 10u, true};
    driver_b_voltage = DriverVoltageTelemetry{24000u, 10u, true};
    for (int i = 0; i < 12; ++i)
    {
        md200t_update_telemetry_health(10u);
        assert(send_status_error_bits() == SYSTEM_ERR_BATTERY_LOW);
        assert(battery_low_reports_remaining == 9);
    }
    driver_a_voltage.millivolts = 24000u;
    md200t_update_telemetry_health(10u);
    for (int i = 0; i < 9; ++i)
        assert(send_status_error_bits() == SYSTEM_ERR_BATTERY_LOW);
    assert(send_status_error_bits() == 0u);
    driver_a_voltage.valid = false;
    fake_clock_us = 4000000u;
    for (int i = 0; i < 12; ++i)
    {
        assert(send_status_error_bits() == SYSTEM_ERR_VOLTAGE_STALE);
        assert(voltage_stale_reports_remaining == 9);
    }
    driver_a_voltage.valid = true;
    for (int i = 0; i < 9; ++i)
        assert(send_status_error_bits() == SYSTEM_ERR_VOLTAGE_STALE);
    assert(send_status_error_bits() == 0u);
}

static void test_rc_powerdown_and_relays()
{
    reset_test();
    mode_state = DRIVE_MODE_STOP;
    setup();
    assert(mode_state == DRIVE_MODE_STOP);
    assert(fake_pin_modes()[5] == OUTPUT && fake_pin_modes()[6] == OUTPUT);
    assert(digitalRead(5) == HIGH && digitalRead(6) == HIGH);

    const unsigned int widths[] = {1000, 1099, 1100, 1200, 1299, 1300, 1500, 1699, 1700, 2000, 2050};
    const DriveMode expected[] = {DRIVE_MODE_POWERDOWN, DRIVE_MODE_POWERDOWN,
        DRIVE_MODE_STOP, DRIVE_MODE_STOP, DRIVE_MODE_STOP,
        DRIVE_MODE_MANUAL, DRIVE_MODE_MANUAL, DRIVE_MODE_MANUAL,
        DRIVE_MODE_AUTO, DRIVE_MODE_AUTO, DRIVE_MODE_AUTO};
    for (unsigned int i = 0; i < sizeof(widths) / sizeof(widths[0]); ++i)
    {
        for (int sample = 0; sample < 10; ++sample)
        {
            fake_clock_us += 20000u;
            digitalWrite(mode_control_pin, HIGH);
            mode_decodePWM();
            fake_clock_us += widths[i];
            digitalWrite(mode_control_pin, LOW);
            mode_decodePWM();
        }
        assert(mode_state == expected[i]);
        loop();
        const int level = expected[i] == DRIVE_MODE_POWERDOWN ? LOW : HIGH;
        assert(digitalRead(5) == level && digitalRead(6) == level);
    }

    auto_command = CommandPacket{1000, 0, CMD_FLAG_ENABLE};
    last_valid_command_ms = millis();
    motor_enabled = true;
    motor_set_left_right_rpm(100.0f, 100.0f);
    update_effective_motor_command(DRIVE_MODE_POWERDOWN, 2000, 2000, millis());
    assert(!motor_enabled);
    assert(target_wheel_rpm_cmd.lf_rpm == 0.0f && target_wheel_rpm_cmd.rf_rpm == 0.0f);
    assert(target_wheel_rpm_cmd.lr_rpm == 0.0f && target_wheel_rpm_cmd.rr_rpm == 0.0f);
    serial_mode = SERIAL_MODE_BRIDGE;
    mode_state = DRIVE_MODE_POWERDOWN;
    loop();
    assert(digitalRead(5) == LOW && digitalRead(6) == LOW);
    mode_state = DRIVE_MODE_STOP;
    loop();
    assert(digitalRead(5) == HIGH && digitalRead(6) == HIGH);
}

static void mode_edge(uint64_t time_us, int level)
{
    fake_clock_us = time_us;
    digitalWrite(mode_control_pin, level);
    mode_decodePWM();
}

static void test_mode_pwm_resynchronization()
{
    // Establish a known average before each fault, then verify recovery.
    for (int fault = 0; fault < 6; ++fault)
    {
        uint64_t t = fake_clock_us + 20000u;
        for (int i = 0; i < 10; ++i, t += 20000u)
        {
            mode_edge(t, HIGH);
            mode_edge(t + 1200u, LOW);
        }
        assert(mode_state == DRIVE_MODE_STOP);
        switch (fault)
        {
        case 0: // Rising interrupt missed: falling edge must not start a pulse.
            mode_edge(t + 1000u, LOW);
            break;
        case 1: // Falling interrupt missed: next rise replaces the old timestamp.
            mode_edge(t, HIGH);
            break;
        case 2: // Rising interrupt missed after a missed falling interrupt.
            mode_edge(t, HIGH);
            mode_edge(t + 21000u, LOW);
            break;
        case 3: // Out-of-range short pulse.
            mode_edge(t, HIGH);
            mode_edge(t + 100u, LOW);
            break;
        case 4: // Out-of-range long pulse.
            mode_edge(t, HIGH);
            mode_edge(t + 5000u, LOW);
            break;
        case 5: // Timestamp zero is valid; subtraction also handles wraparound.
            mode_edge(0u, HIGH);
            mode_edge(1200u, LOW);
            mode_edge(UINT32_MAX - 499u, HIGH);
            mode_edge(uint64_t{UINT32_MAX} + 701u, LOW);
            break;
        }
        assert(mode_state == DRIVE_MODE_STOP);
        t = fake_clock_us + 20000u;
        for (int i = 0; i < 10; ++i, t += 20000u)
        {
            mode_edge(t, HIGH);
            mode_edge(t + 1000u, LOW);
        }
        assert(mode_state == DRIVE_MODE_POWERDOWN);
    }
}

int main()
{
    test_rc_powerdown_and_relays();
    test_mode_pwm_resynchronization();
    test_receive_and_timestamp();
    test_invalid_duplicate_and_missing_response();
    test_clock_wrap_and_session_reset();
    test_wire_snapshot_and_usb_stream();
    test_response_protection_unchanged();
    test_snapshot_timestamp_before_serial_write();
    test_ten_reports_for_all_errors();
    test_retrigger_and_independent_countdowns();
    test_live_inputs_reload_without_delaying_input_release();
    test_persistent_battery_and_stale_voltage_reload();
}
