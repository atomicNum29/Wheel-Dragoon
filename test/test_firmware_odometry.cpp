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

bool can_begin(uint32_t) { return true; }
bool can_is_initialized() { return true; }
bool can_rx_overrun_detected() { return false; }
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
    Serial = {};
    serial_mode = SERIAL_MODE_CONTROL;
    system_status_pending = false;
    system_status_tx_seq = 0u;
    driver_status_fault_latched = false;
    driver_rx_timeout_latched = false;
    safety_can_tx_failure_latched = false;
    battery_low_latched = false;
    checksum_error_latched = false;
    serial_framing_error_latched = false;
    command_out_of_range_latched = false;
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
    assert(!lf_telemetry.valid && !driver_status_fault_latched);
    md200t_process_rx_frame(frame, 10u, 10700u);
    assert(lf_telemetry.status == 0x20u && driver_status_fault_latched);
    assert(lf_telemetry.encoder_count == std::numeric_limits<int32_t>::min());
    md200t_process_rx_frame(monitor(1, 196, 7, 7), 10u, 10800u);
    assert(lf_telemetry.actual_rpm == 7 && lf_telemetry.rx_timestamp_us == 10800u);
    close_driver_response_window(1);
    assert(driver_rx_timeout_latched && lf_telemetry.valid && !rr_telemetry.valid);
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
    checksum_error_latched = true;
    serial_framing_error_latched = true;
    command_out_of_range_latched = true;
    assert(!protocol_send_system_status());
    assert(Serial.output.empty() && system_status_tx_seq == 0u);
    assert(checksum_error_latched && serial_framing_error_latched && command_out_of_range_latched);
    Serial.space = 62;
    assert(!protocol_send_system_status());
    assert(Serial.output.empty() && checksum_error_latched);
    Serial.space = 63;
    assert(protocol_send_system_status());
    assert(Serial.output.size() == 63u);
    assert(Serial.write_requests == std::vector<size_t>{63u});
    assert(system_status_tx_seq == 1u && !checksum_error_latched &&
           !serial_framing_error_latched && !command_out_of_range_latched);
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

int main()
{
    test_receive_and_timestamp();
    test_invalid_duplicate_and_missing_response();
    test_clock_wrap_and_session_reset();
    test_wire_snapshot_and_usb_stream();
    test_response_protection_unchanged();
    test_snapshot_timestamp_before_serial_write();
}
