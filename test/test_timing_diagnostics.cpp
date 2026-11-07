// Only this harness enables profiling; no real CAN or CPU debug registers.
#define FIRMWARE_TIMING_DIAGNOSTICS 1
#include <cassert>
#include <fstream>
#include <cstdio>
#include "../src/main.cpp"

uint64_t fake_clock_us = 0;
uint32_t SCB_AIRCR = 0, ARM_DEMCR = 0, ARM_DWT_CTRL = 0, ARM_DWT_CYCCNT = 0;
FakeSerial Serial;
static bool fake_pending = false;
static uint32_t receive_calls = 0, pending_reads = 0;
bool can_begin(uint32_t) { return true; }
bool can_transmit(const CanFrame &, uint32_t) { return true; }
bool can_receive(CanFrame &, uint32_t) { ++receive_calls; return false; }
bool can_receive_timed(CanFrame &frame, uint32_t &, uint32_t timeout_us) {
    return can_receive(frame, timeout_us);
}
bool can_rx_overrun_detected() { return false; }
void can_rx_overrun_report_sent() {}
void can_clear_rx_overrun_reports() {}
bool can_is_initialized() { return true; }
bool can_rx_pending() { ++pending_reads; return fake_pending; }
uint32_t can_rx_overrun_count() { return 7; }

static bool status_flush_starvation_regression()
{
    // Model the installed Teensy core: 64-byte USB packet, delayed 5 ms flush.
    // Once a wheel response times out, the normal 63-byte status is sent at +6 ms.
    // The old diagnostic gate (+7..10 ms and >=63 bytes free) then never opens.
    Serial = FakeSerial{};
    Serial.space = 64;
    Serial.auto_flush_us = 5000;
    serial_mode = SERIAL_MODE_CONTROL;
    driver_powerdown = driver_starting = false;
    timing_sequence = 4;
    timing_captured_ms = 2000;
    timing_last_packet_ms = 0;
    timing_record = timing::RECORD_COUNT;
    reset_can_tx_scheduler(2000000);
    uint8_t status[63] = {0xAA, 0x55, 59, 0x82};
    status[62] = protocol_xor_checksum(status, 62);
    bool status_pending = false;
    for (uint32_t elapsed = 0; elapsed < 6000000; elapsed += 100) {
        fake_clock_us = 2000000 + elapsed;
        const uint32_t phase = elapsed % 10000;
        can_tx_scheduler.cycle_start_us = static_cast<uint32_t>(fake_clock_us) - phase;
        can_tx_scheduler.driver_a_response_checked = phase >= 6000;
        can_tx_scheduler.driver_b_response_checked = phase >= 6000;
        can_tx_scheduler.voltage_poll_handled = phase >= 6000;
        if (phase == 6000) status_pending = true;
        if (status_pending && Serial.availableForWrite() >= 63) {
            Serial.write(status, sizeof(status));
            status_pending = false;
        }
        timing_publish_if_due();
    }
    if (timing_sequence < 8) return false;
    uint32_t records[16] = {};
    for (size_t offset = 0; offset < Serial.output.size(); offset += 63) {
        const uint8_t *packet = Serial.output.data() + offset;
        if (protocol_xor_checksum(packet, 62) != packet[62]) return false;
        if (packet[3] != 0x84) continue;
        const uint32_t sequence = little_endian::read_u32(packet + 8);
        if (sequence >= 16) return false;
        ++records[sequence];
    }
    for (uint32_t sequence = 5; sequence <= 8; ++sequence)
        if (records[sequence] != timing::RECORD_COUNT) return false;

    // Enqueuing the partial packet may leave no spare USB packet. Do not wait
    // or advance a diagnostic record; retry only when capacity returns.
    fake_clock_us = 8007000;
    can_tx_scheduler.cycle_start_us = 8000000;
    const uint32_t sequence_before = timing_sequence;
    const uint8_t record_before = timing_record;
    const size_t bytes_before = Serial.output.size();
    Serial.space = 1;
    Serial.flush_deadline_us = 0;
    Serial.space_after_send_now = 0;
    timing_publish_if_due();
    if (Serial.output.size() != bytes_before || timing_sequence != sequence_before ||
        timing_record != record_before) return false;
    Serial.space = 64;
    timing_publish_if_due();
    if (Serial.output.size() != bytes_before + 63 || timing_record != 1 ||
        timing_sequence != sequence_before + 1) return false;
    return true;
}

int main(int argc, char **argv)
{
    // Unsigned cycle deltas survive a counter rollover and totals exceed 32 bits.
    ARM_DWT_CYCCNT = 0xFFFFFFF0u;
    profile_step(timing::MOTION, []() { ARM_DWT_CYCCNT += 144; });
    assert(timing_live.stages[timing::MOTION].total == 144);
    assert(timing_live.stages[timing::MOTION].maximum == 144);
    timing::Metric large;
    large.add(0xFFFFFFFFu); large.add(10);
    assert(large.total == 0x100000009ULL);

    fake_clock_us = 0xFFFFFFF0u;
    md200t_drain_rx(millis());
    fake_clock_us += 100;
    md200t_drain_rx(millis());
    assert(timing_live.rx_poll_gap.maximum == 100);

    reset_can_tx_scheduler(1000);
    can_tx_scheduler.driver_a_command_sent = true;
    can_tx_scheduler.driver_a_tx_us = 1000;
    can_tx_scheduler.expected_wheel_mask = CAN_TX_DRIVER_A_WHEELS_MASK;
    CanFrame frame = {};
    frame.id = 0x701; frame.dlc = 8; frame.data[0] = 196;
    md200t_process_rx_frame(frame, 1, 1700);
    assert(timing_live.wheels[0].accepted == 1);
    assert(can_tx_scheduler.received_wheel_mask == CAN_TX_WHEEL_LF_MASK);

    // Deadline handling drains RAM once, then the diagnostic probe only observes.
    fake_clock_us = 4010;
    fake_pending = true;
    const uint32_t calls_before = receive_calls;
    close_driver_response_window(MD200T_DRIVER_A_ID);
    assert(receive_calls == calls_before + 1 && pending_reads == 1 && fake_pending);
    assert(timing_live.drivers[0].timeouts == 1);
    assert(timing_live.drivers[0].pending_at_timeout == 1);
    assert(timing_live.drivers[0].last_missing_mask == CAN_TX_WHEEL_RR_MASK);
    assert(timing_live.drivers[0].last_age_us == 3010);
    assert(timing_live.wheels[1].missing == 1);
    frame.data[0] = 201;
    md200t_process_rx_frame(frame, 4, 4020);
    assert(timing_live.wheels[1].received == 1);
    assert(timing_live.wheels[1].outside_window == 1);
    assert(timing_live.wheels[1].accepted == 0 && !rr_telemetry.valid);
    frame.dlc = 2;
    md200t_process_rx_frame(frame, 4, 4030);
    assert(timing_live.wheels[1].short_frame == 1);
    assert(timing_live.wheels[1].outside_window == 1); // Mutually exclusive rejection reasons.

    // No output in a live response window or while USB has no capacity.
    fake_clock_us = 1007000;
    reset_can_tx_scheduler(1000000);
    Serial.space = 64;
    timing_publish_if_due();
    assert(Serial.output.empty());
    can_tx_scheduler.driver_a_response_checked = true;
    can_tx_scheduler.driver_b_response_checked = true;
    can_tx_scheduler.voltage_poll_handled = true;
    Serial.space = 0;
    timing_publish_if_due();
    assert(Serial.output.empty() && timing_sequence == 0);
    Serial.space = 64;
    timing_publish_if_due();
    assert(Serial.output.size() == 63 && timing_sequence == 1 && timing_record == 1);
    assert(Serial.output[3] == 0x84 && Serial.output[5] == 0);
    assert(protocol_xor_checksum(Serial.output.data(), 62) == Serial.output[62]);
    assert(timing_snapshot.overrun_observations == 7);
    // Frozen snapshot remains stable while live counters grow.
    timing_live.stages[timing::MOTION].add(720);
    assert(timing_snapshot.stages[timing::MOTION].total == 144);
    Serial.space = 64;
    timing_publish_if_due();
    assert(timing_record == 1); // 20 ms spacing.
    fake_clock_us += 20000;
    can_tx_scheduler.cycle_start_us += 20000;
    timing_publish_if_due();
    assert(timing_record == 2 && Serial.output.size() == 126);
    serial_mode = SERIAL_MODE_BRIDGE;
    fake_clock_us += 20000;
    Serial.space = 64;
    timing_publish_if_due();
    assert(timing_record == 2); // Never add output to the existing benchmark path.

    // Optional fixture consumed by the real Python parser: catches cross-language
    // layout, stage ordering, 64-bit values, and checksum mismatches.
    if (argc == 2) {
        std::ofstream output(argv[1], std::ios::binary);
        assert(output.good());
        uint8_t packet[timing::PACKET_SIZE];
        for (uint8_t record = 0; record < timing::RECORD_COUNT; ++record) {
            timing::encode(timing_snapshot, record, 1, 1007, F_CPU, 0, packet);
            output.write(reinterpret_cast<const char *>(packet), sizeof(packet));
        }
        assert(output.good());
    }
    if (!status_flush_starvation_regression()) {
        std::fprintf(stderr, "FAIL: 63-byte status / 5-ms USB flush starves timing snapshots\n");
        return 2;
    }
    std::puts("PASS: timing snapshots continue beyond 4 with +6 ms status traffic");
}
