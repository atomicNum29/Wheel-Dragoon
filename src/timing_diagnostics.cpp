#include <Arduino.h>
#include "timing_diagnostics.hpp"
#include "can_control.hpp"
#include "can_tx_schedule.hpp"
#include "control.hpp"
#include "serial_protocol.hpp"
#include "little_endian.hpp"

#if FIRMWARE_TIMING_DIAGNOSTICS
namespace timing {
namespace {

constexpr uint8_t SUMMARY_RECORDS = 1;
constexpr uint8_t FIRST_WHEEL_RECORD = SUMMARY_RECORDS + STAGE_COUNT;
constexpr uint8_t FIRST_DRIVER_RECORD = FIRST_WHEEL_RECORD + WHEEL_SLOT_COUNT;
constexpr uint8_t RECORD_COUNT = FIRST_DRIVER_RECORD + DRIVER_SLOT_COUNT;
constexpr uint8_t PACKET_SIZE = 63;

struct Metric {
    uint32_t count = 0;
    uint64_t total = 0;
    uint32_t maximum = 0;
    void add(uint32_t value) {
        ++count;
        total += value;
        if (value > maximum) maximum = value;
    }
};
struct Wheel {
    uint32_t received = 0, accepted = 0, outside_window = 0, short_frame = 0;
    uint32_t max_read_age_us = 0, missing = 0, last_rx_us = 0;
};
struct Driver {
    uint32_t timeouts = 0, pending_at_timeout = 0, last_missing_mask = 0;
    uint32_t last_age_us = 0, last_timeout_us = 0, last_pending = 0;
};
struct Snapshot {
    Metric stages[STAGE_COUNT]; // CPU cycles, including interrupt preemption.
    Wheel wheels[WHEEL_SLOT_COUNT]; // A196, A201, B196, B201; different from wheel wire order.
    Driver drivers[DRIVER_SLOT_COUNT];
    Metric rx_poll_gap;         // microseconds
    uint32_t overrun_observations = 0, control_loops = 0, bridge_loops = 0;
};
namespace layout {
constexpr uint8_t LENGTH = PACKET_SIZE - protocol::wire::FRAME_OVERHEAD;
constexpr uint8_t TYPE = 0x84, VERSION_VALUE = 1;
constexpr uint8_t VERSION = 4, KIND = 5, INDEX = 6, MODE = 7;
constexpr uint8_t SEQUENCE = 8, CAPTURED_MS = 12, CPU_HZ = 16;
constexpr uint8_t FIELDS = 20, TOTAL = 24, MAXIMUM = 32;
constexpr uint8_t OVERRUNS = 36, CONTROL_LOOPS = 40, BRIDGE_LOOPS = 44, RECORDS = 48;
constexpr uint8_t CHECKSUM = PACKET_SIZE - 1;
constexpr uint8_t U32_BYTES = sizeof(uint32_t), HIGH_WORD_SHIFT = 32;
enum Kind : uint8_t { SUMMARY = 0, STAGE = 1, WHEEL = 2, DRIVER = 3 };
}
void write_u64(uint8_t *out, uint64_t value)
{
    little_endian::write_u32(out, static_cast<uint32_t>(value));
    little_endian::write_u32(out + layout::U32_BYTES, static_cast<uint32_t>(value >> layout::HIGH_WORD_SHIFT));
}
template <size_t Count>
void write_fields(uint8_t *out, const uint32_t (&fields)[Count])
{
    for (size_t i = 0; i < Count; ++i)
        little_endian::write_u32(out + layout::FIELDS + layout::U32_BYTES * i, fields[i]);
}
void encode(const Snapshot &snapshot, uint8_t record, uint32_t sequence,
            uint32_t captured_ms, uint32_t cpu_hz, uint8_t mode, uint8_t *out)
{
    for (uint8_t i = 0; i < PACKET_SIZE; ++i) out[i] = 0;
    out[protocol::wire::HEADER_0] = protocol::wire::PKT_HEADER_0;
    out[protocol::wire::HEADER_1] = protocol::wire::PKT_HEADER_1;
    out[protocol::wire::LENGTH] = layout::LENGTH;
    out[protocol::wire::TYPE] = layout::TYPE;
    out[layout::VERSION] = layout::VERSION_VALUE;
    out[layout::MODE] = mode;
    little_endian::write_u32(out + layout::SEQUENCE, sequence);
    little_endian::write_u32(out + layout::CAPTURED_MS, captured_ms);
    little_endian::write_u32(out + layout::CPU_HZ, cpu_hz);
    if (record == 0) {
        out[layout::KIND] = layout::SUMMARY;
        little_endian::write_u32(out + layout::FIELDS, snapshot.rx_poll_gap.count);
        write_u64(out + layout::TOTAL, snapshot.rx_poll_gap.total);
        little_endian::write_u32(out + layout::MAXIMUM, snapshot.rx_poll_gap.maximum);
        little_endian::write_u32(out + layout::OVERRUNS, snapshot.overrun_observations);
        little_endian::write_u32(out + layout::CONTROL_LOOPS, snapshot.control_loops);
        little_endian::write_u32(out + layout::BRIDGE_LOOPS, snapshot.bridge_loops);
        out[layout::RECORDS] = RECORD_COUNT;
    } else if (record < FIRST_WHEEL_RECORD) {
        out[layout::KIND] = layout::STAGE;
        out[layout::INDEX] = record - SUMMARY_RECORDS;
        const Metric &metric = snapshot.stages[out[layout::INDEX]];
        little_endian::write_u32(out + layout::FIELDS, metric.count);
        write_u64(out + layout::TOTAL, metric.total);
        little_endian::write_u32(out + layout::MAXIMUM, metric.maximum);
    } else if (record < FIRST_DRIVER_RECORD) {
        out[layout::KIND] = layout::WHEEL;
        out[layout::INDEX] = record - FIRST_WHEEL_RECORD;
        const Wheel &wheel = snapshot.wheels[out[layout::INDEX]];
        const uint32_t fields[] = {wheel.received, wheel.accepted, wheel.outside_window,
            wheel.short_frame, wheel.max_read_age_us, wheel.missing, wheel.last_rx_us};
        write_fields(out, fields);
    } else {
        out[layout::KIND] = layout::DRIVER;
        out[layout::INDEX] = record - FIRST_DRIVER_RECORD;
        const Driver &driver = snapshot.drivers[out[layout::INDEX]];
        const uint32_t fields[] = {driver.timeouts, driver.pending_at_timeout, driver.last_missing_mask,
            driver.last_age_us, driver.last_timeout_us, driver.last_pending};
        write_fields(out, fields);
    }
    out[layout::CHECKSUM] = protocol::xor_checksum(out, layout::CHECKSUM);
}

constexpr uint32_t PACKET_INTERVAL_MS = 20, SNAPSHOT_INTERVAL_MS = 1000;
Snapshot timing_live, timing_snapshot;
uint32_t loop_start = 0, timing_previous_loop = 0, timing_previous_rx_us = 0;
bool timing_have_loop = false, timing_have_rx = false;
uint32_t timing_sequence = 0, timing_captured_ms = 0, timing_last_packet_ms = 0;
uint8_t timing_record = RECORD_COUNT, timing_snapshot_mode = 0;
} // namespace

void begin()
{
    ARM_DEMCR |= ARM_DEMCR_TRCENA;
    ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
}
void begin_loop()
{
    loop_start = ARM_DWT_CYCCNT;
    if (timing_have_loop) timing_live.stages[LOOP_PERIOD].add(loop_start - timing_previous_loop);
    timing_previous_loop = loop_start;
    timing_have_loop = true;
    if (control::status().serial_mode == control::SERIAL_MODE_CONTROL) ++timing_live.control_loops;
    else ++timing_live.bridge_loops;
}
void end_loop() { timing_live.stages[LOOP_BODY].add(ARM_DWT_CYCCNT - loop_start); }
void record_stage(Stage stage, uint32_t elapsed_cycles) { timing_live.stages[stage].add(elapsed_cycles); }
void observe_rx_poll()
{
    const uint32_t poll_us = micros();
    if (timing_have_rx) timing_live.rx_poll_gap.add(poll_us - timing_previous_rx_us);
    timing_previous_rx_us = poll_us;
    timing_have_rx = true;
}
void observe_wheel(WheelSlot slot, uint8_t dlc, uint32_t received_us, bool sent, uint32_t tx_us, bool accepted)
{
    Wheel &diagnostic = timing_live.wheels[slot];
    ++diagnostic.received;
    diagnostic.last_rx_us = received_us;
    if (sent) {
        const uint32_t age = received_us - tx_us;
        if (age > diagnostic.max_read_age_us) diagnostic.max_read_age_us = age;
    }
    if (dlc < CAN_PAYLOAD_SIZE) ++diagnostic.short_frame;
    else if (!accepted) ++diagnostic.outside_window;
    else ++diagnostic.accepted;
}
void observe_timeout(DriverSlot driver, uint8_t missing_mask, uint32_t tx_us)
{
    const bool pending = can_rx_pending();
    const uint32_t timeout_us = micros();
    Driver &diagnostic = timing_live.drivers[driver];
    ++diagnostic.timeouts;
    if (pending) ++diagnostic.pending_at_timeout;
    diagnostic.last_missing_mask = missing_mask;
    diagnostic.last_pending = pending;
    diagnostic.last_timeout_us = timeout_us;
    diagnostic.last_age_us = timeout_us - tx_us;
    const uint8_t masks[] = {CAN_TX_WHEEL_LF_MASK, CAN_TX_WHEEL_RR_MASK,
                             CAN_TX_WHEEL_RF_MASK, CAN_TX_WHEEL_LR_MASK};
    for (uint8_t i = 0; i < WHEEL_SLOT_COUNT; ++i)
        if (missing_mask & masks[i]) ++timing_live.wheels[i].missing;
}
void publish_if_due()
{
    // Diagnostics never switch modes, issue CAN requests, or wait for USB space.
    const uint32_t now_ms = millis();
    if (now_ms - timing_last_packet_ms < PACKET_INTERVAL_MS ||
        !can_control::diagnostic_output_ready(micros())) return;
    const bool new_snapshot = timing_record >= RECORD_COUNT;
    if (new_snapshot && now_ms - timing_captured_ms < SNAPSHOT_INTERVAL_MS)
        return;

    int writable = Serial.availableForWrite();
    if (writable > 0 && writable < PACKET_SIZE) {
        // Teensy reports only the remaining space in its current 64-byte USB
        // packet. A 63-byte 0x82 at cycle +6 ms leaves one byte until its 5-ms
        // auto-flush, permanently excluding diagnostics from +7..10 ms.
        // send_now() only queues that partial packet; it does not wait for the
        // host, insert bytes, or alter CAN scheduling. Still honor backpressure.
        Serial.send_now();
        writable = Serial.availableForWrite();
    }
    if (writable < PACKET_SIZE)
        return;
    if (new_snapshot) {
        timing_live.overrun_observations = can_rx_overrun_count();
        timing_snapshot = timing_live;
        timing_captured_ms = now_ms;
        timing_snapshot_mode = static_cast<uint8_t>(control::status().serial_mode);
        ++timing_sequence;
        timing_record = 0;
    }
    uint8_t packet[PACKET_SIZE];
    encode(timing_snapshot, timing_record, timing_sequence, timing_captured_ms,
                   F_CPU, timing_snapshot_mode, packet);
    if (Serial.write(packet, sizeof(packet)) == sizeof(packet)) {
        ++timing_record;
        timing_last_packet_ms = now_ms;
    }
}

} // namespace timing
#endif
