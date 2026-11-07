#pragma once

#include <stdint.h>
#include "little_endian.hpp"

#ifndef FIRMWARE_TIMING_DIAGNOSTICS
#define FIRMWARE_TIMING_DIAGNOSTICS 0
#endif

namespace timing {
enum Stage : uint8_t {
    POWER, SERIAL_RX, BENCH_TX_PRE, CAN_RX, BENCH_UPDATE, BENCH_TX_POST,
    HEALTH, MOTION, ERRORS, CAN_SCHEDULE, SYSTEM_TX, STEERING, DIAGNOSTIC_TX,
    LOOP_BODY, LOOP_PERIOD,
    // STEERING (ID 11) is reserved; common firmware leaves its metric at zero.
    STAGE_COUNT
};

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
    Metric stages[STAGE_COUNT]; // CPU cycles; inclusive of interrupt preemption.
    Wheel wheels[4];           // A196, A201, B196, B201.
    Driver drivers[2];
    Metric rx_poll_gap;        // microseconds, entry-to-entry of RX drain.
    uint32_t overrun_observations = 0;
    uint32_t control_loops = 0, bridge_loops = 0;
};

constexpr uint8_t PACKET_SIZE = 63;
constexpr uint8_t RECORD_COUNT = 1 + STAGE_COUNT + 4 + 2;

inline void write_u64(uint8_t *out, uint64_t value) {
    little_endian::write_u32(out, static_cast<uint32_t>(value));
    little_endian::write_u32(out + 4, static_cast<uint32_t>(value >> 32));
}

// One frozen snapshot is sent in small, numbered records. No text on the MCU.
inline void encode(const Snapshot &s, uint8_t record, uint32_t sequence,
                   uint32_t captured_ms, uint32_t cpu_hz, uint8_t mode,
                   uint8_t *out) {
    for (uint8_t i = 0; i < PACKET_SIZE; ++i) out[i] = 0;
    out[0] = 0xAA; out[1] = 0x55; out[2] = 59; out[3] = 0x84;
    out[4] = 1; out[7] = mode;
    little_endian::write_u32(out + 8, sequence);
    little_endian::write_u32(out + 12, captured_ms);
    little_endian::write_u32(out + 16, cpu_hz);
    if (record == 0) {
        out[5] = 0; // Summary.
        little_endian::write_u32(out + 20, s.rx_poll_gap.count);
        write_u64(out + 24, s.rx_poll_gap.total);
        little_endian::write_u32(out + 32, s.rx_poll_gap.maximum);
        little_endian::write_u32(out + 36, s.overrun_observations);
        little_endian::write_u32(out + 40, s.control_loops);
        little_endian::write_u32(out + 44, s.bridge_loops);
        out[48] = RECORD_COUNT;
    } else if (record <= STAGE_COUNT) {
        out[5] = 1; out[6] = record - 1;
        const Metric &m = s.stages[out[6]];
        little_endian::write_u32(out + 20, m.count);
        write_u64(out + 24, m.total);
        little_endian::write_u32(out + 32, m.maximum);
    } else if (record <= STAGE_COUNT + 4) {
        out[5] = 2; out[6] = record - STAGE_COUNT - 1;
        const Wheel &w = s.wheels[out[6]];
        const uint32_t fields[] = {w.received, w.accepted, w.outside_window,
            w.short_frame, w.max_read_age_us, w.missing, w.last_rx_us};
        for (uint8_t i = 0; i < 7; ++i) little_endian::write_u32(out + 20 + 4*i, fields[i]);
    } else {
        out[5] = 3; out[6] = record - STAGE_COUNT - 5;
        const Driver &d = s.drivers[out[6]];
        const uint32_t fields[] = {d.timeouts, d.pending_at_timeout, d.last_missing_mask,
            d.last_age_us, d.last_timeout_us, d.last_pending};
        for (uint8_t i = 0; i < 6; ++i) little_endian::write_u32(out + 20 + 4*i, fields[i]);
    }
    for (uint8_t i = 0; i < PACKET_SIZE - 1; ++i) out[PACKET_SIZE - 1] ^= out[i];
}
} // namespace timing
