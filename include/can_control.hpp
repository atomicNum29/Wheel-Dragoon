#pragma once
#include <stdint.h>
#include "build_features.hpp"
#include "flexcan0.hpp"

struct MotorTelemetry
{
    int16_t actual_rpm;
    int32_t encoder_count;
    uint32_t rx_timestamp_us;
    uint8_t status;
    unsigned long last_update_ms;
    bool valid; // Cleared when the latest 3 ms command-response window misses it.
};

struct DriverVoltageTelemetry
{
    uint16_t millivolts;
    unsigned long last_update_ms;
    bool valid; // Cleared when PID 143 exceeds its freshness timeout.
};

namespace can_control {
constexpr unsigned long VOLTAGE_TIMEOUT_MS = 3000;
struct Telemetry {
    MotorTelemetry lf, rf, lr, rr;
    DriverVoltageTelemetry driver_a, driver_b;
};
const Telemetry &telemetry();
void begin();
void reset_session(uint32_t now_us);
// Attempt both drivers immediately, without opening periodic response windows.
void send_torque_off();
bool status_pending();
// Called only after a complete 0x82 write succeeds.
void status_sent();
// Read-only scheduling gate; diagnostics cannot modify response windows.
#if FIRMWARE_TIMING_DIAGNOSTICS
bool diagnostic_output_ready(uint32_t now_us);
#endif
void drain_rx(unsigned long now);
// Shared main-context drain for loop, response deadline and benchmark deadline.
void process_queued_rx(unsigned long now);
void update_health(unsigned long now);
void transmit_if_due(uint32_t now_us);
} // namespace can_control
