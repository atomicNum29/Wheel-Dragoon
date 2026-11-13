#pragma once

#include <stdint.h>
#include "build_features.hpp"
#if FIRMWARE_TIMING_DIAGNOSTICS
#include <kinetis.h>
#endif

namespace timing {
enum Stage : uint8_t {
    POWER, SERIAL_RX, BENCH_TX_PRE, CAN_RX, BENCH_UPDATE, BENCH_TX_POST,
    HEALTH, MOTION, ERRORS, CAN_SCHEDULE, SYSTEM_TX, STEERING, DIAGNOSTIC_TX,
    LOOP_BODY, LOOP_PERIOD,
    // STEERING (ID 11) is reserved; common firmware leaves its metric at zero.
    STAGE_COUNT
};

#if FIRMWARE_TIMING_DIAGNOSTICS
enum WheelSlot : uint8_t {
    A_MONITOR = 0, A_MONITOR2 = 1, B_MONITOR = 2, B_MONITOR2 = 3, WHEEL_SLOT_COUNT = 4
};
enum DriverSlot : uint8_t { DRIVER_A = 0, DRIVER_B = 1, DRIVER_SLOT_COUNT = 2 };
void begin();
void begin_loop();
void end_loop();
void record_stage(Stage stage, uint32_t elapsed_cycles);
void observe_rx_poll();
void observe_wheel(WheelSlot slot, uint8_t dlc, uint32_t received_us, bool sent, uint32_t tx_us, bool accepted);
void observe_timeout(DriverSlot driver, uint8_t missing_mask, uint32_t tx_us);
void publish_if_due();
#else
inline void begin() {}
inline void begin_loop() {}
inline void end_loop() {}
inline void observe_rx_poll() {}
inline void publish_if_due() {}
#endif

template <typename Action>
inline void measure(Stage stage, Action action)
{
#if FIRMWARE_TIMING_DIAGNOSTICS
    const uint32_t start = ARM_DWT_CYCCNT;
    action();
    record_stage(stage, ARM_DWT_CYCCNT - start);
#else
    (void)stage;
    action();
#endif
}

} // namespace timing
