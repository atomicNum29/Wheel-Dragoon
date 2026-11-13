#pragma once
#include "build_features.hpp"
#include "flexcan0.hpp"

namespace can_debug {
#if FIRMWARE_CAN_DEBUG
void handle_bridge_request(const uint8_t *packet);
void handle_benchmark_request(const uint8_t *packet);
void update(uint32_t now_us);
void flush_result(void);
void cancel(void);
void consume_rx(const CanFrame &frame, uint32_t received_us);
#else
// Common control transitions can cancel diagnostics without a link dependency.
inline void cancel() {}
#endif
} // namespace can_debug
