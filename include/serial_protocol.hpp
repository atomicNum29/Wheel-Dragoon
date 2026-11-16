#pragma once
#include <stddef.h>
#include <stdint.h>

namespace protocol {
namespace wire {
constexpr uint8_t PKT_HEADER_0 = 0xAA;
constexpr uint8_t PKT_HEADER_1 = 0x55;
constexpr uint8_t LEGACY_CMD_PACKET_LENGTH = 7, LEGACY_CMD_PACKET_TYPE = 0x01;
constexpr uint8_t CMD_PACKET_LENGTH = 11;
constexpr uint8_t CMD_PACKET_TYPE = 0x03;
constexpr uint8_t CMD_PACKET_SIZE = 15;
constexpr uint8_t CMD_PACKET_VERSION = 1;
constexpr uint8_t RESET_PACKET_LENGTH = 4;
constexpr uint8_t RESET_PACKET_TYPE = 0x02;
constexpr uint8_t RESET_PACKET_SIZE = 8;
constexpr uint8_t RESET_MAGIC_0 = 0xA5;
constexpr uint8_t RESET_MAGIC_1 = 0x5A;
constexpr uint8_t CAN_BRIDGE_PACKET_LENGTH = 15;
constexpr uint8_t CAN_BRIDGE_PACKET_TYPE = 0x20;
constexpr uint8_t CAN_BRIDGE_PACKET_SIZE = 19;
constexpr uint8_t CAN_BENCHMARK_PACKET_LENGTH = 24;
constexpr uint8_t CAN_BENCHMARK_PACKET_TYPE = 0x21;
constexpr uint8_t CAN_BENCHMARK_PACKET_SIZE = 28;
constexpr uint8_t SERIAL_PACKET_MAX_SIZE = CAN_BENCHMARK_PACKET_SIZE;
constexpr uint8_t TELEMETRY_VERSION = 1;
constexpr uint8_t WHEEL_PACKET_TYPE = 0x85, WHEEL_PACKET_SIZE = 20;
constexpr uint8_t VOLTAGE_PACKET_TYPE = 0x86, VOLTAGE_PACKET_SIZE = 15;
constexpr uint8_t STATE_PACKET_TYPE = 0x87, STATE_PACKET_SIZE = 29;
constexpr int16_t CMD_LINEAR_MILLI_MPS_MIN = -2000;
constexpr int16_t CMD_LINEAR_MILLI_MPS_MAX = 2000;
constexpr int16_t CMD_ANGULAR_MILLI_RADPS_MIN = -5000;
constexpr int16_t CMD_ANGULAR_MILLI_RADPS_MAX = 5000;
constexpr uint8_t CMD_FLAG_ENABLE = 0x01, CMD_FLAG_ESTOP = 0x02, CMD_FLAG_PIVOT = 0x04;
constexpr uint8_t CMD_FLAGS_MASK = CMD_FLAG_ENABLE | CMD_FLAG_ESTOP | CMD_FLAG_PIVOT;

constexpr uint8_t HEADER_0 = 0, HEADER_1 = 1, LENGTH = 2, TYPE = 3, SEQUENCE = 4;
constexpr uint8_t PREFIX_SIZE = 3, CHECKSUM_SIZE = 1;
constexpr uint8_t FRAME_OVERHEAD = PREFIX_SIZE + CHECKSUM_SIZE;
namespace command { constexpr uint8_t VERSION = 4, SEQUENCE = 5, FLAGS = 7, LINEAR = 8, THETA = 10, ANGULAR = 12; }
namespace reset { constexpr uint8_t MAGIC_0 = 5, MAGIC_1 = 6; }
namespace system {
constexpr uint8_t VERSION = 4, SEQUENCE = 5;
constexpr uint8_t STATE = 7, MODE_MOTION = 8, FLAGS = 9, ERROR = 11, VALIDITY = 13;
constexpr uint8_t MODE_MASK = 0x0F, TRANSITION_SHIFT = 4;
constexpr uint8_t REQUEST_V = 14, REQUEST_THETA = 16, REQUEST_W = 18;
constexpr uint8_t APPLIED_V = 20, APPLIED_THETA = 22, APPLIED_W = 24, APPLIED_MODE = 26, SESSION = 27;
constexpr uint16_t ENABLED = 1u << 0, READY = 1u << 1, FEASIBLE = 1u << 2, RPM_LIMITED = 1u << 3;
constexpr uint16_t REQUEST_VALID = 1u << 4, ESTOP_ACTIVE = 1u << 5, TIMEOUT_ACTIVE = 1u << 6;
constexpr uint16_t POWERDOWN = 1u << 7, DRIVER_STARTING = 1u << 8;
constexpr uint16_t DRIVE_ALLOWED = 1u << 9, STEERING_VALID = 1u << 10;
constexpr uint16_t APPLIED_VALID = 1u << 11;
constexpr uint16_t REQUEST_PIVOT = 1u << 12;
constexpr uint8_t LF_VALID = 1u << 0, RF_VALID = 1u << 1;
constexpr uint8_t LR_VALID = 1u << 2, RR_VALID = 1u << 3;
constexpr uint8_t VOLTAGE_A_VALID = 1u << 4, VOLTAGE_B_VALID = 1u << 5;
}
} // namespace wire
uint8_t xor_checksum(const uint8_t *data, size_t len);
void begin();
void receive_commands(unsigned long now);
// Discard bytes already buffered on AUTO entry, including an incomplete frame.
void discard_pending_rx();
// All publishers run in main context. Never call these from an ISR.
void queue_wheel(uint8_t wheel_id, uint8_t status, int16_t rpm, int32_t encoder, uint32_t rx_us);
void queue_voltage(uint8_t driver_id, uint16_t millivolts, uint32_t rx_us);
void reset_telemetry_session();
void service_tx();
// Copies one diagnostic frame if its slot is free. True means queued, not host ACK.
bool queue_auxiliary(const uint8_t *packet, size_t size);
void discard_debug_tx(); // Does not cancel a queued timing snapshot.
void report_tx_drop(); // Diagnostic only; never changes drive authority.
} // namespace protocol
