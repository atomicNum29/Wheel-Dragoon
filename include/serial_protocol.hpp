#pragma once
#include <stddef.h>
#include <stdint.h>

namespace protocol {
namespace wire {
constexpr uint8_t PKT_HEADER_0 = 0xAA;
constexpr uint8_t PKT_HEADER_1 = 0x55;
constexpr uint8_t CMD_PACKET_LENGTH = 7;
constexpr uint8_t CMD_PACKET_TYPE = 0x01;
constexpr uint8_t CMD_PACKET_SIZE = 11;
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
constexpr uint8_t SYSTEM_STATUS_PACKET_LENGTH = 59;
constexpr uint8_t SYSTEM_STATUS_PACKET_TYPE = 0x82;
constexpr uint8_t SYSTEM_STATUS_PACKET_VERSION = 2;
constexpr uint8_t SYSTEM_STATUS_PACKET_SIZE = 63;
constexpr int16_t CMD_LINEAR_MILLI_MPS_MIN = -2000;
constexpr int16_t CMD_LINEAR_MILLI_MPS_MAX = 2000;
constexpr int16_t CMD_ANGULAR_MILLI_RADPS_MIN = -5000;
constexpr int16_t CMD_ANGULAR_MILLI_RADPS_MAX = 5000;
constexpr uint8_t CMD_FLAG_ESTOP = 0x02;

constexpr uint8_t HEADER_0 = 0, HEADER_1 = 1, LENGTH = 2, TYPE = 3, SEQUENCE = 4;
constexpr uint8_t PREFIX_SIZE = 3, CHECKSUM_SIZE = 1;
constexpr uint8_t FRAME_OVERHEAD = PREFIX_SIZE + CHECKSUM_SIZE;
namespace command { constexpr uint8_t LINEAR = 5, ANGULAR = 7, FLAGS = 9; }
namespace reset { constexpr uint8_t MAGIC_0 = 5, MAGIC_1 = 6; }
namespace system {
constexpr uint8_t VERSION = 4, SEQUENCE = 5, STATE = 6, ERROR = 7, VALIDITY = 9;
constexpr uint8_t VOLTAGE_A = 10, VOLTAGE_B = 12, TIMESTAMP = 14;
constexpr uint8_t WHEEL_BASE = 18, WHEEL_SIZE = 11;
constexpr uint8_t LF = WHEEL_BASE, RF = LF + WHEEL_SIZE;
constexpr uint8_t LR = RF + WHEEL_SIZE, RR = LR + WHEEL_SIZE;
constexpr uint8_t WHEEL_STATUS = 0, WHEEL_RPM = 1, WHEEL_ENCODER = 3, WHEEL_TIMESTAMP = 7;
constexpr uint8_t LF_VALID = 1u << 0, RF_VALID = 1u << 1;
constexpr uint8_t LR_VALID = 1u << 2, RR_VALID = 1u << 3;
constexpr uint8_t VOLTAGE_A_VALID = 1u << 4, VOLTAGE_B_VALID = 1u << 5;
}
} // namespace wire
uint8_t xor_checksum(const uint8_t *data, size_t len);
void begin();
void receive_commands(unsigned long now);
void send_status_if_ready();
} // namespace protocol
