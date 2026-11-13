#include <Arduino.h>
#include "can_debug.hpp"
#include "control.hpp"
#include "can_control.hpp"
#include "serial_protocol.hpp"
#include "little_endian.hpp"

#if FIRMWARE_CAN_DEBUG
namespace can_debug {
using namespace protocol::wire;
namespace {
constexpr uint8_t MAX_RESPONSES = 2;
constexpr uint32_t MICROSECONDS_PER_MILLISECOND = 1000;
constexpr uint8_t CAN_BRIDGE_RESPONSE_LENGTH = 14;
constexpr uint8_t CAN_BRIDGE_RESPONSE_TYPE = 0xA0;
constexpr uint8_t CAN_BRIDGE_RESPONSE_SIZE = 18;
constexpr uint8_t CAN_BENCHMARK_RESULT_LENGTH = 52;
constexpr uint8_t CAN_BENCHMARK_RESULT_TYPE = 0xA1;
constexpr uint8_t CAN_BENCHMARK_RESULT_SIZE = 56;
constexpr uint8_t CAN_BRIDGE_STATUS_OK = 0;
constexpr uint8_t CAN_BRIDGE_STATUS_TX_FAILED = 1;
constexpr uint8_t CAN_BRIDGE_STATUS_RX_TIMEOUT = 2;
constexpr uint8_t CAN_BRIDGE_STATUS_INVALID_REQUEST = 3;
constexpr uint8_t CAN_BENCHMARK_STATUS_RESPONSE = 0;
constexpr uint8_t CAN_BENCHMARK_STATUS_TX_FAILED = 1;
constexpr uint8_t CAN_BENCHMARK_STATUS_RX_TIMEOUT = 2;
constexpr uint8_t CAN_BENCHMARK_STATUS_INVALID_REQUEST = 3;
constexpr uint8_t CAN_BENCHMARK_FLAG_FINAL = 0x01;
constexpr uint32_t CAN_BRIDGE_TX_TIMEOUT_US = 2000;
constexpr uint32_t CAN_BRIDGE_MAX_RX_TIMEOUT_US = 100000;
// Allow settling after BRIDGE-entry PID 174 commands before the first request.
// Automatic response PIDs depend on driver configuration, not the CLI defaults.
constexpr uint32_t CAN_BENCHMARK_START_SETTLE_US = 5000;

namespace request_wire { constexpr uint8_t ID = 5, DLC = 7, DATA = 8, TIMEOUT_MS = 16; }
namespace bridge_wire { constexpr uint8_t STATUS = 5, ID = 6, DLC = 8, DATA = 9, CHECKSUM = CAN_BRIDGE_RESPONSE_SIZE - 1; }
namespace benchmark_wire {
constexpr uint8_t RESPONSE_ID = 16, PID_0 = 18, REPEAT = 19, PERIOD_MS = 21;
constexpr uint8_t TIMEOUT_MS = 23, RESPONSE_COUNT = 25, PID_1 = 26;
}
namespace result_wire {
constexpr uint8_t ITERATION = 5, STATUS = 7, FLAGS = 8, TX_US = 9;
constexpr uint8_t RX_0_US = 13, RTT_0_US = 17, TIMEOUTS = 21, TX_FAILURES = 23;
constexpr uint8_t RESPONSE_ID = 25, DLC_0 = 27, DATA_0 = 28, RESPONSE_COUNT = 36;
constexpr uint8_t RECEIVED_MASK = 37, RX_1_US = 38, RTT_1_US = 42, DLC_1 = 46, DATA_1 = 47;
constexpr uint8_t CHECKSUM = CAN_BENCHMARK_RESULT_SIZE - 1;
}
struct CanBenchmarkState
{
    CanFrame request;
    uint16_t response_can_id;
    uint8_t response_pids[MAX_RESPONSES];
    uint8_t response_count;
    uint8_t received_mask;
    CanFrame responses[MAX_RESPONSES];
    uint32_t rx_timestamps_us[MAX_RESPONSES];
    uint16_t repeat_count;
    uint16_t iteration;
    uint16_t timeout_count;
    uint16_t tx_failure_count;
    uint32_t period_us;
    uint32_t timeout_us;
    uint32_t next_tx_us;
    uint32_t tx_timestamp_us;
    uint8_t seq;
    bool active;
    bool waiting_for_response;
};

struct CanBenchmarkResult
{
    CanFrame responses[MAX_RESPONSES];
    uint16_t response_can_id;
    uint32_t tx_timestamp_us;
    uint32_t rx_timestamps_us[MAX_RESPONSES];
    uint32_t rtt_us[MAX_RESPONSES];
    uint16_t iteration;
    uint16_t timeout_count;
    uint16_t tx_failure_count;
    uint8_t seq;
    uint8_t status;
    uint8_t flags;
    uint8_t response_count;
    uint8_t received_mask;
    bool tx_pending;
};

CanBenchmarkState can_benchmark = {};
CanBenchmarkResult can_benchmark_result = {};
uint8_t bridge_tunnel_seq = 0;
} // namespace

static void send_can_bridge_response(uint8_t seq, uint8_t status, const CanFrame *frame)
{
    uint8_t packet[CAN_BRIDGE_RESPONSE_SIZE];
    packet[HEADER_0] = PKT_HEADER_0;
    packet[HEADER_1] = PKT_HEADER_1;
    packet[LENGTH] = CAN_BRIDGE_RESPONSE_LENGTH;
    packet[TYPE] = CAN_BRIDGE_RESPONSE_TYPE;
    packet[SEQUENCE] = seq;
    packet[bridge_wire::STATUS] = status;

    uint16_t id = 0u;
    uint8_t dlc = 0u;
    uint8_t data[CAN_PAYLOAD_SIZE] = {};
    if (frame != nullptr)
    {
        id = frame->id;
        dlc = frame->dlc;
        if (dlc > CAN_PAYLOAD_SIZE)
            dlc = CAN_PAYLOAD_SIZE;
        for (uint8_t i = 0u; i < dlc; ++i)
            data[i] = frame->data[i];
    }

    little_endian::write_u16(&packet[bridge_wire::ID], id);
    packet[bridge_wire::DLC] = dlc;
    for (uint8_t i = 0u; i < CAN_PAYLOAD_SIZE; ++i)
        packet[bridge_wire::DATA + i] = data[i];
    packet[bridge_wire::CHECKSUM] = protocol::xor_checksum(packet, CAN_BRIDGE_RESPONSE_SIZE - 1u);

    if (Serial.availableForWrite() >= CAN_BRIDGE_RESPONSE_SIZE)
        Serial.write(packet, CAN_BRIDGE_RESPONSE_SIZE);
}

static void queue_invalid_can_benchmark_result(uint8_t seq)
{
    can_benchmark_result = {};
    can_benchmark_result.seq = seq;
    can_benchmark_result.status = CAN_BENCHMARK_STATUS_INVALID_REQUEST;
    can_benchmark_result.flags = CAN_BENCHMARK_FLAG_FINAL;
    can_benchmark_result.tx_pending = true;
}

static void complete_can_benchmark_attempt(uint8_t status)
{
    for (uint8_t i = 0u; i < MAX_RESPONSES; ++i)
    {
        can_benchmark_result.responses[i] = can_benchmark.responses[i];
        can_benchmark_result.rx_timestamps_us[i] = can_benchmark.rx_timestamps_us[i];
        can_benchmark_result.rtt_us[i] =
            (can_benchmark.received_mask & (1u << i)) != 0u
                ? static_cast<uint32_t>(can_benchmark.rx_timestamps_us[i] -
                                        can_benchmark.tx_timestamp_us)
                : 0u;
    }

    const bool final = can_benchmark.iteration >= can_benchmark.repeat_count;
    can_benchmark_result.tx_timestamp_us = can_benchmark.tx_timestamp_us;
    can_benchmark_result.response_can_id = can_benchmark.response_can_id;
    can_benchmark_result.iteration = can_benchmark.iteration;
    can_benchmark_result.timeout_count = can_benchmark.timeout_count;
    can_benchmark_result.tx_failure_count = can_benchmark.tx_failure_count;
    can_benchmark_result.seq = can_benchmark.seq;
    can_benchmark_result.status = status;
    can_benchmark_result.flags = final ? CAN_BENCHMARK_FLAG_FINAL : 0u;
    can_benchmark_result.response_count = can_benchmark.response_count;
    can_benchmark_result.received_mask = can_benchmark.received_mask;
    can_benchmark_result.tx_pending = true;

    can_benchmark.waiting_for_response = false;
    if (final)
        can_benchmark.active = false;
}

void flush_result(void)
{
    if (!can_benchmark_result.tx_pending || Serial.availableForWrite() < CAN_BENCHMARK_RESULT_SIZE)
        return;

    uint8_t packet[CAN_BENCHMARK_RESULT_SIZE] = {};
    packet[HEADER_0] = PKT_HEADER_0;
    packet[HEADER_1] = PKT_HEADER_1;
    packet[LENGTH] = CAN_BENCHMARK_RESULT_LENGTH;
    packet[TYPE] = CAN_BENCHMARK_RESULT_TYPE;
    packet[SEQUENCE] = can_benchmark_result.seq;
    little_endian::write_u16(&packet[result_wire::ITERATION], can_benchmark_result.iteration);
    packet[result_wire::STATUS] = can_benchmark_result.status;
    packet[result_wire::FLAGS] = can_benchmark_result.flags;
    little_endian::write_u32(&packet[result_wire::TX_US], can_benchmark_result.tx_timestamp_us);
    little_endian::write_u32(&packet[result_wire::RX_0_US], can_benchmark_result.rx_timestamps_us[0]);
    little_endian::write_u32(&packet[result_wire::RTT_0_US], can_benchmark_result.rtt_us[0]);
    little_endian::write_u16(&packet[result_wire::TIMEOUTS], can_benchmark_result.timeout_count);
    little_endian::write_u16(&packet[result_wire::TX_FAILURES], can_benchmark_result.tx_failure_count);
    little_endian::write_u16(&packet[result_wire::RESPONSE_ID], can_benchmark_result.response_can_id);
    packet[result_wire::DLC_0] = can_benchmark_result.responses[0].dlc > CAN_PAYLOAD_SIZE
                     ? CAN_PAYLOAD_SIZE
                     : can_benchmark_result.responses[0].dlc;
    for (uint8_t i = 0u; i < packet[result_wire::DLC_0]; ++i)
        packet[result_wire::DATA_0 + i] = can_benchmark_result.responses[0].data[i];
    packet[result_wire::RESPONSE_COUNT] = can_benchmark_result.response_count;
    packet[result_wire::RECEIVED_MASK] = can_benchmark_result.received_mask;
    little_endian::write_u32(&packet[result_wire::RX_1_US], can_benchmark_result.rx_timestamps_us[1]);
    little_endian::write_u32(&packet[result_wire::RTT_1_US], can_benchmark_result.rtt_us[1]);
    packet[result_wire::DLC_1] = can_benchmark_result.responses[1].dlc > CAN_PAYLOAD_SIZE
                     ? CAN_PAYLOAD_SIZE
                     : can_benchmark_result.responses[1].dlc;
    for (uint8_t i = 0u; i < packet[result_wire::DLC_1]; ++i)
        packet[result_wire::DATA_1 + i] = can_benchmark_result.responses[1].data[i];
    packet[result_wire::CHECKSUM] = protocol::xor_checksum(packet, CAN_BENCHMARK_RESULT_SIZE - 1u);

    Serial.write(packet, CAN_BENCHMARK_RESULT_SIZE);
    can_benchmark_result.tx_pending = false;
}

void cancel(void)
{
    can_benchmark.active = false;
    can_benchmark.waiting_for_response = false;
    can_benchmark_result.tx_pending = false;
}

static bool can_benchmark_consume_rx(const CanFrame &frame, uint32_t rx_timestamp_us)
{
    if (!can_benchmark.active || !can_benchmark.waiting_for_response ||
        frame.id != can_benchmark.response_can_id || frame.dlc == 0u)
        return false;

    uint8_t response_index = can_benchmark.response_count;
    for (uint8_t i = 0u; i < can_benchmark.response_count; ++i)
    {
        if (frame.data[0] == can_benchmark.response_pids[i] &&
            (can_benchmark.received_mask & (1u << i)) == 0u)
        {
            response_index = i;
            break;
        }
    }
    if (response_index >= can_benchmark.response_count)
        return false;

    // A queued frame from an earlier attempt must not fail the current attempt.
    if (static_cast<int32_t>(rx_timestamp_us - can_benchmark.tx_timestamp_us) < 0)
        return false;
    if (static_cast<uint32_t>(rx_timestamp_us - can_benchmark.tx_timestamp_us) >= can_benchmark.timeout_us)
    {
        ++can_benchmark.timeout_count;
        complete_can_benchmark_attempt(CAN_BENCHMARK_STATUS_RX_TIMEOUT);
        return false;
    }

    can_benchmark.responses[response_index] = frame;
    can_benchmark.rx_timestamps_us[response_index] = rx_timestamp_us;
    can_benchmark.received_mask |= static_cast<uint8_t>(1u << response_index);
    const uint8_t complete_mask = static_cast<uint8_t>((1u << can_benchmark.response_count) - 1u);
    if (can_benchmark.received_mask == complete_mask)
        complete_can_benchmark_attempt(CAN_BENCHMARK_STATUS_RESPONSE);
    return true;
}

void update(uint32_t now_us)
{
    if (control::status().driver_powerdown || control::status().driver_starting || !can_benchmark.active || can_benchmark_result.tx_pending)
        return;

    if (can_benchmark.waiting_for_response)
    {
        if (static_cast<uint32_t>(now_us - can_benchmark.tx_timestamp_us) >= can_benchmark.timeout_us)
        {
            can_control::process_queued_rx(millis());
            if (!can_benchmark.waiting_for_response)
                return;
            ++can_benchmark.timeout_count;
            complete_can_benchmark_attempt(CAN_BENCHMARK_STATUS_RX_TIMEOUT);
        }
        return;
    }

    if (static_cast<int32_t>(now_us - can_benchmark.next_tx_us) < 0)
        return;

    ++can_benchmark.iteration;
    can_benchmark.tx_timestamp_us = now_us;
    can_benchmark.next_tx_us = now_us + can_benchmark.period_us;
    can_benchmark.received_mask = 0u;
    can_benchmark.responses[0] = {};
    can_benchmark.responses[1] = {};
    can_benchmark.rx_timestamps_us[0] = 0u;
    can_benchmark.rx_timestamps_us[1] = 0u;

    if (!can_transmit(can_benchmark.request, CAN_BRIDGE_TX_TIMEOUT_US))
    {
        ++can_benchmark.tx_failure_count;
        control::report_safety_tx_failure();
        complete_can_benchmark_attempt(CAN_BENCHMARK_STATUS_TX_FAILED);
        return;
    }

    can_benchmark.waiting_for_response = true;
}

void handle_benchmark_request(const uint8_t *packet)
{
    const uint8_t seq = packet[SEQUENCE];
    cancel();

    if (protocol::xor_checksum(packet, CAN_BENCHMARK_PACKET_SIZE - 1u) != packet[CAN_BENCHMARK_PACKET_SIZE - 1u])
    {
        control::report(control::Error::Checksum);
        queue_invalid_can_benchmark_result(seq);
        return;
    }

    CanFrame request = {};
    request.id = little_endian::read_u16(&packet[request_wire::ID]);
    request.dlc = packet[request_wire::DLC];
    for (uint8_t i = 0u; i < CAN_PAYLOAD_SIZE; ++i)
        request.data[i] = packet[request_wire::DATA + i];

    const uint16_t response_can_id = little_endian::read_u16(&packet[benchmark_wire::RESPONSE_ID]);
    const uint8_t response_pid_1 = packet[benchmark_wire::PID_0];
    const uint16_t repeat_count = little_endian::read_u16(&packet[benchmark_wire::REPEAT]);
    const uint16_t period_ms = little_endian::read_u16(&packet[benchmark_wire::PERIOD_MS]);
    const uint16_t timeout_ms = little_endian::read_u16(&packet[benchmark_wire::TIMEOUT_MS]);
    const uint8_t response_count = packet[benchmark_wire::RESPONSE_COUNT];
    const uint8_t response_pid_2 = packet[benchmark_wire::PID_1];

    if (control::status().driver_powerdown || control::status().driver_starting || request.id > CAN_STANDARD_ID_MAX || request.dlc > CAN_PAYLOAD_SIZE || response_can_id > CAN_STANDARD_ID_MAX ||
        repeat_count == 0u || period_ms == 0u || timeout_ms == 0u ||
        timeout_ms > period_ms || static_cast<uint32_t>(timeout_ms) * MICROSECONDS_PER_MILLISECOND > CAN_BRIDGE_MAX_RX_TIMEOUT_US ||
        response_count == 0u || response_count > MAX_RESPONSES ||
        (response_count == MAX_RESPONSES && response_pid_1 == response_pid_2))
    {
        queue_invalid_can_benchmark_result(seq);
        return;
    }

    control::enter_bridge_mode();
    bridge_tunnel_seq = seq;
    can_benchmark.request = request;
    can_benchmark.response_can_id = response_can_id;
    can_benchmark.response_pids[0] = response_pid_1;
    can_benchmark.response_pids[1] = response_pid_2;
    can_benchmark.response_count = response_count;
    can_benchmark.received_mask = 0u;
    can_benchmark.repeat_count = repeat_count;
    can_benchmark.iteration = 0u;
    can_benchmark.timeout_count = 0u;
    can_benchmark.tx_failure_count = 0u;
    can_benchmark.period_us = static_cast<uint32_t>(period_ms) * MICROSECONDS_PER_MILLISECOND;
    can_benchmark.timeout_us = static_cast<uint32_t>(timeout_ms) * MICROSECONDS_PER_MILLISECOND;
    can_benchmark.next_tx_us = micros() + CAN_BENCHMARK_START_SETTLE_US;
    can_benchmark.tx_timestamp_us = 0u;
    can_benchmark.seq = seq;
    can_benchmark.active = true;
    can_benchmark.waiting_for_response = false;
}

void handle_bridge_request(const uint8_t *packet)
{
    const uint8_t seq = packet[SEQUENCE];
    if (protocol::xor_checksum(packet, CAN_BRIDGE_PACKET_SIZE - 1u) != packet[CAN_BRIDGE_PACKET_SIZE - 1u])
    {
        control::report(control::Error::Checksum);
        send_can_bridge_response(seq, CAN_BRIDGE_STATUS_INVALID_REQUEST, nullptr);
        return;
    }

    CanFrame tx = {};
    tx.id = little_endian::read_u16(&packet[request_wire::ID]);
    tx.dlc = packet[request_wire::DLC];
    if (control::status().driver_powerdown || control::status().driver_starting || tx.id > CAN_STANDARD_ID_MAX || tx.dlc > CAN_PAYLOAD_SIZE)
    {
        send_can_bridge_response(seq, CAN_BRIDGE_STATUS_INVALID_REQUEST, nullptr);
        return;
    }

    for (uint8_t i = 0u; i < CAN_PAYLOAD_SIZE; ++i)
        tx.data[i] = packet[request_wire::DATA + i];

    cancel();
    control::enter_bridge_mode();
    bridge_tunnel_seq = seq;

    const uint16_t timeout_ms = little_endian::read_u16(&packet[request_wire::TIMEOUT_MS]);
    if (!can_transmit(tx, CAN_BRIDGE_TX_TIMEOUT_US))
    {
        control::report_safety_tx_failure();
        send_can_bridge_response(seq, CAN_BRIDGE_STATUS_TX_FAILED, nullptr);
        return;
    }

    uint32_t timeout_us = static_cast<uint32_t>(timeout_ms) * MICROSECONDS_PER_MILLISECOND;
    if (timeout_us > CAN_BRIDGE_MAX_RX_TIMEOUT_US)
        timeout_us = CAN_BRIDGE_MAX_RX_TIMEOUT_US;

    CanFrame rx = {};
    // Bridge mode is a transparent CAN monitor: the first frame received
    // within the request timeout is returned regardless of ID or PID.
    if (!can_receive(rx, timeout_us))
    {
        send_can_bridge_response(seq, CAN_BRIDGE_STATUS_RX_TIMEOUT, nullptr);
        return;
    }

    send_can_bridge_response(seq, CAN_BRIDGE_STATUS_OK, &rx);
}

void consume_rx(const CanFrame &frame, uint32_t received_us)
{
    if (!can_benchmark_consume_rx(frame, received_us))
        send_can_bridge_response(bridge_tunnel_seq, CAN_BRIDGE_STATUS_OK, &frame);
}
} // namespace can_debug
#endif
