// Verify main-loop deadline behavior with already timestamped ISR queue entries.
#include <cstdio>
#include <deque>
#include "../src/main.cpp"

uint64_t fake_clock_us = 0;
uint32_t SCB_AIRCR = 0;
FakeSerial Serial;
struct QueuedRx { CanFrame frame; uint32_t received_us; };
static std::deque<QueuedRx> rx_queue;
bool can_begin(uint32_t) { return true; }
bool can_is_initialized() { return true; }
bool can_transmit(const CanFrame &, uint32_t) { return true; }
bool can_rx_overrun_detected() { return false; }
void can_rx_overrun_report_sent() {}
void can_clear_rx_overrun_reports() {}
bool can_rx_pending() { return !rx_queue.empty(); }
uint32_t can_rx_overrun_count() { return 0; }
bool can_receive_timed(CanFrame &frame, uint32_t &received_us, uint32_t)
{
    if (rx_queue.empty()) return false;
    frame = rx_queue.front().frame;
    received_us = rx_queue.front().received_us;
    rx_queue.pop_front();
    return true; // Main clock does not travel back to the ISR timestamp.
}
bool can_receive(CanFrame &frame, uint32_t timeout_us)
{
    uint32_t received_us;
    return can_receive_timed(frame, received_us, timeout_us);
}

static void enqueue(uint8_t driver, uint8_t pid, uint32_t received_us)
{
    CanFrame frame = {};
    frame.id = 0x700u | driver; frame.dlc = 8; frame.data[0] = pid;
    rx_queue.push_back({frame, received_us});
}

static void reset_control(uint32_t tx_us = 10000)
{
    rx_queue.clear(); Serial = {};
    fake_clock_us = tx_us;
    serial_mode = SERIAL_MODE_CONTROL;
    driver_powerdown = driver_starting = false;
    driver_rx_timeout_reports_remaining = 0;
    driver_status_fault_reports_remaining = 0;
    battery_low_reports_remaining = 0;
    reset_telemetry_validity();
    reset_can_tx_scheduler(tx_us);
    can_tx_scheduler.driver_a_command_sent = true;
    can_tx_scheduler.driver_a_tx_us = tx_us;
    can_tx_scheduler.expected_wheel_mask = CAN_TX_DRIVER_A_WHEELS_MASK;
    cancel_can_benchmark();
}

#define CHECK(condition) do { if (!(condition)) { \
    std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); return false; \
} } while (0)

static bool check_control_deadlines()
{
    reset_control();
    md200t_drain_rx(millis()); // No data at the loop's initial drain.
    enqueue(1, 196, 10700); enqueue(1, 201, 11100);
    fake_clock_us = 13200; // Deadline expired during other main-loop work.
    close_driver_response_window(MD200T_DRIVER_A_ID);
    CHECK(driver_rx_timeout_reports_remaining == 0 && lf_telemetry.valid && rr_telemetry.valid);
    CHECK(lf_telemetry.rx_timestamp_us == 10700 && rr_telemetry.rx_timestamp_us == 11100);
    CHECK(fake_clock_us == 13200 && rx_queue.empty());

    reset_control();
    // Queue depth greater than the old eight-frame drain limit.
    for (int i = 0; i < 20; ++i) enqueue(3, 196, 10500 + i);
    enqueue(1, 196, 10700); enqueue(1, 201, 11100);
    fake_clock_us = 13400;
    close_driver_response_window(MD200T_DRIVER_A_ID);
    CHECK(driver_rx_timeout_reports_remaining == 0 && rx_queue.empty());

    reset_control();
    enqueue(1, 196, 9999); // Pre-command queue entry must not count as this round.
    enqueue(1, 201, 13000); // Exactly at deadline is outside the existing window.
    fake_clock_us = 13400;
    close_driver_response_window(MD200T_DRIVER_A_ID);
    CHECK(driver_rx_timeout_reports_remaining > 0 && !lf_telemetry.valid && !rr_telemetry.valid);

    reset_control(0xFFFFFF00u);
    enqueue(1, 196, 0x000001BCu); enqueue(1, 201, 0x0000034Cu);
    fake_clock_us = 0x100000C00ULL;
    close_driver_response_window(MD200T_DRIVER_A_ID);
    CHECK(driver_rx_timeout_reports_remaining == 0 && lf_telemetry.valid && rr_telemetry.valid);
    return true;
}

static void reset_benchmark()
{
    reset_control();
    serial_mode = SERIAL_MODE_BRIDGE;
    can_benchmark = {};
    can_benchmark_result = {};
    can_benchmark.active = can_benchmark.waiting_for_response = true;
    can_benchmark.response_can_id = 0x701;
    can_benchmark.response_count = 2;
    can_benchmark.response_pids[0] = 196;
    can_benchmark.response_pids[1] = 201;
    can_benchmark.tx_timestamp_us = 10000;
    can_benchmark.timeout_us = 3000;
    can_benchmark.repeat_count = 10;
    can_benchmark.iteration = 1;
}

static bool check_benchmark_deadlines()
{
    reset_benchmark();
    enqueue(1, 196, 9900); // Stale response must not end this attempt as timeout.
    enqueue(1, 196, 10700); enqueue(1, 201, 11100);
    fake_clock_us = 13200;
    can_benchmark_update(micros());
    CHECK(can_benchmark.timeout_count == 0 && !can_benchmark.waiting_for_response);
    CHECK(can_benchmark_result.tx_pending && can_benchmark.received_mask == 3);
    CHECK(can_benchmark.rx_timestamps_us[0] == 10700 && can_benchmark.rx_timestamps_us[1] == 11100);

    reset_benchmark();
    enqueue(1, 196, 10700); enqueue(1, 201, 13001);
    fake_clock_us = 13400;
    can_benchmark_update(micros());
    CHECK(can_benchmark.timeout_count == 1 && !can_benchmark.waiting_for_response);
    return true;
}

int main()
{
    if (!check_control_deadlines() || !check_benchmark_deadlines()) return 1;
    std::puts("PASS: queued RX timestamps, deadline drains, stale/late frames and benchmark");
}
