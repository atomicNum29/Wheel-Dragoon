#include <assert.h>
#include <stdint.h>

#include "can_tx_schedule.hpp"
#include "little_endian.hpp"
#include "md200t_can.hpp"

static CanFrame captured_frame = {};
static uint32_t captured_timeout_us = 0u;

bool can_transmit(const CanFrame &frame, uint32_t timeout_us)
{
    captured_frame = frame;
    captured_timeout_us = timeout_us;
    return true;
}

static void assert_frame(uint16_t id, const uint8_t (&expected)[8])
{
    assert(captured_frame.id == id);
    assert(captured_frame.dlc == 8u);
    assert(captured_timeout_us == 2000u);
    for (uint8_t i = 0u; i < 8u; ++i)
        assert(captured_frame.data[i] == expected[i]);
}

int main()
{
    assert(md200t_set_velocity(MD200T_DRIVER_A_ID, 100, -80));
    const uint8_t velocity[8] = {0xCFu, 0x01u, 0x64u, 0x00u, 0x01u, 0xB0u, 0xFFu, 0x00u};
    assert_frame(0x001u, velocity);

    // Exercise signed limits independently of the production constants. This
    // catches accidental signed right shifts or endian reversal.
    assert(md200t_set_velocity(MD200T_DRIVER_B_ID, -32768, 32767));
    const uint8_t velocity_limits[8] = {0xCFu, 0x01u, 0x00u, 0x80u, 0x01u, 0xFFu, 0x7Fu, 0x00u};
    assert_frame(0x002u, velocity_limits);

    assert(md200t_torque_off(MD200T_DRIVER_B_ID));
    const uint8_t torque_off[8] = {0xAEu, 0x01u, 0x01u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};
    assert_frame(0x002u, torque_off);

    assert(md200t_request_pid_data(MD200T_DRIVER_A_ID, MD200T_PID_VOLT_IN));
    const uint8_t voltage_request[8] = {0x04u, 0x8Fu, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};
    assert_frame(0x001u, voltage_request);

    assert(md200t_request_pid_data(MD200T_DRIVER_A_ID, MD200T_PID_MAIN_DATA));
    const uint8_t main_data_request[8] = {0x04u, 0xC1u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};
    assert_frame(0x001u, main_data_request);

    assert(md200t_request_pid_data(MD200T_DRIVER_B_ID, MD200T_PID_MAIN_DATA2));
    const uint8_t main_data2_request[8] = {0x04u, 0xC8u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u};
    assert_frame(0x002u, main_data2_request);

    assert(CAN_TX_COMMAND_PERIOD_US == 10000u);
    assert(CAN_TX_POLL_PHASE_US == 5000u);
    uint8_t lf_count = 0u;
    uint8_t rf_count = 0u;
    uint8_t lr_count = 0u;
    uint8_t rr_count = 0u;
    uint8_t voltage_a_count = 0u;
    uint8_t voltage_b_count = 0u;
    for (uint8_t cycle = 0u; cycle < CAN_TX_SUPERFRAME_CYCLES; ++cycle)
    {
        ScheduledPollRequest request = {};
        if (!scheduled_poll_for_cycle(cycle, request))
            continue;

        if (request.driver_id == MD200T_DRIVER_A_ID && request.pid == MD200T_PID_MAIN_DATA)
            ++lf_count;
        else if (request.driver_id == MD200T_DRIVER_B_ID && request.pid == MD200T_PID_MAIN_DATA)
            ++rf_count;
        else if (request.driver_id == MD200T_DRIVER_B_ID && request.pid == MD200T_PID_MAIN_DATA2)
            ++lr_count;
        else if (request.driver_id == MD200T_DRIVER_A_ID && request.pid == MD200T_PID_MAIN_DATA2)
            ++rr_count;
        else if (request.driver_id == MD200T_DRIVER_A_ID && request.pid == MD200T_PID_VOLT_IN)
            ++voltage_a_count;
        else if (request.driver_id == MD200T_DRIVER_B_ID && request.pid == MD200T_PID_VOLT_IN)
            ++voltage_b_count;
        else
            assert(false);
    }
    assert(lf_count == 10u && rf_count == 10u && lr_count == 10u && rr_count == 10u);
    assert(voltage_a_count == 1u && voltage_b_count == 1u);

    ScheduledPollRequest scheduled = {};
    assert(scheduled_poll_for_cycle(0u, scheduled));
    assert(scheduled.driver_id == MD200T_DRIVER_A_ID && scheduled.pid == MD200T_PID_VOLT_IN);
    assert(scheduled_poll_for_cycle(1u, scheduled));
    assert(scheduled.driver_id == MD200T_DRIVER_A_ID && scheduled.pid == MD200T_PID_MAIN_DATA);
    assert(scheduled_poll_for_cycle(2u, scheduled));
    assert(scheduled.driver_id == MD200T_DRIVER_B_ID && scheduled.pid == MD200T_PID_MAIN_DATA);
    assert(scheduled_poll_for_cycle(3u, scheduled));
    assert(scheduled.driver_id == MD200T_DRIVER_B_ID && scheduled.pid == MD200T_PID_MAIN_DATA2);
    assert(scheduled_poll_for_cycle(4u, scheduled));
    assert(scheduled.driver_id == MD200T_DRIVER_A_ID && scheduled.pid == MD200T_PID_MAIN_DATA2);
    assert(!scheduled_poll_for_cycle(5u, scheduled));
    assert(scheduled_poll_for_cycle(50u, scheduled));
    assert(scheduled.driver_id == MD200T_DRIVER_B_ID && scheduled.pid == MD200T_PID_VOLT_IN);

    uint8_t driver_id = 0u;
    assert(md200t_decode_response_id(0x701u, driver_id));
    assert(driver_id == MD200T_DRIVER_A_ID);
    assert(md200t_decode_response_id(0x702u, driver_id));
    assert(driver_id == MD200T_DRIVER_B_ID);
    assert(!md200t_decode_response_id(0x001u, driver_id));
    assert(!md200t_decode_response_id(0x800u, driver_id));

    uint8_t bytes[2] = {};
    little_endian::write_i16(bytes, -80);
    assert(bytes[0] == 0xB0u && bytes[1] == 0xFFu);
    assert(little_endian::read_i16(bytes) == -80);
    return 0;
}
