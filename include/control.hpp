#pragma once
#include <stdint.h>
#include "build_features.hpp"
#include "vehicle_motion.hpp"

struct CommandPacket
{
    int16_t v_milli_mps;
    int16_t w_milli_radps;
    uint8_t flags;
};

typedef enum
{
    MOTOR_STATE_DISABLED = 0,
    MOTOR_STATE_ENABLED = 1,
    MOTOR_STATE_TIMEOUT_STOP = 2,
    MOTOR_STATE_ESTOP = 3,
    MOTOR_STATE_FAULT = 4,
    MOTOR_STATE_BOOTING = 5,
    // Wire value 6 remains reserved.
    MOTOR_STATE_DRIVER_POWERDOWN = 7
} MotorState;

typedef enum
{
    DRIVE_MODE_STOP = 0,
    DRIVE_MODE_MANUAL = 1,
    DRIVE_MODE_AUTO = 2,
    DRIVE_MODE_POWERDOWN = 3,
    DRIVE_MODE_PIVOT = 4
} DriveMode;
struct WheelRpmCommand
{
    float lf_rpm;
    float lr_rpm;
    float rf_rpm;
    float rr_rpm;
};

struct FaultSnapshot
{
    bool serial_checksum_error;
    bool command_timeout;
    bool driver_fault;
    bool can_tx_failure;
    bool emergency_stop_active;
    bool battery_low;
    bool serial_framing_error;
    bool command_out_of_range;
    bool driver_status_fault;
    bool driver_rx_timeout;
    bool voltage_stale;
    bool can_rx_overrun;
    // Live inputs remain separate from their repeated diagnostic reports.
    bool command_timeout_active;
    bool emergency_stop_input_active;
};

namespace control {
enum SerialMode : uint8_t {
    SERIAL_MODE_CONTROL = 0,
#if FIRMWARE_CAN_DEBUG
    SERIAL_MODE_BRIDGE = 1,
#endif
    SERIAL_MODE_RESETTING = 2
};
enum class Error : uint8_t {
    SafetyCanTx, VoltageCanTx, DriverStatus, DriverTimeout, BatteryLow,
    Checksum, Framing, CommandRange, CommandTimeout, Estop, VoltageStale, Count
};
struct Status {
    bool booting = true;
    bool driver_powerdown = false;
    bool driver_starting = false;
    bool motor_enabled = false;
    bool estop_active = false;
    bool command_timeout = true;
    SerialMode serial_mode = SERIAL_MODE_CONTROL;
    unsigned long control_mode_started_ms = 0;
    WheelRpmCommand target_wheel_rpm_cmd = {};
};
const Status &status();
const vehicle::MotionController &motion_output();
// Reports expire only after successfully transmitted canonical 0x82 packets.
void report(Error error);
bool reported(Error error);
void initialize_power();
void finish_boot();
void initialize_motion();
void update_power(DriveMode drive_mode, unsigned long now);
void accept_command(const CommandPacket &command, unsigned long now);
void update_drive(DriveMode drive_mode,
                  unsigned int v_pwm, unsigned int w_pwm, unsigned long now);
void reset_mcu(void);
// Reset telemetry/scheduler/report history without changing the selected mode.
void initialize_session(unsigned long now);
void enter_control_mode(unsigned long now);
void report_safety_tx_failure(void);
bool has_driver_fault(void);
void refresh_errors(unsigned long now);
void consume_error_reports(void);
FaultSnapshot fault_snapshot(unsigned long now);
MotorState motor_state(const FaultSnapshot &fault);
int16_t rpm_to_i16(float rpm);
#if FIRMWARE_CAN_DEBUG
void enter_bridge_mode();
#endif
} // namespace control
