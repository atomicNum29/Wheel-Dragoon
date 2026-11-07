# Independent front steering and four wheel drive

> Current RC curvature input, doubled acceleration/deceleration and provisional
> +/-70 degree PWM mapping are documented in [the RC steering update](rc-steering.ko.md).
> This initial implementation record contains historical defaults (skid mode,
> disabled calibration, +/-20 degree tables and polled CAN). Current settings use
> independent steering, user-enabled provisional calibration and FIFO/ISR CAN.

Implementation status: host-tested and compiled for Teensy 3.2. No new physical driving, PWM, servo, or CAN timing validation has been performed. The checked-in configuration deliberately inhibits drive until **both** steering calibrations are verified. Skid-steer remains the selected mode.

## Repository inspection and preserved architecture

`platformio.ini` selects `src/main.cpp` with Arduino/Teensy (`teensy32` environment, `teensy31` board ID for Teensy 3.1/3.2). The project uses its own register-level, polling FlexCAN0 implementation in `src/flexcan0.cpp`, rather than a FlexCAN library. No existing steering implementation was found.

Inputs are RC PWM on pins 0/1/2 or binary USB Serial commands from the Raspberry Pi. RC mode selection (STOP/MANUAL/AUTO/POWERDOWN) remains separate from the new vehicle kinematic mode. The existing `DriveMode` name in `main.cpp` refers to RC authority; `vehicle::DriveMode` refers to skid/independent steering.

The unchanged 11-byte command is `AA 55 07 01 seq v_i16_le w_i16_le flags xor`, with milli-m/s and milli-rad/s units, enable bit 0, E-stop bit 1. AUTO timeout is 500 ms; invalid packets do not renew it. MANUAL retains RC deadbands/scaling and relies on receiver failsafe, with no MCU RC signal-loss timeout. Inverse kinematics remains on the Teensy.

MD200T PID 207 controls two closed-loop signed integer RPM channels. PID 174 sends TQ-OFF when outputs are disabled. Motor telemetry uses PID **196/201**, with raw signed RPM, status, encoder count, and MCU RX timestamps. Voltage PID 143 uses explicit requests, MID 7 responses. These files, wire layouts, telemetry decoding, driver settings, and polarity conventions are preserved:

| Driver | CAN command ID | CH1 | CH2 |
| --- | --- | --- | --- |
| A | 1 | FL | RR |
| B | 2 | FR | RL |

No new shaft direction inversions are applied. Positive rolling RPM is sent with the existing positive channel convention. Confirm the installed driver direction configuration at low speed before running with steering enabled.

`include/can_tx_schedule.hpp` and the scheduler in `main.cpp` retain their existing 10 ms cycle: A at cycle start, B at least 3 ms after actual A TX, followed by B's full 3 ms response window. Early responses do not shorten those windows; late loops skip missed cycles without bursts. Voltage requests occur at +6 ms on cycles 0 and 50 of the 100-cycle superframe: each driver gets one per second. No new CAN queries or transactions were added. The only scheduler logic change cancels a pending enabled B command when `motor_enabled` becomes false between A and B, including steering startup inhibition or host disable. This closes a stale-snapshot safety gap without changing transaction timing.

Existing safety paths remain: host timeout, enable/E-stop, RC STOP/POWERDOWN, CAN TX failure and driver RX timeout reporting/output inhibition, raw driver faults, low voltage, and relay startup delay. Communication timeouts never trigger an automatic driver reset. A normal enabled zero command uses PID 207 and deceleration; disable/E-stop/timeout use the existing PID 174 TQ-OFF path. TQ-OFF is **not an independently verified mechanical emergency brake**.

### Verified documentation discrepancies and limitations

- Existing skid-steer mixing used **0.485 m**, rather than the new measured-model track of 0.439 m. Skid mode keeps 0.485 m for compatibility; independent mode uses 0.420 m wheelbase and 0.439 m front/rear tracks.
- The scheduler header previously described automatic PID 196/201, while bridge comments and some prose describe PID 193/200. Control actually consumes 196/201. The bridge/benchmark matcher still uses its existing 193/200 assumptions; this refactor does not reconcile driver return configuration or alter either path.
- The watchdog diagnostic has no active detection source, and there is no newly added MCU watchdog. It would be inaccurate to claim an existing watchdog was verified.
- Firmware did not configure driver acceleration/deceleration or implement a host RPM slew limit. Driver settings remain untouched; the new host limits are additional commissioning constraints, not measured driver behavior.
- CAN RX timeout and fault inhibition follows the existing ten-status-report expiration policy; it is not a new latched fault or recovery handshake. Voltage stale and RX-overrun diagnostics keep their existing behavior.
- Bridge/benchmark paths retain their existing diagnostic behavior, including raw commands that bypass normal motion constraints. Use them with the vehicle secured; steering readiness does not constrain raw CAN bridge commands.

## Added components

| File | Responsibility |
| --- | --- |
| `include/vehicle_kinematics.hpp` | Geometry, velocity validation, skid and independent IK, whole-command feasibility rejection |
| `include/steering_config.hpp` | Separate servo tables, neutral, inversion, safe pulse/angle bounds, PWM frame frequency, motion configuration |
| `include/vehicle_motion.hpp` | Startup gate, mode guard, steering tracking, brake/slew/settle state machine, coordinated RPM limits |
| `src/main.cpp` | Existing command/safety handling, mapping motion outputs to unchanged CAN scheduler, hardware PWM, additive diagnostics |
| `test/test_vehicle_motion.cpp` | Required numerical cases and reconstruction of signed wheel velocity vectors |
| `test/test_motion_transitions.cpp` | Synthetic calibration, startup, direction reversal, fast command changes, infeasibility, mode guards, limits, time wrap |
| `test/test_firmware_odometry.cpp` | Existing CAN/status regressions plus firmware startup/E-stop/timeout/mapping/spacing integration |

All firmware state uses fixed-size storage. No new packages, heap allocations, blocking waits, or interrupt masking were added. PWM uses Teensy core `analogWriteFrequency` and 16-bit `analogWrite` on pins 9 and 10. Both pins share FTM0 and use **50 Hz** (a configurable commissioning assumption). FTM0 PWM frequency changes affect other PWM channels on that timer; existing relay pins 5/6 use digital GPIO, and RC capture uses edge interrupts. PWM duty resolution is about 0.305 microseconds at 50 Hz; verify the actual frame/pulse timing on a scope.

## Kinematics and feasibility

The reference is the rear axle center: +X forward, +Y left, +omega counterclockwise. For front wheel `(x,y)`, `(vx,vy)=(v-omega*y,omega*x)`. Compute `atan2(vy,vx)`, normalize into ±90 degrees by reversing the signed rolling speed when necessary, then test against each independently calibrated operating range. Rear speeds are `v ± omega*rear_track/2`. Float RPM is `speed*60/(2*pi*0.065)`.

The theoretical range is ±70 degrees. The stricter configured per-wheel calibration limits govern actual feasibility. Neither motor reversal nor common scaling can fix infeasible curvature. An infeasible command sets all target RPMs to zero and holds the generated steering angles. Existing wheel commands decelerate under the host limiter. No arbitrary angle clamp or high-level command interpolation is used. This policy may stop the vehicle for requests close to the infeasible wedge; it does not provide a closest feasible path, guaranteed braking distance, or tire-slip compensation. The pure `inverse()` function is the replacement point for a future command projection policy.

Below 0.001 m/s front-vector magnitude, hold that wheel's previous generated steering and set its RPM to zero. This avoids meaningless `atan2(0,0)` and direction noise below the integer RPM resolution. Theoretical resolution is approximately 0.006807 m/s per RPM.

Representative theoretical ±70 degree results (before acceleration/transition limits):

| v | omega | FL angle | FR angle | Result |
| ---: | ---: | ---: | ---: | --- |
| 0.5 | 0 | 0° | 0° | four equal positive RPMs |
| -0.5 | 0 | 0° | 0° | four equal negative RPMs |
| 0.5 | 1 | ~56.26° | ~30.27° | feasible left turn, distinct wheel speeds |
| 0 | 1 | ~-62.41° | ~62.41° | left RPM negative, right positive; front/rear speed ratio ~2.159 |
| 0.2195 | 1 | infeasible | — | reject whole command |
| 0 | 0 | retained | retained | zero RPM |

The ±20 degree placeholder configuration intentionally rejects turns that require greater angles, including rear-axle-centered rotation. These numerical cases do not establish motor shaft direction on the physical vehicle.

## Motion coordination

`MotionController` tracks previous generated, target, and current generated front angles. `Transition` exposes STARTUP, DRIVING, BRAKING, STEERING, and SETTLING. These are command/estimate states, never measured steering position.

A nontrivial wheel direction reversal or an infeasible request triggers BRAKING; steering-angle error alone does not. The former 5-degree braking threshold and its configuration have been removed. Both commanded steering angles use a common progress fraction, with the larger remaining travel limited to 180 degrees/s, so fixed targets finish together. During braking, steering holds until generated RPM reaches zero and the 200 ms stopped dwell completes. The 500 ms settling stage remains for startup/re-enable/mode-change/reversal transitions; ordinary steering while DRIVING does not enter it. These are generated commands, not measured servo arrival times.

Actual unsigned millisecond elapsed time controls slew/ramping. Each increment caps elapsed time at 50 ms after a loop stall, preventing an unsafe catch-up jump; settling/startup timers use full elapsed time and survive 32-bit clock wrap. RPM target limiting uses one common scale to preserve target ratios. Acceleration/deceleration uses one interpolation fraction constrained by every wheel's rate, without per-wheel sign jumps. Quantization occurs only at the final CAN boundary with nearest-integer rounding and finite-value checking; there is no dithering.

Defaults are **200 RPM**, 100 RPM/s acceleration, and 200 RPM/s deceleration. The motor manufacturer's [in-wheel motor specification](https://www.mdrobot.co.kr/inwheel-motor) lists MDH100 at 24 V, 400 RPM, 100–200 W. The 200 RPM commissioning limit is deliberately below that specification; confirm the installed variant's rated/maximum speed and acceptable loaded acceleration before increasing it. Configuration above the published 400 RPM ceiling is rejected and the final conversion also enforces that ceiling. MD200T's general numeric range is not used as the motor limit. Host disable, E-stop, timeout, and existing driver faults bypass ordinary ramping and clear generated RPM state while the existing CAN safety path inhibits drive.

Both commanded-zero RPM and elapsed settling are estimates of physical standstill/completion. They do not prove wheel deceleration or actual servo arrival. Conservative dwell/rates must be established with the real loaded vehicle. Startup assumes that a verified neutral pulse and a configured **1500 ms** initialization interval are sufficient; set that interval from measured worst-case startup travel. If that procedure cannot establish steering readiness reliably, leave calibration unverified and drive remains inhibited.

## Calibration and mode commissioning

1. Secure/lift the vehicle and isolate drive torque. Independently establish each servo's safe PWM frame rate, pulse limits, neutral, mounting inversion, clearance, and physical angle response with appropriate bench equipment. The firmware intentionally does not provide an unverified live steering sweep.
2. Replace `left_servo` and `right_servo` independently in `steering_config.hpp`. The provided 1500/1300/1700 microsecond values are **disabled placeholders**, not RDS51150 calibration. Mark `verified=true` only after measuring safe neutral and mapping. Leaving either false emits no servo PWM and inhibits normal drive in both modes.
3. Tables have 3–5 strictly increasing actuator-angle / pulse points, including exact zero with `neutral_us`; physical inversion is applied before lookup. This supports nonlinear piecewise interpolation. Physical min/max operating angles must lie inside table coverage. Expand beyond the initial ±20 degrees only after clearance and mapping verification; absolute software bounds remain ±70 degrees.
4. Verify PWM timing, startup neutral travel, minimum required settling time, direction signs, loaded braking/dwell, and small-angle coordination. Driver relay powerdown does not remove servo power; power supply/GND and actual servo power state remain hardware responsibilities.
5. `MotionConfig::initial_mode` defaults to `SKID_STEER`; change it to `INDEPENDENT_STEERING` only after calibration. Rebuild/reboot with the vehicle stopped. This first version adds **no runtime USB mode-switch packet** and preserves RC authority semantics.
6. The reusable `request_mode(mode, standstill_confirmed)` rejects changes unless startup is complete, all generated RPMs are zero, and the caller confirms physical standstill. A future host integration must obtain fresh wheel telemetry/another reliable standstill source before passing true, and must run a zero command through neutral slew/settling before driving in skid mode. Do not use the generated RPM estimate alone as standstill confirmation.

## Diagnostics and protocol compatibility

`0x01` command and `0x82` System Status v2 byte layouts and versions are unchanged. Previously unused System Status error bits now provide live motion diagnostics:

| Bit | Meaning |
| ---: | --- |
| 12 | Last evaluated enabled command was infeasible/invalid |
| 13 | Steering startup/calibration not ready; normal drive inhibited |
| 14 | Steering/drive transition state is not DRIVING |
| 15 | Last evaluated target was common-scaled by RPM limit |

Bits 12–15 are live state, not ten-report historical errors. A rejected command uses enabled zero-RPM deceleration and may therefore retain MCU state ENABLED; read the diagnostics rather than assuming ENABLED means vehicle motion. Disable retains the last feasibility/limit result until another enabled command is evaluated. Legacy `0x81` bits remain unchanged and its periodic transmission remains disabled.

An optional steering status packet `0x83` is **disabled by default** to preserve existing host stream behavior. Set `MotionConfig::publish_steering_status=true` only after verifying host framing/unknown-type handling. When enabled, it is offered at most twice a second, after existing CAN and `0x82` handling. It is skipped when USB TX space is insufficient and is absent in bridge mode. Host readers must frame by length and ignore unknown packet types; a reader that assumes every frame is exactly 63 bytes must be updated before deployment.

| Byte | Field |
| --- | --- |
| 0–3 | `AA 55 0E 83` (18 bytes total) |
| 4 | version = 1 |
| 5 | vehicle mode: 0 skid, 1 independent |
| 6 | transition: 0 startup, 1 driving, 2 braking, 3 steering, 4 settling |
| 7 | flags: bit 0 ready estimate, bit 1 feasible, bit 2 RPM limited |
| 8–11 | uint32 LE MCU milliseconds |
| 12–13 | int16 LE FL generated steering angle, hundredths of a degree |
| 14–15 | int16 LE FR generated steering angle, hundredths of a degree |
| 16 | reserved = 0 (no feedback) |
| 17 | XOR of bytes 0–16 |

Angles are explicitly **commanded**, not measured. Readiness and transition completion are estimates without servo feedback.

## Validation

Run all C++ host harnesses and the existing Python tests:

```bash
bash test/run_host_tests.sh
```

Compile the actual target with the installed PlatformIO environment:

```bash
pio run -e teensy32
```

Host tests cover cases A–H and J, calibration rejection/nonlinear/inverted mappings, limits and rounding, time wrap, mode guard, E-stop/timeout integration, wheel channel mapping, and the existing CAN transaction-spacing/status/telemetry regressions for case I. They do not validate CAN electrical behavior, PWM accuracy, servo performance, loaded braking, tire scrub, receiver failsafe, or ROS host handling of additive diagnostics. Repeat secured low-speed hardware tests and communication monitoring before driving.
