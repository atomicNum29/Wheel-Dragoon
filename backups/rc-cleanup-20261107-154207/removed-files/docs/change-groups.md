# 변경 그룹 메모 — 2026-11-07 기록

> 과거 커밋 분리와 검증 결과를 보존한 메모다. 현재 작업 트리 상태나 실행 지시가 아니다.
> 최신 문서: [문서 안내](../../../../docs/README.md).

현재는 main 기능을 모듈로 분리했고 빌드 환경은 네 개다. 조향 세부 계측 26레코드,
`0x83`, 추가 조향 오류 비트는 제거됐으며 타이밍은 22레코드다. 활성 `test/`도 제거됐다.
아래 테스트·빌드 성공 수치는 당시 분리 트리의 기록이며 현재 소스 검증으로 인용하지 않는다.

<details>
<summary>당시 변경 그룹과 staging 메모</summary>

# Pending change groups

2026-11-07. CAN group committed as `3062dad` (`fix(can): buffer RX with FIFO and an interrupt-driven queue`).
The isolated CAN tree passed the Teensy target build and both focused CAN host tests.
Windows CLI committed as `3514ee0` (`fix(cli): support Windows serial and console input`).
Common timing committed next as `025b281` (`feat(diagnostics): profile control stages and CAN response loss`).
The isolated Windows tree passed 26 CLI tests. The common timing tree passed 28 CLI
tests, the timing harness/22-record fixture and both Teensy build environments.
Steering, steering-specific timing probes and existing user changes remain in the working tree.
Do not stage all of `src/main.cpp`, `test/fakes/Arduino.h`,
`test/test_firmware_odometry.cpp`, README or the shared test runner as a single feature.

## 1. CAN controller / receive path

Suggested subject: `fix(can): buffer RX with FIFO and an interrupt-driven queue`

- `include/flexcan0.hpp`, `src/flexcan0.cpp`: final FIFO/ISR/queue implementation,
  timestamp API, pending/loss counters and driver-local stats.
- `test/flexcan_fakes/`, `test/test_flexcan_rx.cpp`: register/FIFO/IRQ model.
- `test/test_can_rx_queue_integration.cpp`: main-loop deadline integration.
- CAN-only hunks of `src/main.cpp`: queue capacity drain bound; queued processing
  helper and ISR timestamp use; final drain before CONTROL/benchmark timeout;
  rejection of pre-command benchmark entries. Exclude timing probes and motion/PWM.
- CAN API stub adaptation in `test/test_firmware_odometry.cpp`; exclude steering
  policy tests. Add CAN test targets in the shared runner separately.
- `docs/can-rx-fifo.md`.

The driver owns its counters and has no dependency on `timing_diagnostics.hpp`.
Earlier standalone-mailbox acknowledge/BUSY changes have been superseded by FIFO;
do not create a separate intermediate mailbox commit from the final working tree.

## 2. Timing analysis / diagnostic tooling

Suggested subject: `feat(diagnostics): profile control stages and CAN response loss`

- `include/timing_diagnostics.hpp`, `test/test_timing_diagnostics.cpp`,
  `test/check_timing_fixture.py`, `docs/timing-diagnostics.md`.
- Main-loop probes, response/timeout diagnostic counters, DWT setup and bounded
  USB diagnostic publishing hunks of `src/main.cpp`.
- Profile environment in `platformio.ini`, related fake DWT/USB support and runner hunks.
- CLI 0x84 decoder, timing-only option and relevant tests/docs/README links.
- Windows terminal/serial portability changes in `src/can_cli.py` and corresponding
  tests/docs can be a separate tooling commit rather than mixed into the CAN driver.

Dependency: common CAN diagnostics consume counters from group 1 and have now been
committed independently of steering. Stage 11 is reserved with zero samples in the
common commit (22 records). The active steering probe and four additional substages
remain with group 3 in the working tree (26 records); existing stage IDs are preserved.

## 3. Steering system

Suggested subject: `feat(steering): add calibrated motion and servo control`

- `include/steering_config.hpp`, `include/vehicle_kinematics.hpp`,
  `include/vehicle_motion.hpp`, steering documents and motion tests.
- Motion state, mode/transition handling, command mapping, steering safety/status,
  PWM setup/output and steering error bits in `src/main.cpp`.
- Steering-related fake Arduino functions and existing odometry test changes.
- The calculation helper split (`pulse_for_valid_angle`) belongs to steering; timing
  probes around validation/calculation/PWM belong to group 2. For an independent
  steering commit, use the original validated `pulse_for_angle` wrapper until the
  substage diagnostic commit is applied.

The current split leaves PWM enabled and the 26-record timing implementation in the
working tree for hardware comparison. No calibration validation caching or steering
algorithm optimization was included. RC mapping changes also remain uncommitted.

## Later staging procedure

Use patch/hunk staging or construct isolated commits against HEAD; shared files contain
multiple features and cannot be separated by filename alone. Check that each proposed
commit builds independently before committing. Completed order is CAN, Windows CLI,
common timing. Steering and its additional timing probes can be reviewed next.
Keep unrelated editor settings in `.vscode/extensions.json` outside these feature commits.

</details>
