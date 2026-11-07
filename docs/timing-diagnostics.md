# 공통 처리시간 및 CAN 응답 진단

일반 CONTROL 루프의 단계별 경과시간과 CAN 응답 누락을 관측한다.
이 문서는 조향 확장에 의존하지 않는 공통 계측을 설명한다. 기존 주행 계산과
3 ms 응답 제한을 유지하며, CAN 수신은 [FIFO·ISR·RAM 큐](can-rx-fifo.md)를 사용한다.

## 빌드와 관측

기본 `teensy32` 빌드에서는 시간 프로브와 `0x84` 출력을 비활성화한다.
계측은 `teensy32_profile` 환경에서만 활성화된다.

```powershell
pio run -e teensy32_profile
```

산출물은 `.pio/build/teensy32_profile/firmware.hex`다. 해당 펌웨어를 장비에
적용한 뒤 실제 COM 포트로 다음을 실행한다. 이 명령 자체는 업로드하지 않는다.

```powershell
uv run python src/can_cli.py --port COM5 --timing-only
```

`--timing-only`는 출력 필터다. 시작 시 요청을 보내거나 BRIDGE로 전환하지 않는다.
`benchmark`/`pid`/`frame` 명령은 기존처럼 모드를 바꾸므로 일반 주행 계측 중에는
입력하지 않는다. RC MANUAL 주행은 기존 RC로 조작한다.

AUTO 주행에는 기존 호스트의 주기적 제어 명령이 필요하다. CLI를 열기 위해 ROS 등
기존 연결을 끊으면 AUTO 조건이 달라진다. 같은 COM 포트를 두 프로그램이 동시에
열 수 없으므로 AUTO 환경을 유지하려면 기존 호스트가 `0x84`를 길이 기준으로
분리/기록해야 한다. 그 호스트의 호환성은 검증하지 않았다. `0x01`/`0x82` 정의는 유지한다.

## 측정값

통계는 **부팅 이후 누적**이며 스냅샷마다 초기화하지 않는다. 공통 구현은
요약 1개 + stage 15개 + wheel 4개 + driver 2개, 총 **22개 레코드**를 출력한다.
같은 `snapshot`의 레코드는 같은 시점에 복사한 값이다. `at_ms`는 복사 시점의
MCU `millis()`이며 PC 출력 시각과 다르다. 조건 비교는 재부팅하여 시작한다.
CONTROL/BRIDGE 양쪽에서 누적하지만 출력은 CONTROL에서만 한다.

| ID / stage | 계측 범위 |
| --- | --- |
| 0 `power` | RC 모드 기반 릴레이/부팅 대기/제어 세션 전환 |
| 1 `serial_rx` | USB 명령 수신·파싱·처리 |
| 2 `bench_tx_pre` | RX 전 벤치마크 결과 전송 시도 |
| 3 `can_rx` | 메인 루프의 CAN 큐 drain 및 프레임 처리 |
| 4 `bench_update` | 벤치마크 송신/타임아웃 처리 |
| 5 `bench_tx_post` | 벤치마크 갱신 후 결과 전송 시도 |
| 6 `health` | telemetry freshness 및 저전압 상태 확인 |
| 7 `motion` | 기존 유효 주행 명령 및 휠 목표 속도 결정 |
| 8 `errors` | 오류 보고 상태 갱신 |
| 9 `can_schedule` | 일반 CAN 스케줄, 응답 창 마감, 실제 송신 대기 포함 |
| 10 `system_tx` | 기존 `0x82` 상태 직렬화/전송 시도 |
| 11 `steering` | 예약 ID. 공통 구현에서는 호출하지 않아 값이 0 |
| 12 `diagnostic_tx` | 스냅샷 복사/직렬화/출력 및 출력 가능 여부 검사 |
| 13 `loop_body` | `loop()` 진입부터 진단 처리 종료까지 |
| 14 `loop_period` | 연속 `loop()` 진입 간격; Arduino 루프 밖 처리 포함 |

각 stage는 `n`, `avg_us`, `max_us`, `total_ms`를 표시한다. CPU DWT 사이클을
측정하고 패킷의 `F_CPU`로 단위를 변환한다. FTM0, CAN, SysTick을 재설정하지 않는다.
**인터럽트 선점과 동기 대기를 포함한 경과시간**이며 순수 CPU 사용량이나 ISR별
실행시간은 아니다. 즉시 반환한 호출도 평균에 포함된다. `loop_body`/`loop_period`는
다른 단계를 포함하므로 단계별 시간에 중복 합산하지 않는다.

`rx_poll_gap_us`는 메인 루프 `md200t_drain_rx()` 진입 간격이다. FIFO는 ISR이
읽으므로 이 값은 하드웨어를 읽지 못한 시간이나 프레임 도착 간격, CAN RTT가 아니다.
합계는 64비트, 횟수와 최대값은 32비트다. 단일 DWT 측정은 1회 wrap보다 짧아야
한다(72 MHz에서 약 59.7초). 디버거 중단/긴 정지는 정상 처리시간으로 해석하지 않는다.

## CAN 누락 구분

`A/PID196`, `A/PID201`, `B/PID196`, `B/PID201`별로 집계한다.

- `received`: CONTROL 파서에 도달한 해당 PID 프레임 수. 버스 도착 수가 아니다.
- `accepted`: DLC와 응답 창 조건을 통과한 수.
- `outside_window`: DLC는 충분하지만 응답 창 조건으로 폐기한 수.
- `short_frame`: DLC < 8로 폐기한 수. `outside_window`와 중복 집계하지 않는다.
- `max_read_age_us`: command-sent 상태에서 송신 호출 전 시각부터 ISR이 FIFO에서
  읽은 시각까지의 최대 차이. 수용/폐기 프레임 모두 포함한다. IRQ 지연과 FIFO 체류는
  포함하고 RAM 큐 대기는 제외한다. 응답 sequence가 없어 동일 요청의 RTT라고 보증하지 않는다.
- `missing`: 응답 창 마감 시 해당 PID가 빠져 있던 횟수.
- `last_rx_us`: 마지막 프레임의 ISR 읽기 시각. `micros()` wrap을 고려한다.

드라이버별 `timeouts`는 창 마감 사건 수이며 기존 10회 반복 오류 보고 수가 아니다.
`last_missing_mask`는 `0x01=A196(FL)`, `0x02=B196(FR)`, `0x04=B201(RL)`,
`0x08=A201(RR)`이다. `last_age_us`는 송신부터 마지막 timeout 처리까지의 시간,
`last_timeout_us`는 그 처리 시각이다.

`pending_at_timeout`/`last_pending`은 마감 전 최종 큐 drain 후 `can_rx_pending()`으로
관측한 FIFO 또는 RAM 큐 대기 상태다. 진단 프로브는 프레임을 소비하거나 FIFO의
잠금/해제 레지스터에 접근하지 않는다. 대기 중이었다고 해서 누락 PID의 응답이라고
특정할 수는 없다. ISR과 관측이 동시에 진행할 수 있으므로 사건의 완전한 캡처도 아니다.

`overrun_observations`는 현재 CAN 드라이버의 FIFO overflow 관측과 RAM 큐 drop
누계다. 정확한 하드웨어 유실 프레임 수나 PID별 유실 수는 아니다. 옛 단일 메일박스의
OVERRUN CODE 계측과 의미가 다르다. FIFO/큐별 통계 API는 CAN 드라이버가 소유한다.

단계 최대값과 누락 횟수는 누적치이므로 같은 사건에서 발생했다고 보증하지 않는다.
정확한 wire 도착 시각과 인과관계에는 CAN 캡처 등 별도 증거가 필요하다.

## 출력 부하와 스냅샷 지속

- 최대 1초에 한 스냅샷: 공통 구현은 63 bytes × 22 = 1,386 bytes다.
- 한 번에 한 패킷, 패킷 간 최소 20 ms. CONTROL에서만 출력한다.
- 정상 주행은 cycle +7..10 ms에서 A/B 응답 창과 전압 조회 처리가 끝나야 출력한다.
  POWERDOWN/부팅 대기에는 응답 창이 없어 이 조건을 생략한다.
- USB 공간이 부족하면 다음 loop로 미루고 미완료 스냅샷을 보존한다. 따라서 출력이
  1초보다 늦어질 수 있다. 스냅샷 4개 제한은 없다.
- 현재 USB 패킷에 1..62 bytes만 남으면 `Serial.send_now()`로 부분 패킷을 큐에
  넘긴 뒤 공간을 다시 확인한다. 호스트 수신을 기다리거나 패딩을 넣지 않는다.
- 프로브/출력에도 비용이 있다. `diagnostic_tx`와 loop 시간에 영향이 포함되며
  프로브 없는 원래 펌웨어의 실행시간을 그대로 측정한 것은 아니다.

초기 구현은 63-byte 공간만 검사했다. Teensy의 64-byte USB 패킷에 일반 상태가
cycle +6 ms에 쓰이면 1 byte만 남아 약 5 ms 자동 flush 전에는 +7..10 ms 진단
출력 창을 놓칠 수 있었다. 부분 패킷을 큐에 넘기는 처리는 이 출력 기아를 해소한다.
이는 CAN 타임아웃 원인 자체에 대한 수정과는 별개다.

## 패킷 `0x84`, version 1

총 63 bytes, 시작 `AA 55 3B 84`, 정수는 little-endian이다.

| Offset | 내용 |
| --- | --- |
| 4 | version=1 |
| 5 | kind: 0 summary, 1 stage, 2 wheel, 3 driver |
| 6 | kind 안의 index: stage 0..14, wheel A196/A201/B196/B201, driver A/B |
| 7 | snapshot serial mode (출력은 CONTROL=0) |
| 8..11 | snapshot sequence, u32 |
| 12..15 | snapshot millis, u32 |
| 16..19 | F_CPU, u32 |
| 20..61 | kind별 데이터, 미사용 bytes=0 |
| 62 | bytes 0..61 XOR |

Stage: 20=count(u32), 24=total cycles(u64), 32=max cycles(u32).
Summary: 20=RX gap count(u32), 24=gap total us(u64), 32=gap max us(u32),
36=overrun observations, 40=CONTROL loops, 44=BRIDGE loops(각 u32), 48=record count(u8).
Wheel: offset 20부터 u32 7개, received/accepted/outside_window/short_frame/
max_read_age_us/missing/last_rx_us 순서다.
Driver: offset 20부터 u32 6개, timeouts/pending_at_timeout/last_missing_mask/
last_age_us/last_timeout_us/last_pending 순서다.

향후 조향 세부 stage는 기존 ID를 바꾸지 않고 뒤에 추가한다. 레코드 수는 summary의
값을 사용한다. 이 공통 커밋에는 조향 알고리즘, PWM 초기화/출력 변경이 없다.

## 직접 관련된 검증

조향 파일을 제외한 커밋 구성에서 다음을 확인한다. 실물 COM/CAN 시험과 업로드는
포함하지 않는다. 호스트 모형은 실제 장비 성능이나 무손실 수신을 보증하지 않는다.

```powershell
New-Item -ItemType Directory -Force .pio/build
g++ -std=c++11 -Wall -Wextra -Werror -pedantic -Iinclude -Itest/fakes test/test_timing_diagnostics.cpp src/md200t_can.cpp -o .pio/build/test_timing_diagnostics.exe
.\.pio\build\test_timing_diagnostics.exe .pio/build/timing_fixture.bin
uv run python test/check_timing_fixture.py .pio/build/timing_fixture.bin
uv run python -m unittest discover -s test -p test_can_cli.py
pio run -e teensy32 -e teensy32_profile
```

C++ harness는 counter wrap, 64-bit 합계, 응답 수용/폐기/누락, 비소비 pending 관측,
USB 공간/출력 창/간격 제한, frozen snapshot을 검사한다. +6 ms 상태 트래픽과
5 ms USB flush를 모형화하여 snapshot 5..8의 모든 레코드가 이어지는지도 검사한다.
Python fixture 검사는 C++가 만든 22개 레코드의 분할 수신·단위·순서·체크섬을 확인한다.

2026-11-07 분리된 공통 커밋 구성에서 C++ harness, 22개 fixture 레코드,
CLI 테스트 28개와 `teensy32`/`teensy32_profile` 빌드가 모두 통과했다.
장비 업로드와 실물 통신 시험은 실행하지 않았다.
