# Wheel-Dragoon

Teensy 3.2 기반 4륜 skid-steer 로봇 MCU 펌웨어입니다. 목표 하드웨어는 MDROBOT `MDH100` 인휠 모터 4개와 `MD200T` 듀얼채널 모터 드라이버 2개입니다. ROS와 Teensy 사이의 USB Serial(UART) command/status packet 연결은 유지하고, Teensy는 계산한 휠 목표 속도를 CAN bus로 두 개의 MD200T에 전달합니다.

이 README는 MD200T/CAN 전환 목표 구조와 ROS 패키지 연동 프로토콜을 함께 기록합니다. ROS용 command packet 파싱과 basic status packet 송신은 기존 `src/main.cpp` 구현을 기준으로 유지합니다.

## 주요 기능

### 유지되는 기능

- RC PWM 입력 기반 수동 주행
- ROS-UART 실제 속도 `int16` command packet 파싱
- 상태머신 기반 USB Serial command packet 수신
- XOR checksum 검증
- `flags`의 enable 및 emergency_stop 처리
- Basic Status Packet 20 Hz 송신
- command timeout 감지 및 timeout 시 모터 목표 RPM 0 처리
- state/error bitfield 산출
- 차동 구동식 기반 좌/우 바퀴 목표 RPM 계산
- 10 ms 주기 MD200T command 송신 스케줄링

### MD200T/CAN 전환 목표

- MD200T 2대 CAN 제어
- MDH100 4개 휠 목표 속도 명령 분배
- 드라이버별 2채널 속도/enable/stop 명령 송신
- Stop, disable, emergency stop, command timeout 시 모든 MD200T 채널 정지 명령 송신

### 아직 구현되지 않음

- `seq` 기반 응답/상태 동기화
- MD200T CAN bitrate, CAN ID, command/status frame format 확정
- Teensy 3.2 CAN transceiver 배선 및 CAN 라이브러리 선정
- 채널별 direction polarity 검증
- 실제 MD200T CAN frame 송신 구현
- 실제 배터리 전압 ADC 측정
- 드라이버 fault, 과전류, 과열, 파라미터 오류 감지 입력

## 하드웨어 기준

전환 목표는 다음 하드웨어 구성을 기준으로 합니다.

| 항목 | 값 |
| --- | --- |
| 차폭 `W` | `0.42 m` |
| 바퀴 반지름 `R` | `0.14 m` |
| 보드 | Teensy 3.2 (`teensy31`) |
| 프레임워크 | Arduino |
| 모터 | MDROBOT `MDH100` x 4, 24 V급, 100-200 W |
| 모터 드라이버 | MDROBOT `MD200T` x 2, 12-48 V, 10 A x 2 ch |
| Teensy-MD200T 통신 | CAN bus |

## 핀맵

### RC 입력

| 기능 | 핀 | 설명 |
| --- | --- | --- |
| 각속도 입력 `w` | `0` | RC PWM 입력 |
| 선속도 입력 `v` | `1` | RC PWM 입력 |
| 모드 선택 | `2` | RC PWM 입력 |

### CAN / MD200T 연결

| 기능 | 연결 | 설명 |
| --- | --- | --- |
| Teensy CAN TX/RX | TBD | Teensy 3.2의 실제 CAN 핀은 구현 시 확인 |
| CAN transceiver | TBD | Teensy와 MD200T 사이에 CAN transceiver 필요 |
| MD200T CAN_H/CAN_L | CAN bus | 두 MD200T를 같은 CAN bus에 연결 |
| CAN 종단저항 | TBD | bus 양 끝단 기준으로 적용 여부 확인 |
| GND 공통 | Teensy / transceiver / MD200T | 통신 기준 전위 공유 |

## 동작 모드

모드 채널의 PWM 평균값으로 주행 모드를 선택합니다.

| PWM 범위 | 모드 | 동작 |
| --- | --- | --- |
| `< 1300 us` | Stop (`DRIVE_MODE_STOP`) | 좌/우 목표 RPM을 `0`으로 설정 |
| `1300-1699 us` | Manual (`DRIVE_MODE_MANUAL`) | RC `v`, `w` 입력으로 주행 |
| `>= 1700 us` | Auto (`DRIVE_MODE_AUTO`) | ROS-UART command packet으로 `v_milli_mps`, `w_milli_radps` 수신 |

Manual 모드에서 RC PWM은 다음 범위로 변환됩니다.

- `v = (v_pwm - 1500) / 250` → 대략 `-2 ~ 2 m/s`
- `w = (w_pwm - 1500) / 100` → 대략 `-5 ~ 5 rad/s`

현재 펌웨어는 좌/우 바퀴 속도를 다음 식으로 계산한 뒤 RPM으로 변환합니다.

```text
left_velocity  = v - w * W / 2
right_velocity = v + w * W / 2
```

Auto 모드에서는 ROS 노드가 `/cmd_vel`의 실제 속도를 milli-unit `int16` 값으로 변환해 보냅니다. MCU는 packet 값을 다음처럼 실제 속도로 환산합니다.

```text
v = v_milli_mps / 1000.0          # m/s
w = w_milli_radps / 1000.0        # rad/s
```

MCU는 기본 주행 제한으로 `v_milli_mps`를 `-2000 ~ +2000`, `w_milli_radps`를 `-5000 ~ +5000` 범위로 검증합니다. 프로토콜 범위를 벗어난 값은 송신 측 오류로 보고 `COMMAND_OUT_OF_RANGE`로 반영합니다.

## 구동 명령 흐름

MD200T/CAN 전환 후의 목표 명령 흐름은 다음과 같습니다.

```text
ROS /cmd_vel
  -> ROS node: v_milli_mps, w_milli_radps packet 생성
  -> USB Serial(UART)
  -> Teensy 3.2: command packet 파싱
  -> Teensy 3.2: v, w 환산 및 skid-steer 좌/우 RPM 계산
  -> Teensy 3.2: 4개 휠 목표 RPM으로 분배
  -> CAN bus
  -> MD200T 2대: 각 2채널 MDH100 속도 제어
```

Teensy는 좌/우 목표 RPM을 다음 휠 명령으로 복제합니다.

| 휠 | 목표 RPM |
| --- | --- |
| LF | `left_rpm_ref` |
| LR | `left_rpm_ref` |
| RF | `right_rpm_ref` |
| RR | `right_rpm_ref` |

Stop, disable, emergency stop, command timeout 상태에서는 네 개 채널 모두에 zero/disable 명령을 송신해야 합니다.

## 드라이버/모터 매핑

두 개의 MD200T는 대각선으로 묶인 모터를 담당합니다.

| 드라이버 | CAN ID | CH1 | CH2 | 비고 |
| --- | --- | --- | --- | --- |
| MD200T A | TBD | LF | RR | 대각 페어 |
| MD200T B | TBD | RF | LR | 대각 페어 |

각 채널의 `direction polarity`는 실제 배선 후 검증해야 합니다. CAN bitrate, CAN ID 설정 방식, command/status frame format은 MD200T CAN 매뉴얼 확인 후 확정합니다.

## ROS 연동 목표 책임 분리

다음 내용은 ROS 패키지와 MCU 펌웨어가 가져야 할 책임 경계입니다. ROS와 Teensy 사이의 packet protocol은 유지하고, Teensy 이후의 모터 출력 계층만 MD200T CAN 제어로 교체합니다.

### ROS 노드 책임

- `/cmd_vel` 구독
- `linear.x`, `angular.z`를 milli-unit `int16` 실제 속도 명령으로 변환
- 고정 주기로 MCU에 명령 패킷 송신
- MCU 상태 패킷 수신
- 파싱한 상태를 `~/status`에 `std_msgs/msg/String`으로 발행
- 명령 타임아웃 및 종료 시 disable/zero 명령 송신
- Serial 포트가 닫히면 재연결

### MCU 책임

- `v_milli_mps`, `w_milli_radps`를 `v`, `w` 실제 속도로 환산
- skid-steer 모터 믹싱으로 좌/우 목표 RPM 계산
- 좌/우 목표 RPM을 LF/LR/RF/RR 4개 휠 명령으로 분배
- MD200T A/B의 2채널 CAN 명령 송신
- Basic Status Packet 주기 송신
- command timeout 시 `TIMEOUT_STOP` 및 `COMMAND_TIMEOUT` 송신
- Stop, disable, estop, timeout 시 모든 MD200T 채널 zero/disable 명령 강제

## ROS Packet Protocol

다음 프로토콜은 ROS 패키지 연동을 위한 통신 명세입니다. 현재 `src/main.cpp`는 command packet 수신, checksum 검증, basic status packet 송신을 구현합니다.

모든 패킷은 `0xAA 0x55`로 시작합니다. Checksum은 byte 0부터 checksum 직전 byte까지 모든 바이트의 XOR 값입니다.

MCU 수신부는 USB Serial 내부 RX buffer를 `Serial.available()`이 빌 때까지 읽고, 각 바이트를 command frame 상태머신에 전달합니다. 상태머신은 `0xAA 0x55`, length, type 순서로 packet 후보를 조립합니다. 헤더, length, type, checksum 검증에 실패하면 현재 packet 후보를 버리고 다음 header를 기다립니다. 실패한 packet 내부를 재스캔하지 않으므로, 복구는 다음 정상 command packet 주기에 이루어집니다.

### Command Packet: ROS to MCU

ROS 노드가 MCU로 보내는 명령 패킷입니다. 현재 MCU 펌웨어는 이 형식을 파싱합니다.

| Byte | Field | Type | Description |
| --- | --- | --- | --- |
| 0 | `header[0]` | `uint8` | `0xAA` |
| 1 | `header[1]` | `uint8` | `0x55` |
| 2 | `length` | `uint8` | `7` |
| 3 | `type` | `uint8` | `0x01` |
| 4 | `seq` | `uint8` | Sequence counter |
| 5-6 | `v_milli_mps` | `int16 LE` | 목표 선속도 `m/s * 1000`, 기본 허용 범위 `-2000` to `+2000` |
| 7-8 | `w_milli_radps` | `int16 LE` | 목표 각속도 `rad/s * 1000`, 기본 허용 범위 `-5000` to `+5000` |
| 9 | `flags` | `uint8` | bit 0: enable, bit 1: emergency_stop |
| 10 | `checksum` | `uint8` | XOR checksum |

Payload는 `type + seq + v_milli_mps + w_milli_radps + flags`이며, length는 `7`입니다.

```text
AA 55 07 01 seq v_lo v_hi w_lo w_hi flags checksum
```

### Status Packet: MCU to ROS

MCU가 ROS 노드로 20 Hz로 보내는 상태 패킷입니다. 명령 수신 여부와 무관하게 disabled, timeout, fault 상태에서도 주기적으로 송신합니다.

| Byte | Field | Type | Description |
| --- | --- | --- | --- |
| 0 | `header[0]` | `uint8` | `0xAA` |
| 1 | `header[1]` | `uint8` | `0x55` |
| 2 | `length` | `uint8` | `7` |
| 3 | `type` | `uint8` | `0x81` |
| 4 | `seq` | `uint8` | MCU status 송신 counter. 송신할 때마다 1 증가, `255` 이후 `0`으로 wrap |
| 5 | `state` | `uint8` | MCU state |
| 6-7 | `error` | `uint16 LE` | MCU error bitfield/code |
| 8-9 | `battery_mv` | `uint16 LE` | Battery voltage in millivolts. 현재 센서 미구현으로 `0` 송신 |
| 10 | `checksum` | `uint8` | XOR checksum |

Payload는 `type + seq + state + error + battery_mv`이며, length는 `7`입니다.

```text
AA 55 07 81 seq state error_lo error_hi batt_lo batt_hi checksum
```

State 값은 다음 enum 매핑을 사용합니다.

| 값 | State | 의미 |
| --- | --- | --- |
| `0` | `DISABLED` | enable 명령이 없거나 사용자가 disable한 상태 |
| `1` | `ENABLED` | 정상 명령을 받고 있으며 모터 출력이 허용된 상태 |
| `2` | `TIMEOUT_STOP` | 정상 명령 timeout으로 정지한 상태 |
| `3` | `ESTOP` | emergency stop 활성 상태 |
| `4` | `FAULT` | 드라이버 fault 또는 심각 오류 상태 |
| `5` | `BOOTING` | MCU 부팅 또는 초기화 중 |
| `6` | `CALIBRATION` | 보정 또는 설정 동작 중 |

상태 우선순위는 `FAULT > ESTOP > BOOTING > CALIBRATION > TIMEOUT_STOP > DISABLED > ENABLED`입니다.

Error bitfield는 다음 매핑을 사용합니다.

| Bit | 값 | 이름 |
| --- | --- | --- |
| 0 | `0x0001` | `CHECKSUM_ERROR` |
| 1 | `0x0002` | `COMMAND_TIMEOUT` |
| 2 | `0x0004` | `DRIVER_FAULT` |
| 3 | `0x0008` | `EMERGENCY_STOP_ACTIVE` |
| 4 | `0x0010` | `BATTERY_LOW` |
| 5 | `0x0020` | `SERIAL_FRAMING_ERROR` |
| 6 | `0x0040` | `COMMAND_OUT_OF_RANGE` |
| 7 | `0x0080` | `WATCHDOG_RESET_DETECTED` |
| 8 | `0x0100` | `OVER_CURRENT` |
| 9 | `0x0200` | `OVER_TEMPERATURE` |
| 10 | `0x0400` | `PARAMETER_ERROR` |

현재 센서가 없는 항목은 내부 상태값이 `false`로 고정되어 있습니다. `CHECKSUM_ERROR`, `SERIAL_FRAMING_ERROR`, `COMMAND_OUT_OF_RANGE`는 발생 후 다음 status packet에 반영되고, 실제 status 송신 후 latch를 clear합니다. `COMMAND_TIMEOUT`은 정상 command packet을 다시 수신하면 해제됩니다. `EMERGENCY_STOP_ACTIVE`는 estop flag가 해제된 정상 command를 수신하면 해제됩니다.

### Checksum 계산

```python
checksum = 0
for byte in packet_without_checksum:
    checksum ^= byte
```

## 빌드 및 업로드

이 프로젝트는 PlatformIO 프로젝트입니다.

### 사전 준비

- PlatformIO Core 또는 VS Code PlatformIO 확장
- Teensy Loader / Teensyduino 환경
- USB로 연결된 Teensy 3.2

### 빌드

```bash
pio run
```

### 업로드

```bash
pio run --target upload
```

환경 이름은 `platformio.ini`의 `[env:teensy32]`를 사용합니다.

```ini
[env:teensy32]
platform = teensy
board = teensy31
framework = arduino
```

## Python 제어 스크립트

`src/control.py`는 ROS command packet 형식으로 단일 명령을 전송하는 보조 스크립트입니다. `seq`는 기본값 `0`이며, `--seq`로 지정할 수 있습니다.

의존성은 `pyproject.toml`에 정의되어 있습니다.

```bash
uv sync
```

자동 포트 감지로 명령을 보내려면:

```bash
uv run python src/control.py 0.5 0
```

포트를 직접 지정하려면:

```bash
uv run python src/control.py 0.5 0 --port /dev/ttyACM0
```

sequence counter를 지정하려면:

```bash
uv run python src/control.py 0.5 0 --seq 12
```

disable 또는 emergency stop flag를 보내려면:

```bash
uv run python src/control.py 0 0 --disable
uv run python src/control.py 0 0 --estop
```

예시:

| 명령 | 의미 |
| --- | --- |
| `uv run python src/control.py 0.5 0` | `0.5 m/s` 전진 |
| `uv run python src/control.py 0 1.0` | `1.0 rad/s` 제자리 회전 |
| `uv run python src/control.py 0 0` | enable 상태의 정지 명령 |

## 프로젝트 구조

```text
.
├── platformio.ini          # PlatformIO 보드/프레임워크 설정
├── pyproject.toml          # Python 제어 스크립트 의존성
├── src/
│   ├── main.cpp            # Teensy 펌웨어
│   └── control.py          # UART 명령 송신 스크립트
```

## 목표 제어 방식

Teensy는 모터를 직접 구동하지 않습니다. ROS 또는 RC 입력에서 계산한 목표 속도, enable, stop 상태를 MD200T CAN command로 변환해 송신하고, 실제 MDH100 속도 제어는 MD200T 내부 제어기를 사용합니다.

구현 시 확정해야 할 항목은 다음과 같습니다.

- MD200T CAN bitrate
- MD200T A/B CAN ID
- 속도 command frame format
- enable/disable 및 stop command frame format
- MD200T status/fault frame 수신 여부와 error bitfield 매핑
- 채널별 direction polarity

## 주의 사항

- MD200T/CAN 상세 프로토콜은 아직 README에서 확정하지 않습니다. 구현 전 MD200T CAN 매뉴얼로 bitrate, CAN ID, frame format을 확인해야 합니다.
- MD200T A/B의 CH1/CH2 방향 극성은 실제 배선 후 저속 테스트로 검증해야 합니다.
- CAN 통신에는 Teensy와 MD200T 사이의 CAN transceiver, CAN_H/CAN_L 배선, 공통 GND, 종단저항 검토가 필요합니다.
- Auto 모드를 사용하려면 모드 입력 PWM이 `1700 us` 이상이어야 합니다.
- RC PWM 입력은 10개 샘플 이동 평균으로 필터링됩니다.
- 현재 MCU 펌웨어는 ROS command packet 형식의 바이너리 패킷만 인식합니다. 일반 텍스트 `"0.5,0.0"` 형태로 보내면 인식되지 않습니다.
- status packet과 command packet은 같은 Serial 포트를 공유하므로, ROS 수신부는 binary packet framing을 기준으로 파싱해야 합니다.

## 참고 자료

- MDROBOT MD200T 제품 페이지: <https://www.mdrobot.co.kr/BLDCmotordriver-store-dualchannel/?idx=160>
- MDROBOT MDH100 제품 페이지: <https://www.mdrobot.co.kr/inwheelmotor-store/?idx=274>
