# Wheel-Dragoon

Teensy 3.2 기반 4륜 skid-steer 로봇 MCU 펌웨어입니다. 현재 펌웨어는 RC 수신기의 PWM 입력으로 수동 조종하거나, USB Serial(UART)로 ROS 목표 command packet을 받아 좌/우 바퀴 목표 RPM을 계산합니다.

이 README에는 현재 구현 상태와 ROS 패키지 연동을 위한 목표 프로토콜을 함께 기록합니다. ROS용 command packet 파싱은 구현되어 있지만, status packet 송신은 아직 `src/main.cpp`에 구현되어 있지 않습니다.

## 주요 기능

### 현재 구현됨

- RC PWM 입력 기반 수동 주행
- ROS-UART 정규화 `int16` command packet 파싱
- 원형 큐 기반 UART 수신 및 패킷 재동기화
- XOR checksum 검증
- `flags`의 enable 및 emergency_stop 처리
- 차동 구동식 기반 좌/우 바퀴 목표 RPM 계산
- 10 ms 주기 `IntervalTimer` 제어 루프
- 10 kHz, 10 bit PWM 모터 출력
- 목표 RPM 절댓값에 비례한 오픈루프 PWM 출력

### 아직 구현되지 않음

- ROS로 status packet 송신
- `seq` 기반 응답/상태 동기화
- Serial 명령 타임아웃 기반 disable/zero 처리
- kick-start, ramp limiting, minimum PWM, per-wheel gain
- motor watchdog 및 error bitfield/status state 관리

## 하드웨어 기준

현재 펌웨어는 다음 차량 파라미터를 기준으로 작성되어 있습니다.

| 항목 | 값 |
| --- | --- |
| 차폭 `W` | `0.42 m` |
| 바퀴 반지름 `R` | `0.14 m` |
| 보드 | Teensy 3.2 (`teensy31`) |
| 프레임워크 | Arduino |

## 핀맵

### RC 입력

| 기능 | 핀 | 설명 |
| --- | --- | --- |
| 각속도 입력 `w` | `0` | RC PWM 입력 |
| 선속도 입력 `v` | `1` | RC PWM 입력 |
| 모드 선택 | `2` | RC PWM 입력 |

### 모터 출력

| 기능 | 핀 | 설명 |
| --- | --- | --- |
| 좌측 전륜 PWM | `3` | 10 kHz PWM |
| 좌측 후륜 PWM | `4` | 10 kHz PWM |
| 우측 후륜 PWM | `5` | 10 kHz PWM |
| 우측 전륜 PWM | `6` | 10 kHz PWM |
| 좌측 방향 | `14` | 방향 제어 |
| 우측 방향 | `15` | 방향 제어 |

## 동작 모드

모드 채널의 PWM 평균값으로 주행 모드를 선택합니다.

| PWM 범위 | 모드 | 동작 |
| --- | --- | --- |
| `< 1300 us` | Stop | 좌/우 목표 RPM을 `0`으로 설정 |
| `1300-1699 us` | Manual | RC `v`, `w` 입력으로 주행 |
| `>= 1700 us` | Auto | ROS-UART command packet으로 `v_cmd`, `w_cmd` 수신 |

Manual 모드에서 RC PWM은 다음 범위로 변환됩니다.

- `v = (v_pwm - 1500) / 250` → 대략 `-2 ~ 2 m/s`
- `w = (w_pwm - 1500) / 100` → 대략 `-5 ~ 5 rad/s`

현재 펌웨어는 좌/우 바퀴 속도를 다음 식으로 계산한 뒤 RPM으로 변환합니다.

```text
left_velocity  = v - w * W / 2
right_velocity = v + w * W / 2
```

Auto 모드에서는 ROS 노드가 `/cmd_vel`을 `-1000`부터 `+1000`까지의 정규화 명령으로 변환해 보낸다고 가정합니다. MCU는 현재 이 정규화 명령을 기존 RC 입력 범위와 맞춰 다음처럼 환산합니다.

```text
v = v_cmd / 1000.0 * 2.0     # m/s
w = w_cmd / 1000.0 * 5.0     # rad/s
```

`v_cmd`, `w_cmd`는 packet의 `int16 LE` 값을 그대로 사용하며 MCU에서 clamp하지 않습니다. 프로토콜 범위를 벗어난 값은 송신 측 오류로 보고 ROS 노드에서 제한해야 합니다.

## ROS 연동 목표 책임 분리

다음 내용은 ROS 패키지와 MCU 펌웨어가 최종적으로 가져야 할 책임 경계입니다. 현재 저장소의 `src/main.cpp`는 이 책임 중 일부만 구현합니다.

### ROS 노드 책임

- `/cmd_vel` 구독
- `linear.x`, `angular.z`를 `-1000`부터 `+1000`까지의 정규화된 정수 명령으로 변환
- 고정 주기로 MCU에 명령 패킷 송신
- MCU 상태 패킷 수신
- 파싱한 상태를 `~/status`에 `std_msgs/msg/String`으로 발행
- 명령 타임아웃 및 종료 시 disable/zero 명령 송신
- Serial 포트가 닫히면 재연결

### MCU 책임

- 정규화된 `v_cmd`, `w_cmd`를 휠 레벨 명령으로 변환: 구현됨
- skid-steer 모터 믹싱 수행: 구현됨
- kick-start, ramp limiting, minimum PWM, per-wheel gain, motor watchdog 동작 수행: 아직 미구현
- 저수준 타임아웃 및 안전 동작 강제: Stop 모드의 RPM 0 설정만 구현됨

## ROS Packet Protocol

다음 프로토콜은 ROS 패키지 연동을 위한 통신 명세입니다. 현재 `src/main.cpp`는 command packet 수신과 checksum 검증을 구현하고, status packet 송신은 아직 구현하지 않았습니다.

모든 패킷은 `0xAA 0x55`로 시작합니다. Checksum은 byte 0부터 checksum 직전 byte까지 모든 바이트의 XOR 값입니다.

MCU 수신부는 원형 큐를 사용합니다. loop마다 Serial에서 최대 1바이트를 큐에 넣고, 큐의 앞쪽에서 패킷 후보를 검사합니다. 헤더, length, type, checksum 검증에 실패하면 1바이트만 버려 다음 헤더 후보를 다시 찾습니다.

### Command Packet: ROS to MCU

ROS 노드가 MCU로 보내는 명령 패킷입니다. 현재 MCU 펌웨어는 이 형식을 파싱합니다.

| Byte | Field | Type | Description |
| --- | --- | --- | --- |
| 0 | `header[0]` | `uint8` | `0xAA` |
| 1 | `header[1]` | `uint8` | `0x55` |
| 2 | `length` | `uint8` | `7` |
| 3 | `type` | `uint8` | `0x01` |
| 4 | `seq` | `uint8` | Sequence counter |
| 5-6 | `v_cmd` | `int16 LE` | 정규화된 선속도 명령, `-1000` to `+1000` |
| 7-8 | `w_cmd` | `int16 LE` | 정규화된 각속도 명령, `-1000` to `+1000` |
| 9 | `flags` | `uint8` | bit 0: enable, bit 1: emergency_stop |
| 10 | `checksum` | `uint8` | XOR checksum |

Payload는 `type + seq + v_cmd + w_cmd + flags`이며, length는 `7`입니다.

```text
AA 55 07 01 seq v_lo v_hi w_lo w_hi flags checksum
```

### Status Packet: MCU to ROS

MCU가 ROS 노드로 보내는 상태 패킷입니다. 현재 MCU 펌웨어는 status packet을 송신하지 않습니다.

| Byte | Field | Type | Description |
| --- | --- | --- | --- |
| 0 | `header[0]` | `uint8` | `0xAA` |
| 1 | `header[1]` | `uint8` | `0x55` |
| 2 | `length` | `uint8` | `7` |
| 3 | `type` | `uint8` | `0x81` |
| 4 | `seq` | `uint8` | Sequence counter |
| 5 | `state` | `uint8` | MCU state |
| 6-7 | `error` | `uint16 LE` | MCU error bitfield/code |
| 8-9 | `battery_mv` | `uint16 LE` | Battery voltage in millivolts |
| 10 | `checksum` | `uint8` | XOR checksum |

Payload는 `type + seq + state + error + battery_mv`이며, length는 `7`입니다.

```text
AA 55 07 81 seq state error_lo error_hi batt_lo batt_hi checksum
```

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
uv run python src/control.py 250 0
```

포트를 직접 지정하려면:

```bash
uv run python src/control.py 250 0 --port /dev/ttyACM0
```

sequence counter를 지정하려면:

```bash
uv run python src/control.py 250 0 --seq 12
```

disable 또는 emergency stop flag를 보내려면:

```bash
uv run python src/control.py 0 0 --disable
uv run python src/control.py 0 0 --estop
```

예시:

| 명령 | 의미 |
| --- | --- |
| `uv run python src/control.py 250 0` | 정규화 명령 기준 약 `0.5 m/s` 전진 |
| `uv run python src/control.py 0 200` | 정규화 명령 기준 약 `1.0 rad/s` 제자리 회전 |
| `uv run python src/control.py 0 0` | enable 상태의 정지 명령 |

## 프로젝트 구조

```text
.
├── platformio.ini          # PlatformIO 보드/프레임워크 설정
├── pyproject.toml          # Python 제어 스크립트 의존성
├── src/
│   ├── main.cpp            # Teensy 펌웨어
│   └── control.py          # UART 명령 송신 스크립트
└── lib/
    └── MCP41100/           # MCP41100 디지털 가변저항 보조 라이브러리
```

## 현재 제어 방식

현재 `control_tick()`은 목표 RPM의 절댓값에 비례해 PWM 값을 출력하는 오픈루프 방식입니다.

```cpp
int lf_u = constrain(left_ref_abs * 5, 0, 1023);
```

현재 구현에는 엔코더 피드백, 폐루프 PI/PID 제어, 개별 휠 게인, ramp limiting, minimum PWM, kick-start가 연결되어 있지 않습니다.

## 주의 사항

- 방향 핀의 HIGH/LOW 극성은 현재 모터 드라이버 배선 기준입니다. 배선이 다르면 좌/우 또는 전/후진 방향이 반대로 동작할 수 있습니다.
- Auto 모드를 사용하려면 모드 입력 PWM이 `1700 us` 이상이어야 합니다.
- RC PWM 입력은 10개 샘플 이동 평균으로 필터링됩니다.
- 현재 MCU 펌웨어는 ROS command packet 형식의 바이너리 패킷만 인식합니다. 일반 텍스트 `"0.5,0.0"` 형태로 보내면 인식되지 않습니다.
- status packet 송신을 사용하려면 MCU 펌웨어에 상태 생성 및 송신 처리를 추가해야 합니다.
