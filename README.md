# Wheel-Dragoon

Teensy 3.2 기반 4륜 차동 구동 로봇 펌웨어입니다. RC 수신기의 PWM 입력으로 수동 조종하거나, USB Serial(UART)로 선속도 `v`와 각속도 `w` 명령을 받아 좌/우 바퀴 목표 RPM을 계산해 모터 PWM과 방향 핀을 제어합니다.

## 주요 기능

- RC PWM 입력 기반 수동 주행
- UART 패킷 기반 자동 주행 명령 수신
- 차동 구동 운동학 기반 좌/우 바퀴 속도 계산
- 10 ms 주기 `IntervalTimer` 제어 루프
- 10 kHz, 10 bit PWM 모터 출력
- Python 제어 스크립트(`src/control.py`) 제공

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
| `>= 1700 us` | Auto | UART 패킷으로 `v`, `w` 수신 |

Manual 모드에서 RC PWM은 다음 범위로 변환됩니다.

- `v = (v_pwm - 1500) / 250` → 대략 `-2 ~ 2 m/s`
- `w = (w_pwm - 1500) / 100` → 대략 `-5 ~ 5 rad/s`

좌/우 바퀴 속도는 다음 차동 구동식으로 계산합니다.

```text
left_velocity  = v - w * W / 2
right_velocity = v + w * W / 2
```

## UART 자동 주행 패킷

Auto 모드에서는 USB Serial을 통해 12바이트 바이너리 패킷을 받습니다.

```text
0xAA 0x55 | float32 little-endian v | float32 little-endian w | 0x55 0xAA
```

- `v`: 선속도 명령, 단위 `m/s`
- `w`: 각속도 명령, 단위 `rad/s`
- Baud rate: `115200`
- 정상 수신 시 펌웨어는 `>ACK`를 출력합니다.

Python에서는 다음과 같이 패킷을 구성합니다.

```python
payload = b"\xaa\x55" + struct.pack("<ff", v, w) + b"\x55\xaa"
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

`src/control.py`는 Auto 모드에서 Teensy로 `v`, `w` 명령을 보내는 보조 스크립트입니다.

의존성은 `pyproject.toml`에 정의되어 있습니다.

```bash
uv sync
```

자동 포트 감지로 명령을 보내려면:

```bash
uv run python src/control.py 0.5 0.0
```

포트를 직접 지정하려면:

```bash
uv run python src/control.py 0.5 0.0 --port /dev/ttyACM0
```

예시:

| 명령 | 의미 |
| --- | --- |
| `uv run python src/control.py 0.5 0.0` | 전진 |
| `uv run python src/control.py 0.0 1.0` | 제자리 좌회전 방향 |
| `uv run python src/control.py 0.0 0.0` | 정지 명령 |

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

`control_tick()`은 목표 RPM의 절댓값에 비례해 PWM 값을 출력하는 오픈루프 방식입니다.

```cpp
int lf_u = constrain(left_ref_abs * 5, 0, 1023);
```

엔코더 피드백 또는 폐루프 PI/PID 제어는 현재 메인 펌웨어에 연결되어 있지 않습니다.

## 주의 사항

- 방향 핀의 HIGH/LOW 극성은 현재 모터 드라이버 배선 기준입니다. 배선이 다르면 좌/우 또는 전/후진 방향이 반대로 동작할 수 있습니다.
- Auto 모드를 사용하려면 모드 입력 PWM이 `1700 us` 이상이어야 합니다.
- RC PWM 입력은 10개 샘플 이동 평균으로 필터링됩니다.
- 펌웨어의 UART 패킷은 바이너리 float 형식이므로 일반 텍스트 `"0.5,0.0"` 형태로 보내면 인식되지 않습니다.
