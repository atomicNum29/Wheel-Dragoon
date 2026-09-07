# Wheel-Dragoon

Teensy 3.2 기반 4륜 skid-steer 로봇 MCU 펌웨어입니다. 목표 하드웨어는 MDROBOT `MDH100` 인휠 모터 4개와 `MD200T` 듀얼채널 모터 드라이버 2개입니다. ROS와 Teensy 사이의 USB Serial(UART) command/status packet 연결은 유지하고, Teensy는 계산한 휠 목표 속도를 CAN bus로 두 개의 MD200T에 전달합니다.

이 README는 MD200T/CAN 제어 구조와 ROS 패키지 연동 프로토콜을 함께 기록합니다. ROS용 command packet을 수신하고 canonical System Status v2 packet을 송신하는 현재 `src/main.cpp` 구현을 기준으로 합니다.

## 주요 기능

### 유지되는 기능

- RC PWM 입력 기반 수동 주행
- ROS-UART 실제 속도 `int16` command packet 파싱
- 상태머신 기반 USB Serial command packet 수신
- XOR checksum 검증
- `flags`의 enable 및 emergency_stop 처리
- System Status v2 (`0x82`) 10 Hz 송신
- Legacy Basic Status (`0x81`) 생성 코드 유지, 기본 송신 비활성
- command timeout 감지 및 timeout 시 PID 174 TQ-OFF 처리
- state/error bitfield 산출
- 차동 구동식 기반 좌/우 바퀴 목표 RPM 계산
- 10 ms 주기 Driver A/B MD200T command pair와 10 Hz/1 Hz telemetry polling 통합 스케줄링
- USB Serial-CAN bridge를 통한 MD200T PID/data 직접 송수신

### MD200T/CAN 전환 목표

- MD200T 2대 CAN 제어
- MDH100 4개 휠 목표 속도 명령 분배
- 드라이버별 2채널 속도/enable/stop 명령 송신
- Stop, disable, emergency stop, command timeout 시 모든 MD200T 채널 정지 명령 송신

### 아직 구현되지 않음

- ROS 주행 command에 대한 `seq` 기반 ack 동기화
- Teensy 3.2 외부 CAN transceiver 배선 검증
- 채널별 direction polarity 검증
- FlexCAN ESR1/ECR diagnostics 출력
- 실제 배터리 전압 ADC 측정
- watchdog reset 감지 source 연결
- MD200T telemetry 주기와 freshness timeout의 실제 장비 검증

## 하드웨어 기준

전환 목표는 다음 하드웨어 구성을 기준으로 합니다.

| 항목 | 값 |
| --- | --- |
| 차폭 `W` | `0.485 m` |
| 바퀴 반지름 `R` | `0.13 m` |
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
| Teensy CAN TX | pin `3` / `PTA12` / `CAN0_TX` | `PORTA_PCR12 = PORT_PCR_MUX(2)` |
| Teensy CAN RX | pin `4` / `PTA13` / `CAN0_RX` | `PORTA_PCR13 = PORT_PCR_MUX(2)` |
| CAN transceiver | 외부 CAN transceiver | Teensy 3.2는 CAN controller만 내장하므로 TX/RX를 transceiver TXD/RXD에 연결 |
| MD200T CAN_H/CAN_L | CAN bus | 두 MD200T를 같은 CAN bus에 연결 |
| CAN 종단저항 | bus 양 끝 | 실제 배선 길이와 MD200T 내장 종단 여부 확인 후 양 끝단 기준 적용 |
| GND 공통 | Teensy / transceiver / MD200T | 통신 기준 전위 공유 |

## FlexCAN0 직접 레지스터 구현 조사 결과

이번 구현 단계에서는 외부 CAN 라이브러리를 사용하지 않는다. Teensyduino 코어의 `kinetis.h`가 제공하는 `CAN0_MCR`, `CAN0_CTRL1`, `CAN0_IFLAG1` 등 CAN0 레지스터 주소 정의를 쓰고, bit mask와 MB field 배치는 `datasheet/K20P64M72SF1RM.pdf`의 FlexCAN 장에서 확인한 값으로 `flexcan0.cpp` 안에 로컬 상수로 둔다.

### 확인한 문서

- 확인함: `datasheet/K20P64M72SF1RM.pdf`
- 확인함: `datasheet/MDROBOT-CAN communication protocol on controllers[EN].pdf`
- 저장소에서 못 찾음: `datasheet/K20P64M72SF1.pdf`

문서 부재 때문에 MCU 패키지별 핀 멀티플렉싱은 레퍼런스 매뉴얼의 64-pin pinout과 Teensyduino 코어 pin map을 교차 확인했다. MD200T는 별도 전용 매뉴얼 PDF가 없는 것으로 보고, 운용 CAN baudrate가 `250 kbit/s`라는 전제만 명시한다.

### 구현 파일 구조

```text
include/flexcan0.hpp
src/flexcan0.cpp
include/md200t_can.hpp
include/can_tx_schedule.hpp
src/md200t_can.cpp
src/main.cpp
src/can_cli.py
```

FlexCAN 계층 API:

```cpp
struct CanFrame {
    uint16_t id;      // standard 11-bit ID only
    uint8_t dlc;      // 0..8
    uint8_t data[8];
};

bool can_begin(uint32_t bitrate);
bool can_transmit(const CanFrame& frame, uint32_t timeout_us);
bool can_receive(CanFrame& frame, uint32_t timeout_us);
bool can_rx_overrun_detected();
```

MD200T 계층 API:

```cpp
bool md200t_set_velocity(uint8_t driver_id, int16_t rpm1, int16_t rpm2);
bool md200t_torque_off(uint8_t driver_id);
bool md200t_request_pid_data(uint8_t driver_id, uint8_t target_pid);
```

`can_begin()`, `can_transmit()`, `can_receive()`의 모든 polling loop는 `micros()` 기반 timeout을 가져야 한다. timeout 대상은 low-power acknowledge 해제, freeze acknowledge 진입/해제, soft reset 완료, TX MB active/abort 대기, TX 완료 `IFLAG1` 대기, RX 완료 `IFLAG1` 대기다. 동적 메모리와 interrupt는 사용하지 않는다. 첫 구현은 검증한 `250000` bitrate만 허용하고, 다른 bitrate는 별도 timing table을 문서로 확인한 뒤 추가한다.

### FlexCAN 클록과 250 kbit/s 비트 타이밍

K20 레퍼런스 매뉴얼은 FlexCAN clock source가 `CANx_CTRL1[CLKSRC]`로 `OSCERCLK` 또는 bus clock 중 선택된다고 설명한다. Teensyduino FlexCAN 초기화 코드는 `OSC0_CR |= OSC_ERCLKEN` 후 `CAN0_CTRL1[CLKSRC]=0`을 사용해 16 MHz crystal clock을 FlexCAN source로 선택한다. 직접 구현도 이 경로를 따른다.

기본 bitrate `250000 bit/s` 설정:

| 항목 | 값 |
| --- | --- |
| FlexCAN source clock | `16,000,000 Hz` (`OSCERCLK`, `CTRL1[CLKSRC]=0`) |
| PRESDIV register | `3` |
| Clock divisor | `PRESDIV + 1 = 4` |
| Time quantum clock | `16,000,000 / 4 = 4,000,000 Hz` |
| Time quantum | `250 ns` |
| PROPSEG register | `2` |
| PROPSEG actual | `PROPSEG + 1 = 3 TQ` |
| PSEG1 register | `7` |
| PSEG1 actual | `PSEG1 + 1 = 8 TQ` |
| PSEG2 register | `3` |
| PSEG2 actual | `PSEG2 + 1 = 4 TQ` |
| RJW register | `1` |
| RJW actual | `RJW + 1 = 2 TQ` |
| Total time quanta | `1 sync + 3 prop + 8 pseg1 + 4 pseg2 = 16 TQ` |
| Sample point | `(1 + 3 + 8) / 16 = 75%` |
| Actual bitrate | `16,000,000 / 4 / 16 = 250,000 bit/s` |
| `CAN0_CTRL1` timing bits | `0x037B0002` before optional non-timing bits |

`CAN0_CTRL1[CLKSRC]`는 Disable mode에서만 쓸 수 있으므로 `MCR[MDIS]`를 clear하기 전에 0으로 둔다. `CAN0_CTRL1`의 timing field는 Freeze mode에서만 쓴다. 이번 polling 구현에서는 interrupt mask bit를 켜지 않는다. RX FIFO는 사용하지 않으므로 `MCR[RFEN]=0`을 유지한다.

### FlexCAN0 초기화 순서

1. `SIM_SCGC5 |= SIM_SCGC5_PORTA`로 PORTA clock을 켠다.
2. `SIM_SCGC6 |= SIM_SCGC6_FLEXCAN0`로 FlexCAN0 clock을 켠다.
3. `OSC0_CR |= OSC_ERCLKEN`으로 external reference clock을 켠다.
4. `PORTA_PCR12 = PORT_PCR_MUX(2)`, `PORTA_PCR13 = PORT_PCR_MUX(2)`로 CAN0_TX/RX를 선택한다.
5. Reset 직후 `MCR[MDIS]=1`인 Disable mode에서 `CAN0_CTRL1 &= ~CTRL1_CLKSRC`로 oscillator clock source를 선택한다.
6. `CAN0_MCR |= MCR_FRZ`, `CAN0_MCR &= ~MCR_MDIS` 후 `MCR[LPM_ACK]`가 0이 될 때까지 timeout polling한다.
7. `CAN0_MCR |= MCR_SOFT_RST` 후 `MCR[SOFT_RST]`가 0이 될 때까지 timeout polling한다.
8. `CAN0_MCR |= MCR_FRZ | MCR_HALT` 후 `MCR[FRZ_ACK]`가 1이 될 때까지 timeout polling한다.
9. `MCR[MAXMB]=15`, `MCR[SRX_DIS]=1`, `MCR[RFEN]=0`으로 16개 MB, self reception disabled, Rx FIFO disabled를 설정한다.
10. `CAN0_CTRL1 = 0x037B0002`로 250 kbit/s timing을 설정한다. `CLK_SRC` bit는 0이어야 한다.
11. `CAN0_IMASK1 = 0`, `CAN0_IFLAG1 = 0xFFFFFFFF`로 interrupt를 끄고 pending flag를 w1c clear한다.
12. MB0..MB15 RAM을 모두 초기화한다. `CS=0`, `ID=0`, `WORD0=0`, `WORD1=0`으로 지운 뒤 TX로 쓸 MB0은 `CS = CODE_TX_INACTIVE << 24`, RX로 쓸 MB1은 `CS = CODE_RX_EMPTY << 24`로 둔다.
13. `CAN0_MCR &= ~MCR_HALT` 후 `MCR[FRZ_ACK]`가 0이 될 때까지 timeout polling한다.

MK20DX256의 FlexCAN0은 16개 message buffer를 가진다. MB 하나는 16 byte이고, MB memory는 `CAN0` base `0x40024000 + 0x80 + n * 0x10` 위치다. Teensyduino 코어의 `kinetis.h`는 `CAN0_MBn_CS(n)`, `CAN0_MBn_ID(n)`, `CAN0_MBn_WORD0(n)`, `CAN0_MBn_WORD1(n)` macro를 제공하지 않으므로 직접 구현에서는 base offset helper를 로컬 `volatile uint32_t&` 함수로 둔다.

### TX Message Buffer 작성 규칙

레퍼런스 매뉴얼에서 확인한 MB field:

| 항목 | 값 |
| --- | --- |
| TX inactive CODE | `0b1000` |
| TX abort CODE | `0b1001` |
| TX data frame once CODE | `0b1100` |
| Standard data frame | `IDE=0`, `RTR=0` |
| DLC 위치 | `CS[19:16]` |
| CODE 위치 | `CS[27:24]` |
| Standard ID 위치 | `ID[28:18]`, 즉 `(id & 0x7FF) << 18` |
| Data byte order | `WORD0[31:24]=data[0]`, `WORD0[23:16]=data[1]`, `WORD0[15:8]=data[2]`, `WORD0[7:0]=data[3]`, `WORD1`도 같은 순서로 `data[4]..data[7]` |
| TX complete flag | `CAN0_IFLAG1 & (1u << tx_mb)` |
| IFLAG clear | 해당 bit에 `1` 쓰기, 예: `CAN0_IFLAG1 = (1u << tx_mb)` |

`can_transmit()` 절차:

1. `id <= 0x7FF`, `dlc <= 8`, `can_begin()` 완료 여부를 검사한다.
2. TX MB의 pending flag가 있으면 먼저 `CAN0_IFLAG1 = mask`로 clear한다.
3. TX MB가 `CODE_TX_INACTIVE`가 아니면 `CODE_TX_ABORT`를 쓰고 해당 `IFLAG1`이 set될 때까지 timeout polling한 뒤 flag를 clear한다. abort 이후에도 inactive가 아니면 실패 처리한다.
4. `ID`, `WORD0`, `WORD1`을 쓴다. `dlc`보다 뒤의 byte는 0으로 채운다.
5. 마지막으로 `CS = (CODE_TX_DATA_ONCE << 24) | (dlc << 16)`을 써서 MB를 arbitration에 올린다.
6. `IFLAG1` 해당 bit가 set될 때까지 timeout polling한다.
7. 완료 후 `CAN0_IFLAG1 = mask`로 clear하고 성공을 반환한다.

### RX Message Buffer 수신 규칙

MD200T polling response와 CAN bridge traffic을 받기 위해 MB1 하나를 polling RX MB로 사용한다. RX FIFO와 interrupt는 사용하지 않는다.

| 항목 | 값 |
| --- | --- |
| RX MB | `MB1` |
| RX empty CODE | `0b0100` |
| RX full CODE | `0b0010` |
| RX overrun CODE | `0b0110` |
| Acceptance mask | `CAN0_RXMGMASK = 0`, 모든 standard ID 수신 |
| Standard ID 위치 | `ID[28:18]`, 즉 `(ID >> 18) & 0x7FF` |
| Data byte order | TX와 동일하게 `WORD0[31:24]=data[0]` 순서 |
| RX complete flag | `CAN0_IFLAG1 & (1u << 1)` |
| RX unlock / clear | MB1의 `CS/ID/WORD`를 읽고 `CAN0_TIMER`를 읽은 뒤 `IFLAG1` bit 1을 clear |

`can_receive()`는 MB1의 `IFLAG1`이 set될 때까지 timeout polling하고 standard ID, DLC, data byte를 복사한다. 처리 후 MB1을 다시 `CODE_RX_EMPTY`로 돌린다. `CODE_RX_OVERRUN`은 sticky diagnostic으로 기록한다. 정상 loop는 timeout 0으로 수신 frame을 drain하며 별도의 polling-response 대기 loop를 만들지 않는다.

### ESR1/ECR diagnostics 추후 구현

`can_print_diagnostics()`는 이번 1차 구현의 우선순위에서 제외한다. 추후 구현 시 최소한 다음 값을 출력한다.

```text
CAN0_MCR
CAN0_CTRL1
CAN0_ESR1
CAN0_ECR
CAN0_IFLAG1
ECR_TXERR = ECR[7:0]
ECR_RXERR = ECR[15:8]
ESR1_FLTCONF = ESR1[5:4]
ESR1_ACKERR, BIT0ERR, BIT1ERR, CRCERR, FRMERR, STFERR
```

ACK error는 외부 transceiver, CAN_H/CAN_L 배선, 종단저항, 상대 노드 bitrate가 틀렸을 때 가장 먼저 볼 가능성이 높다. 이번 범위에서는 bus-off 자동 복구를 구현하지 않고 진단 출력만 제공한다.

## MD200T CAN frame 결정

MDROBOT 표준 CAN frame 구조:

```text
CAN ID[10:8] = MID
CAN ID[7:0]  = driver ID
DLC = 8
DATA[0] = PID
DATA[1..7] = PID data
```

표준 모드 송신 예제는 `MID=0`을 사용한다. 문서는 드라이버가 표준 모드에서 수신할 때 MID 3 bit를 don't care로 본다고 설명하지만, 구현 상수는 예제값을 따라 `MDROBOT_STANDARD_CMD_MID = 0x00`으로 둔다. 드라이버 응답 frame의 MID는 문서에 `0x07`로 명시되어 있으므로 수신 구현 시 `MDROBOT_STANDARD_RESPONSE_MID = 0x07`로 둔다.

현재 속도 제어 구현에 사용할 PID:

| 함수 | CAN ID | DATA |
| --- | --- | --- |
| `md200t_set_velocity(driver_id, rpm1, rpm2)` | `(0x00 << 8) | driver_id` | `PID_PNT_VEL_CMD(207), 1, rpm1_lo, rpm1_hi, 1, rpm2_lo, rpm2_hi, 0` |
| `md200t_torque_off(driver_id)` | `(0x00 << 8) | driver_id` | `PID_PNT_TQ_OFF(174), 1, 1, 0, 0, 0, 0, 0` |
| `md200t_request_pid_data(driver_id, 193)` | `(0x00 << 8) | driver_id` | `PID_REQ_PID_DATA(4), PID_MAIN_DATA(193), 0, 0, 0, 0, 0, 0` |
| `md200t_request_pid_data(driver_id, 200)` | `(0x00 << 8) | driver_id` | `PID_REQ_PID_DATA(4), PID_MAIN_DATA2(200), 0, 0, 0, 0, 0, 0` |
| `md200t_request_pid_data(driver_id, 143)` | `(0x00 << 8) | driver_id` | `PID_REQ_PID_DATA(4), PID_VOLT_IN(143), 0, 0, 0, 0, 0, 0` |

RPM은 MDROBOT 문서의 2-byte data 규칙에 따라 signed 16-bit little-endian으로 보낸다. `PID_PNT_VEL_CMD(207)` 한 프레임에 CH1/CH2 RPM을 각각 넣으며, 음수는 2의 보수 bit pattern을 그대로 직렬화한다.

`PID_PNT_TQ_OFF(174)`는 dual-channel driver에서 motor1/motor2 torque-off condition을 각각 `D0`, `D1` bit로 지정한다. `md200t_torque_off()`는 안전 정지용으로 두 채널 모두 free stop시키는 함수이므로 `D0=1`, `D1=1`, `D2=0(no return data)`로 둔다.

MD200T A/B는 이미 `250 kbit/s`로 설정된 상태를 전제로 한다. 펌웨어는 baudrate 설정 frame을 보내지 않고, Teensy FlexCAN0를 `250 kbit/s`로 초기화한 뒤 바로 명령 frame을 송신한다.

## 동작 모드

모드 채널의 PWM 평균값으로 주행 모드를 선택합니다.

| PWM 범위 | 모드 | 동작 |
| --- | --- | --- |
| `< 1300 us` | Stop (`DRIVE_MODE_STOP`) | 좌/우 목표 RPM을 `0`으로 설정 |
| `1300-1699 us` | Manual (`DRIVE_MODE_MANUAL`) | RC `v`, `w` 입력으로 주행 |
| `>= 1700 us` | Auto (`DRIVE_MODE_AUTO`) | ROS-UART command packet으로 `v_milli_mps`, `w_milli_radps` 수신 |

`mode_state`는 위 STOP/MANUAL/AUTO 구동 source를 선택합니다. 이 값과 별도로 `serial_mode`는 정상 제어/상태 통신인 CONTROL과 raw CAN tunnel인 BRIDGE를 선택합니다. 두 변수는 우선순위가 같은 주행 모드가 아니라, `serial_mode`가 바깥쪽 통신 동작을 정하고 `mode_state`가 CONTROL 안에서만 구동 source를 고르는 중첩 구조입니다.

```text
serial_mode
├── CONTROL
│   ├── mode_state == STOP   → 주기적인 PID 174 TQ-OFF
│   ├── mode_state == MANUAL → RC PWM으로 PID 207 목표 계산
│   └── mode_state == AUTO   → 마지막 정상 0x01 command로 PID 207 목표 계산
└── BRIDGE
    ├── mode_state와 관계없이 automatic PID 207/174 및 telemetry polling 중단
    ├── unsolicited 0x81/0x82 송신 중단
    └── 0x20 raw CAN 송신과 0xA0 CAN 수신 tunnel만 처리
```

BRIDGE 진입은 `mode_state == AUTO`로 제한되지 않습니다. STOP, MANUAL, AUTO 어느 위치에서도 유효한 `0x20` packet을 받으면 BRIDGE로 전환합니다. BRIDGE에서는 RC mode switch가 계속 갱신되더라도 그 값에 따라 CAN 자동 출력이 다시 시작되거나 CONTROL로 복귀하지 않습니다. 유효한 `0x01` command를 받아야 CONTROL로 복귀하며, 복귀 직후 실제 출력은 그때의 `mode_state`가 결정합니다. 따라서 STOP이면 TQ-OFF, MANUAL이면 RC 명령, AUTO이면 수신한 `0x01` 명령을 적용합니다.

CONTROL이라고 항상 ROS target을 사용하는 것은 아닙니다. `mode_state == AUTO`일 때만 저장된 ROS command가 모터 출력에 적용됩니다. STOP과 MANUAL에서도 유효한 `0x01`은 AUTO용 command로 저장되고 BRIDGE 상태였다면 CONTROL 복귀를 일으키지만, 현재 CAN 출력에는 ROS target이 섞이지 않습니다.

RC receiver 자체의 fail-safe를 사용하므로 MCU는 별도의 PWM signal-loss timeout을 추가하지 않습니다. Loop는 ISR이 갱신하는 mode와 두 PWM 값을 iteration마다 한 번씩 snapshot하여 같은 iteration 안에서 mode가 섞이지 않게 처리합니다.

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

Stop, disable, emergency stop, command timeout 상태에서는 두 MD200T에 PID 174를 보내 네 개 채널을 모두 TQ-OFF합니다.

## 드라이버/모터 매핑

두 개의 MD200T는 대각선으로 묶인 모터를 담당합니다.

| 드라이버 | CAN ID | CH1 | CH2 | 비고 |
| --- | --- | --- | --- | --- |
| MD200T A | `1` | LF | RR | 대각 페어 |
| MD200T B | `2` | RF | LR | 대각 페어 |

각 채널의 `direction polarity`는 실제 배선 후 검증해야 합니다. 표준 송신 frame은 `MD200T CAN frame 결정` 섹션의 듀얼채널 PID를 사용합니다.

## CAN TX scheduler와 polling 주기

CONTROL mode의 자동 CAN 송신은 `can_transmit_if_due(micros())` 한 함수가 관리합니다. 각 10 ms cycle의 시작에 같은 RPM/enable snapshot으로 Driver A와 Driver B의 PID 207 frame을 가능한 간격 없이 back-to-back 송신합니다. 출력이 금지된 상태에서는 같은 위치에 PID 174 두 장을 back-to-back 송신합니다. CAN은 직렬 bus이므로 두 frame이 물리적으로 동시에 전송되는 것은 아닙니다.

Polling request는 command pair와 겹치지 않도록 각 cycle의 `+5 ms` phase에 최대 한 장만 보냅니다. PID 4 요청을 송신한 뒤 응답을 기다리지 않으며, 정상 loop의 non-blocking CAN RX drain이 나중에 수신한 PID를 decode합니다.

```text
cycle +0 ms  Driver A PID 207/174 → 즉시 Driver B PID 207/174
cycle +5 ms  예약된 PID 4 polling request 한 장 또는 idle
```

Polling은 다음 1초 superframe을 반복합니다.

```text
   5 ms  Driver A PID 143 voltage
  15 ms  LF  = Driver A PID 193
  25 ms  RF  = Driver B PID 193
  35 ms  LR  = Driver B PID 200
  45 ms  RR  = Driver A PID 200

115/125/135/145 ms부터 같은 wheel pattern을 100 ms마다 반복
505 ms에 Driver B PID 143 voltage
```

따라서 LF/RF/LR/RR는 각각 10 Hz, Driver A/B voltage는 각각 1 Hz로 요청됩니다. Loop가 늦어 지난 cycle을 놓치면 밀린 event를 몰아서 보내지 않고 현재 cycle로 건너뜁니다. Command pair가 `+5 ms` polling phase까지 지연된 경우 motor command를 우선하고 그 cycle의 poll을 생략합니다.

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
- System Status v2 Packet 주기 송신 (`0x82`, 10 Hz)
- Legacy Basic Status (`0x81`) serializer는 유지하지만 기본 송신은 비활성화
- command timeout 시 `TIMEOUT_STOP` 및 `COMMAND_TIMEOUT` 송신
- Stop, disable, estop, timeout 시 두 MD200T의 모든 채널에 PID 174 TQ-OFF 강제

## ROS Packet Protocol

다음 프로토콜은 ROS 패키지와 MCU 사이의 USB Serial 통신 명세입니다. 기본 운용에서는 ROS가 `0x01` command를 보내고 MCU가 `0x82` System Status v2를 10 Hz로 보냅니다. `0x81` Basic Status 생성 코드는 구버전 호환을 위해 남아 있지만 기본 loop의 호출은 의도적으로 주석 처리되어 있어 송신되지 않습니다.

모든 패킷은 `0xAA 0x55`로 시작합니다. Byte 2의 `length`는 `type`부터 checksum 직전 마지막 payload byte까지의 바이트 수이며, checksum byte 자체는 포함하지 않습니다. 2-byte 정수는 모두 little-endian이고 signed 값은 2의 보수 bit pattern으로 전송합니다. Checksum은 byte 0부터 checksum 직전 byte까지 모든 바이트의 XOR 값입니다.

| 방향 | Type | 기본 동작 | 용도 |
| --- | --- | --- | --- |
| ROS → MCU | `0x01` | 활성 | 주행 command 및 control mode 진입 |
| MCU → ROS | `0x82` | 활성, 10 Hz | canonical MCU/MD200T 통합 상태 |
| MCU → ROS | `0x81` | 비활성 | 구버전 Basic Status 호환용 serializer |
| PC → MCU | `0x20` | bridge mode에서 요청 시 | Serial-CAN bridge request |
| MCU → PC | `0xA0` | bridge mode에서 연속 | 필터링하지 않은 CAN 수신 frame 및 bridge status |

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

### Legacy Basic Status Packet: MCU to ROS (`0x81`, 기본 비활성)

구버전 consumer와 wire format 호환을 위해 생성 함수와 2 Hz scheduler를 유지합니다. 그러나 현재 기본 loop에서는 `protocol_send_basic_status_if_due()` 호출이 주석 처리되어 있으므로 이 패킷을 출력하지 않습니다. 임의로 다시 활성화하지 말고, 신규 ROS 연동은 `0x82`를 사용합니다.

| Byte | Field | Type | Description |
| --- | --- | --- | --- |
| 0 | `header[0]` | `uint8` | `0xAA` |
| 1 | `header[1]` | `uint8` | `0x55` |
| 2 | `length` | `uint8` | `7` |
| 3 | `type` | `uint8` | `0x81` |
| 4 | `seq` | `uint8` | MCU status 송신 counter. 송신할 때마다 1 증가, `255` 이후 `0`으로 wrap |
| 5 | `state` | `uint8` | MCU state |
| 6-7 | `error` | `uint16 LE` | MCU error bitfield/code |
| 8-9 | `battery_mv` | `uint16 LE` | 두 driver 전압이 모두 fresh이면 낮은 값, 아니면 `0` |
| 10 | `checksum` | `uint8` | XOR checksum |

Payload는 `type + seq + state + error + battery_mv`이며, length는 `7`입니다.

```text
AA 55 07 81 seq state error_lo error_hi batt_lo batt_hi checksum
```

### MCU state (`0x81`/`0x82` 공통)

`state`와 `mcu_state`는 다음 enum 매핑을 공통으로 사용합니다.

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

### Legacy Basic Status error (`0x81`)

Legacy `error` bitfield는 다음 매핑을 사용합니다. `0x81`은 상세 MD200T 원인을 모두 표현할 수 없으므로 신규 구현에서는 `0x82 system_error`와 wheel별 `status`를 기준으로 진단해야 합니다.

| Bit | Mask | 이름 | 의미와 유지 조건 |
| --- | --- | --- | --- |
| 0 | `0x0001` | `CHECKSUM_ERROR` | 수신 packet checksum 오류 event. 정상 `0x82` 송신 후 clear |
| 1 | `0x0002` | `COMMAND_TIMEOUT` | AUTO mode에서 정상 command가 500 ms 이상 없음. 정상 command 수신 또는 AUTO mode 이탈 시 clear |
| 2 | `0x0004` | `DRIVER_FAULT` | 구동/안전 CAN TX failure, MD200T status, MAIN_DATA stale 중 하나 이상의 요약. 재부팅 전까지 유지 |
| 3 | `0x0008` | `EMERGENCY_STOP_ACTIVE` | 현재 E-stop 입력이 활성. 정상 command에서 E-stop 해제 또는 AUTO mode 이탈 시 clear |
| 4 | `0x0010` | `BATTERY_LOW` | 두 전압이 fresh인 상태에서 하나라도 21,000 mV 미만. 재부팅 전까지 유지 |
| 5 | `0x0020` | `SERIAL_FRAMING_ERROR` | header/length/type framing 오류 event. 정상 `0x82` 송신 후 clear |
| 6 | `0x0040` | `COMMAND_OUT_OF_RANGE` | command 속도 범위 오류 event. 정상 `0x82` 송신 후 clear |
| 7 | `0x0080` | `WATCHDOG_RESET_DETECTED` | watchdog reset 진단용. 현재 값을 설정하는 source는 구현되지 않음 |
| 8 | `0x0100` | `OVER_CURRENT` | legacy 예약 bit. 현재 항상 0 |
| 9 | `0x0200` | `OVER_TEMPERATURE` | legacy 예약 bit. 현재 항상 0 |
| 10 | `0x0400` | `PARAMETER_ERROR` | legacy 예약 bit. 현재 항상 0 |
| 11-15 | `0xF800` | Reserved | 항상 0 |

### System Status v2 Packet: MCU to ROS (`0x82`)

MCU의 운용 상태와 두 MD200T에서 수집한 네 wheel telemetry를 한 시점의 snapshot으로 전달하는 canonical 상태 packet입니다. Control mode에서 10 Hz로 송신하며 bridge mode에서는 송신하지 않습니다. 전체 크기는 43 bytes입니다.

```text
AA 55 27 82 version seq state error_lo error_hi validity
voltage_a_lo voltage_a_hi voltage_b_lo voltage_b_hi
LF[7] RF[7] LR[7] RR[7] checksum
```

| Byte | Field | Type | 설명 |
| --- | --- | --- | --- |
| 0 | `header[0]` | `uint8` | `0xAA` |
| 1 | `header[1]` | `uint8` | `0x55` |
| 2 | `length` | `uint8` | `39 (0x27)`, byte 3~41의 길이 |
| 3 | `type` | `uint8` | `0x82` |
| 4 | `version` | `uint8` | 현재 protocol version `1` |
| 5 | `seq` | `uint8` | 실제 Serial write가 수행될 때 1 증가, 255 이후 0으로 wrap |
| 6 | `mcu_state` | `uint8` | 위 State enum의 현재 최종 운용 상태 |
| 7-8 | `system_error` | `uint16 LE` | MCU/system 계층 원인 및 driver fault 요약 |
| 9 | `validity` | `uint8` bitfield | 각 telemetry 값의 freshness 표시 |
| 10-11 | `driver_a_voltage_mv` | `uint16 LE` | Driver A PID 143 입력전압, mV |
| 12-13 | `driver_b_voltage_mv` | `uint16 LE` | Driver B PID 143 입력전압, mV |
| 14-20 | `lf` | 7 bytes | Left Front wheel telemetry |
| 21-27 | `rf` | 7 bytes | Right Front wheel telemetry |
| 28-34 | `lr` | 7 bytes | Left Rear wheel telemetry |
| 35-41 | `rr` | 7 bytes | Right Rear wheel telemetry |
| 42 | `checksum` | `uint8` | byte 0~41의 XOR |

#### System error (Byte 7-8)

`system_error`는 원인을 나타내는 bitfield이므로 여러 bit가 동시에 설정될 수 있습니다. `mcu_state`는 우선순위를 적용한 최종 운용 상태 하나만 나타냅니다. 따라서 `system_error != 0`이 항상 `mcu_state == FAULT`를 뜻하지는 않습니다.

| Bit | Mask | 이름 | 설정 조건 | 운용 영향 | 해제 조건 |
| --- | --- | --- | --- | --- | --- |
| 0 | `0x0001` | `SERIAL_CHECKSUM_ERROR` | `0x01` 또는 `0x20` 수신 checksum 오류 | 진단 event, 상태 변경 없음 | 정상 `0x82` 송신 후 clear |
| 1 | `0x0002` | `COMMAND_TIMEOUT` | AUTO mode에서 정상 command가 없거나 마지막 command 후 500 ms 초과 | `TIMEOUT_STOP`, 출력 금지 | 정상 command 수신 또는 AUTO mode 이탈 |
| 2 | `0x0004` | `CAN_TX_FAILURE` | CAN 초기화, PID 207/174/bridge 송신 실패 또는 telemetry poll 송신 실패 | 구동/안전 TX 실패는 `FAULT`; poll 단발 실패는 진단만 표시 | 구동/안전 TX 실패는 재부팅, poll 실패는 다음 poll TX 성공 |
| 3 | `0x0008` | `EMERGENCY_STOP_ACTIVE` | command flag bit 1 활성 | `ESTOP`, 출력 금지 | E-stop이 해제된 정상 command 또는 AUTO mode 이탈 |
| 4 | `0x0010` | `BATTERY_LOW` | 두 driver 전압이 모두 fresh이고 하나라도 21,000 mV 미만 | `FAULT`, TQ-OFF | 재부팅 |
| 5 | `0x0020` | `SERIAL_FRAMING_ERROR` | 잘못된 header/length/type 수신 | 진단 event, 상태 변경 없음 | 정상 `0x82` 송신 후 clear |
| 6 | `0x0040` | `COMMAND_OUT_OF_RANGE` | `v` 또는 `w`가 허용 범위를 벗어남 | 해당 command 거부 | 정상 `0x82` 송신 후 clear |
| 7 | `0x0080` | `WATCHDOG_RESET_DETECTED` | watchdog reset 감지 | 진단용 | 현재 감지 source 미구현 |
| 8 | `0x0100` | `DRIVER_STATUS_FAULT_PRESENT` | 어느 wheel이든 MD200T raw `status != 0` | `FAULT`, TQ-OFF | 재부팅 |
| 9 | `0x0200` | `MAIN_DATA_STALE` | 500 ms grace 이후 어느 wheel이든 MAIN_DATA invalid | `FAULT`, TQ-OFF | 재부팅 |
| 10 | `0x0400` | `VOLTAGE_STALE` | 3 s grace 이후 Driver A/B 중 하나라도 voltage invalid | 진단 및 validity만 변경 | 두 voltage가 다시 fresh하면 자동 clear |
| 11 | `0x0800` | `CAN_RX_OVERRUN` | RX mailbox에서 hardware overrun 검출 | 진단용, 즉시 정지하지 않음 | CAN 재초기화 또는 재부팅 |
| 12 | `0x1000` | Reserved | 이전 BC 초기화 오류 위치. 현재 사용하지 않음 | 없음 | 항상 0 |
| 13-15 | `0xE000` | Reserved | 사용하지 않음 | 없음 | 항상 0 |

`FAULT`를 만드는 안전 fault는 구동/안전 경로의 `CAN_TX_FAILURE`, `BATTERY_LOW`, `DRIVER_STATUS_FAULT_PRESENT`, `MAIN_DATA_STALE`입니다. PID 193/200/143 polling request 한 건의 TX 실패도 같은 `CAN_TX_FAILURE` bit로 보이지만 즉시 `FAULT`로 승격하지 않고, 다음 poll TX가 성공하면 해제합니다. 내부에서는 sticky safety TX failure와 recoverable poll TX failure를 분리합니다. 안전 fault에는 runtime reset이 없으며 디버깅 후 MCU를 재부팅해야 해제됩니다. Fault가 있으면 state 우선순위에 따라 E-stop이나 timeout보다 `FAULT`가 먼저 표시됩니다.

Control mode에서는 PID 193/200/143을 모두 PID 4로 polling하며 firmware가 CMD 5/6/50/51을 자동 생성하지 않습니다. Bridge mode 진입 시 두 driver를 PID 174로 TQ-OFF하고 PID 207/174의 주기 송신, 모든 telemetry polling, `0x82` 송신을 중단합니다. `0x81`은 어느 mode에서도 기본 송신하지 않습니다.

#### Telemetry validity (Byte 9)

`validity`는 값이 packet 안에 존재하는지를 뜻하지 않고, 해당 값이 최근 timeout 이내에 정상 수신되어 지금 사용 가능한지를 뜻합니다. Bit가 0이어도 telemetry byte에는 마지막 수신값이 남아 있을 수 있으므로 ROS는 반드시 validity bit를 먼저 확인하고, invalid 값을 현재 측정값으로 사용하면 안 됩니다.

| Bit | Mask | 대상 | 1이 되는 조건 | 0이 되는 조건 |
| --- | --- | --- | --- | --- |
| 0 | `0x01` | LF MAIN_DATA | Driver A/MOT1/PID 193 정상 frame 수신 후 500 ms 이내 | 미수신, 500 ms 초과 또는 control session 재시작 |
| 1 | `0x02` | RF MAIN_DATA | Driver B/MOT1/PID 193 정상 frame 수신 후 500 ms 이내 | 미수신, 500 ms 초과 또는 control session 재시작 |
| 2 | `0x04` | LR MAIN_DATA | Driver B/MOT2/PID 200 정상 frame 수신 후 500 ms 이내 | 미수신, 500 ms 초과 또는 control session 재시작 |
| 3 | `0x08` | RR MAIN_DATA | Driver A/MOT2/PID 200 정상 frame 수신 후 500 ms 이내 | 미수신, 500 ms 초과 또는 control session 재시작 |
| 4 | `0x10` | Driver A voltage | Driver A/PID 143 정상값 수신 후 3 s 이내 | 미수신, 3 s 초과 또는 control session 재시작 |
| 5 | `0x20` | Driver B voltage | Driver B/PID 143 정상값 수신 후 3 s 이내 | 미수신, 3 s 초과 또는 control session 재시작 |
| 6-7 | `0xC0` | Reserved | 사용하지 않음 | 항상 0 |

정상 MAIN_DATA frame은 MDROBOT response MID 7, 올바른 driver ID, PID 193/200, DLC 8을 만족해야 합니다. 정상 전압 frame은 MID 7, 올바른 driver ID, PID 143, DLC 3 이상이고 raw 전압이 `1..655` 범위여야 합니다. PID 143 raw 단위는 0.1 V이며 MCU는 `raw × 100`으로 mV에 변환합니다. 잘못된 frame은 마지막 정상값과 timestamp를 갱신하지 않습니다.

Control mode 시작 또는 bridge mode에서 control mode로 복귀하면 모든 validity와 저장 telemetry를 0으로 초기화합니다. MAIN_DATA는 처음 500 ms, voltage는 처음 3 s 동안 수신 대기 grace period를 가지며, 이 기간에는 validity가 0일 수 있지만 아직 stale error를 발생시키지 않습니다.

#### Wheel telemetry block

각 wheel은 다음 7-byte 구조입니다. Packet 내 wheel 순서는 `LF → RF → LR → RR`이며, 실제 대각선 driver 배치는 아래 표와 같습니다.

| Wheel | CAN source | PID | Packet byte |
| --- | --- | --- | --- |
| LF | Driver A / MOT1 | 193 (`0xC1`) | 14-20 |
| RF | Driver B / MOT1 | 193 (`0xC1`) | 21-27 |
| LR | Driver B / MOT2 | 200 (`0xC8`) | 28-34 |
| RR | Driver A / MOT2 | 200 (`0xC8`) | 35-41 |

| Block offset | Field | Type/단위 | 설명 |
| --- | --- | --- | --- |
| `+0` | `status` | `uint8` bitfield | MD200T가 보낸 raw motor status를 변경 없이 전달 |
| `+1..2` | `actual_rpm` | `int16 LE`, rpm | 실제 모터 회전수. 음수는 2의 보수 |
| `+3..4` | `current` | `uint16 LE`, 0.1 A | 예: `123`은 12.3 A |
| `+5..6` | `controller_output` | `uint16 LE` | MD200T 제어출력, 일반 범위 0~1023 |

MDROBOT CAN 통신사양 V2.1의 raw `status` bit 의미는 다음과 같습니다. 현재 안전 정책에서는 아래 bit 중 하나라도 설정되어 `status != 0`이면 `DRIVER_STATUS_FAULT_PRESENT`를 latch하고 차량을 `FAULT`로 전환합니다.

| Bit | Mask | MD200T 이름 | 의미 |
| --- | --- | --- | --- |
| 0 | `0x01` | `ALARM` | 제어기 alarm 존재 |
| 1 | `0x02` | `CTRL_FAIL` | 기준속도의 1/3 이하로 접근하지 못한 제어 실패 |
| 2 | `0x04` | `OVER_VOLT` | 규정 이상 입력전압 감지 |
| 3 | `0x08` | `OVER_TEMP` | 지원 제어기에서 65 °C 이상 감지 |
| 4 | `0x10` | `OVER_LOAD` | 설정 전류 이상이 4초 이상 지속되거나 순간 최대 과전류 감지 |
| 5 | `0x20` | `HALL_FAIL` | Hall sensor 감지 실패 |
| 6 | `0x40` | `INV_VEL` | 모터 회전속도 방향이 출력 방향과 반대 |
| 7 | `0x80` | `STALL` | 출력 중 모터가 2초 이상 구속되어 움직이지 않음 |

#### `0x82` 정상 packet 예시

다음은 모든 telemetry가 valid이고 error가 없는 packet 예시입니다. `seq=0x10`, `state=ENABLED`, Driver A/B 전압은 각각 24.0 V/23.8 V이며 checksum은 `0x5A`입니다.

```text
AA 55 27 82 01 10 01 00 00 3F C0 5D F8 5C
00 64 00 7B 00 F4 01
00 9C FF 78 00 EA 01
00 B0 FF 6E 00 C2 01
00 50 00 73 00 CC 01
5A
```

```text
system_error = 0x0000 → error bytes 00 00
validity     = 0x3F   → LF/RF/LR/RR와 Driver A/B voltage가 모두 fresh
LF           = status 0, +100 rpm, 12.3 A, output 500
RF           = status 0, -100 rpm, 12.0 A, output 490
LR           = status 0,  -80 rpm, 11.0 A, output 450
RR           = status 0,  +80 rpm, 11.5 A, output 460
```

예를 들어 `DRIVER_STATUS_FAULT_PRESENT(bit 8)`와 `MAIN_DATA_STALE(bit 9)`가 동시에 설정되면 `system_error=0x0300`이고, little-endian 전송 byte는 `00 03`입니다.

### CAN Bridge Request: PC CLI to MCU

MDAS처럼 PC 터미널에서 MD200T의 PID/data frame을 직접 보내기 위한 요청 패킷입니다. 같은 USB Serial 포트를 공유하므로 기존 ROS command/status packet과 동일한 `AA 55 length type ... checksum` framing을 사용하되, type은 `0x20`으로 분리합니다.

요청을 받은 Teensy는 bridge mode로 전환한 뒤 표준 11-bit CAN data frame을 250 kbit/s CAN bus로 송신합니다. `0x20`의 packet layout과 timeout field는 기존 형식을 그대로 유지합니다. Bridge mode로 처음 전환할 때 두 driver에는 PID 174 TQ-OFF를 back-to-back으로 보내고 automatic command/polling scheduler를 중단합니다. Firmware는 broadcasting ON/OFF 명령을 자동으로 보내지 않습니다.

지정한 timeout 동안 가장 먼저 들어온 CAN frame은 ID, driver ID, PID를 검사하지 않고 `0xA0`으로 전달합니다. 이후 bridge mode에서 수신하는 모든 standard CAN data frame도 같은 형식으로 계속 전달합니다. 따라서 외부에서 broadcasting을 켰거나 다른 node가 송신 중이면 요청 직후의 첫 `0xA0`이 요청 대상의 응답이라는 보장은 없습니다. 요청-응답 일치를 판단해야 한다면 PC 측에서 CAN ID와 PID를 보고 판단해야 합니다.

| Byte | Field | Type | Description |
| --- | --- | --- | --- |
| 0 | `header[0]` | `uint8` | `0xAA` |
| 1 | `header[1]` | `uint8` | `0x55` |
| 2 | `length` | `uint8` | `15` |
| 3 | `type` | `uint8` | `0x20` |
| 4 | `seq` | `uint8` | CLI transaction sequence |
| 5-6 | `can_id` | `uint16 LE` | Standard 11-bit CAN ID, `0x000..0x7FF` |
| 7 | `dlc` | `uint8` | `0..8` |
| 8-15 | `data` | `uint8[8]` | CAN data bytes. `dlc`보다 뒤는 0 padding |
| 16-17 | `timeout_ms` | `uint16 LE` | CAN 응답 대기 timeout. MCU에서 최대 100 ms로 제한 |
| 18 | `checksum` | `uint8` | XOR checksum |

```text
AA 55 0F 20 seq id_lo id_hi dlc d0 d1 d2 d3 d4 d5 d6 d7 timeout_lo timeout_hi checksum
```

MDROBOT PID frame을 보낼 때 CLI는 다음처럼 구성합니다.

```text
CAN ID = (MID << 8) | driver_id
DLC = 8
DATA[0] = PID
DATA[1..7] = PID data, 남는 byte는 0
```

기본 MID는 문서 예제와 기존 속도 제어 구현에 맞춰 `0`입니다. MD200T A/B의 driver ID는 각각 `1`, `2`입니다.

### CAN Bridge Response: MCU to PC CLI

Teensy가 CAN bus에서 받은 frame과 bridge 요청 처리 status를 PC로 터널링하는 패킷입니다. Bridge mode에서는 unsolicited `0x81`/`0x82`를 보내지 않고, CAN RX frame을 `0xA0`으로 계속 보냅니다. MCU는 driver ID, response MID, PID를 기준으로 frame을 거르지 않습니다.

| Byte | Field | Type | Description |
| --- | --- | --- | --- |
| 0 | `header[0]` | `uint8` | `0xAA` |
| 1 | `header[1]` | `uint8` | `0x55` |
| 2 | `length` | `uint8` | `14` |
| 3 | `type` | `uint8` | `0xA0` |
| 4 | `seq` | `uint8` | 가장 최근에 정상 수신한 `0x20` 요청의 sequence. 연속 수신 frame도 이 값을 사용 |
| 5 | `status` | `uint8` | `0=ok`, `1=can_tx_failed`, `2=can_rx_timeout`, `3=invalid_request` |
| 6-7 | `can_id` | `uint16 LE` | 수신된 CAN ID. 실패 시 `0` |
| 8 | `dlc` | `uint8` | 수신 DLC. 실패 시 `0` |
| 9-16 | `data` | `uint8[8]` | 수신 CAN data. 실패 또는 `dlc` 뒤는 0 |
| 17 | `checksum` | `uint8` | XOR checksum |

```text
AA 55 0E A0 seq status id_lo id_hi dlc d0 d1 d2 d3 d4 d5 d6 d7 checksum
```

`status=0`은 CAN frame이 포함되어 있음을 뜻합니다. `status=2 (can_rx_timeout)`은 해당 `0x20` 송신 후 timeout 안에 frame이 없었다는 요청 단위 알림일 뿐이며 monitoring을 종료하지 않습니다. 이후 CAN frame이 들어오면 다시 `status=0` packet으로 전달됩니다. Serial TX buffer에 한 packet을 쓸 공간이 없을 때에는 control loop를 block하지 않기 위해 해당 tunnel packet을 생략할 수 있으므로, CLI는 안전 제어용 recorder가 아니라 raw CAN monitoring과 진단 용도로 사용합니다.

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

## Python CAN 설정 CLI

`src/can_cli.py`는 Teensy를 USB Serial-CAN bridge로 사용해 MCU가 전달하는 packet을 계속 표시하고, 같은 화면의 마지막 `can>` 줄에서 CAN 송신 명령을 받는 대화형 도구입니다. 수신 로그가 출력되어도 작성 중인 입력 문자열을 다시 그리므로 입력이 사라지지 않습니다. MD200T A/B는 이미 `250 kbit/s`로 설정된 상태이므로 이 CLI는 CAN bitrate를 변경하지 않습니다.

의존성은 기존 `pyproject.toml`의 `pyserial`을 사용합니다.

```bash
uv sync
```

모니터만 먼저 열려면 다음과 같이 실행합니다. 이 시점에 MCU가 아직 control mode라면 `0x82`도 표시될 수 있으며, 처음 `0x20` 명령을 입력한 뒤 bridge mode로 전환됩니다.

```bash
uv run python src/can_cli.py --port /dev/ttyACM0
```

실행과 동시에 첫 명령을 보낼 수도 있습니다. 기존 one-shot CLI의 `pid`/`frame` 인자 형식은 유지하지만, 전송 후 종료하지 않고 monitoring을 계속합니다.

```bash
# Driver A의 입력 전압(PID 143)을 요청하며 bridge monitor 시작
uv run python src/can_cli.py pid 1 4 143 --port /dev/ttyACM0
```

터미널 마지막 줄의 입력 형식은 다음과 같습니다. 숫자는 10진수와 `0x` 16진수를 모두 허용하며, Enter를 누르면 기존 `0x20` packet으로 MCU에 전송합니다.

```text
can> pid DRIVER_ID PID [DATA_BYTE ...] [--mid MID] [--timeout-ms N]
can> frame CAN_ID [DATA_BYTE ...] [--dlc N] [--timeout-ms N]
```

예시:

```text
# Driver A(ID 1)에 PID 207로 MOT1=+100 rpm, MOT2=-80 rpm 전송
can> pid 1 207 0x01 0x64 0x00 0x01 0xB0 0xFF 0x00

# Driver B(ID 2)에 두 채널 TQ-OFF(PID 174) 전송
can> pid 2 174 0x01 0x01 0x00 0x00 0x00 0x00 0x00

# 표준 CAN data frame 직접 송신
can> frame 0x001 0x04 0x8F --dlc 8
```

CLI는 sequence를 전송할 때마다 자동으로 증가시키며, 수신 `0xA0`을 sequence로 필터링하지 않습니다. 모든 tunnel frame은 도착 순서대로 표시됩니다.

CLI 출력 예시는 다음과 같습니다. 아래 두 RX frame은 요청 응답과 그 밖의 CAN traffic을 구분하거나 제거하지 않은 결과입니다.

```text
TX seq=0 id=0x001 dlc=8 data=04 8F 00 00 00 00 00 00
[14:03:12] RX seq=0 id=0x701 dlc=8 data=C1 00 64 00 7B 00 F4 01 pid=193
[14:03:12] RX seq=0 id=0x701 dlc=8 data=8F F0 00 00 00 00 00 00 pid=143
```

`BRIDGE status=can_rx_timeout`은 Teensy의 CAN 송신 자체는 끝났지만 지정한 시간 안에 어떤 CAN frame도 받지 못했다는 뜻입니다. monitoring은 계속됩니다. `BRIDGE status=can_tx_failed`는 ACK failure 가능성이 높으므로 transceiver, CAN_H/CAN_L, 공통 GND, 종단저항, MD200T 전원 및 250 kbit/s 설정을 먼저 확인합니다.

종료하려면 입력 줄에 `quit`, `exit`, `q` 중 하나를 입력하거나 Ctrl-C/Ctrl-D를 누릅니다. 이 명령은 PC CLI만 종료하며 MCU mode를 바꾸는 별도 packet을 보내지 않습니다. MCU를 control mode로 되돌리려면 정상 `0x01` command를 보내거나 재부팅해야 합니다.

## 프로젝트 구조

```text
.
├── platformio.ini          # PlatformIO 보드/프레임워크 설정
├── pyproject.toml          # Python 제어 스크립트 의존성
├── include/
│   ├── can_tx_schedule.hpp # 1초 polling superframe 정의
│   ├── flexcan0.hpp        # FlexCAN0 polling TX/RX API
│   ├── little_endian.hpp   # signed/unsigned 16-bit little-endian helper
│   └── md200t_can.hpp      # MD200T channel command API
├── src/
│   ├── main.cpp            # Teensy 펌웨어 main loop / ROS packet / MD200T scheduling
│   ├── flexcan0.cpp        # MK20DX256 FlexCAN0 register-level polling TX/RX implementation
│   ├── md200t_can.cpp      # MDROBOT standard CAN frame builder
│   ├── control.py          # UART 주행 명령 송신 스크립트
│   └── can_cli.py          # USB Serial-CAN bridge CLI
```

## 목표 제어 방식

Teensy는 모터를 직접 구동하지 않습니다. ROS 또는 RC 입력에서 계산한 목표 속도, enable, stop 상태를 MD200T CAN command로 변환해 송신하고, 실제 MDH100 속도 제어는 MD200T 내부 제어기를 사용합니다.

실제 장비 시험에서 확정해야 할 항목은 다음과 같습니다.

- PID 193/200 10 Hz 및 PID 143 1 Hz polling 응답률과 500 ms/3 s freshness timeout의 적정성
- RX mailbox overrun 발생 여부와 PID 143 입력전압 단위
- 채널별 direction polarity

## 주의 사항

- FlexCAN0 송수신 bitrate는 `250 kbit/s`입니다. MD200T A/B도 이미 `250 kbit/s`로 설정되어 있다는 전제입니다.
- MD200T A/B의 CAN driver ID는 각각 `1`, `2`입니다. 같은 bus에서 두 드라이버 ID가 충돌하면 안 됩니다.
- MD200T A/B의 CH1/CH2 방향 극성은 실제 배선 후 저속 테스트로 검증해야 합니다.
- CAN 통신에는 Teensy와 MD200T 사이의 CAN transceiver, CAN_H/CAN_L 배선, 공통 GND, 종단저항 검토가 필요합니다.
- CAN 설정 CLI를 사용할 때도 Teensy FlexCAN0와 MD200T A/B는 모두 `250 kbit/s` 상태여야 합니다. 이 프로젝트는 MD200T baudrate 변경 frame을 보내지 않습니다.
- Auto 모드를 사용하려면 모드 입력 PWM이 `1700 us` 이상이어야 합니다.
- RC PWM 입력은 10개 샘플 이동 평균으로 필터링됩니다.
- 현재 MCU 펌웨어는 ROS command packet과 CAN bridge request packet 형식의 바이너리 패킷만 인식합니다. 일반 텍스트 `"0.5,0.0"` 형태로 보내면 인식되지 않습니다.
- status packet, command packet, CAN bridge packet은 같은 Serial 포트를 공유하므로, PC/ROS 수신부는 binary packet framing을 기준으로 파싱해야 합니다.

## 참고 자료

- MDROBOT MD200T 제품 페이지: <https://www.mdrobot.co.kr/BLDCmotordriver-store-dualchannel/?idx=160>
- MDROBOT MDH100 제품 페이지: <https://www.mdrobot.co.kr/inwheelmotor-store/?idx=274>
