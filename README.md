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
- System Status v2 (`0x82`) 100 Hz 송신
- Legacy Basic Status (`0x81`) 생성 코드 유지, 기본 송신 비활성
- command timeout 감지 및 timeout 시 PID 174 TQ-OFF 처리
- 보호 magic을 포함한 MCU software reset command
- state/error bitfield 산출
- 차동 구동식 기반 좌/우 바퀴 목표 RPM 계산
- 10 ms 주기 Driver A/B command pair와 command-triggered wheel telemetry 수신
- Driver A/B 입력전압 PID 143의 1 Hz polling
- USB Serial-CAN bridge를 통한 MD200T PID/data 직접 송수신
- Bridge mode에서 request/response CAN RTT 반복 benchmark 및 sample별 통계 반환

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
- MD200T command-response 지연과 voltage freshness timeout의 실제 장비 검증

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

문서 부재 때문에 MCU 패키지별 핀 멀티플렉싱은 레퍼런스 매뉴얼의 64-pin pinout과 Teensyduino 코어 pin map을 교차 확인했다. MD200T는 별도 전용 매뉴얼 PDF가 없는 것으로 보고, 운용 CAN baudrate가 `500 kbit/s`라는 전제만 명시한다.

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
void can_rx_overrun_report_sent();
```

MD200T 계층 API:

```cpp
bool md200t_set_velocity(uint8_t driver_id, int16_t rpm1, int16_t rpm2);
bool md200t_torque_off(uint8_t driver_id);
bool md200t_request_pid_data(uint8_t driver_id, uint8_t target_pid);
```

`can_begin()`, `can_transmit()`, `can_receive()`의 모든 polling loop는 `micros()` 기반 timeout을 가져야 한다. timeout 대상은 low-power acknowledge 해제, freeze acknowledge 진입/해제, soft reset 완료, TX MB active/abort 대기, TX 완료 `IFLAG1` 대기, RX 완료 `IFLAG1` 대기다. 동적 메모리와 interrupt는 사용하지 않는다. 현재 구현은 `500000` bitrate만 허용하고, 다른 bitrate는 별도 timing table을 문서로 확인한 뒤 추가한다.

### FlexCAN 클록과 500 kbit/s 비트 타이밍

K20 레퍼런스 매뉴얼은 FlexCAN clock source가 `CANx_CTRL1[CLKSRC]`로 `OSCERCLK` 또는 bus clock 중 선택된다고 설명한다. Teensyduino FlexCAN 초기화 코드는 `OSC0_CR |= OSC_ERCLKEN` 후 `CAN0_CTRL1[CLKSRC]=0`을 사용해 16 MHz crystal clock을 FlexCAN source로 선택한다. 직접 구현도 이 경로를 따른다.

기본 bitrate `500000 bit/s` 설정:

| 항목 | 값 |
| --- | --- |
| FlexCAN source clock | `16,000,000 Hz` (`OSCERCLK`, `CTRL1[CLKSRC]=0`) |
| PRESDIV register | `1` |
| Clock divisor | `PRESDIV + 1 = 2` |
| Time quantum clock | `16,000,000 / 2 = 8,000,000 Hz` |
| Time quantum | `125 ns` |
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
| Actual bitrate | `16,000,000 / 2 / 16 = 500,000 bit/s` |
| `CAN0_CTRL1` timing bits | `0x017B0002` before optional non-timing bits |

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
10. `CAN0_CTRL1 = 0x017B0002`로 500 kbit/s timing을 설정한다. `CLK_SRC` bit는 0이어야 한다.
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

MD200T의 command-triggered PID 196/201, PID 143 응답 및 CAN bridge traffic을 받기 위해 MB1 하나를 polling RX MB로 사용한다. 여기서 polling RX는 FlexCAN register를 MCU loop가 확인하는 방식을 뜻하며, wheel telemetry를 PID 4로 요청한다는 의미가 아니다. RX FIFO와 interrupt는 사용하지 않는다.

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

`can_receive()`는 MB1의 `IFLAG1`이 set될 때까지 timeout polling하고 standard ID, DLC, data byte를 복사한다. 처리 후 MB1을 다시 `CODE_RX_EMPTY`로 돌린다. `CODE_RX_OVERRUN`을 감지하면 진단 보고 잔여 횟수를 10으로 설정한다. 해당 진단은 `0x82` 송신마다 한 번 차감한다. 정상 loop는 timeout 0으로 수신 frame을 drain하며 별도의 polling-response 대기 loop를 만들지 않는다.

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

`md200t_request_pid_data()`는 bridge/진단용 API로 유지하지만 CONTROL mode에서는 wheel 상태를 별도 조회하지 않는다. 사용자가 MD200T A/B의 PID149를 7로 사전 설정하면 PID207 또는 PID174에 대해 PID196/201을 반환한다. 펌웨어는 PID149를 조회·설정하지 않는다. PID 143은 이 command response에 포함되지 않으므로 firmware가 driver별 1 Hz로 계속 요청한다.

RPM은 MDROBOT 문서의 2-byte data 규칙에 따라 signed 16-bit little-endian으로 보낸다. `PID_PNT_VEL_CMD(207)` 한 프레임에 CH1/CH2 RPM을 각각 넣으며, 음수는 2의 보수 bit pattern을 그대로 직렬화한다.

`PID_PNT_TQ_OFF(174)`는 dual-channel driver에서 motor1/motor2 torque-off condition을 각각 `D0`, `D1` bit로 지정한다. `md200t_torque_off()`는 안전 정지용으로 두 채널 모두 free stop시키는 함수이므로 `D0=1`, `D1=1`, payload의 나머지는 0으로 둔다. 별도로 설정된 MD200T command response 기능에 의해 이 명령 뒤에도 PID 196/201이 반환된다.

MD200T A/B는 이미 `500 kbit/s`로 설정된 상태를 전제로 한다. 펌웨어는 baudrate 설정 frame을 보내지 않고, Teensy FlexCAN0를 `500 kbit/s`로 초기화한 뒤 바로 명령 frame을 송신한다.

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
    ├── mode_state와 관계없이 automatic PID 207/174 및 PID 143 polling 중단
    ├── unsolicited 0x81/0x82 송신 중단
    ├── 0x20 raw CAN 송신과 0xA0 CAN 수신 tunnel 처리
    └── 0x21 반복 benchmark와 0xA1 sample 결과 처리
```

BRIDGE 진입은 `mode_state == AUTO`로 제한되지 않습니다. STOP, MANUAL, AUTO 어느 위치에서도 유효한 `0x20` 또는 `0x21` packet을 받으면 BRIDGE로 전환합니다. BRIDGE에서는 RC mode switch가 계속 갱신되더라도 그 값에 따라 CAN 자동 출력이 다시 시작되거나 CONTROL로 복귀하지 않습니다. 유효한 `0x01` command를 받아야 CONTROL로 복귀하며, 복귀 직후 실제 출력은 그때의 `mode_state`가 결정합니다. 따라서 STOP이면 TQ-OFF, MANUAL이면 RC 명령, AUTO이면 수신한 `0x01` 명령을 적용합니다.

CONTROL이라고 항상 ROS target을 사용하는 것은 아닙니다. `mode_state == AUTO`일 때만 저장된 ROS command가 모터 출력에 적용됩니다. STOP과 MANUAL에서도 유효한 `0x01`은 AUTO용 command로 저장되고 BRIDGE 상태였다면 CONTROL 복귀를 일으키지만, 현재 CAN 출력에는 ROS target이 섞이지 않습니다.

RC receiver 자체의 fail-safe를 사용하므로 MCU는 별도의 PWM signal-loss timeout을 추가하지 않습니다. Loop는 ISR이 갱신하는 mode와 두 PWM 값을 iteration마다 한 번씩 snapshot하여 같은 iteration 안에서 mode가 섞이지 않게 처리합니다.

Manual 모드에서 RC PWM은 다음 범위로 변환됩니다.

- 중립값은 `1500 us`이며 `1450~1550 us` 범위(양 끝 포함)를 각 축의 데드밴드로 사용
- 데드밴드 안에서는 해당 축의 입력을 `0`으로 처리
- 두 축이 모두 데드밴드 안이면 좌/우 목표 RPM이 모두 `0`이 되어 정지
- 데드밴드 밖에서는 기존 환산 비율을 유지
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

## CAN TX scheduler와 command response

CONTROL mode의 자동 CAN 송신은 `can_transmit_if_due(micros())` 한 함수가 관리합니다. 각 10 ms cycle의 `+0 ms`에 Driver A, `+3 ms` 이후에 Driver B의 PID 207을 송신합니다. 두 frame은 같은 RPM/enable snapshot을 사용합니다. 출력이 금지된 상태에서는 같은 위치에 PID 174를 송신합니다. Driver A의 응답 timeout이나 다른 safety fault가 첫 3 ms 구간에서 검출되면 예약된 Driver B 명령은 PID 174로 전환됩니다.

MD200T A/B는 PID 207 또는 PID 174를 수신하면 별도 PID 4 요청 없이 각각 PID 196(MOT1 MONITOR)과 PID 201(MOT2 MONITOR)을 반환하도록 설정되어 있습니다. 따라서 firmware는 wheel 상태용 PID 4 frame을 생성하지 않고 들어오는 네 response만 처리합니다. 응답의 driver ID/PID는 기존 대각선 mapping `A/MOT1=LF`, `A/MOT2=RR`, `B/MOT1=RF`, `B/MOT2=LR`에 그대로 반영합니다.

각 driver command는 송신 시작 시각부터 3 ms 안에 해당 driver의 PID 196과 PID 201을 모두 받아야 합니다. 두 frame 중 하나라도 누락되거나 3 ms 이후 도착하면 `DRIVER_RX_TIMEOUT` 보고 잔여 횟수를 10으로 설정하고 누락 wheel의 validity를 0으로 내립니다. 이 검사는 MCU가 RX mailbox를 읽은 `micros()` 시각을 사용하며, 응답 frame 자체에는 command sequence가 없으므로 CAN wire상의 실제 생성 시각을 직접 측정하는 것은 아닙니다.

PID 143 입력전압은 command response에 포함되지 않으므로 기존 driver별 1 Hz 요청을 유지합니다. 1초 superframe의 cycle 0에서 Driver A, cycle 50에서 Driver B를 `+6 ms` 이후 요청하여 두 요청을 500 ms 떨어뜨립니다.

```text
cycle +0 ms  Driver A PID 207/174
  +0..3 ms  Driver A PID 196/201 response window
cycle +3 ms  Driver B PID 207/174
  +3..6 ms  Driver B PID 196/201 response window
cycle +6 ms  1초 schedule에 해당하면 Driver A 또는 B PID 143 voltage
```

CAN RX는 blocking wait하지 않습니다. 현재 command cycle의 PID 196/201 네 종류가 모두 모이면 즉시 `0x82` 송신을 예약합니다. 응답이 누락되면 두 번째 3 ms window가 끝나는 시점에 timeout bit와 validity를 담은 fallback `0x82`를 예약합니다. Loop가 늦어져 실제 command가 지연되면 고정 cycle 위치가 아니라 실제 TX 시작 시각부터 3 ms를 보장한 뒤 다음 command로 진행하며, 놓친 command를 burst로 재생하지 않습니다.

기본 100 Hz 동작은 command 200 frame/s, command-triggered PID 196/201 response 400 frame/s, voltage request/response 4 frame/s로 총 약 604 CAN frame/s입니다. 500 kbit/s Standard CAN에서 명목 bus load는 frame 길이와 bit stuffing에 따라 대략 14~17%입니다. `0x82` USB 출력은 `63 bytes × 100 Hz × 10 bits/byte = 63,000 bit/s`로 115200 bps 환산 약 54.7%이며, 기존 ROS command 50 Hz를 단순 합산하면 약 59.5%입니다. 실제 장비에서는 네 command response의 수신률, response burst에 따른 RX overrun, PID 207 jitter 및 host의 지속 수신 처리율을 함께 측정해야 합니다.

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
- 네 wheel 응답 취합 직후 System Status v2 Packet 송신 (`0x82`, 목표 100 Hz)
- Legacy Basic Status (`0x81`) serializer는 유지하지만 기본 송신은 비활성화
- command timeout 시 `TIMEOUT_STOP` 및 `COMMAND_TIMEOUT` 송신
- Stop, disable, estop, timeout 시 두 MD200T의 모든 채널에 PID 174 TQ-OFF 강제

## ROS Packet Protocol

다음 프로토콜은 ROS 패키지와 MCU 사이의 USB Serial 통신 명세입니다. 기본 운용에서는 ROS가 `0x01` command를 보내고 MCU가 네 wheel 응답 취합 직후 `0x82` System Status v2를 목표 100 Hz로 보냅니다. `0x81` Basic Status 생성 코드는 구버전 호환을 위해 남아 있지만 기본 loop의 호출은 의도적으로 주석 처리되어 있어 송신되지 않습니다.

모든 패킷은 `0xAA 0x55`로 시작합니다. Byte 2의 `length`는 `type`부터 checksum 직전 마지막 payload byte까지의 바이트 수이며, checksum byte 자체는 포함하지 않습니다. 2-byte 정수는 모두 little-endian이고 signed 값은 2의 보수 bit pattern으로 전송합니다. Checksum은 byte 0부터 checksum 직전 byte까지 모든 바이트의 XOR 값입니다.

| 방향 | Type | 기본 동작 | 용도 |
| --- | --- | --- | --- |
| ROS → MCU | `0x01` | 활성 | 주행 command 및 control mode 진입 |
| Host → MCU | `0x02` | 요청 시 | 두 driver TQ-OFF 후 MCU software reset |
| MCU → ROS | `0x82` | 활성, 목표 100 Hz | canonical MCU/MD200T 통합 상태 |
| MCU → ROS | `0x81` | 비활성 | 구버전 Basic Status 호환용 serializer |
| PC → MCU | `0x20` | bridge mode에서 요청 시 | Serial-CAN bridge request |
| MCU → PC | `0xA0` | bridge mode에서 연속 | 필터링하지 않은 CAN 수신 frame 및 bridge status |
| PC → MCU | `0x21` | bridge mode에서 요청 시 | 반복 CAN request/response benchmark 시작 |
| MCU → PC | `0xA1` | benchmark 반복마다 | PID별 RTT, 타임스탬프, 누적 timeout과 최대 두 response frame |

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

### MCU Reset Command: Host to MCU (`0x02`)

MCU 상태 전체를 명시적으로 재부팅해 초기화할 때 사용하는 software reset 명령입니다. 우발적인 짧은 packet이 reset으로 해석되지 않도록 고정 magic `A5 5A`와 정상 XOR checksum을 모두 요구합니다.

| Byte | Field | Type | Description |
| --- | --- | --- | --- |
| 0 | `header[0]` | `uint8` | `0xAA` |
| 1 | `header[1]` | `uint8` | `0x55` |
| 2 | `length` | `uint8` | `4` |
| 3 | `type` | `uint8` | `0x02` |
| 4 | `seq` | `uint8` | Host의 reset 요청 추적용 sequence. MCU 동작에는 영향 없음 |
| 5 | `magic[0]` | `uint8` | 반드시 `0xA5` |
| 6 | `magic[1]` | `uint8` | 반드시 `0x5A` |
| 7 | `checksum` | `uint8` | byte 0~6의 XOR |

```text
AA 55 04 02 seq A5 5A checksum

seq=0 예시: AA 55 04 02 00 A5 5A 06
```

정상 reset packet은 `mode_state`와 `serial_mode`에 관계없이 허용됩니다. MCU는 먼저 `serial_mode`를 BRIDGE로 바꿔 periodic CAN TX를 중단하고, Driver A와 B에 PID 174 TQ-OFF를 back-to-back으로 전송한 다음 Cortex-M4 `SCB_AIRCR.SYSRESETREQ`로 재부팅합니다. CAN 초기화 또는 TQ-OFF 송신이 실패하더라도 reset 자체는 계속 수행합니다.

Reset 직후 USB Serial 연결이 끊겼다가 다시 열리므로 별도 ACK packet은 송신하지 않습니다. Host는 포트 재연결을 reset 완료 신호로 사용해야 합니다. 이 reset은 watchdog reset이 아니므로 다음 부팅의 `WATCHDOG_RESET_DETECTED`를 설정하지 않습니다. 별도의 인증 기능은 없으므로 신뢰할 수 있는 USB Serial host에서만 사용해야 합니다.

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
| 0 | `0x0001` | `CHECKSUM_ERROR` | 수신 checksum 오류를 마지막 감지 후 `0x82` 10개에 보고 |
| 1 | `0x0002` | `COMMAND_TIMEOUT` | AUTO 정상 command가 500ms 넘게 없으면 재감지. 마지막 감지 후 `0x82` 10개에 보고 |
| 2 | `0x0004` | `DRIVER_FAULT` | 구동/안전 CAN TX failure, MD200T status, driver RX timeout의 잔여 보고가 있으면 설정 |
| 3 | `0x0008` | `EMERGENCY_STOP_ACTIVE` | E-stop 입력 활성 시 재감지. 입력 해제 후에도 잔여 보고는 유지 |
| 4 | `0x0010` | `BATTERY_LOW` | 두 전압이 fresh이고 하나라도 21,000mV 미만이면 재감지. 마지막 감지 후 `0x82` 10개에 보고 |
| 5 | `0x0020` | `SERIAL_FRAMING_ERROR` | framing 오류를 마지막 감지 후 `0x82` 10개에 보고 |
| 6 | `0x0040` | `COMMAND_OUT_OF_RANGE` | 범위 오류를 마지막 감지 후 `0x82` 10개에 보고 |
| 7 | `0x0080` | `WATCHDOG_RESET_DETECTED` | watchdog reset 진단용. 현재 값을 설정하는 source는 구현되지 않음 |
| 8 | `0x0100` | `OVER_CURRENT` | legacy 예약 bit. 현재 항상 0 |
| 9 | `0x0200` | `OVER_TEMPERATURE` | legacy 예약 bit. 현재 항상 0 |
| 10 | `0x0400` | `PARAMETER_ERROR` | legacy 예약 bit. 현재 항상 0 |
| 11-15 | `0xF800` | Reserved | 항상 0 |

### System Status Packet: MCU to ROS (`0x82`, wire version 2)

Control mode에서 PID207/174 명령에 따른 PID196/201 네 wheel 응답이 모두 모이면 상태 packet을 송신합니다. 목표는 100Hz이며 누락 시 두 번째 3ms 응답 구간 뒤에 invalid 표시를 포함한 fallback snapshot을 만듭니다. Bridge mode에서는 송신하지 않습니다.

전체 크기는 **63 bytes**, length는 **59 (`0x3B`)**, wire version은 **2**입니다. 기존 wire version 1(43 bytes)과 layout이 다르므로 호스트 파서를 함께 갱신해야 합니다. 정수는 little-endian, signed 값은 2의 보수입니다.

```text
AA 55 3B 82 02 seq state error_lo error_hi validity
voltage_a[2] voltage_b[2] snapshot_timestamp_us[4]
LF[11] RF[11] LR[11] RR[11] checksum
```

| Byte | Field | Type | 설명 |
| --- | --- | --- | --- |
| 0-1 | header | uint8[2] | AA 55 |
| 2 | length | uint8 | 59, type부터 checksum 직전까지 |
| 3 | type | uint8 | 0x82 |
| 4 | version | uint8 | 2 |
| 5 | seq | uint8 | Serial.write가 전체 packet 쓰기에 성공할 때 증가, 255 이후 0 |
| 6 | mcu_state | uint8 | 기존 상태 enum |
| 7-8 | system_error | uint16 | 기존 오류 bit 위치 유지 |
| 9 | validity | uint8 | bit 0..3: LF/RF/LR/RR, bit 4..5: A/B voltage |
| 10-11 | driver_a_voltage_mv | uint16 | mV |
| 12-13 | driver_b_voltage_mv | uint16 | mV |
| 14-17 | snapshot_timestamp_us | uint32 | 데이터 조립·USB 공간 확인 후 checksum 계산·Serial.write 직전 raw micros() |
| 18-28 | lf | 11 bytes | A/MOT1/PID196 |
| 29-39 | rf | 11 bytes | B/MOT1/PID196 |
| 40-50 | lr | 11 bytes | B/MOT2/PID201 |
| 51-61 | rr | 11 bytes | A/MOT2/PID201 |
| 62 | checksum | uint8 | byte 0..61 XOR |

패킷 데이터 조립 후 `Serial.availableForWrite()`가 전체 63bytes 이상인지 확인합니다. 공간이 있으면 snapshot 시각을 기록하고 checksum 계산 직후 `Serial.write(packet, 63)`을 호출합니다. 공간이 부족하면 다음 loop에서 최신 데이터로 다시 시도합니다. sequence 증가와 오류 잔여 보고 횟수 차감은 Serial.write가 63bytes 쓰기에 성공한 뒤 처리합니다. USB 공간 부족이나 write 실패 시 차감하지 않습니다. write 성공은 MCU USB 송신 버퍼에 기록했다는 뜻이며 host 수신 확인은 아닙니다.

#### System error (Byte 7-8)

`system_error`는 원인을 나타내는 bitfield이므로 여러 bit가 동시에 설정될 수 있습니다. `mcu_state`는 우선순위를 적용한 최종 운용 상태 하나만 나타냅니다. 따라서 `system_error != 0`이 항상 `mcu_state == FAULT`를 뜻하지는 않습니다.

| Bit | Mask | 이름 | 감지 조건 | 운용 영향 |
| --- | --- | --- | --- | --- |
| 0 | `0x0001` | `SERIAL_CHECKSUM_ERROR` | 수신 packet XOR 불일치 | 진단만 보고 |
| 1 | `0x0002` | `COMMAND_TIMEOUT` | AUTO 정상 command가 500ms 넘게 없음 | 현재 timeout 입력 동안 `TIMEOUT_STOP`, 출력 금지 |
| 2 | `0x0004` | `CAN_TX_FAILURE` | CAN 초기화, 구동/안전/bridge/benchmark TX 또는 PID143 요청 TX 실패 | 구동/안전 실패 잔여 횟수 동안 `FAULT`; 전압 요청 실패는 진단만 보고 |
| 3 | `0x0008` | `EMERGENCY_STOP_ACTIVE` | E-stop 입력 활성 | 현재 입력 동안 `ESTOP`, 출력 금지 |
| 4 | `0x0010` | `BATTERY_LOW` | 두 전압이 fresh이고 하나라도 21000mV 미만 | 잔여 횟수 동안 `FAULT`, TQ-OFF |
| 5 | `0x0020` | `SERIAL_FRAMING_ERROR` | header/length/type framing 오류 | 진단만 보고 |
| 6 | `0x0040` | `COMMAND_OUT_OF_RANGE` | command 속도 범위 오류 | 해당 command 거부 |
| 7 | `0x0080` | `WATCHDOG_RESET_DETECTED` | 현재 감지 source 미구현 | 현재는 항상 0 |
| 8 | `0x0100` | `DRIVER_STATUS_FAULT_PRESENT` | 정상 수신된 wheel raw status가 0이 아님 | 잔여 횟수 동안 `FAULT`, TQ-OFF |
| 9 | `0x0200` | `DRIVER_RX_TIMEOUT` | 명령 후 3ms 동안 해당 PID196/201 쌍이 불완전 | 잔여 횟수 동안 `FAULT`, TQ-OFF |
| 10 | `0x0400` | `VOLTAGE_STALE` | CONTROL 시작 3초 이후 한쪽 이상의 전압이 invalid | 진단 및 voltage validity 보고 |
| 11 | `0x0800` | `CAN_RX_OVERRUN` | RX mailbox overrun 실제 감지 | 진단만 보고 |
| 12-15 | `0xF000` | Reserved | 현재 사용하지 않음 | 항상 0 |

각 오류는 bool latch 대신 **정수 잔여 보고 횟수**로 관리합니다. 마지막 감지 시점부터 10개의 canonical `0x82` packet에 포함하고 자동 해제합니다.

1. 감지 시 해당 오류의 잔여 횟수를 `10`으로 설정합니다. 이미 보고 중이어도 재감지되면 다시 `10`입니다.
2. packet에는 차감 전 값이 0보다 큰 오류 비트를 설정합니다. `Serial.write()`가 전체 packet 쓰기에 성공한 뒤 각 양수 잔여 횟수를 1씩 차감합니다.
3. 따라서 새 감지가 없으면 10번째 packet까지 오류가 포함되고 11번째부터 해제됩니다. 0에서 더 차감하지 않습니다.
4. USB 공간 부족으로 packet을 전송하지 못하거나 단순 상태 조회만 한 경우 차감하지 않습니다. `0x81`, bridge, benchmark packet은 차감하지 않습니다. BRIDGE에서는 `0x82`가 중단되므로 잔여 횟수를 유지합니다.
5. command timeout·E-stop·voltage stale은 현재 조건이 참인 동안 반복해서 10으로 갱신합니다. 저전압은 fresh 전압이 낮은 동안, 드라이버 상태 오류는 새 오류 응답을 받을 때, CAN 실패·timeout·overrun은 새로 감지할 때 갱신합니다.

이는 **발생 횟수나 시간 기반 timeout이 아니라 남은 보고 횟수**입니다. `0x82`가 100Hz로 전송될 때 새 감지 없는 오류는 약 100ms 뒤에 만료되지만, 전송 지연이 있으면 더 오래 유지됩니다. 세션 전환이나 전압 조회 TX 성공만으로 남은 오류 보고를 지우지 않습니다.

구동/안전 TX 실패, driver status, driver RX timeout, 저전압은 해당 잔여 횟수가 남아 있는 동안 기존 TQ-OFF 경로로 출력을 차단합니다. 잔여 횟수가 0이 되면 이 오류에 의한 차단이 해제되고 현재 enable·E-stop·timeout 조건으로 출력 허용을 판단합니다. 별도의 복구 요청이나 재허용 handshake는 추가하지 않습니다. 전압 조회 TX 실패는 같은 CAN_TX_FAILURE bit를 사용하지만 계속 진단 전용입니다.

E-stop과 command timeout의 **현재 입력 상태는 기존 정지 판단에 유지**합니다. 입력이 해제된 뒤에도 오류 비트는 남은 횟수만큼 보고되지만 이 과거 보고 때문에 `ESTOP`/`TIMEOUT_STOP` 상태를 연장하지 않습니다. 반대로 현재 입력이 활성인데 보고 횟수만 소진되어 구동이 허용되지는 않습니다. wheel raw status byte는 드라이버 원본 값이며 이 보고 카운트다운으로 변조하지 않습니다.

Control mode에서는 PID207/174에 대한 PID196/201 응답과 driver별 1Hz PID143을 사용합니다. 추가 CAN 요청·Setup packet·누적 오류 통계 packet은 이번 변경에 포함하지 않습니다.

#### Telemetry validity (Byte 9)

`validity`는 값이 packet 안에 존재하는지를 뜻하지 않고, 해당 command의 3 ms response window 안에서 정상 수신되어 지금 사용 가능한지를 뜻합니다. Bit가 0이어도 telemetry byte에는 마지막 수신값이 남아 있을 수 있으므로 ROS는 반드시 validity bit를 먼저 확인하고, invalid 값을 현재 측정값으로 사용하면 안 됩니다.

| Bit | Mask | 대상 | 1이 되는 조건 | 0이 되는 조건 |
| --- | --- | --- | --- | --- |
| 0 | `0x01` | LF MONITOR | Driver A 명령의 3 ms window 안에 MOT1/PID 196 정상 frame 수신 | 해당 window에서 미수신 또는 control session 재시작 |
| 1 | `0x02` | RF MONITOR | Driver B 명령의 3 ms window 안에 MOT1/PID 196 정상 frame 수신 | 해당 window에서 미수신 또는 control session 재시작 |
| 2 | `0x04` | LR MONITOR | Driver B 명령의 3 ms window 안에 MOT2/PID 201 정상 frame 수신 | 해당 window에서 미수신 또는 control session 재시작 |
| 3 | `0x08` | RR MONITOR | Driver A 명령의 3 ms window 안에 MOT2/PID 201 정상 frame 수신 | 해당 window에서 미수신 또는 control session 재시작 |
| 4 | `0x10` | Driver A voltage | Driver A/PID 143 정상값 수신 후 3 s 이내 | 미수신, 3 s 초과 또는 control session 재시작 |
| 5 | `0x20` | Driver B voltage | Driver B/PID 143 정상값 수신 후 3 s 이내 | 미수신, 3 s 초과 또는 control session 재시작 |
| 6-7 | `0xC0` | Reserved | 사용하지 않음 | 항상 0 |

정상 MONITOR frame은 MDROBOT response MID 7, 올바른 driver ID, PID 196/201, DLC 8을 만족해야 합니다. 정상 전압 frame은 MID 7, 올바른 driver ID, PID 143, DLC 3 이상이고 raw 전압이 `1..655` 범위여야 합니다. PID 143 raw 단위는 0.1 V이며 MCU는 `raw × 100`으로 mV에 변환합니다. 잘못된 frame은 마지막 정상값과 timestamp를 갱신하지 않습니다.

Control mode 시작 또는 bridge mode에서 control mode로 복귀하면 모든 validity와 저장 telemetry를 0으로 초기화합니다. MONITOR에는 별도 startup grace period가 없으며 첫 command의 3 ms response window부터 timeout 검사를 적용합니다. Voltage만 처음 3 s 동안 수신 대기 grace period를 가집니다.

#### Wheel telemetry block

각 wheel은 다음 11-byte 구조입니다. 순서는 LF → RF → LR → RR입니다. CAN PID196/201의 status, RPM, 위치를 해석하고, 해당 frame을 polling RX에서 읽은 직후의 MCU 시각을 함께 저장합니다.

| Block offset | Field | Type/단위 | 설명 |
| --- | --- | --- | --- |
| +0 | status | uint8 | 기존과 동일한 raw 상태 비트 |
| +1..2 | actual_rpm | int16, rpm | 모터 회전 속도 |
| +3..6 | encoder_count | int32, count | 드라이버 raw 누적 위치값. 설정에 따라 홀/엔코더 카운트 |
| +7..10 | rx_timestamp_us | uint32, us | 이 wheel의 CAN frame을 읽은 직후 MCU 시각 |

시각은 UNIX/ROS 시간이 아닌 **raw `micros()` 값(uint32, us)**입니다. 약 71.6분마다 wrap하며 MCU에서 64bit로 확장하지 않습니다. ROS는 같은 연결 세션 내에서 `(current_us - previous_us) & 0xFFFFFFFF`로 경과 시간을 계산합니다. snapshot 대비 표본 age도 같은 modulo 뺄셈을 사용합니다. 100Hz 연속 수신을 기준으로 하며, USB 재연결·MCU 재부팅 시 host의 시간 대응을 다시 초기화합니다. CONTROL/BRIDGE 전환은 하드웨어 타이머를 초기화하지 않습니다.

`rx_timestamp_us`는 드라이버 내부 측정 시각이나 CAN 하드웨어의 실제 도착 시각이 아니라 **MCU가 수신 mailbox를 읽은 시각**입니다. 각 frame을 읽은 직후, PID 해석과 USB 처리 전에 기록합니다. 휠별 시각은 서로 다를 수 있습니다. 값 0만으로 유효성을 판단하지 말고 validity 비트를 사용합니다. invalid wheel에는 마지막 위치·RPM·시각이 남아 있을 수 있습니다. 새로운 위치 표본 여부는 수신 시각과 validity로 판단하고 동일 표본을 반복 적분하지 않습니다.

count를 미터로 바꾸려면 실제 count/rev, 감속비, 휠 반지름과 방향 부호가 필요합니다. 전원 재인가나 위치 원점 변경 후에는 이전 count와 연속이라고 가정하지 않습니다. 이번 패킷에는 드라이버 재부팅 식별값이나 위치 불연속 전용 비트가 없으므로 호스트에서 무효 구간·재연결을 경계로 차분 기준을 다시 잡아야 합니다.

현재 모드 7 packet에는 전류·제어출력 필드가 없습니다. 이 값들을 위치 카운트 bytes로 잘못 해석하지 않도록 wire version과 length를 함께 검사합니다.

ROS 수신부는 length 59와 version 2를 함께 검사하고, prefix는 `struct.Struct("<BBBBHBHHI")`, wheel은 `struct.Struct("<BhiI")`로 해석할 수 있습니다. MCU 시각은 ROS epoch와 다르므로 시간 기준 변환은 호스트에서 수행합니다. 단방향 packet의 시각만으로 USB 지연과 clock offset을 완전히 분리할 수는 없습니다.

MDROBOT CAN 통신사양 V2.1의 raw `status` bit 의미는 다음과 같습니다. 아래 bit 중 하나라도 설정된 `status != 0` 응답을 받으면 `DRIVER_STATUS_FAULT_PRESENT`의 잔여 보고 횟수를 10으로 설정하고 차량을 `FAULT`로 전환합니다.

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

다음은 모든 telemetry가 valid이고 error가 없는 wire version 2 예시입니다. `seq=0x10`, `state=ENABLED`, Driver A/B 전압은 24.0V/23.8V, snapshot은 14500us입니다. LF/RF/LR/RR 위치는 각각 123456/-123456/-654321/654321 count이며 각 수신 시각은 10700/13700/14100/11020us입니다.

```text
AA 55 3B 82 02 10 01 00 00 3F C0 5D F8 5C A4 38 00 00
00 64 00 40 E2 01 00 CC 29 00 00
00 9C FF C0 1D FE FF 84 35 00 00
00 B0 FF 0F 04 F6 FF 14 37 00 00
00 50 00 F1 FB 09 00 0C 2B 00 00
F9
```

### CAN Bridge Request: PC CLI to MCU

MDAS처럼 PC 터미널에서 MD200T의 PID/data frame을 직접 보내기 위한 요청 패킷입니다. 같은 USB Serial 포트를 공유하므로 기존 ROS command/status packet과 동일한 `AA 55 length type ... checksum` framing을 사용하되, type은 `0x20`으로 분리합니다.

요청을 받은 Teensy는 bridge mode로 전환한 뒤 표준 11-bit CAN data frame을 500 kbit/s CAN bus로 송신합니다. `0x20`의 packet layout과 timeout field는 기존 형식을 그대로 유지합니다. Bridge mode로 처음 전환할 때 두 driver에는 PID 174 TQ-OFF를 back-to-back으로 보내고 automatic PID 207/174 및 PID 143 scheduler를 중단합니다. Firmware는 broadcasting ON/OFF 명령을 자동으로 보내지 않습니다.

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

### CAN Benchmark Start: PC CLI to MCU (`0x21`)

Bridge mode에서 MD200T request/response RTT를 MCU의 `micros()` 시계로 반복 측정하는 요청입니다. 동시에 하나의 benchmark session과 하나의 outstanding CAN request만 허용합니다. 한 request에서 서로 다른 PID response를 한 개 또는 두 개까지 기다릴 수 있습니다. 각 반복의 시작 시각 간격은 `period_ms`를 기준으로 하며, 필요한 response를 모두 일찍 받아도 다음 기준 시각까지 기다립니다. loop 지연이나 Serial backpressure로 기준 시각을 놓친 경우 밀린 request를 burst로 재생하지 않고 가능한 다음 시점에 한 번만 송신합니다.

듀얼 응답 필드를 추가하면서 `0x21`/`0xA1` 길이와 layout이 변경되었으므로 MCU firmware와 CLI는 같은 revision을 사용해야 합니다. 기존 `0x20`/`0xA0` bridge layout은 변경하지 않았습니다.

각 응답 matcher는 공통 `response_can_id`와 CAN `DATA[0] == response_pid_N`이 모두 일치해야 합니다. 두 PID를 기다릴 때는 둘 다 도착해야 해당 반복이 성공합니다. 한 PID만 도착하고 timeout되면 partial response가 포함된 `status=2` 결과를 반환합니다. 중복 PID나 관련 없는 CAN frame은 benchmark 응답으로 소비하지 않고 기존 `0xA0` raw tunnel로 전달합니다.

CLI 기본 matcher는 request에 따라 자동 선택됩니다.

- PID 207 또는 PID 174 command: `(PID 193, PID 200)` 두 response
- PID 4 request: `request DATA[1]` 한 response
- 그 외 PID: request `DATA[0]` 한 response

| Byte | Field | Type | Description |
| --- | --- | --- | --- |
| 0-1 | `header` | `uint8[2]` | `AA 55` |
| 2 | `length` | `uint8` | `24 (0x18)` |
| 3 | `type` | `uint8` | `0x21` |
| 4 | `seq` | `uint8` | benchmark session sequence |
| 5-6 | `request_can_id` | `uint16 LE` | 송신할 standard 11-bit CAN ID |
| 7 | `request_dlc` | `uint8` | `0..8` |
| 8-15 | `request_data` | `uint8[8]` | 송신 data, DLC 뒤는 0 padding |
| 16-17 | `response_can_id` | `uint16 LE` | 기대하는 standard 11-bit response ID |
| 18 | `response_pid_1` | `uint8` | 첫 번째로 기대하는 response `DATA[0]` |
| 19-20 | `repeat_count` | `uint16 LE` | `1..65535` |
| 21-22 | `period_ms` | `uint16 LE` | request start-to-start 주기, `1..65535 ms` |
| 23-24 | `timeout_ms` | `uint16 LE` | 반복별 응답 제한, `1..100 ms`이며 period 이하 |
| 25 | `response_count` | `uint8` | 기대 response 수, `1` 또는 `2` |
| 26 | `response_pid_2` | `uint8` | 두 번째 기대 PID. `response_count=1`이면 0 |
| 27 | `checksum` | `uint8` | XOR checksum |

```text
AA 55 18 21 seq req_id_lo req_id_hi dlc d0 d1 d2 d3 d4 d5 d6 d7
rsp_id_lo rsp_id_hi rsp_pid1 count_lo count_hi period_lo period_hi timeout_lo timeout_hi
rsp_count rsp_pid2 checksum
```

정상 `0x21`은 BRIDGE mode로 진입합니다. BRIDGE 진입 시 안전용 PID 174가 자동 반환한 PID 193/200을 첫 측정 응답으로 오인하지 않도록 5 ms 안정화 구간 동안 CAN RX를 raw tunnel로 비운 뒤 첫 request를 시작합니다. 새 `0x21`은 진행 중 session을 교체합니다. `0x20` raw bridge request, `0x01` control command, `0x02` reset은 진행 중 benchmark를 취소합니다. 유효하지 않은 `0x21`에는 반복 index 0인 `0xA1 status=3`을 반환합니다.

### CAN Benchmark Result: MCU to PC CLI (`0xA1`)

각 반복이 모든 response 수신, timeout 또는 TX failure로 끝날 때 한 packet을 반환합니다. 두 response는 기대 PID 순서의 고정 slot에 저장되므로 실제 도착 순서가 바뀌어도 PID별 통계가 섞이지 않습니다. Serial TX 공간이 부족하면 결과를 한 개 pending slot에 보관하고 다음 반복을 시작하지 않으므로 benchmark result는 의도적으로 버리지 않습니다. 따라서 host가 충분히 빠르게 읽지 않으면 실제 request 주기가 늘어날 수 있습니다.

| Byte | Field | Type | Description |
| --- | --- | --- | --- |
| 0-1 | `header` | `uint8[2]` | `AA 55` |
| 2 | `length` | `uint8` | `52 (0x34)` |
| 3 | `type` | `uint8` | `0xA1` |
| 4 | `seq` | `uint8` | `0x21` session sequence |
| 5-6 | `iteration` | `uint16 LE` | 1부터 시작하는 현재 반복 번호. invalid request는 0 |
| 7 | `status` | `uint8` | `0=모든 response 수신`, `1=can_tx_failed`, `2=response_timeout`, `3=invalid_request` |
| 8 | `flags` | `uint8` | bit 0: 마지막 반복 결과 (`final`) |
| 9-12 | `tx_timestamp_us` | `uint32 LE` | `can_transmit()` 호출 직전의 MCU `micros()` 값 |
| 13-16 | `rx1_timestamp_us` | `uint32 LE` | PID1 response 수신 직후 `micros()`. 미수신은 0 |
| 17-20 | `rtt1_us` | `uint32 LE` | PID1의 unsigned `rx1 - tx`. 미수신은 0 |
| 21-22 | `timeout_count` | `uint16 LE` | session 누적 response timeout 횟수 |
| 23-24 | `tx_failure_count` | `uint16 LE` | session 누적 CAN TX failure 횟수 |
| 25-26 | `response_can_id` | `uint16 LE` | matcher에 설정한 공통 response CAN ID |
| 27 | `response1_dlc` | `uint8` | PID1 response DLC. 미수신은 0 |
| 28-35 | `response1_data` | `uint8[8]` | PID1 response data, DLC 뒤는 0 |
| 36 | `response_count` | `uint8` | 기대 response 수, `1` 또는 `2` |
| 37 | `received_mask` | `uint8` | bit 0: PID1 수신, bit 1: PID2 수신 |
| 38-41 | `rx2_timestamp_us` | `uint32 LE` | PID2 response 수신 직후 `micros()`. 미수신은 0 |
| 42-45 | `rtt2_us` | `uint32 LE` | PID2의 unsigned `rx2 - tx`. 미수신은 0 |
| 46 | `response2_dlc` | `uint8` | PID2 response DLC. 미수신은 0 |
| 47-54 | `response2_data` | `uint8[8]` | PID2 response data, DLC 뒤는 0 |
| 55 | `checksum` | `uint8` | XOR checksum |

```text
AA 55 34 A1 seq iter_lo iter_hi status flags
tx_us[4] rx1_us[4] rtt1_us[4] timeout_count[2] tx_failure_count[2]
rsp_id[2] dlc1 rsp1_data[8] rsp_count received_mask
rx2_us[4] rtt2_us[4] dlc2 rsp2_data[8] checksum
```

`micros()`는 약 71.6분마다 wrap하지만 차이는 unsigned 32-bit 뺄셈으로 계산하므로 각 timeout/RTT가 `2^32 us`보다 짧은 현재 범위에서는 정상입니다. 타임스탬프는 MCU 부팅 이후 상대 시간이며 PC wall-clock이 아닙니다. `rtt1_us`와 `rtt2_us`는 같은 TX timestamp를 기준으로 각 PID가 수신될 때까지의 시간입니다. CLI의 `round_rtt_us`는 둘 중 큰 값, 즉 필요한 response가 모두 모일 때까지의 시간이고 `response_gap_us`는 두 RTT의 절댓값 차이입니다.

Driver A에 PID 207을 보내고 자동 반환되는 PID 193/200을 10 ms 주기, 5 ms timeout으로 100회 측정하는 예시는 다음과 같습니다. `--response-id`, `--response-pid`, `--response-pid2`를 생략하면 `0x701`, `193`, `200`을 자동 선택합니다.

```bash
uv run python src/can_cli.py benchmark 1 207 1 100 0 1 176 255 0 \
  --dlc 8 --count 100 --period-ms 10 --timeout-ms 5
```

PID 174도 같은 자동 matcher를 사용합니다. 단일 response를 측정하려면 `--response-pid`만 지정하고, 명시적으로 두 개를 측정하려면 `--response-pid`와 `--response-pid2`를 함께 지정합니다. 이미 monitor가 열린 interactive prompt에서도 `benchmark` 이후 부분을 같은 형식으로 입력할 수 있습니다.

CLI는 새 benchmark 명령을 보낼 때 해당 sequence의 통계를 초기화하고, `0xA1`의 MCU timestamp로 PID별 RTT/RX 간격, round RTT, 두 response 간격, command TX 간격과 round 완료 간격을 누적합니다. timeout 때 한 PID만 도착한 경우 그 PID의 RTT/RX 표본은 유지하고 `partial_rounds`도 증가시킵니다. 따라서 어느 PID가 빠졌는지 PID별 표본 수로 구분할 수 있습니다. 중간 timeout/TX failure가 있어도 TX 간격은 계속 측정되며, PID별 RX 간격은 직전 해당 PID 응답부터 다음 응답까지의 공백을 포함합니다.

`final` 결과를 받으면 MCU가 보고한 누적 timeout/TX failure와 수집한 sample로 다음과 같은 요약을 한 번 출력합니다. 모든 분포의 `p50/p95/p99`는 정렬된 sample 사이를 선형 보간하며 `stddev`는 모집단 표준편차입니다. 중간에 새 benchmark를 시작하면 이전 통계는 버리고 새 session으로 초기화하며, raw `pid`/`frame` 명령은 MCU benchmark를 취소하므로 CLI 통계도 함께 초기화합니다.

```text
BENCH SUMMARY seq=0 attempts=100/100 complete_rounds=98 partial_rounds=1 timeouts=2 tx_failures=0 success_rate=98.00%
  pid193_rtt_us[n=99 min=312 avg=428.6 p50=417.0 p95=561.4 p99=603.2 max=619 stddev=54.8]
  pid200_rtt_us[n=98 min=571 avg=702.1 p50=694.0 p95=811.2 p99=840.4 max=851 stddev=62.3]
  round_rtt_us[n=98 min=571 avg=702.1 p50=694.0 p95=811.2 p99=840.4 max=851 stddev=62.3]
  response_gap_us[n=98 min=201 avg=273.5 p50=270.0 p95=319.1 p99=334.2 max=340 stddev=29.4]
  tx_interval_us[n=99 min=9960 avg=10001.2 p50=10000.0 p95=10018.0 p99=10031.4 max=10042 stddev=12.1]
  pid193_rx_interval_us[n=98 ...]
  pid200_rx_interval_us[n=97 ...]
  round_rx_interval_us[n=97 ...]
```

`attempts`의 앞 값은 마지막 `0xA1`의 iteration이고 뒤 값은 CLI가 요청한 count입니다. `complete_rounds`와 성공률은 MCU의 최종 누적 timeout/TX failure를 기준으로 계산합니다. 각 PID의 RTT 표본 수는 실제 해당 PID를 받은 횟수이므로 partial round에서는 서로 다를 수 있습니다.

듀얼 result는 반복당 56 bytes입니다. 115200 baud에서 10 ms 주기는 약 48.6%를 사용하지만, 5 ms는 결과 packet만으로 약 97.2%여서 터미널 출력과 USB/host 지연에 매우 민감합니다. 2 ms는 이 baudrate의 물리적 전송량을 초과합니다. 자동 응답 벤치마크는 우선 10 ms 이상으로 실행하는 것을 권장합니다.

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

두 MD200T를 TQ-OFF한 뒤 MCU를 software reset하려면:

```bash
uv run python src/control.py --reset --port /dev/ttyACM0
```

Reset 명령에는 속도 인자와 `--disable`/`--estop`을 함께 사용하지 않습니다. 명령 전송 후 USB 장치가 재연결될 때까지 기다려야 합니다.

예시:

| 명령 | 의미 |
| --- | --- |
| `uv run python src/control.py 0.5 0` | `0.5 m/s` 전진 |
| `uv run python src/control.py 0 1.0` | `1.0 rad/s` 제자리 회전 |
| `uv run python src/control.py 0 0` | enable 상태의 정지 명령 |

## Python CAN 설정 CLI

`src/can_cli.py`는 Teensy를 USB Serial-CAN bridge로 사용해 MCU가 전달하는 packet을 계속 표시하고, 같은 화면의 마지막 `can>` 줄에서 CAN 송신 명령을 받는 대화형 도구입니다. 수신 로그가 출력되어도 작성 중인 입력 문자열을 다시 그리므로 입력이 사라지지 않습니다. MD200T A/B는 이미 `500 kbit/s`로 설정된 상태이므로 이 CLI는 CAN bitrate를 변경하지 않습니다.

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

`BRIDGE status=can_rx_timeout`은 Teensy의 CAN 송신 자체는 끝났지만 지정한 시간 안에 어떤 CAN frame도 받지 못했다는 뜻입니다. monitoring은 계속됩니다. `BRIDGE status=can_tx_failed`는 ACK failure 가능성이 높으므로 transceiver, CAN_H/CAN_L, 공통 GND, 종단저항, MD200T 전원 및 500 kbit/s 설정을 먼저 확인합니다.

종료하려면 입력 줄에 `quit`, `exit`, `q` 중 하나를 입력하거나 Ctrl-C/Ctrl-D를 누릅니다. 이 명령은 PC CLI만 종료하며 MCU mode를 바꾸는 별도 packet을 보내지 않습니다. MCU를 control mode로 되돌리려면 정상 `0x01` command를 보내거나 재부팅해야 합니다.

## 프로젝트 구조

```text
.
├── platformio.ini          # PlatformIO 보드/프레임워크 설정
├── pyproject.toml          # Python 제어 스크립트 의존성
├── include/
│   ├── can_tx_schedule.hpp # 10 ms wheel phase와 1초 voltage schedule 정의
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

- PID 207/174 command에 대한 PID 193/200 자동 response 수신률과 3 ms response timeout의 적정성
- command response burst에 따른 RX mailbox overrun 발생 여부, PID 143 driver별 1 Hz 응답률과 입력전압 단위
- 채널별 direction polarity

## 주의 사항

- FlexCAN0 송수신 bitrate는 `500 kbit/s`입니다. MD200T A/B도 이미 `500 kbit/s`로 설정되어 있다는 전제입니다.
- MD200T A/B의 CAN driver ID는 각각 `1`, `2`입니다. 같은 bus에서 두 드라이버 ID가 충돌하면 안 됩니다.
- MD200T A/B의 CH1/CH2 방향 극성은 실제 배선 후 저속 테스트로 검증해야 합니다.
- CAN 통신에는 Teensy와 MD200T 사이의 CAN transceiver, CAN_H/CAN_L 배선, 공통 GND, 종단저항 검토가 필요합니다.
- CAN 설정 CLI를 사용할 때도 Teensy FlexCAN0와 MD200T A/B는 모두 `500 kbit/s` 상태여야 합니다. 이 프로젝트는 MD200T baudrate 변경 frame을 보내지 않습니다.
- Auto 모드를 사용하려면 모드 입력 PWM이 `1700 us` 이상이어야 합니다.
- RC PWM 입력은 10개 샘플 이동 평균으로 필터링됩니다.
- 현재 MCU 펌웨어는 ROS command, MCU reset, CAN bridge request 형식의 바이너리 패킷만 인식합니다. 일반 텍스트 `"0.5,0.0"` 형태로 보내면 인식되지 않습니다.
- status packet, command packet, CAN bridge packet은 같은 Serial 포트를 공유하므로, PC/ROS 수신부는 binary packet framing을 기준으로 파싱해야 합니다.

## 참고 자료

- MDROBOT MD200T 제품 페이지: <https://www.mdrobot.co.kr/BLDCmotordriver-store-dualchannel/?idx=160>
- MDROBOT MDH100 제품 페이지: <https://www.mdrobot.co.kr/inwheelmotor-store/?idx=274>
