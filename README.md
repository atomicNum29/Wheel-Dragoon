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
- Basic Status Packet 2 Hz 송신
- command timeout 감지 및 timeout 시 모터 목표 RPM 0 처리
- state/error bitfield 산출
- 차동 구동식 기반 좌/우 바퀴 목표 RPM 계산
- 10 ms 주기 MD200T command 송신 스케줄링
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
```

MD200T 계층 API:

```cpp
bool md200t_set_velocity(uint8_t driver_id, int16_t rpm1, int16_t rpm2);
bool md200t_torque_off(uint8_t driver_id);
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

CAN bridge CLI에서 MD200T 응답을 받기 위해 MB1 하나를 polling RX MB로 사용한다. RX FIFO, filter, interrupt는 사용하지 않는다.

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
| RX unlock / clear | MB1 `CS/ID/WORD`를 읽고 `CAN0_TIMER`를 읽은 뒤 `CAN0_IFLAG1 = (1u << 1)` |

`can_receive()`는 `IFLAG1`이 set될 때까지 timeout polling하고, `CODE_RX_FULL` 또는 `CODE_RX_OVERRUN`인 경우 standard ID, DLC, data byte를 복사한다. 처리 후 MB1을 다시 `CODE_RX_EMPTY`로 돌린다. Extended ID, RTR frame, filter 처리는 이번 범위에 넣지 않는다.

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
| MD200T A | `1` | LF | RR | 대각 페어 |
| MD200T B | `2` | RF | LR | 대각 페어 |

각 채널의 `direction polarity`는 실제 배선 후 검증해야 합니다. 표준 송신 frame은 `MD200T CAN frame 결정` 섹션의 듀얼채널 PID를 사용합니다.

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

MCU가 ROS 노드로 2 Hz로 보내는 상태 패킷입니다. 명령 수신 여부와 무관하게 disabled, timeout, fault 상태에서도 주기적으로 송신합니다.

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

### CAN Bridge Request: PC CLI to MCU

MDAS처럼 PC 터미널에서 MD200T의 PID/data frame을 직접 보내기 위한 요청 패킷입니다. 같은 USB Serial 포트를 공유하므로 기존 ROS command/status packet과 동일한 `AA 55 length type ... checksum` framing을 사용하되, type은 `0x20`으로 분리합니다.

요청을 받은 Teensy는 표준 11-bit CAN data frame을 250 kbit/s CAN bus로 송신하고, 지정한 timeout 동안 MB1에서 응답 frame을 polling합니다. 응답이 있으면 수신 CAN frame을 PC로 돌려주고, 없으면 timeout status를 돌려줍니다.

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
| 16-17 | `timeout_ms` | `uint16 LE` | CAN 응답 대기 timeout |
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

Teensy가 CAN bridge 요청 하나에 대해 하나씩 돌려주는 응답 패킷입니다. CLI는 중간에 들어오는 기존 status packet `0x81`을 무시하고, 같은 `seq`의 `0xA0` 응답만 사용합니다.

| Byte | Field | Type | Description |
| --- | --- | --- | --- |
| 0 | `header[0]` | `uint8` | `0xAA` |
| 1 | `header[1]` | `uint8` | `0x55` |
| 2 | `length` | `uint8` | `14` |
| 3 | `type` | `uint8` | `0xA0` |
| 4 | `seq` | `uint8` | 요청 packet의 sequence |
| 5 | `status` | `uint8` | `0=ok`, `1=can_tx_failed`, `2=can_rx_timeout`, `3=invalid_request` |
| 6-7 | `can_id` | `uint16 LE` | 수신된 CAN ID. 실패 시 `0` |
| 8 | `dlc` | `uint8` | 수신 DLC. 실패 시 `0` |
| 9-16 | `data` | `uint8[8]` | 수신 CAN data. 실패 또는 `dlc` 뒤는 0 |
| 17 | `checksum` | `uint8` | XOR checksum |

```text
AA 55 0E A0 seq status id_lo id_hi dlc d0 d1 d2 d3 d4 d5 d6 d7 checksum
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

`src/can_cli.py`는 Teensy를 USB Serial-CAN bridge로 사용해 MD200T에 임의의 PID/data frame을 보내고, MD200T의 응답 frame을 터미널에 출력합니다. MD200T A/B는 이미 `250 kbit/s`로 설정된 상태이므로 이 CLI는 CAN baudrate를 변경하지 않습니다.

의존성은 기존 `pyproject.toml`의 `pyserial`을 사용합니다.

```bash
uv sync
```

MDROBOT PID frame 전송 형식:

```bash
uv run python src/can_cli.py pid DRIVER_ID PID [DATA_BYTE ...]
```

예시:

```bash
# Driver A(ID 1)에 PID 130, data 0x34 0x12 전송
uv run python src/can_cli.py pid 1 130 0x34 0x12

# Driver B(ID 2)에 PID 131, data 0x00 0x00 전송
uv run python src/can_cli.py pid 2 131 0x00 0x00

# 포트를 직접 지정
uv run python src/can_cli.py pid 1 130 0x34 0x12 --port /dev/ttyACM0
```

표준 CAN data frame을 직접 보낼 수도 있습니다.

```bash
uv run python src/can_cli.py frame 0x001 0x82 0x34 0x12 --dlc 8
```

CLI 출력은 송신 frame과 수신 frame을 그대로 보여줍니다.

```text
TX id=0x001 dlc=8 data=82 34 12 00 00 00 00 00
RX status=ok id=0x701 dlc=8 data=82 34 12 00 00 00 00 00
MDROBOT pid=130 payload=34 12 00 00 00 00 00
```

`RX status=can_rx_timeout`은 Teensy의 CAN 송신 자체는 끝났지만 지정한 시간 안에 CAN 응답 frame을 받지 못했다는 뜻입니다. 이 경우 PID가 write-only이거나, MD200T가 해당 PID에 응답하지 않거나, 응답 설정/배선/종단/ID가 맞지 않을 수 있습니다. `RX status=can_tx_failed`는 ACK failure 가능성이 높으므로 transceiver, CAN_H/CAN_L, 공통 GND, 종단저항, MD200T 전원 및 250 kbit/s 설정을 먼저 확인합니다.

## 프로젝트 구조

```text
.
├── platformio.ini          # PlatformIO 보드/프레임워크 설정
├── pyproject.toml          # Python 제어 스크립트 의존성
├── include/
│   ├── flexcan0.hpp        # FlexCAN0 polling TX/RX API
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

구현 시 확정해야 할 항목은 다음과 같습니다.

- MD200T status/fault frame 수신 여부와 error bitfield 매핑
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
