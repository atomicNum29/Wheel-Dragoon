# USB 프로토콜

[문서 안내](README.md) · [호스트 도구](can-cli-windows.md) · [CAN 응답 의미](can-rx-fifo.md)

현재 바이트 규격의 기준 문서다. 소스: [serial_protocol.hpp](../include/serial_protocol.hpp),
[serial_protocol.cpp](../src/serial_protocol.cpp), [can_debug.cpp](../src/can_debug.cpp),
[control.cpp](../src/control.cpp). 타이밍 패킷 필드는 [타이밍 진단](timing-diagnostics.md)에 둔다.
ROS 측 구현 절차와 파서 예시는 [ROS USB 수신 가이드](ros-usb-integration.ko.md)를 따른다.
ROS 제어는 새 `0x03` v·θ/PIVOT 명령을 사용하며, RC AUTO에서만 구동에 적용한다.
기존 `0x01` v·ω 명령은 폐기한 규격이며 수신해도 구동·timeout 갱신에 사용하지 않는다.

## 공통 프레이밍과 지원 범위

| Offset | 크기 | 내용 |
| ---: | ---: | --- |
| 0 | 1 | `0xAA` |
| 1 | 1 | `0x55` |
| 2 | 1 | length: type부터 마지막 payload까지의 바이트 수 |
| 3 | 1 | type |
| 4.. | 가변 | payload |
| 마지막 | 1 | 앞선 모든 바이트의 XOR |

전체 크기는 `length + 4`다. 다바이트 정수는 little-endian이며 offset은 프레임 시작 기준이다.
아래 표의 `u8/u16/u32`, `i16/i32`는 unsigned/signed 정수의 비트 수다.
USB는 패킷 경계가 보존되는 메시지 API가 아니므로 host는 length로 분리하고 checksum을 확인해야 한다.

| 방향 | type | length / 전체 크기 | 지원 |
| --- | --- | --- | --- |
| Host → MCU | `0x03` 제어 v1 | 11 / 15 | 모든 프로필, 구동은 RC AUTO에서만 |
| Host → MCU | `0x01` 이전 제어 | 7 / 11 | 프레임 전체 소비 후 framing 오류, 제어 적용 안 함 |
| Host → MCU | `0x02` 보호 리셋 | 4 / 8 | 모든 프로필 |
| MCU → Host | `0x85` 휠 피드백 v1 | 16 / 20 | 모든 프로필의 CONTROL, 유효 CAN 응답마다 |
| MCU → Host | `0x86` 전압 피드백 v1 | 11 / 15 | 모든 프로필의 CONTROL, 유효 CAN 응답마다 |
| MCU → Host | `0x87` MCU 상태·지령 v1 | 25 / 29 | 모든 프로필의 CONTROL, 목표 100Hz |
| Host → MCU | `0x20` CAN 브릿지 | 15 / 19 | debug, debug_timing |
| MCU → Host | `0xA0` CAN 브릿지 응답 | 14 / 18 | debug, debug_timing |
| Host → MCU | `0x21` CAN benchmark | 24 / 28 | debug, debug_timing |
| MCU → Host | `0xA1` benchmark 결과 | 52 / 56 | debug, debug_timing |
| MCU → Host | `0x84` Timing v1 | 59 / 63 | timing, debug_timing의 CONTROL |

디버그가 없는 프로필은 알려진 `0x20`/`0x21` 프레임 전체를 소비한 뒤 framing 오류로 보고한다.
모드 전환·CAN 전송·디버그 응답은 없다. `0x81`, 기존 `0x82`, `0x83`은 현재 송신하지 않는다.
MCU 수신 파서는 위 요청의 정해진 length/type 조합만 허용한다.

## 제어 `0x03`, version 1

| Offset | 타입 | 내용 |
| ---: | --- | --- |
| 4 | u8 | version=1 |
| 5 | u16 | 명령 seq, 65535 다음 0 |
| 7 | u8 | bit0 enable, bit1 estop, bit2 PIVOT, bit3..7 예약 0 |
| 8 | i16 | v, milli-m/s; −2000..2000 |
| 10 | i16 | 가상 중앙 휠 θ, milli-rad; 범위는 아래 설명 |
| 12 | i16 | PIVOT ω, milli-rad/s; −5000..5000 |
| 14 | u8 | checksum |

bit2=0은 일반 v·θ 주행이며 ω는 0이어야 한다. bit2=1은 PIVOT이며 v=θ=0이어야 한다.
θ는 RC와 같은 기하에서 내륜 ±70°에 대응하는 중앙각 이내여야 한다. 현재 약 ±0.845rad이며,
정수 상한은 `floor(rc_maximum_center_angle(geometry) * 1000)`이다. 범위 밖 값을 잘라 수락하지 않는다.
검증은 estop/enable 해제 명령에도 적용한다. 정지 전용 명령은 사용하지 않는 속도·각도 값을 0으로 보낸다.
checksum 오류는 error bit0, version/예약 flags 오류는 bit5, 범위·운동 모드 필드 조합 오류는 bit6이다.

유효 프레임은 디버그를 취소하고 CONTROL로 전환한다. 명령 저장·시각 갱신·estop 처리는
RC AUTO이고 POWERDOWN/드라이버 시작 대기가 아닐 때만 수행한다. MANUAL/PIVOT에서는 RC 입력을 유지한다.
AUTO 진입 시 이전 명령과 조립 중 RX 프레임, 그 시점에 MCU USB 버퍼에 이미 있는 바이트를 버린다.
CONTROL 세션 초기화와 BRIDGE 진입도 저장 명령을 무효화한다. AUTO는 이후 새 명령을 기다린다.

연속 수신 중에는 `delta=(new_seq-last_seq)&0xffff`가 1..32767인 명령만 수락한다.
중복·역순은 error bit5로 보고하며 timeout을 갱신하지 않는다. 최초 명령, AUTO/CONTROL 재진입,
500ms 초과 timeout 후 첫 명령은 새로운 seq 기준을 설정한다. seq gap은 허용한다.
seq는 전송 시각이 아니며 호스트/USB에 오래 대기한 명령의 나이를 판별하지 못한다.
호스트는 송신 backlog를 쌓지 말고 현재 목표만 주기적으로 갱신해야 한다. ACK나 수락 seq echo는 없다.

enable=1, estop=0, 마지막 수락 후 500ms 이내이면 기존 모션 스케줄러에 전달한다.
새 명령이 없거나 500ms를 초과하면 구동을 차단하고 조향은 유지한다. 정상 v=0/PIVOT ω=0은
enable=1을 유지한 채 보내며 기존 스케줄러로 감속한다. enable=0은 감속 보간을 우회해 TQ-OFF 경로를 사용한다.
timeout 후 새 유효 enable=1, estop=0 명령으로 재개할 수 있다.

estop 활성화 시 두 드라이버 TQ-OFF를 주기 슬롯을 기다리지 않고 시도하며, 이후 주기 송신도 TQ-OFF다.
조향은 PIVOT 반대 자세인 좌 +atan2(L,Tf/2), 우 −atan2(L,Tf/2), 현재 약 +62.4°/−62.4°로
기존 180°/s 제한을 적용해 이동한다. 모션 상태는 EMERGENCY_STOP=5이고 적용 차량 지령은 무효다.
최초 모션 준비 대기·보정 유효성·조향 범위 검사는 유지한다. 이 자세의 제동 효과나 실제 도달은 측정하지 않는다.
통신 timeout이 추가로 발생해도 활성 estop과 해당 자세 요청은 유지한다. 새 유효 estop=0 명령으로 해제한다.
해제 후 enable=1이면 BRAKING의 정지 dwell → STEERING → SETTLING을 거쳐 주행으로 복귀한다.
enable=0이면 그때의 조향각을 유지한다. RC 모드 변경은 AUTO 명령을 무효화하며 새 RC 모드가 우선한다.
USB로 RC 모드 자체를 선택하는 패킷은 없다.
자세한 제어 권한은 [RC 제어](rc-steering.ko.md)를 따른다.

## 보호 리셋 `0x02`

| Offset | 내용 |
| ---: | --- |
| 4 | seq, host 추적용이며 MCU에서 사용하지 않음 |
| 5..6 | magic `A5 5A` |
| 7 | checksum |

checksum과 magic이 유효하면 디버그를 취소하고 RESETTING으로 전환하여 일반 제어를 멈춘다.
POWERDOWN·드라이버 시작 대기가 아니면 두 드라이버 TQ-OFF를 시도한다. TX 실패 여부와 무관하게
MCU 소프트웨어 리셋을 진행하며 ACK는 없다. USB 연결이 다시 열려야 할 수 있다.

## 휠 피드백 `0x85`, version 1

| Offset | 타입 / 크기 | 내용 |
| ---: | --- | --- |
| 4 | u8 | version=1 |
| 5 | u16 | 휠별 생성 seq, 65536에서 순환 |
| 7 | u8 | 휠 ID: 0 LF, 1 RF, 2 LR, 3 RR |
| 8 | u32 | CAN ISR 수신 `micros()` |
| 12 | u8 | MD200T 원시 상태 |
| 13 | i16 | 실제 RPM 원시값 |
| 15 | i32 | encoder count 원시값 |
| 19 | u8 | checksum |

RPM·encoder는 원시값이며 거리·차체 속도로 보정한 값이 아니다.
상태 byte는 드라이버에서 받은 값이다. 펌웨어는 비영 여부로 fault를 판단하며 개별 bit의 물리 조건을 검증하지 않는다.
기존 DLC·드라이버 ID·응답 창 검사를 통과한 응답만 등록한다. 모터 상태가 비영인 응답도 전송한다.
네 휠이 모일 때까지 기다리지 않으며, RX 시각은 모터 내부 측정 시각이 아니다.
조향 실측값·전류는 이 패킷에 없다. 같은 응답의 RPM·엔코더·상태는 한 프레임에 보존한다.

## 전압 피드백 `0x86`, version 1

| Offset | 타입 | 내용 |
| ---: | --- | --- |
| 4 | u8 | version=1 |
| 5 | u16 | 드라이버별 생성 seq |
| 7 | u8 | 드라이버 ID: 0 A, 1 B (CAN ID 자체가 아님) |
| 8 | u32 | CAN ISR 수신 `micros()` |
| 12 | u16 | 전압 mV |
| 14 | u8 | checksum |

기존 PID143 검증 후 등록한다. 유효 원시값 1..655에 100을 곱한 mV이며 실제 분해능은 100mV다.

## MCU 상태·지령 `0x87`, version 1

| Offset | 타입 | 내용 |
| ---: | --- | --- |
| 4 | u8 | version=1 |
| 5 | u16 | 상태 생성 seq |
| 7 | u8 | MCU state, 아래 표 |
| 8 | u8 | 하위 4bit RC mode, 상위 4bit 모션 상태 |
| 9 | u16 | 상태 flags, 아래 표 |
| 11 | u16 | system error bits, 아래 표 |
| 13 | u8 | 휠·전압 validity bits |
| 14 / 16 / 18 | i16 각각 | 입력 목표 v / θ / ω, 단위 0.001m/s / 0.001rad / 0.001rad/s |
| 20 / 22 / 24 | i16 각각 | 적용 지령 v / θ / ω, 입력과 동일한 단위 |
| 26 | u8 | 적용 모드: MANUAL=1, PIVOT=4, 표현 불가/무효=255 |
| 27 | u8 | CONTROL 세션 번호, 255 다음 0으로 순환 |
| 28 | u8 | checksum |

RC mode는 STOP=0, MANUAL=1, AUTO=2, POWERDOWN=3, PIVOT=4, FAILSAFE=5다.
수신기 failsafe에서 정상 USB AUTO 권한을 유지하는 동안 mode는 AUTO다. 제어 신호 상실
비상 정지와 연속 5회/누적 30회·timeout 지속 300 ms 조건, POWERDOWN → STOP 복구 절차는
[RC 제어 문서](rc-steering.ko.md#제어-신호-상실-비상-정지)를 따른다.
모션 상태는 STARTUP=0, DRIVING=1, BRAKING=2, STEERING=3, SETTLING=4, EMERGENCY_STOP=5다.
`mode = packet[8] & 0x0F`, `transition = (packet[8] >> 4) & 0x0F`로 분리한다.
예를 들어 PIVOT/STEERING은 `0x34`, AUTO/EMERGENCY_STOP은 `0x52`다.
RC mode의 6..15와 모션 상태의 6..15는 예약이다.

시간·uptime·누락 누적값·개별 휠 지령은 싣지 않는다. 이전 미확정 초안은 이 29바이트 규격으로 대체했다.

상태 취득은 main loop의 모션 갱신·CAN 송신 스케줄 처리 후이며, 유효 응답 유무와 독립적인 목표 10ms 주기다.
POWERDOWN·드라이버 시작 대기에도 송신하고 BRIDGE/RESETTING에서는 일반 세 종류를 생성하지 않는다.
루프 지연 시 밀린 슬롯을 재생하지 않고 최신 상태 하나만 생성한다. 지연 뒤 다음 기한은 실제 생성 시각+10ms다.

| Flag bit | 의미 |
| ---: | --- |
| 0 | 제어부 motor_enabled |
| 1 | 모션 ready |
| 2 | 모션 feasible (마지막 목표 평가 결과) |
| 3 | 모션 limited: RPM 크기 상한 적용, 변화율 제한 전체를 뜻하지 않음 |
| 4 | 선택된 목표 필드 유효; RC MANUAL/PIVOT 또는 timeout 전의 수락된 AUTO 명령 |
| 5 | 활성 host estop 또는 제어 신호 상실 비상 정지 |
| 6 | 활성 host command timeout |
| 7 | driver_powerdown |
| 8 | driver_starting |
| 9 | 현재 CAN 구동 허용 gate; 0이면 적용 지령을 실제 구동 중이라고 해석하지 않음 |
| 10 | 조향 보정 유효; 0이면 각도 필드를 실제 PWM 출력으로 해석하지 않음 |
| 11 | 적용 지령 v·θ·ω와 적용 모드가 유효함 |
| 12 | 목표 운동 모드 PIVOT; bit4=1일 때 해석, 0이면 일반 v·θ |
| 13 | RC_SIGNAL_LOST: 최신 유효 모드 펄스가 1900 µs 이상인 수신기 failsafe 상태 |
| 14..15 | 예약 0 |

`RC_SIGNAL_LOST`는 제어 권한·비상 정지 래치와 독립적인 현재 RC 신호 상실 표시다.
수신기는 송신기 신호 상실 시 모드 채널에 2000 µs를 계속 출력하며, 최신 유효 원시 펄스의
모드가 failsafe인지 보고한다. AUTO 제어권을 유지하고 `DRIVE_ALLOWED=1`이어도 이 bit는 1이다.
정상 범위의 RC 모드 펄스가 돌아오면 bit는 0으로 복귀하며, 비상 정지 래치가 남아 있어도
RC 복구 사실을 표시한다. 펄스 수신 중단 타이머나 비상 정지 이력 플래그는 아니다.
패킷 크기(29바이트), flags 위치(offset 9, u16), 버전은 기존과 같다.
CLI 상태 출력에서도 `rc_signal_lost=0/1`로 표시한다.

MANUAL 목표는 후륜 축 중앙 v와 가상 전륜 중앙 θ다. PIVOT은 v=θ=0, ω가 유효하다.
AUTO에서도 bit4/12로 목표의 유효성과 운동 모드를 판별한다. RC mode=AUTO만으로 일반/PIVOT을 구분할 수 없다.
AUTO의 enable=0 또는 estop=1이어도 timeout 전 수락된 목표는 보고하므로, bit4를 구동 허용으로 해석하지 않는다.
timeout 후에는 bit4/12=0이고 목표는 0이다. 이는 모든 USB 수신 명령의 echo가 아니다.
적용 지령은 스케줄러의 `scheduled` 상태다. 적용 MANUAL은 v·θ, 적용 PIVOT은 ω를 사용하고 나머지 필드는 0이다.
모드 변경을 위한 BRAKING 중에는 이전 적용 모드가 유지되므로 RC mode와 적용 모드가 다를 수 있다.
일반 DRIVING과 표현 가능한 BRAKING에서 적용 지령을 보고한다. 실제 두 조향축을 직접 보간하는
STEERING/SETTLING/EMERGENCY_STOP 중에는 bit11=0, 적용 모드=255, 적용 v·θ·ω=0으로 보고한다. 초기화·잘못된 보정/설정도 무효다.
이 무효값을 직진·정지 자세로 해석하지 않는다. 그 상태에서는 중앙 조향각만으로 좌우 자세를 복원할 수 없다.
적용 지령은 실제 측정값, CAN A/B 라운드 스냅샷, TX 완료·드라이버 적용 ACK가 아니다.
호스트는 같은 기하 모델로 휠 출력을 계산할 수 있지만, 0.001 단위 양자화 때문에 MCU 내부 float와 완전히 같지는 않다.

Validity bit0..3은 LF/RF/LR/RR, bit4/5는 전압 A/B이며 bit6/7은 0이다.
유효하지 않은 항목의 숫자가 0이거나 최신이라고 가정하지 않는다.
상태 패킷에는 시각이 없다. 최신 상태의 validity와 개별 피드백의 수신 시각을 함께 사용한다.

### 순번·세션·전송 관리

- 휠은 휠별, 전압은 드라이버별, 상태는 단일 u16 seq를 사용한다. 등록/생성 시 증가하고 최초 값은 0이다.
  정상 패킷을 기다리느라 누락된 CAN 응답 자체에는 seq를 부여하지 않는다. 따라서 seq gap과 CAN 누락은 다른 지표다.
- CONTROL 세션 초기화 시 TX 대기 큐를 비우고 session을 증가시킨다. 전원 모드 전환·CONTROL 재진입도 포함하며,
  seq·아직 보고하지 못한 USB 오류 플래그는 유지한다. session은 MCU 재시작 때 0부터 초기화되므로 영구적으로 유일한 boot ID가 아니다.
  전송 세션 번호는 u8이므로 256회마다 순환한다. 값의 증가/감소 대신 변경 여부를 확인하며,
  수신 중단 사이 정확히 256의 배수만큼 바뀌면 같은 값으로 보일 수 있다. 내부 전송 완료 검사는 더 넓은 세션 번호를 유지한다.
- uptime이나 상태 timestamp는 보내지 않는다. 호스트는 USB 재연결·세션 변경 시 수신 기준을 재설정한다.
  seq·세션 번호만으로 모든 MCU 재시작을 확정할 수 없다. EEPROM 쓰기나 영구 boot counter는 추가하지 않았다.
- 휠 FIFO 16개, 전압 FIFO 4개, 상태 최신값 슬롯 1개, 진단 슬롯 1개와 전송 중 프레임 1개를 고정 할당한다.
  휠·전압 큐가 차면 가장 오래된 미전송 항목을 버린다. 상태는 최신값으로 교체한다. 혼잡에 의한 폐기는 error bit12로 보고한다.
- `service_tx()`가 네 종류의 슬롯을 순환 선택하며 루프당 최대 두 번 write를 시도한다. USB 여유가 없으면 다음 루프로 넘긴다.
  프레임 전송이 시작되면 남은 바이트를 먼저 보낸다. 모든 일반·디버그·타이밍 프레임이 이 송신부를 사용한다.
- 전송 시작 전 항목은 세션 초기화 시 버린다. 이미 일부 쓴 프레임은 경계를 깨지 않도록 끝낸다.
  이전 세션의 마지막 프레임이 도착할 수 있으므로 호스트는 세션 전환 시 수신 이력을 새로 설정해야 한다.
- 부분 USB 버퍼는 `send_now()`로 전송 예약한다. 이는 host read 완료나 USB 지연 상한을 보장하지 않는다.
  애플리케이션에 공간 대기 루프는 없지만 USB 코어 내부 실행시간/WCET는 이번 변경에서 측정하지 않았다.
- 오류는 플래그로만 보고하며 발생 횟수·누적값은 전송하지 않는다. USB 코어/호스트에서의 모든 유실을 검출하는 것은 아니다.

### MCU state와 오류

| 값 | state | 우선순위 높은 순서 |
| ---: | --- | ---: |
| 3 | ESTOP: 제어 신호 상실 비상 정지 활성 | 1 |
| 7 | DRIVER_POWERDOWN | 2 |
| 4 | FAULT: driver fault 또는 저전압 | 3 |
| 3 | ESTOP: 활성 host estop 입력 | 4 |
| 5 | BOOTING: boot 또는 드라이버 시작 대기 | 5 |
| 2 | TIMEOUT_STOP: 현재 활성 timeout | 6 |
| 0 | DISABLED | 7 |
| 1 | ENABLED | 8 |
| 6 | 예약 | 사용하지 않음 |

| Error bit | 의미 |
| ---: | --- |
| 0 | USB checksum 오류 |
| 1 | command timeout 보고 |
| 2 | CAN TX 실패: 안전 명령 또는 전압 조회 |
| 3 | host estop 또는 제어 신호 상실 비상 정지 보고 |
| 4 | 저전압 |
| 5 | framing / 지원하지 않는 요청 |
| 6 | 명령 속도 범위 초과 |
| 7 | 예약: watchdog 감지 소스 없음 |
| 8 | 드라이버 원시 상태 비영 |
| 9 | 드라이버 응답 창 누락 |
| 10 | 전압 stale |
| 11 | CAN RX overrun 관측 |
| 12 | USB 송신 데이터 누락: 휠·전압 큐 초과, 미전송 상태 교체, 주기 슬롯 생략, 브릿지 응답 폐기 |
| 13 | USB write 반환 길이 불일치 (부분 전송 또는 실패) |
| 14..15 | 예약 |

오류 보고 카운터와 현재 활성 timeout/estop은 별개다. 과거 오류 bit가 남아도 state는
현재 활성 입력을 기준으로 다르게 보일 수 있다. 오류 발생 시 보고 카운터는 10회로 갱신된다.
`0x87` 전체를 USB에 쓴 뒤, 해당 스냅샷과 현재 CONTROL 세션·오류 revision·CAN overrun 수가 같을 때만
보고 카운터를 한 번 감소시킨다. 대기 중 새 오류가 생겼거나 세션이 바뀌었다면 감소시키지 않는다.
휠·전압·진단 패킷 전송은 보고 카운터를 감소시키지 않는다. 오류 카운터의 구동 차단 참여는 기존과 같다.
이는 호스트 ACK·로그 저장 확인이 아니다. ENABLED도 물리적 움직임을 확인한 값이 아니다.

USB 오류 bit12/13은 별도로 래치한다. 같은 세션의 `0x87` 전체 write가 완료되었고,
스냅샷 이후 해당 오류가 다시 발생하지 않았을 때 그 bit를 해제한다. 다른 종류의 오류 bit는 독립적으로 유지한다.
전송할 수 없는 동안에는 유지하며 CONTROL 세션 초기화도 아직 보고하지 못한 USB 오류를 지우지 않는다.
정상 모드 전환·세션 초기화·명시적 디버그 취소에 따른 큐 비우기는 오류로 보고하지 않는다.
큐 여유가 없어 benchmark/타이밍 패킷을 보류했다가 다시 등록하는 것 역시 실제 폐기가 아니므로 오류가 아니다.
USB bit12/13의 보고 비트 자체는 기존 구동 차단 조건에 추가하지 않는다.
다만 해당 송신 실패·스킵 이벤트는 AUTO 구간의 USB 이상 횟수에 집계되어, 수신기 failsafe와
겹치고 연속 5회 또는 누적 30회 한계에 도달하면 제어 신호 상실 비상 정지로 이어진다.

## CAN 브릿지 `0x20` / `0xA0`

`0x20` 요청:

| Offset | 타입 | 내용 |
| ---: | --- | --- |
| 4 | u8 | seq |
| 5 | u16 | standard CAN ID, 0..0x7FF |
| 7 | u8 | DLC, 0..8 |
| 8..15 | 8 bytes | data; DLC 밖은 송신 payload에 포함하지 않음 |
| 16 | u16 | RX timeout ms, 100으로 상한 제한; 0 허용 |
| 18 | u8 | checksum |

전원 사용 가능·checksum·ID·DLC를 검사한다. 유효 요청은 benchmark를 취소하고 BRIDGE로 진입한다.
최초 진입에서는 TQ-OFF 두 건을 시도한 뒤 raw 요청을 보낸다. RX 대기는 최대 100 ms로
메인 루프를 지연시킬 수 있다. 응답 ID/PID를 매칭하지 않고 큐의 첫 CAN 프레임을 사용하므로
그 프레임이 해당 요청의 응답이라는 보장은 없다.

`0xA0` 응답:

| Offset | 타입 | 내용 |
| ---: | --- | --- |
| 4 | u8 | seq |
| 5 | u8 | status: 0 수신, 1 TX 실패, 2 RX timeout, 3 무효 요청 |
| 6 | u16 | 수신 CAN ID |
| 8 | u8 | 수신 DLC |
| 9..16 | 8 bytes | 수신 data; 남는 바이트 0 |
| 17 | u8 | checksum |

프레임이 없는 실패 응답은 ID/DLC/data가 0이다. BRIDGE에서 나중에 들어온 비매칭 프레임도
최근 유효 브릿지 세션 seq로 `0xA0`에 실을 수 있으므로 seq만으로 일대일 요청 대응을 만들지 않는다.
공통 진단 슬롯이 비어 있으면 응답을 복사하고 USB 공간이 생길 때까지 송신을 보류한다.
슬롯이 이미 차 있으면 새 응답을 버리고 USB 송신 누락 error bit12를 올린다. 폐기한 응답은 재시도하지 않는다.

## Benchmark `0x21` / `0xA1`

`0x21` 요청:

| Offset | 타입 | 내용 |
| ---: | --- | --- |
| 4 | u8 | seq |
| 5 | u16 | 요청 CAN ID |
| 7 | u8 | 요청 DLC |
| 8..15 | 8 bytes | 요청 data |
| 16 | u16 | 기대 응답 CAN ID |
| 18 | u8 | 응답 PID0 |
| 19 | u16 | 반복 수, 1..65535 |
| 21 | u16 | 주기 ms, 1..65535 |
| 23 | u16 | timeout ms, 1..100이며 주기 이하 |
| 25 | u8 | 응답 수, 1 또는 2 |
| 26 | u8 | 응답 PID1, 두 응답이면 PID0과 달라야 함 |
| 27 | u8 | checksum |

ID는 각각 0..0x7FF, DLC는 0..8이며 전원이 사용 가능해야 한다. 응답 하나이면 PID1은 무시한다.
이 handler는 **checksum 검증 전 진행 중인 benchmark를 취소**한다. 무효 요청은 iteration=0,
status=3, final 결과를 예약하며 새 BRIDGE 진입은 하지 않는다.

유효 요청은 BRIDGE로 전환하고 5 ms 뒤 첫 전송을 예약한다. 매 요청 시작에 이 대기가 적용된다.
응답은 기대 ID와 data[0] PID로 매칭하며 DLC≥1이어야 한다. 송신보다 오래된 큐 프레임은 제외하고
timeout 미만 RX timestamp만 수용한다. 일반 제어의 DLC≥8 규칙과 다르다.
응답 두 개 중 하나만 오면 부분 결과를 보존한다. timeout 처리 직전에 큐를 다시 처리한다.

`0xA1` 결과:

| Offset | 타입 | 내용 |
| ---: | --- | --- |
| 4 | u8 | seq |
| 5 | u16 | iteration, 정상 시도는 1부터 |
| 7 | u8 | status: 0 응답 완성, 1 TX 실패, 2 timeout, 3 무효 요청 |
| 8 | u8 | flags bit0=final |
| 9 | u32 | TX 기준 `micros()` |
| 13 | u32 | 응답0 RX `micros()` |
| 17 | u32 | 응답0 RTT µs |
| 21 | u16 | 누적 timeout 수 |
| 23 | u16 | 누적 TX 실패 수 |
| 25 | u16 | 기대 응답 CAN ID |
| 27 | u8 | 응답0 DLC |
| 28..35 | 8 bytes | 응답0 data |
| 36 | u8 | 기대 응답 수 |
| 37 | u8 | 수신 mask bit0/1 |
| 38 | u32 | 응답1 RX `micros()` |
| 42 | u32 | 응답1 RTT µs |
| 46 | u8 | 응답1 DLC |
| 47..54 | 8 bytes | 응답1 data |
| 55 | u8 | checksum |

TX 기준은 `can_debug::update(micros())`에 전달된 시각이며 실제 버스 송신 완료 시각이 아니다.
RX는 ISR FIFO 읽기 시각이고 RTT는 이 둘의 unsigned 차이다. mask가 없는 응답의 시간·data는 0이다.
호스트 전송시간, 실제 CAN wire RTT, 모터 내부 처리시간을 분리해서 측정한 수치가 아니다.

전송 예정 시각은 실제 시도 시점에서 다음 주기만큼 뒤로 잡는다. 결과가 benchmark의 pending에 남아 있으면
다음 요청을 보내지 않는다. 공통 USB 진단 슬롯에 복사되면 pending을 해제하므로, USB write 완료보다 먼저
다음 CAN 요청을 시작할 수 있다. 실제 write와 부분 전송은 공통 송신부가 관리한다.
`0xA0`은 진단 슬롯이 차 있으면 응답을 버리고, `0xA1`은 슬롯이 열릴 때까지 pending을 유지한다.
취소 시 아직 전송을 시작하지 않은 진단 프레임을 버린다. 무손실 전달이나 호스트 수신 ACK는 제공하지 않는다.

반복 종료 후에도 BRIDGE에 남으며 자동 TQ-OFF는 없다. 새 benchmark 요청, 유효 raw 요청,
유효 `0x03`, 보호 리셋, POWERDOWN 전환 등으로 취소할 수 있다. 호스트 CLI 종료는 취소 명령이 아니다.
