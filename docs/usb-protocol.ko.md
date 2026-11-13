# USB 프로토콜

[문서 안내](README.md) · [호스트 도구](can-cli-windows.md) · [CAN 응답 의미](can-rx-fifo.md)

현재 바이트 규격의 기준 문서다. 소스: [serial_protocol.hpp](../include/serial_protocol.hpp),
[serial_protocol.cpp](../src/serial_protocol.cpp), [can_debug.cpp](../src/can_debug.cpp),
[control.cpp](../src/control.cpp). 타이밍 패킷 필드는 [타이밍 진단](timing-diagnostics.md)에 둔다.

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
| Host → MCU | `0x01` 제어 | 7 / 11 | 모든 프로필 |
| Host → MCU | `0x02` 보호 리셋 | 4 / 8 | 모든 프로필 |
| MCU → Host | `0x82` System Status v2 | 59 / 63 | 모든 프로필의 CONTROL |
| Host → MCU | `0x20` CAN 브릿지 | 15 / 19 | debug, debug_timing |
| MCU → Host | `0xA0` CAN 브릿지 응답 | 14 / 18 | debug, debug_timing |
| Host → MCU | `0x21` CAN benchmark | 24 / 28 | debug, debug_timing |
| MCU → Host | `0xA1` benchmark 결과 | 52 / 56 | debug, debug_timing |
| MCU → Host | `0x84` Timing v1 | 59 / 63 | timing, debug_timing의 CONTROL |

디버그가 없는 프로필은 알려진 `0x20`/`0x21` 프레임 전체를 소비한 뒤 framing 오류로 보고한다.
모드 전환·CAN 전송·디버그 응답은 없다. `0x81`과 `0x83`은 현재 송신하지 않는다.
MCU 수신 파서는 위 네 요청의 정해진 length/type 조합만 허용한다.

## 제어 `0x01`

| Offset | 타입 | 내용 |
| ---: | --- | --- |
| 4 | u8 | seq: 현재 MCU 제어 처리에서 사용하지 않음 |
| 5 | i16 | 선속도, milli-m/s; −2000..2000 |
| 7 | i16 | 각속도, milli-rad/s; −5000..5000 |
| 9 | u8 | flags: host 관례상 bit0 enable, bit1 estop |
| 10 | u8 | checksum |

checksum과 범위가 유효하면 디버그를 취소하고 CONTROL로 전환하여 명령과 수신 시각을 저장한다.
CONTROL로 바뀌는 경우 세션 이력도 초기화한다. ACK나 seq echo는 없다.
잘못된 checksum·범위는 유효 명령 시각을 갱신하지 않는다. flags의 미사용 비트는 검증하지 않는다.

현재 bit0은 구동 권한을 부여하지 않는다. MANUAL/PIVOT은 RC를 사용하며 bit1도 무시한다.
AUTO는 구동을 차단한 채 bit1과 마지막 유효 명령으로부터 500 ms 초과 timeout을 상태에 반영한다.
따라서 `v=0,w=0` 또는 enable 해제 패킷을 MANUAL/PIVOT 정지 명령으로 해석할 수 없다.
PIVOT의 각속도 입력은 RC 조향 채널이다. `0x01`의 각속도는 PIVOT 구동에 적용되지 않으며,
USB로 RC 모드를 선택하는 패킷은 없다.
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

## System Status `0x82`, version 2

| Offset | 타입 / 크기 | 내용 |
| ---: | --- | --- |
| 4 | u8 | version=2 |
| 5 | u8 | MCU 송신 seq, 256에서 순환 |
| 6 | u8 | MCU state |
| 7 | u16 | system error bits |
| 9 | u8 | validity bits |
| 10 | u16 | Driver A 전압, mV |
| 12 | u16 | Driver B 전압, mV |
| 14 | u32 | USB 쓰기 직전 MCU `micros()` |
| 18 | 11 bytes | LF telemetry |
| 29 | 11 bytes | RF telemetry |
| 40 | 11 bytes | LR telemetry |
| 51 | 11 bytes | RR telemetry |
| 62 | u8 | checksum |

각 휠 블록은 상대 offset +0=status(u8), +1=RPM(i16), +3=encoder count(i32),
+7=ISR RX `micros()`(u32)다. RPM·encoder는 원시값이며 거리·차체 속도로 보정한 값이 아니다.
상태 byte는 드라이버에서 받은 값이다. 펌웨어는 비영 여부로 fault를 판단하며 개별 bit의 물리 조건을 검증하지 않는다.
조향 실측값·전류는 이 패킷에 없다.

Validity bit0..3은 LF/RF/LR/RR, bit4/5는 전압 A/B이며 bit6/7은 0이다.
유효하지 않은 항목의 숫자가 0이거나 최신이라고 가정하지 않는다.
각 필드의 갱신 시각은 다르며 offset14는 모든 센서가 동시에 샘플된 시각이 아니다.

### MCU state와 오류

| 값 | state | 우선순위 높은 순서 |
| ---: | --- | ---: |
| 7 | DRIVER_POWERDOWN | 1 |
| 4 | FAULT: driver fault 또는 저전압 | 2 |
| 3 | ESTOP: 현재 활성 입력 | 3 |
| 5 | BOOTING: boot 또는 드라이버 시작 대기 | 4 |
| 2 | TIMEOUT_STOP: 현재 활성 timeout | 5 |
| 0 | DISABLED | 6 |
| 1 | ENABLED | 7 |
| 6 | 예약 | 사용하지 않음 |

| Error bit | 의미 |
| ---: | --- |
| 0 | USB checksum 오류 |
| 1 | command timeout 보고 |
| 2 | CAN TX 실패: 안전 명령 또는 전압 조회 |
| 3 | host estop 보고 |
| 4 | 저전압 |
| 5 | framing / 지원하지 않는 요청 |
| 6 | 명령 속도 범위 초과 |
| 7 | 예약: watchdog 감지 소스 없음 |
| 8 | 드라이버 원시 상태 비영 |
| 9 | 드라이버 응답 창 누락 |
| 10 | 전압 stale |
| 11 | CAN RX overrun 관측 |
| 12..15 | 예약: 현재 조향 진단 비트 없음 |

오류 보고 카운터와 현재 활성 timeout/estop은 별개다. 과거 오류 bit가 남아도 state는
현재 활성 입력을 기준으로 다르게 보일 수 있다. 여러 오류 카운터는 전체 63바이트의 `Serial.write()`가
성공했을 때 한 번 감소하고, 같은 시점에 seq가 증가한다. 오류 발생 시 10회로 갱신된다.
전송 여유가 없으면 pending을 유지하고 다음 루프의 최신 telemetry로 다시 시도한다.
이는 호스트 ACK·로그 저장 확인이 아니다. ENABLED도 물리적 움직임을 확인한 값이 아니다.

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
USB 여유가 18바이트 미만이면 응답을 보내지 않으며 별도 재시도 큐가 없다.

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

전송 예정 시각은 실제 시도 시점에서 다음 주기만큼 뒤로 잡는다. 결과가 USB pending이면 다음 요청을
보내지 않아 실효 주기가 길어질 수 있다. 여유가 56바이트 이상이면 쓰고 pending을 해제하지만
`0xA0`/`0xA1`은 write 반환 길이를 확인하지 않는다. 무손실 결과 전달을 보증하지 않는다.

반복 종료 후에도 BRIDGE에 남으며 자동 TQ-OFF는 없다. 새 benchmark 요청, 유효 raw 요청,
유효 `0x01`, 보호 리셋, POWERDOWN 전환 등으로 취소할 수 있다. 호스트 CLI 종료는 취소 명령이 아니다.
