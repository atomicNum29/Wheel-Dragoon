# ROS 측 USB 제어·피드백 연동 가이드

[문서 안내](README.md) · [바이트 규격](usb-protocol.ko.md) · [RC 제어](rc-steering.ko.md)

이 문서는 MCU 제어 명령과 피드백을 연결하는 호스트 노드를 개발하기 위한 기준이다.
일반 피드백은 `0x85` 휠, `0x86` 전압, `0x87` 상태·지령의 세 종류다.
기존 `0x82 v2`는 더 이상 생성하지 않는다. `control.py`는 `0x03` 명령과 새 피드백을 지원하며,
`can_cli.py`와 `packet_monitor.py --decode`도 새 피드백을 해석한다. 조작법은 [조작 CLI](control-cli.ko.md)를 따른다.
공통 codec은 `src/mcu_protocol.py`다. ROS 노드·토픽 연결은 별도 구현 대상이다.

## 현재 구현 범위

- MCU 휠·전압 유효 응답 등록과 USB 공통 송신 큐, 목표 100Hz 상태·지령 보고가 구현되어 있다.
- 호스트 → MCU `0x03 v1`의 수신·검증과 AUTO 스케줄러 연결이 구현되어 있다. 이전 `0x01`은 거부한다.
- RC가 AUTO일 때 일반 v·θ 또는 PIVOT ω 명령을 적용한다. STOP/POWERDOWN이 우선하고 MANUAL/PIVOT은 RC가 제어한다.
- 비상정지는 TQ-OFF와 PIVOT 반대 전륜 자세 요청을 사용한다. 세부 조건은 아래와 [RC 제어](rc-steering.ko.md)를 따른다.
- 아래 코드는 송수신 로직 예시이며 이번 작업에서 실행 검증하지 않았다. Teensy 빌드·업로드·실물 USB 검증도 미수행이다.

## ROS → MCU 명령

`0x03`은 15바이트이며 `AA 55 0B 03`, version=1, u16 seq, u8 flags, i16 v/θ/ω, XOR 순서다.
flags bit0=enable, bit1=estop, bit2=PIVOT이며 나머지는 0이다. 정수 단위는 각각 0.001m/s, 0.001rad, 0.001rad/s다.
일반 주행은 ω=0, PIVOT은 v=θ=0으로 보낸다. v는 ±2000, ω는 ±5000이고 θ는 현재 기하에서 ±845 이내다.
기하를 변경하면 θ 상한도 [바이트 규격](usb-protocol.ko.md)의 식으로 다시 산출한다.

프레임을 만드는 예시이며 포트를 열거나 송신하지 않는다:

```python
import struct
from functools import reduce
from operator import xor

def command_frame(seq, v_milli=0, theta_milli=0, w_milli=0,
                  *, enable=False, estop=False, pivot=False):
    if not 0 <= seq <= 0xffff:
        raise ValueError("seq out of range")
    if not (-2000 <= v_milli <= 2000 and -845 <= theta_milli <= 845
            and -5000 <= w_milli <= 5000):
        raise ValueError("command out of range for current geometry")
    if (pivot and (v_milli != 0 or theta_milli != 0)) or (not pivot and w_milli != 0):
        raise ValueError("unused motion fields must be zero")
    flags = int(enable) | (int(estop) << 1) | (int(pivot) << 2)
    packet = bytearray(b"\xaa\x55\x0b\x03\x01")
    packet.extend(struct.pack("<HBhhh", seq, flags, v_milli, theta_milli, w_milli))
    packet.append(reduce(xor, packet, 0))
    return bytes(packet)
```

- 한 송신 주체가 seq를 관리한다. 새 명령마다 증가시키고 65535 다음 0으로 순환한다.
- 같은 목표를 유지할 때도 새 seq로 재송신한다. 중복·역순 명령은 timeout을 갱신하지 않는다.
- 마지막 수락 후 500ms 초과 또는 명령 미수신이면 구동을 차단한다. 송신 주기는 지연 여유를 두고 이보다 짧게 설정한다.
- AUTO 진입 전 명령은 재사용하지 않는다. 진입 시 MCU가 이미 버퍼링한 바이트와 부분 프레임도 폐기하므로
  호스트는 현재 목표를 계속 갱신해야 한다. 드라이버 시작 대기 중 명령도 구동용으로 저장하지 않는다.
- CONTROL 세션 변경·AUTO 재진입·timeout 뒤 첫 수락 시 seq 기준을 새로 잡는다. 연결 중간에 호스트 seq만 초기화하면
  역순으로 거부될 수 있다. 호스트 재시작 시에는 MCU의 timeout/새 세션 경계를 고려한다.
- seq에는 생성 시각이 없다. 오래 대기한 명령의 나이·모든 재전송을 검출하지 못한다. host에서 과거 목표를 큐에 쌓거나
  송신 실패 후 밀린 주기를 재생하지 말고, 최신 목표 하나를 유지한다.
- 정상 정지는 enable=1인 v=0 또는 PIVOT ω=0이다. 일반 v=0에서도 θ를 지정해 정지 조향할 수 있다.
- enable=0/timeout은 TQ-OFF 경로와 조향 유지를 사용한다. 통신 복구 후 새 enable=1, estop=0 명령으로 재개한다.
- estop=1은 즉시 TQ-OFF를 시도하고 좌 +62.4°/우 −62.4° 자세로 180°/s 이하로 이동한다.
  timeout이 겹쳐도 비상 자세 요청은 유지한다. 새 estop=0 명령으로 해제하고 enable 조건에 따라 복귀한다.
  조향은 준비 대기와 보정 검사를 거치며 실제 제동 성능·서보 도달은 확인하지 않는다.
- 호스트 estop은 RC AUTO 명령에만 적용한다. MANUAL/PIVOT의 RC 권한을 대신하는 전역 정지 명령이 아니다.
- 수락 ACK/seq echo는 없다. `0x87`의 목표·적용값·구동 gate로 상태를 관측하되 개별 명령 수락 증명으로 해석하지 않는다.

## 패킷을 읽는 순서

```text
serial read(bytes)
  → 누적 버퍼 / AA 55 검색 / length / XOR 확인
  → type와 version 검사
  → 필드 디코딩
  → MCU 시각·순번·세션 처리
  → 휠별 이력 / 전압 / 상태 갱신
  → ROS 메시지 발행 및 필요 시 오도메트리 계산
```

`read()` 한 번은 패킷 하나와 대응하지 않는다. 프레임 일부나 여러 프레임이 함께 반환될 수 있다.
바이너리 데이터를 `readline()` 또는 UTF-8 문자열로 해석하지 않는다. 한 노드가 포트를 소유하고
읽기를 담당하도록 구성한다. 프레임 내용에 `AA 55`가 포함될 수도 있으므로 정상 프레임은 length로 소비한다.

최대 길이가 제한된 파서 예시:

```python
from functools import reduce
from operator import xor
import struct

SIZES = {0x85: 20, 0x86: 15, 0x87: 29,
         0x84: 63, 0xA0: 18, 0xA1: 56}  # 현재 송신 타입
buffer = bytearray()

def frames(chunk):
    buffer.extend(chunk)  # 호출자는 read 크기도 제한한다. 예: 최대 4096 bytes.
    while True:
        pos = buffer.find(b"\xaa\x55")
        if pos < 0:
            buffer[:] = b"\xaa" if buffer.endswith(b"\xaa") else b""
            return
        del buffer[:pos]
        if len(buffer) < 4:
            return
        total = buffer[2] + 4
        if SIZES.get(buffer[3]) != total:
            del buffer[0]  # 잘못된 length/type: 한 바이트 이동하여 재동기화
            continue
        if len(buffer) < total:
            return  # 불완전 프레임 보존
        packet = bytes(buffer[:total])
        if reduce(xor, packet, 0) != 0:
            del buffer[0]  # checksum 오류 시 프레임 전체를 버리지 않는다
            continue
        del buffer[:total]
        yield packet

def decode(packet):
    kind = packet[3]
    if kind not in (0x85, 0x86, 0x87):
        return None  # 진단은 별도 소비자로 전달 가능
    if packet[4] != 1:
        raise ValueError("Unsupported telemetry version")
    u16 = lambda offset: struct.unpack_from("<H", packet, offset)[0]
    u32 = lambda offset: struct.unpack_from("<I", packet, offset)[0]
    i16 = lambda offset: struct.unpack_from("<h", packet, offset)[0]
    if kind == 0x85:
        if packet[7] >= 4:
            raise ValueError("Invalid wheel ID")
        return dict(type=kind, seq=u16(5), wheel=packet[7], rx_us=u32(8),
                    status=packet[12], rpm=i16(13),
                    encoder=struct.unpack_from("<i", packet, 15)[0])
    if kind == 0x86:
        if packet[7] >= 2:
            raise ValueError("Invalid driver ID")
        return dict(type=kind, seq=u16(5), driver=packet[7], rx_us=u32(8),
                    voltage_V=u16(12) / 1000.0)
    mode, transition = packet[8] & 0x0F, (packet[8] >> 4) & 0x0F
    if mode > 4 or transition > 5:
        raise ValueError("Unsupported mode/transition")
    return dict(type=kind, seq=u16(5),
                state=packet[7], mode=mode, transition=transition,
                flags=u16(9), errors=u16(11), validity=packet[13],
                requested_v=i16(14) / 1000.0,
                requested_theta=i16(16) / 1000.0,
                requested_w=i16(18) / 1000.0,
                applied_v=i16(20) / 1000.0,
                applied_theta=i16(22) / 1000.0,
                applied_w=i16(24) / 1000.0,
                applied_mode=packet[26], session=packet[27])
```

이 파서는 알 수 없는 type/길이를 거부한다. 향후 타입이 추가되면 허용표를 갱신한다.
XOR는 기존 프레이밍을 유지한 오류 검출이며 모든 다중 비트 오류를 검출하는 CRC가 아니다.
호스트 재연결 시 `buffer`를 비우고, 이전 세션의 조립 중 프레임을 이어 붙이지 않는다.

## 각 정보의 의미와 활용

### 휠 피드백

ID 순서는 **LF, RF, LR, RR**다. CAN 수신 순서와 다를 수 있다.
실제 드라이버 배선 매핑은 A MOT1=LF, A MOT2=RR, B MOT1=RF, B MOT2=LR이다.
0x85의 RPM·엔코더·상태는 하나의 수용된 CAN 응답에서 취득한 원시값이다.
회전 부호, encoder count/rev, 감속비를 실물 기준으로 확인한 뒤 거리·차체 속도로 변환한다.
엔코더 차분은 signed 32비트 래핑을 고려한다. 모터 전원 재시작 시에는 래핑으로 간주하지 말고 기준을 재설정한다.

각 휠 이력은 `(wheel ID, seq, rx_us)`를 유지한다. 같은 값의 RPM이 반복되어도 서로 다른 샘플일 수 있다.
같은 seq를 반복 수신하면 중복으로 취급하고, 정상 범위에서 `delta=(seq-prev)&0xffff`가 1보다 크면
생성된 피드백 중 `delta-1`개가 보이지 않는다는 뜻이다. 한 바퀴 이상 seq가 순환할 정도의 긴 단절 뒤에는
정확한 누락 수를 복원할 수 없으므로 이력을 다시 설정한다. seq 비교만으로 MCU 재시작을 확정하지 않는다.

CAN 응답이 오지 않은 경우에는 새 seq 자체가 생성되지 않는다. CAN 누락은 상태 패킷의
validity·오류와 마지막 RX 시각으로 판정한다. USB 송신 누락 error bit12는 CAN RX timeout(bit9)과 구분한다.

### 전압

ID 0=A, 1=B의 전압이며 두 값의 수신 시각은 다르다. 같은 전원 계통이라도 평균값 하나로 먼저 합치지 않는다.
새 전압 패킷이 없어도 MCU 상태는 계속 보고된다. validity bit4/5와 각 전압 마지막 RX 시각을 확인한다.
MCU의 전압 stale 기준은 3000ms이며, 실제 전압 샘플 간격은 기존 CAN 조회 스케줄을 따른다.

### 상태와 지령

`requested_v/theta/w`는 선택된 입력 목표이며 flags bit4가 1일 때만 해석한다.
flags bit12=0은 일반 v·θ, bit12=1은 PIVOT ω를 사용한다. AUTO에서는 RC mode만으로 요청 운동 모드를 알 수 없다.
enable=0/estop 상태도 timeout 전이면 수락 목표를 보고한다. 목표 유효와 구동 허용은 별개다.
θ와 실제 좌우 조향각은 다른 값이다.
v는 후륜 축 중앙 기준 m/s, θ는 가상 전륜 중앙 휠 rad이며 양수는 좌조향이다.
후진에서도 θ 부호 정의는 같으며 `ω=v*tan(θ)/L`의 부호는 v에 따라 달라진다.

`applied_v/theta/w`는 스케줄러의 생성 지령이다. flags bit11이 1일 때만 `applied_mode`로 해석한다.
적용 MANUAL=1은 v·θ, 적용 PIVOT=4는 ω를 사용한다. 모드 변경을 위한 감속 중에는 입력 모드와 적용 모드가 다를 수 있다.
좌우 조향각을 직접 보간하는 STEERING/SETTLING 및 EMERGENCY_STOP(5) 중에는 적용 모드=255, bit11=0, 세 값=0이다.
초기화·잘못된 설정/보정도 무효다. 무효 적용값은 실제 정지·직진 자세로 해석하지 않는다.
BRAKING에서는 기존 중앙각/고정 PIVOT 자세가 유지되는 경우에 한해 감속 중인 적용값이 유효하다.
개별 조향각·RPM은 패킷에 포함하지 않는다. 적용 지령은 휠 피드백과 다른 토픽·필드로 보존한다.
조향각 관측 센서가 없으므로 생성 각도를 실제 관측 JointState로 표시하거나
실측값과 같은 신뢰도로 필터에 넣지 않는다. 모션 ready/SETTLING 완료도 실제 도달 확인이 아니다.
현재 각도 지령으로 오도메트리를 계산하면 조향 추종이 모델대로 이루어진다는 가정이 포함된다.

구동 gate(bit9)가 0일 때는 적용 지령만 보고 실제 주행 명령이 적용 중이라고 판단하지 않는다.
POWERDOWN은 CAN 송신을 중단하고 그 밖의 비활성 구동은 TQ-OFF 경로를 사용한다.
bit9=1도 CAN 전송 성공이나 드라이버 적용 ACK는 아니다. A/B 드라이버 전송은 별도 시각에 이뤄진다.
조향 보정 유효 bit10이 0이면 각도 필드를 실제 PWM 출력으로 해석하지 않는다.

`mode`, `state`, `transition`은 각각 운전 모드, 구동 허용/오류 상태, 스케줄러 단계다.
DRIVING은 이동 중이라는 뜻이 아니다. 정지 조향도 DRIVING일 수 있다.
RPM 상한 flag(bit3)는 조향 변화율 제한·후보 재시도를 모두 나타내는 지표가 아니다.
재시도 횟수·원시 RC PWM·서보 PWM 폭은 이번 상태 패킷에 포함하지 않았다.

### 적용값에서 휠 지령 계산

호스트는 MCU와 같은 축거 L, 전륜/후륜 윤거 Tf/Tr, 반경 r을 사용한다.
MANUAL은 `κ=tan(θ)/L`, `C=60/(2πr)`로 놓으면 다음과 같다.

```text
LF angle = atan2(κL, 1-κTf/2)    RF angle = atan2(κL, 1+κTf/2)
LF RPM   = Cv·hypot(1-κTf/2, κL)
RF RPM   = Cv·hypot(1+κTf/2, κL)
LR RPM   = Cv·(1-κTr/2)          RR RPM = Cv·(1+κTr/2)
```

적용 v는 MCU의 공통 RPM 상한 축소까지 반영된 값이다. 적용 PIVOT은 좌우 각도가
`-atan2(L,Tf/2)`, `+atan2(L,Tf/2)`이며, 앞바퀴 RPM은 각각 `-Cω·hypot(Tf/2,L)`,
`+Cω·hypot(Tf/2,L)`, 뒷바퀴는 `-CωTr/2`, `+CωTr/2`다.
입력 대신 **적용값과 적용 모드**를 사용한다. 0.001 단위 양자화로 재계산 결과는 내부 float와 조금 다르다.
적용 지령이 무효인 전환 자세에는 이 공식을 적용하지 않는다.

## 시간, 재시작, 비동기 휠 데이터

- `rx_us`는 CAN ISR에서 기록한 MCU micros 시각이다. USB 전송·PC 도착 시각, MD200T 내부 샘플 시각이 아니다.
- `0x87`에는 timestamp·uptime이 없다. 상태 수신 시각은 host에서 기록하며 측정 시각으로 취급하지 않는다.
- u32 micros는 약 71.6분마다 순환한다. 연속 수신한 휠/전압 피드백을 기준으로 확장하고,
  다른 휠·전압은 가까운 피드백 앵커와의 signed 모듈러 차이로 배치한다.
  `delta=((event_us-anchor_us+2**31)%2**32)-2**31`이며 실제 차이가 약 ±35.8분 이내일 때만 모호하지 않다.
- 휠·전압·상태 큐는 순환 선택되므로 **전체 USB 스트림의 timestamp가 항상 증가하지는 않는다.**
  타입·ID별 순서를 추적하고, 다른 타입의 늦은 패킷을 재시작으로 오인하지 않는다.
- CONTROL 세션 번호 변경은 전원 모드 전환 또는 CONTROL 재진입도 뜻하므로 MCU 재시작과 동일하지 않다.
  USB 재연결·세션 변경 시 기존 시간/순번/엔코더 기준을 재설정한다.
  세션 번호는 u8이고 255→0으로 순환한다. `new != old`로 변경을 판정하며 수치 증가 여부로 비교하지 않는다.
  수신 중단 사이 256의 배수만큼 세션이 바뀌면 같은 값으로 보일 수 있다.
  uptime·boot UUID·영구 boot counter는 없으므로 seq·세션만으로 모든 재시작을 확정할 수 없다.
- 세션이 바뀌면 캐시를 무효화하고 각 스트림의 새 샘플을 기다린다. 일부 전송된 이전 프레임은 경계를 유지하기 위해
  끝까지 전송될 수 있다. 휠·전압에는 session 필드가 없으며 `0x87`에도 시간 앵커가 없으므로
  세션 경계 주변 샘플의 소속을 정확히 복원할 수는 없다. 재연결 직후 단일 샘플로 적분을 시작하지 말고
  각 스트림의 연속 피드백에서 새 기준을 설정한다. 큰 시각 차이의 샘플은 보류/폐기하고 재동기화한다.
- MCU 시간과 ROS 시간의 offset/drift 동기화 프로토콜은 아직 없다. host read 시각을 별도로 기록하되
  그것을 측정 시각이라고 명명하지 않는다. ROS timestamp로 변환할 때 사용하는 동기화 가정·지연 오차를 명시한다.
- 상태·적용 지령의 정확한 MCU 생성 시각은 복원할 수 없다. 이를 휠 피드백 시각의 정확한 동시 지령으로 간주하지 않는다.

오도메트리는 네 값이 같은 USB read에 들어왔다는 이유로 동시 샘플로 취급하지 않는다.
휠별 최신 이력과 유효성을 유지하고, 오도메트리 계산 시각에 맞게 정렬하거나 샘플 나이를 함께 평가한다.
마지막 엔코더 값을 계속 받아 적으면서 이동 증분을 중복 적분하지 않는다.

## ROS 노드에서 보존할 정보

ROS 배포판이나 메시지 패키지는 이번 작업에서 고정하지 않았다. 다음 의미를 분리해 보존한다.

| 데이터 | 보존할 핵심 필드 |
| --- | --- |
| 휠 원시 피드백 | 휠 ID, seq, MCU RX 시각, host 도착 시각, RPM, encoder, 원시 상태 |
| 전압 | 드라이버 ID, seq, MCU RX 시각, 전압 |
| MCU 상태 | session, mode/state/transition, flags, errors, validity |
| 입력 목표 | v·θ 또는 PIVOT ω, 목표 유효 여부 |
| 적용 지령 | v·θ 또는 PIVOT ω, 적용 모드·유효 여부, 구동 gate, host 수신 시각 |
| 추정 오도메트리 | 시간 정렬·보정 후 값, 사용한 피드백 나이·유효성·조향 추종 가정 |

상태 패킷 수신이 멈추면 마지막 ENABLED를 계속 현재 상태로 유지하지 말고 통신 상태를 unknown/stale로 전환한다.
호스트의 timeout 수치는 운용 주기·허용 지연에 맞춰 별도로 정한다. 이것은 MCU 자체 정지 동작과 별개다.

## 버퍼와 유실 해석

MCU는 휠 16개, 전압 4개 FIFO와 최신 상태 슬롯 1개를 사용한다. 혼잡 시 오래된 미전송 데이터를 버린다.
전송 중 프레임 1개는 별도이며, 일부 전송되면 끝까지 이어 쓴다. USB write는 루프당 최대 두 번 시도한다.
상태 패킷은 누락 누적값과 write 오류 횟수를 보내지 않는다. error bit12는 큐 초과·미전송 상태 교체·
10ms 주기 슬롯 생략·브릿지 응답 폐기를, bit13은 USB write 반환 길이 불일치를 나타낸다.
정상 모드 전환·세션 초기화·명시적 디버그 취소에 의한 폐기는 오류가 아니다.
bit12/13은 해당 오류가 담긴 상태 프레임 전체를 USB에 쓰고, 스냅샷 이후 같은 오류가 다시 발생하지 않았을 때 해제된다.
그 전에 전송이 막히면 유지한다. CONTROL 세션 초기화로 아직 보고하지 못한 USB 오류를 지우지 않는다.
진단 플래그이므로 bit12/13 자체가 MCU 구동을 차단하지 않는다. 오류별 발생 횟수는 복원할 수 없다.
write 성공은 USB 코어에 전달됐다는 뜻이며 host 저장 확인이 아니다. 호스트 ACK·재전송 기능은 없다.

네 휠이 각각 100Hz, 상태가 100Hz라면 애플리케이션 프레임은 `20×400 + 29×100 = 10900 bytes/s`다.
전압·진단 및 USB 자체 overhead는 별도다. 패킷당 크기는 줄었지만 기존 63×100보다 총 바이트 수는 늘어난다.
실효 지연·주기·드롭률은 호스트 수신 구현과 실물 연결에서 후속 확인이 필요하다.
