# ROS 2 motor bridge 변경 제안 보고서

## 범위

ROS 2 package `/home/jina/car_ws/ros2_ws/src/my_car_motor_bridge`는 Wheel-Dragoon과 별도로 관리하므로 이번 작업에서는 수정하지 않는다. 아래 내용은 MCU의 System Status v2 지원에 필요한 후속 변경 방향이다.

현재 MD200T는 PID 207 또는 PID 174를 받을 때 각 driver의 PID 193/200을 반환하도록 설정되어 있다. MCU firmware는 wheel 상태용 PID 4 polling을 생성하지 않는다. Driver A를 cycle +0 ms, Driver B를 +3 ms 이후에 송신하고, 각 command의 실제 TX 시작부터 3 ms 안에 해당 PID 193/200 쌍을 받아야 정상 round로 인정한다. 두 frame 중 하나라도 누락되면 `system_error` bit 9 `DRIVER_RX_TIMEOUT`을 latch하고 누락 wheel validity를 0으로 만든다. 네 response가 모두 모이면 `0x82`를 즉시 송신하고, 누락 시에는 두 번째 response window 종료 시 fallback packet을 송신한다. PID 143 입력전압만 driver별 1 Hz로 polling한다. Bit 9의 위치는 유지되지만 이전 `MAIN_DATA_STALE` 의미는 폐기되었으므로 ROS 표시 이름과 설명을 갱신해야 한다. Bit 12의 이전 `BC_INITIALIZATION_FAILURE` 의미는 제거되었으며 reserved/항상 0으로 처리해야 한다. Bit 2 `CAN_TX_FAILURE`는 구동/안전 TX 실패일 때는 sticky fault이고, PID 143 요청 단발 TX 실패일 때는 다음 전압 요청 TX 성공까지 유지되는 진단값일 수 있다.

## protocol.py

- 단일 `PAYLOAD_LENGTH=7` 가정을 제거하고 packet type별 기대 길이를 검사한다.
- 기존 `0x81`/7-byte Basic Status parser는 선택적 구형 firmware 호환용으로 유지한다. 현재 MCU 기본 설정에서는 `0x81`을 송신하지 않는다.
- `0x82`/39-byte System Status v2 parser와 dataclass를 추가한다.
- System Status v2 layout은 Wheel-Dragoon `README.md`의 명세를 따른다.
- stream parser는 fragmentation, back-to-back packet, 알 수 없는 type, checksum failure에서 다음 `AA 55`로 resync해야 한다.
- little-endian 형식 예시는 다음과 같다.

```python
SYSTEM_STATUS_V2_FIXED = struct.Struct("<BBBBHBHH")
WHEEL_TELEMETRY = struct.Struct("<BhHH")
```

실제 구현에서는 byte offset과 전체 payload 길이를 명시적으로 검증한 뒤 Driver A/B voltage 및 LF/RF/LR/RR 순서로 decode한다.

## motor_bridge_node.py

- `serial_mode` parameter를 추가하며 허용값은 `control`, `bridge`로 제한한다.
- control mode에서는 기존 `/cmd_vel` command를 보내고 기본적으로 `0x82`를 파싱한다. `0x81` parsing은 구형 firmware fallback으로만 유지한다.
- bridge mode에서는 control command와 상태 publish를 중단하고 `0x20` request 및 연속 `0xA0` CAN tunnel stream을 처리한다. `0xA0`은 sequence나 driver ID/PID로 버리지 않고 CAN ID/PID를 상위 진단 계층에서 필요에 따라 분류한다.
- 정상 `0x82`를 한 번 수신하면 해당 연결에서는 `0x82`를 canonical 상태로 사용하고 legacy `0x81`을 `~/status`에 중복 publish하지 않는다.
- 구형 MCU처럼 `0x82`가 일정 시간 전혀 수신되지 않을 때만 `0x81`을 fallback으로 publish한다.
- `~/status` JSON은 다음처럼 계층화한다.

```json
{
  "version": 1,
  "seq": 0,
  "state": 0,
  "system_error": 0,
  "validity": 0,
  "drivers": {
    "a": {"voltage_mv": 0, "valid": false},
    "b": {"voltage_mv": 0, "valid": false}
  },
  "wheels": {
    "lf": {"status": 0, "rpm": 0, "current_deci_amp": 0, "controller_output": 0, "valid": false},
    "rf": {"status": 0, "rpm": 0, "current_deci_amp": 0, "controller_output": 0, "valid": false},
    "lr": {"status": 0, "rpm": 0, "current_deci_amp": 0, "controller_output": 0, "valid": false},
    "rr": {"status": 0, "rpm": 0, "current_deci_amp": 0, "controller_output": 0, "valid": false}
  }
}
```

- 대표 battery voltage가 필요하면 Driver A/B가 모두 valid일 때 두 값 중 낮은 값을 사용하고, 하나라도 invalid이면 대표값도 invalid로 취급한다.

## 권장 테스트

- `0x81` 단독 parsing 및 기존 JSON 호환성
- `0x82` 정상 packet과 signed negative RPM
- 네 wheel offset 및 LF/RF/LR/RR 순서
- split read와 여러 packet의 한 번 수신
- `0x81`과 `0x82` 혼합 stream
- checksum 오류와 다음 header resync
- v2 수신 후 legacy publish 억제
- v2가 없는 구형 MCU의 legacy fallback
- `serial_mode=control|bridge`별 timer/subscriber/publisher 동작

## Serial load

- MCU→ROS System Status v2: 43 bytes × 100 Hz × 10 bits = 43,000 bit/s
- MCU→ROS legacy Basic Status: 기본 비활성, 0 bit/s
- 기본 MCU→ROS 합계: 약 37.3% of 115200 bps
- ROS→MCU command 11 bytes × 50 Hz까지 단순 합산하면 양방향 명목 합계는 약 42.1% 수준이다. USB CDC는 full-duplex지만 ROS parser/publisher가 100 Hz packet을 지속 처리하는지 검증해야 한다.
