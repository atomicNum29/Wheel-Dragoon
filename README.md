# Wheel-Dragoon

Teensy 3.2에서 RC 전륜 조향과 MD200T 두 대의 4륜 구동을 처리하는 펌웨어다.
현재 구동 입력은 **RC MANUAL/PIVOT과 RC AUTO에서의 USB `0x03` 명령**이다.
AUTO는 v·θ 또는 PIVOT ω를 기존 스케줄러에 연결하며 enable·timeout·estop 조건을 적용한다.
Python 도구는 Windows/Linux의 명령 입력형 조작 CLI, USB 피드백 관측, 디버깅용 CAN 브릿지를 제공한다.

## 문서 읽는 순서

| 목적 | 문서 |
| --- | --- |
| 전체 문서와 코드 근거 찾기 | [문서 안내](docs/README.md) |
| 네 프로필·모듈·실행 흐름 이해 | [펌웨어 구조](docs/firmware-architecture.ko.md) |
| RC 모드·핀·조향 보정·운동학 확인 | [RC 제어와 조향](docs/rc-steering.ko.md) |
| CAN 채널·주기·응답 유효성 확인 | [CAN 송수신](docs/can-rx-fifo.md) |
| USB 연동 구현 | [USB 프로토콜](docs/usb-protocol.ko.md) |
| 터미널에서 일반/PIVOT 조작 | [조작 CLI](docs/control-cli.ko.md) |
| PC에서 수신·브릿지·벤치마크 사용 | [호스트 도구](docs/can-cli-windows.md) |
| 루프 시간·응답 누락 해석 | [타이밍 진단](docs/timing-diagnostics.md) |
| Raspberry Pi로 작업 환경 인계 | [인계 절차](docs/raspberry-pi-handoff.ko.md) |

## 빌드 프로필

| PlatformIO 환경 | 일반 제어·상태 | CAN 브릿지·벤치마크 | 타이밍 진단 |
| --- | --- | --- | --- |
| `teensy32_control` — 기본 | 포함 | 제외 | 제외 |
| `teensy32_debug` | 포함 | 포함 | 제외 |
| `teensy32_timing` | 포함 | 제외 | 포함 |
| `teensy32_debug_timing` | 포함 | 포함 | 포함 |

선택 기준과 명령은 [펌웨어 구조](docs/firmware-architecture.ko.md)에 있다.
모든 환경은 CONTROL로 부팅한다. 디버그 환경도 유효한 브릿지 요청을 받아야 BRIDGE로 전환한다.

## 저장소 구성

```text
platformio.ini       펌웨어 빌드 환경
src/main.cpp         setup / loop의 처리 순서
src/*.cpp            제어, RC, CAN, USB, 선택적 디버그·계측
include/             모듈 인터페이스, 운동학·모션·보정 설정
src/*.py             USB 관측·명령 전송용 호스트 도구
docs/                현재 사용 문서와 날짜가 명시된 사고 검토
```

현재 설정은 [steering_config.hpp](include/steering_config.hpp), 통신 스케줄은
[can_tx_schedule.hpp](include/can_tx_schedule.hpp)가 기준이다. 활성 `test/` 디렉터리는 없다.

## 현재 동작과 확인 범위

RC STOP은 CAN TQ-OFF를 요청하며, POWERDOWN은 GPIO 5/6의 전원 제어 출력을 내린다.
물리적 제동이나 실제 접점 상태를 확인하는 기능은 없다. RC 펄스 만료 감지와 동작 중인
애플리케이션 watchdog도 없다. MANUAL/PIVOT에서는 호스트 `--estop`·`--disable`로 정지할 수 없다.
세부 제약은 [RC 제어](docs/rc-steering.ko.md)와
[2026-11-09 조종 기능 상실 검토](docs/rc-loss-of-control-review-2026-11-09.ko.md)에 정리했다.

MANUAL의 RC 입력과 주행 중 보간 변수는 선속도 `v`와 가상 중앙 휠 조향각 `θ`다.
좌우 실제 휠의 조향각과 네 휠 RPM은 같은 중간 운동 상태에서 생성하며, 각 출력의 변화율을 함께 제한한다.
PIVOT은 후륜 중심 회전이며, 모드 전환 시 감속·정지 대기·조향 속도 제한·안정 대기를 적용한다.
입력 구간과 전환 조건은 [RC 제어와 조향](docs/rc-steering.ko.md)을 따른다.
USB 피드백은 휠 `0x85`, 전압 `0x86`, MCU 상태·지령 `0x87`로 분리한다.
`0x87`은 29바이트로 입력 목표와 적용 차량 지령을 함께 보고하며, 오류는 플래그로 전달한다.
바이트 규격은 [USB 프로토콜](docs/usb-protocol.ko.md), 호스트 구현은 [ROS USB 수신 가이드](docs/ros-usb-integration.ko.md)를 따른다.
기존 `0x82` 송신과 `0x01` 명령은 종료했다. `control.py`는 `0x03` 주기 송신과 새 피드백 요약을 지원한다.
`can_cli.py`와 `packet_monitor.py --decode`에서도 새 피드백을 해석한다.
AUTO 비상정지는 TQ-OFF와 PIVOT 반대 조향 자세 요청을 사용한다. 해당 제동 효과는 실물 검증하지 않았다.
소스 기준일과 검증 상태는 [문서 안내](docs/README.md)에 있다.
