# 호스트 도구: 조작 CLI, 수신, CAN 디버깅

[문서 안내](README.md) · [USB 규격](usb-protocol.ko.md) · [타이밍 진단](timing-diagnostics.md)

Python 도구는 Windows와 Linux 터미널에서 USB 수신, 주기 조작 명령 전송, CAN 디버깅을 제공한다.
소스는 [can_cli.py](../src/can_cli.py), [packet_monitor.py](../src/packet_monitor.py),
[control.py](../src/control.py)다.
`control.py`는 새 `0x03` 명령을 보내고 `0x85/86/87`을 해석한다. 다른 두 도구도 새 피드백 해석을 지원한다.
공통 프레이밍·명령 인코딩·피드백 디코딩은 [mcu_protocol.py](../src/mcu_protocol.py)에 있다.
실행 절차는 [조작 CLI](control-cli.ko.md), 필드 의미는 [ROS USB 가이드](ros-usb-integration.ko.md)를 따른다.

## 환경과 선택

[pyproject.toml](../pyproject.toml)은 Python ≥3.14와 pyserial ≥3.5를 요구한다.
저장소 루트에서 `uv sync`로 [uv.lock](../uv.lock)의 환경을 준비한다.
아래 COM3는 예시이며 Linux에서는 `/dev/ttyACM0` 등 실제 포트로 바꾼다. 기본 baud 인자는 115200이다.
Teensy의 USB CDC 스트림을 115200 baud UART 선로의 처리량으로 계산하지 않는다.
같은 포트는 한 프로그램에서 사용한다.

| 목적 | 도구 | 장치로 명령 전송 |
| --- | --- | --- |
| checksum 유효 패킷 HEX/필드 관측 | `packet_monitor.py` (`--decode`로 해석) | 없음 |
| 상태·타이밍 해석, 대화형 CAN 요청 | `can_cli.py` | 요청 입력 또는 시작 명령이 있을 때 |
| `0x03` 주기 조작·단발 명령·보호 리셋 | `control.py` | 명령 입력 또는 초기 명령이 있을 때 |

## 명령을 보내지 않는 관측

```powershell
uv run python src/packet_monitor.py --port COM3
```

`packet_monitor.py`는 포트에서 읽기만 한다. `AA 55`와 length 1..64를 기준으로 프레임을 모으고,
checksum이 일치하는 패킷만 출력한다. 불완전 프레임은 보관하며 손상 시 한 바이트씩 재동기화한다.
기본은 HEX 수신기이며 `--decode`를 추가하면 `0x85/86/87` 필드를 해석한다. 종료는 Ctrl+C다.
알려진 프레임 크기도 검사한다. 수신 결과를 줄여서 보려면 `control.py --port COM3 monitor`를 사용한다.

해석된 출력을 보려면 다음 CLI를 사용한다. 시작 명령을 생략하면 시작 시 CAN 요청을 전송하지 않는다.

```powershell
uv run python src/can_cli.py --port COM3
uv run python src/can_cli.py --port COM3 --timing-only
```

`--timing-only`는 `0x84` 표시 필터이며 입력을 잠그는 수신 전용 옵션이 아니다.
타이밍 프로필이 업로드돼 있고 CONTROL 상태여야 `0x84`가 나온다. CLI는 펌웨어 프로필을 자동 탐지하지 않는다.
대화형 입력은 실제 TTY가 필요하다. Windows는 `msvcrt`, POSIX는 `termios`/`select` 경로를 사용한다.

## 브릿지와 벤치마크

`pid`, `frame`, `benchmark`는 `teensy32_debug` 또는 `teensy32_debug_timing`에서만 동작한다.
요청은 BRIDGE로 전환하여 일반 제어 CAN 스케줄을 멈추므로 일반 주행 관측과 같은 조건이 아니다.
raw 명령은 모션 제한을 우회한다. 아래는 문법 예시이며 명령을 실제 전송한다.

```text
pid 1 4 143
frame 0x001 4 143 --dlc 8
benchmark 0x001 174 1 1 0 0 0 0 0 --response-id 0x701 --response-pid 196 --response-pid2 201 --count 100 --period-ms 10 --timeout-ms 3
```

위 문장은 CLI 내부 프롬프트에서 입력할 수 있다. 시작 명령으로 전달하려면
`uv run python src/can_cli.py --port COM3` 뒤에 같은 문장을 붙인다. `help`로 문법을 보고 `quit`로 CLI를 종료한다.
CLI 종료만으로 펌웨어 benchmark 취소, CONTROL 복귀, 모터 정지가 보장되지는 않는다.

| 항목 | 기본 / 동작 |
| --- | --- |
| 반복 수 / 주기 | 100회 / 10 ms |
| 시작 `benchmark` timeout | 10 ms |
| 대화형 `benchmark` timeout 생략 | CLI 기본 timeout과 주기 중 작은 값; 기본 조합은 10 ms |
| `pid`/`frame` timeout | 기본 100 ms, `--timeout-ms`로 지정 |
| 응답 ID 생략 | 요청 driver ID에 MID=7을 합성 |
| PID174/207 응답 PID 생략 | **193/200** 두 응답 |

일반 제어는 **196/201**을 요구한다. 따라서 위 예시는 그 PID를 명시한다.
PID149=7 설정이 보고돼 있더라도 현재 실제 장비 설정을 읽어 확인한 것은 아니다.
명령 PID·모드·주기·응답 PID·호스트 부하가 다른 결과를 일반 주행 응답률과 직접 비교하지 않는다.

벤치마크 시간은 MCU 송신 호출 전 시각과 ISR RX 시각의 차이이며 PC 왕복시간이 아니다.
두 응답 중 일부만 도착해도 결과에 남는다. `0xA1` 전송 보류는 다음 요청을 지연시키며,
USB 버퍼 쓰기와 PC의 결과 수신·저장은 별개다. 상세 취소 조건과 필드는 [USB 규격](usb-protocol.ko.md)을 따른다.

## 조작 CLI

`uv run python src/control.py --port COM3`로 명령 입력형 CLI를 연다. Linux에서는 `/dev/ttyACM0` 등으로 바꾼다.
시작 시 관측만 하며 `drive 0.3 10`, `pivot 0.5`, `stop`, `disable`, `estop` 등을 입력해 조작한다.
일반 조향 단위는 가상 중앙 휠의 도(degree)다. 기본 목표 송신 주기는 50Hz이고 표시 주기는 2Hz다.
MANUAL/PIVOT은 RC가 제어하며 호스트 명령은 RC AUTO에서만 적용된다.

초기 명령, 기간 지정, 단발 송신, 수신 전용, 종료 정책은 [조작 CLI](control-cli.ko.md)에 정리했다.
이전 `control.py v_mps w_radps` 위치 인자와 `--disable`/`--estop` 옵션은 지원하지 않는다.
각각 `drive V THETA_DEG`, `disable`, `estop` 명령을 사용한다. 보호 리셋은 `reset` 또는 기존 `--reset`으로 보낸다.
write 성공은 MCU 수락·물리적 정지 확인이 아니다. 이번 호스트 변경은 정적 검토만 수행했으며 실행 검증하지 않았다.
