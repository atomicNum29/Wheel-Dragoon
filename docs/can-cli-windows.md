# 호스트 도구: 수신, 단발 명령, CAN 디버깅

[문서 안내](README.md) · [USB 규격](usb-protocol.ko.md) · [타이밍 진단](timing-diagnostics.md)

Python 도구는 Windows와 POSIX 터미널에서 USB 수신, 단발 명령 전송, CAN 디버깅을 제공한다.
소스는 [can_cli.py](../src/can_cli.py), [packet_monitor.py](../src/packet_monitor.py),
[control.py](../src/control.py)다.

## 환경과 선택

[pyproject.toml](../pyproject.toml)은 Python ≥3.14와 pyserial ≥3.5를 요구한다.
저장소 루트에서 `uv sync`로 [uv.lock](../uv.lock)의 환경을 준비한다.
아래 COM3는 예시이며 Linux에서는 `/dev/ttyACM0` 등 실제 포트로 바꾼다. 기본 baud 인자는 115200이다.
Teensy의 USB CDC 스트림을 115200 baud UART 선로의 처리량으로 계산하지 않는다.
같은 포트는 한 프로그램에서 사용한다.

| 목적 | 도구 | 장치로 명령 전송 |
| --- | --- | --- |
| checksum 유효 패킷 HEX 관측 | `packet_monitor.py` | 없음 |
| 상태·타이밍 해석, 대화형 CAN 요청 | `can_cli.py` | 요청 입력 또는 시작 명령이 있을 때 |
| `0x01` 한 번 또는 보호 리셋 전송 | `control.py` | 있음 |

## 명령을 보내지 않는 관측

```powershell
uv run python src/packet_monitor.py --port COM3
```

`packet_monitor.py`는 포트에서 읽기만 한다. `AA 55`와 length 1..64를 기준으로 프레임을 모으고,
checksum이 일치하는 패킷만 출력한다. 불완전 프레임은 보관하며 손상 시 한 바이트씩 재동기화한다.
상태 필드의 의미를 해석하지 않는 HEX 수신기다. 종료는 Ctrl+C다.

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

## `control.py`의 현재 역할과 제한

`control.py v_mps w_radps --port COM3`는 `0x01`을 **한 번** 전송한다.
속도 허용 범위는 ±2 m/s, 각속도는 ±5 rad/s이며 호스트는 이를 milli 단위 int16으로 만든다.
현재 AUTO에서는 이 속도로 주행하지 않는다. MANUAL/PIVOT에서는 RC를 계속 사용하고
`--disable`·`--estop`은 구동 중지 수단으로 적용되지 않는다.
BRIDGE 중 유효 `0x01` 수신은 CONTROL 복귀를 유발하므로 단순 관측 명령으로 사용하지 않는다.

일반 명령 뒤 응답 처리는 `readline().decode()`이며 바이너리 `0x82` 디코더가 아니다.
출력 문자열이나 빈 응답으로 명령 수용·정지를 확인할 수 없다. 상태 관측에는 위 수신 도구를 사용한다.

`control.py --reset --port COM3`는 보호 리셋 `0x02`를 전송하며 응답을 기다리지 않는다.
펌웨어는 전원 상태가 허용하면 TQ-OFF를 시도한 뒤 MCU를 리셋한다. 전송 성공이나 실제 정지를
확인한 다음에만 재부팅하는 절차가 아니다.
