# 명령 입력형 조작 CLI

[문서 안내](README.md) · [USB 규격](usb-protocol.ko.md) · [ROS 연동](ros-usb-integration.ko.md)

`src/control.py`는 Windows와 Linux의 터미널에서 새 `0x03` 명령을 주기적으로 보내고
`0x85/86/87` 피드백을 함께 표시한다. 입력은 Enter로 확정한다. MCU 펌웨어와 프로토콜은 변경하지 않는다.
Windows는 `msvcrt`, Linux는 `termios`와 `select`를 사용한다. Windows의 COM 핸들을 `select`에 넘기지 않는다.
현재 소스·문서 정적 검토만 수행했으며 Python 실행, 각 OS 터미널, 실물 USB/주행 검증은 수행하지 않았다.

## 실행

저장소 루트에서 `uv sync`로 의존성을 준비한다. 프로젝트 Python 요구사항은 `pyproject.toml`을 따른다.

```powershell
# 포트 목록만 표시
uv run python src/control.py --list-ports

# Windows
uv run python src/control.py --port COM3
```

```bash
# Linux
uv run python src/control.py --port /dev/ttyACM0
```

장치 이름 이외의 명령은 동일하다. Linux에서 포트를 열 수 없으면 포트 파일의 사용자 접근 권한과
다른 프로그램의 점유 여부를 확인한다. 같은 포트는 한 프로그램이 소유한다.
`--port`를 생략하면 Teensy VID/설명 또는 시리얼 장치 이름으로 후보를 찾고, 후보가 여러 개이면 선택을 요구한다.
대화형 입력은 실제 TTY가 필요하다. 출력 리다이렉션·자동 실행에는 아래의 초기 명령이나 `monitor`를 사용한다.

시작만 하면 수신 전용이다. 연결 후 최초 0.6초 동안 모션 명령 송신을 보류하여 이전 송신자의 500ms 명령 유효기간이
끝날 여유를 둔다. 이 시간에도 피드백을 수신한다. 목표 입력 후에는 기본 50Hz로 같은 목표를 새 seq와 함께 보낸다.
명령을 타이핑하는 동안에도 이전 목표가 유지된다. `help`, `status`, 잘못된 입력은 목표를 변경하지 않는다.

## 조작 명령

| 명령 | 동작 |
| --- | --- |
| `drive 0.3 10` | 선속도 0.3m/s, 가상 중앙 휠 조향각 +10°의 일반 주행 |
| `drive-rad 0.3 0.2` | 위와 같은 형식이나 조향 단위는 rad |
| `steer 20` | 일반 주행의 목표 조향각만 +20°로 변경, enable=1 |
| `speed 0.3` | 일반 주행의 목표 선속도만 변경, enable=1 |
| `pivot 0.5` | 후륜 축 중앙 기준 +0.5rad/s PIVOT 요청 |
| `stop` | 현재 일반 조향 목표/PIVOT 모드를 유지하고 속도만 0, enable=1로 스케줄링 정지 |
| `disable` | enable=0으로 구동 차단 요청. 활성 estop은 유지 |
| `estop` | AUTO 비상정지: TQ-OFF와 PIVOT 반대 조향 자세 요청 |
| `clear-estop` | estop 해제 명령을 보내되 enable=0 유지 |
| `pause` | 현재 목표를 보관하고 주기 송신 중단. MCU timeout 관찰용 |
| `status` | 최신 MCU 피드백과 현재 로컬 목표 표시 |
| `reset` | 보호 리셋 `0x02` 송신 후 CLI 종료 |
| `help`, `quit` | 도움말, 종료. `q`/`exit`도 종료 |

아직 목표가 없을 때 `steer`는 v=0에서 시작하고, `speed`는 θ=0에서 시작한다.
PIVOT에서 `speed`/`steer`를 사용하지 않고 `drive V THETA`로 일반 주행을 선택한다.
`pause` 후에는 목표를 정하는 명령으로 송신을 재개한다. `status`는 송신을 재개하지 않는다.

조향 단위는 기본적으로 **가상 중앙 휠의 도(degree)**이며 실제 좌우 서보 각도가 아니다.
현재 입력 한계는 v ±2m/s, θ ±0.845rad(약 ±48.415°), PIVOT ω ±5rad/s다.
기하가 바뀌면 공통 codec의 θ 한계도 함께 갱신해야 한다. MCU의 RPM 상한과 변화율 제한은 별도로 적용된다.
CLI는 NaN/무한대·범위 초과·잘못된 인자 수를 거부하고 이전 목표를 유지한다.

`drive`, `steer`, `speed`, `pivot`, `stop`은 enable=1을 요청한다. RC가 AUTO일 때만 적용된다.
RC가 다른 모드여도 선택한 목표의 주기 송신은 계속되므로, 이후 AUTO로 바꾸면 새로 수신되는 목표가 적용될 수 있다.
비상정지가 로컬 목표 또는 마지막 피드백에 남아 있으면 일반 주행 명령으로 자동 해제하지 않는다.
`clear-estop` 후 MCU 피드백의 estop 해제를 확인하고 새 목표를 입력한다. MCU의 복귀 스케줄은 그대로 사용한다.

## 입력 순서 비교 예시

아래는 CLI 프롬프트에 직접 입력하는 명령이며 자동 실행되는 실험이 아니다.
단계 간 간격은 사용자가 정한다. 상태의 `DRIVING`은 실제 이동 여부가 아니라 스케줄러 단계다.

```text
drive 0 0
steer 20
speed 0.3
stop
disable
```

반대 순서는 다음과 같다.

```text
drive 0 0
speed 0.3
steer 20
stop
disable
```

## 단발·기간 지정·수신 전용

공통 옵션을 명령 앞에 둔다. 다음 동작은 Windows에서도 포트만 `COM3` 등으로 바꿔 사용한다.

```bash
# 연결 후 0.6초 보류, 지정 목표를 3초간 주기 송신한 뒤 구동 해제 시도
uv run python src/control.py --port /dev/ttyACM0 --duration 3 drive 0.3 10

# 한 프레임만 송신. 별도의 종료 명령은 보내지 않으며 MCU의 500ms timeout 적용
uv run python src/control.py --port /dev/ttyACM0 --once disable

# 포트를 열지 않고 인코딩된 프레임만 HEX로 출력
uv run python src/control.py --dry-run drive 0.3 10

# 명령을 전혀 송신하지 않고 요약 피드백 표시
uv run python src/control.py --port /dev/ttyACM0 monitor

# 보호 리셋 (--reset도 같은 동작)
uv run python src/control.py --port /dev/ttyACM0 reset
```

초기 모션 명령에 `--duration`이 없으면 Ctrl+C까지 반복한다. `--once`와 `--duration`은 함께 쓸 수 없다.
`reset`은 0.6초 보류 없이 한 번 송신하고 종료한다.
`--rate-hz`는 5..100, 기본 50Hz다. 누락된 송신 슬롯을 뒤늦게 몰아서 보내지 않는다.
`--status-hz`는 0..10, 기본 2Hz이며 0이면 주기 표시를 끈다. `status`는 언제든 표시할 수 있다.
`--seq`는 첫 u16 seq(기본 0), `--baud`는 기본 115200이다. seq는 송신마다 증가하며 65535 다음 0이다.
새 프로세스의 0.6초 보류는 MCU 수락 ACK가 아니며, 다른 송신자와 포트를 공유하는 것을 허용하지 않는다.

## 표시와 종료

- MCU state, RC mode, 모션 단계, 구동 gate, estop, timeout, 오류 flags, 목표/적용 지령을 표시한다.
- 적용값이 무효인 조향 전환·비상 조향은 `applied[invalid]`로 표시한다. 직진·정지 자세로 해석하지 않는다.
- 네 휠의 RPM·encoder·원시 상태와 A/B 전압을 표시한다. MCU validity와 host 수신 이후 경과시간을 함께 표시한다.
- `age`는 PC의 monotonic 수신 경과시간이다. MCU 측정 시각이 아니다. 상태가 1초 넘게 없으면 `STALE`이며
  휠·전압 validity는 알 수 없다고 표시한다. 이 1초는 표시 기준이고 MCU의 500ms timeout과 별개다.
- CONTROL session 변경 시 피드백 캐시를 비운다. 새 호스트 연결에서는 새 캐시를 사용하며 자동 재연결하지 않는다.
- `quit`/Ctrl+C/기간 종료 시, 실제 모션 명령을 보냈다면 enable=0을 마지막으로 한 번 시도한다.
  로컬 목표 또는 마지막 피드백에 estop이 있으면 유지한다. `--once`, 리셋, 수신만 한 세션에는 추가 종료 명령이 없다.
- write 실패·부분 write에서는 송신을 중단한다. 전송이 끊긴 상태에서는 마지막 종료 명령도 전달할 수 없다.
  write 성공은 MCU 수락·물리적 정지를 증명하지 않는다. 명령 ACK는 없고, 강제 프로세스 종료 시 cleanup도 보장하지 않는다.

원시 패킷 전체 또는 진단 패킷을 보려면 [호스트 도구](can-cli-windows.md)의 `packet_monitor.py`/`can_cli.py`를 사용한다.
조작 중에는 해당 프로그램을 같은 포트에 동시에 연결하지 않는다.
