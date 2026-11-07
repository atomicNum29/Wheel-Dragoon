# Windows can_cli 실행 및 벤치마크 검토

2026-11-07 CAN FIFO 커밋 이후 기준. Windows 호환 변경은 호스트 CLI에만 적용했다.
이 커밋은 MCU 펌웨어나 CAN 수신 처리를 바꾸지 않는다. 조향 확장과 타이밍 계측은 별도 변경이다.

## Windows 실행

프로젝트 루트의 PowerShell 또는 명령 프롬프트에서 실행한다.
프로젝트 Python 환경은 `uv sync`로 준비할 수 있다.

```powershell
uv run python -m serial.tools.list_ports -v
uv run python src/can_cli.py --port COM5
```

`COM5`는 실제 Teensy 포트로 바꾼다. 기존 `.venv`가 준비되어 있으면
`uv run python` 대신 `.\.venv\Scripts\python.exe`를 사용할 수 있다.
같은 포트를 사용하는 ROS 연결이나 다른 시리얼 모니터는 먼저 종료한다.

Windows에서는 `msvcrt.kbhit/getwch`로 키보드를 읽고, pySerial의 `in_waiting`
및 nonblocking `read`로 수신한다. COM 포트와 콘솔에 `select/fileno`를 사용하지 않는다.
Enter, Backspace, Ctrl-U, Ctrl-C, Ctrl-D, `quit`를 지원하며 방향키·기능키 코드는 무시한다.
Windows 화면 갱신은 ANSI escape 지원을 요구하지 않는다. POSIX는 기존 termios/select 경로를 유지한다.

Teensy 이름 또는 USB VID `0x16C0`를 포트 검색의 단서로 사용한다.
후보가 여럿이면 `--port`를 명시해야 한다. VID만으로 개별 장치의 정체를 보증하지는 않는다.
CLI는 대화형 터미널을 요구하므로 stdin/stdout 리디렉션이나 파이프 실행은 지원하지 않는다.

## 현재 PID 196/201 설정에 맞는 벤치마크

**PID 174/207에 대한 CLI의 기존 자동 matcher는 193/200이다.**
이 기본값은 이번 Windows 호환 변경에서 바꾸지 않았다. 현재 드라이버 설정인
PID149=7의 196/201 응답을 검사하려면 두 PID를 명시해야 한다.
명시하지 않으면 196/201이 raw RX 로그에 보이면서 벤치마크는 timeout으로 집계될 수 있다.
펌웨어 matcher는 CLI가 전달하는 PID를 사용하므로 193/200에 고정되어 있지 않다.

아래 예제는 PID 174로 두 채널을 torque-off시키며, 드라이버 A의 자동 응답을
10 ms 주기, 3 ms 제한, 1,000회 검사한다. 주행 속도 명령을 재현하는 예제는 아니다.

```powershell
uv run python src/can_cli.py benchmark 1 174 1 1 --dlc 8 --response-pid 196 --response-pid2 201 --count 1000 --period-ms 10 --timeout-ms 3 --port COM5
```

A 검사가 끝난 뒤 같은 `can>` 프롬프트에서 B를 검사한다.

```text
benchmark 2 174 1 1 --dlc 8 --response-pid 196 --response-pid2 201 --count 1000 --period-ms 10 --timeout-ms 3
```

응답 CAN ID는 각각 `0x701`, `0x702`로 자동 설정된다. `--dlc 8`은 일반 제어의
8-byte 명령과 맞추기 위한 옵션이며, 생략하면 입력한 데이터 길이가 DLC가 된다.
벤치마크의 기본 timeout은 10 ms이므로 일반 제어와 비교할 때 3 ms를 명시해야 한다.

POWERDOWN/드라이버 부팅 대기 중에는 요청이 거절된다. 유효한 요청은 BRIDGE로
전환하고, 새로 BRIDGE에 진입할 때 A/B에 PID 174를 보낸다. 첫 측정 전 5 ms를 기다린다.
새 benchmark는 이전 세션을 대체한다. raw `0x20`, 유효한 control `0x01`, reset `0x02`는
벤치마크를 취소한다. 호스트에서 `quit`/Ctrl-C로 나가는 것만으로 MCU 세션을 취소하거나
CONTROL 모드로 복귀시키지는 않는다. 정해진 횟수 완료 후에도 BRIDGE 모드는 유지된다.

## 일반 주행과 동일한 환경인가?

**동일하지 않다.** 저수준 CAN 경로는 같지만 실행 순서와 부하가 다르다.

| 항목 | 일반 주행 CONTROL | 현재 benchmark BRIDGE |
| --- | --- | --- |
| CAN 드라이버 | 같은 FlexCAN0, 500 kbit/s, FIFO·ISR·RAM 큐 | 동일 |
| 송신 대상 | 10 ms마다 A, 최소 3 ms 뒤 B | 한 세션에서 한 CAN ID 반복 |
| 명령 | 주행 명령에 따른 PID 207 또는 PID 174 | 지정한 고정 payload |
| 응답 제한 | 드라이버별 3 ms | CLI 설정값, 기본 10 ms |
| 응답 검사 | 해당 응답 창 + PID + DLC 최소 8 | 세션 ID/PID matcher + DLC 최소 1 |
| RX 이후 순서 | health/주행 명령 처리 뒤 timeout 검사 | benchmark timeout 검사 및 일반 구동 억제 |
| 주행 처리 | RC/AUTO 명령을 바퀴별 목표 속도로 변환 | 일반 구동 비활성화 |
| 전압 조회 | A/B 각각 1 Hz PID 143 | 자동 조회 없음 |
| USB 출력 | 네 바퀴 통합 상태 `0x82`, 63 bytes | 반복별 `0xA1`, 56 bytes + matcher 밖의 raw `0xA0` |
| USB 혼잡 | 상태 송신을 미루되 CAN 스케줄은 계속 진행 | 결과가 pending이면 다음 benchmark 송신도 대기 |
| timeout 이후 | 일반 제어 fault 처리로 이후 출력 억제 | timeout 집계 후 다음 반복, 일반 제어와 같은 억제 경로 아님 |

따라서 벤치마크가 성공해도 일반 주행에서의 주행 처리 부하나 A/B 교대 송신에 의한
PID 201 누락을 배제할 수 없다. 반대로 같은 현상이 발생하면 공유하는 CAN 드라이버,
FIFO·큐 처리, 드라이버 응답 지연을 조사할 근거가 된다. 원인을 단독으로 확정하지는 못한다.

## 결과 해석과 확인 범위

- `received_mask=0x01`이면 첫 matcher인 PID 196만 받았다는 뜻이다. `0x03`은 두 PID 모두 수신이다.
- `pid201_rtt_us[n=...]`, `partial_rounds`, `timeouts`를 함께 확인한다. PID별 RTT 분포는 수신한 표본만 포함한다.
- `tx_interval_us`로 실제 송신 간격이 요청한 10,000 us를 유지하는지 확인한다.
  Windows idle polling은 5 ms sleep을 사용하지만 OS 스케줄링 때문에 정확한 5 ms 주기를 보장하지 않는다.
  호스트 수신/출력이 느려 USB 버퍼가 차면 MCU의 다음 benchmark 송신이 밀릴 수 있다.
- RTT는 PC 왕복 시간이 아니라 MCU `micros()` 기반의 송신 호출 직전부터 ISR에서 FIFO를 읽은 시각까지다.
  실제 CAN wire 도착 시각은 아니며 IRQ 지연과 FIFO 체류 시간을 포함한다. RAM 큐 대기 시간은 제외된다.
  이전 폴링 펌웨어와는 측정 지점이 다르다.
- 요청 `0x21`(28 bytes), 결과 `0xA1`(56 bytes)의 길이·필드·체크섬은 현재 CLI와 펌웨어가 일치한다.
- Windows Python에서 `--help` 실행 및 `test_can_cli.py`의 26개 테스트가 통과했다.
  Windows 콘솔/시리얼 경로는 모의 입력으로 검사했고, 분할된 결과와 PID 201 누락 결과의 집계도 검사했다.
- 실물 COM 포트를 열거나 CAN 명령을 송신하지 않았다. 실제 Windows 터미널 조작, 장비 응답률,
  현재 업로드된 펌웨어 버전 및 실시간 성능은 미검증이다. MCU 빌드/업로드도 수행하지 않았다.

호스트 API 근거: [Python msvcrt](https://docs.python.org/3/library/msvcrt.html),
[pySerial API](https://pyserial.readthedocs.io/en/latest/pyserial_api.html),
[포트 검색](https://pyserial.readthedocs.io/en/latest/tools.html).
