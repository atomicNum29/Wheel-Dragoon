# UGV 라즈베리파이 작업 인계

2026-11-07 기준. 저장소는 `atomicNum29/Wheel-Dragoon`, 작업 브랜치는 `main`이다.
이번 인계는 코드·설정·백업을 Git으로 전달하는 작업이며 라즈베리파이에 접속하거나
설치/업로드한 작업이 아니다. 기존 MCU는 Teensy 3.2이며 이 펌웨어가 Pi에서 직접 실행되는 것은 아니다.

## 현재 기준 상태

- RC MANUAL에서 전륜 독립 조향과 네 바퀴 구동을 사용한다. **AUTO 구동은 차단**한다.
  ROS 속도 패킷을 보내도 자율 주행하지 않는다. USB 패킷 파싱과 상태 송신은 유지한다.
- RC 좌우 스틱은 안쪽 앞바퀴 목표각에 대응하고 곡률로 변환한다. 정지·후진에서도
  같은 스틱 위치가 같은 목표 조향각을 뜻한다. 스키드 모드와 ROS 각속도 운동학은 제거했다.
- RC 입력: 1000~2000 µs, 중립 1450~1550 µs, 양쪽 유효 구간 450 µs.
  조향 끝점은 안쪽 바퀴 ±70°, 전후 끝점은 ±1.8 m/s 요청이며 RPM 상한을 적용한다.
- 휠 상한 200 RPM, 가속 200 RPM/s, 감속 400 RPM/s.
- 조향은 더 많이 움직이는 쪽을 최대 180°/s로 제한하고 공통 진행 비율로 두 명령각의
  도착을 맞춘다. 5° 조향 차이로 감속하는 정책은 제거했다.
- 초기 대기 1500 ms, 반전/재활성화 경로의 정지 대기 200 ms·안정화 500 ms는 유지한다.
  실제 조향 피드백은 없고 명령각/명령 RPM에 기반한 상태다.
- 서보 핀 9/10, FTM0 50 Hz, 16비트 PWM. 양쪽 `inverted=true`.
  영점은 **왼쪽 1540 µs / 오른쪽 1420 µs**이며 보정표의 0° 항목만 수정한다.
  현재 −70°는 1889 µs, +70°는 1111 µs로 임시 대응한다.
- CAN FIFO/ISR 수신(`3062dad`), Windows CLI(`3514ee0`), 공통 타이밍(`025b281`)을 포함한다.
  계측 환경은 `teensy32_profile`, 스냅샷은 22개 레코드다. 별도 조향 프로브와 `0x83`은 없다.

상세 수식·파일 역할·보정 규칙은 [RC 조향 문서](rc-steering.ko.md), CAN은
[수신 구조](can-rx-fifo.md), 계측은 [공통 타이밍 문서](timing-diagnostics.md)를 참고한다.

## Pi에서 저장소 이어받기

새 디렉터리에서 시작한다면:

```bash
git clone https://github.com/atomicNum29/Wheel-Dragoon.git
cd Wheel-Dragoon
git switch main
git log -5 --oneline
```

이미 체크아웃이 있다면 먼저 `git status --short`로 로컬 변경을 확인한다. 변경이 없을 때:

```bash
git fetch origin
git switch main
git pull --ff-only origin main
git log -5 --oneline
```

로컬 변경이나 분기 차이로 실패하면 덮어쓰거나 강제 리셋하지 말고 먼저 차이를 확인한다.
최신 RC 커밋과 위 CAN/CLI/타이밍 이력이 포함되어 있는지 확인한다.

호스트 Python 환경은 현재 `pyproject.toml`의 **Python >=3.14**, `pyserial` 및 `uv.lock`을
기준으로 한다. Pi에 `uv`와 해당 Python 환경을 준비한 후 프로젝트 루트에서:

```bash
uv sync --locked
uv run python -m serial.tools.list_ports -v
```

Pi의 OS/CPU 아키텍처, Python 설치 가능 여부, USB 권한, PlatformIO 툴체인 가용성은
이번 작업에서 확인하지 않았다. PC의 `.venv`나 `.pio`를 복사해서 사용하지 않는다.

## USB 관측과 펌웨어 작업 구분

표시된 실제 포트로 바꿔 사용한다. `/dev/ttyACM0`은 예시이며, 가능한 경우
`/dev/serial/by-id/`의 장치 경로를 사용한다. ROS 노드 등 다른 프로세스가 같은 포트를
점유하지 않도록 한다. 수신만 하려면:

```bash
uv run python src/packet_monitor.py --port /dev/ttyACM0
```

이미 업로드된 MCU 펌웨어가 계측 빌드라면 다음 CLI로 `0x84`를 해석할 수 있다.
이 명령은 MCU에 계측 펌웨어를 업로드하거나 계측을 활성화하지 않는다.

```bash
uv run python src/can_cli.py --port /dev/ttyACM0 --timing-only
```

CLI의 `benchmark`/`pid`/`frame`은 CAN 명령을 전송하고 BRIDGE 모드로 전환할 수 있다.
AUTO 차단이 이 명시적 CAN 명령까지 차단하지는 않는다. 단순 RC 관측에는 입력하지 않는다.

펌웨어 변경은 Teensy 대상으로 별도 컴파일해야 한다. PlatformIO가 준비된 개발 환경에서
`pio run -e teensy32` 또는 `pio run -e teensy32_profile`을 사용한다. 이는 빌드 명령이며
업로드하지 않는다. Pi에서의 빌드·업로드 가능 여부와 현재 장치에 올라간 펌웨어는 별도 확인 대상이다.

## 확인한 내용과 다음 작업

- 사용자 보고: 영점을 눈대중과 오실로스코프로 맞췄으며 간단한 RC 주행이 적절하게 동작했다.
  주행 로그·시험 시간·부하 조건·정확한 업로드 커밋은 제공되지 않았다.
- 에이전트 확인: 소스 참조와 Git 변경, 정리 전 백업 무결성 확인.
  이번 최종 RC 구성의 자동 테스트·빌드·하드웨어 업로드는 수행하지 않았다.
  이전 공통 타이밍/CAN 커밋의 테스트 결과를 최신 RC 구성의 통과 결과로 간주하지 않는다.
- **끝점 보정은 엔드 스톱 스위치·엔코더 등 하드웨어 추가 후 진행**하기로 했다.
  임시 ±70° 매핑을 실측 각도나 기계적 끝점으로 간주하지 않는다. `verified=true`는 현재
  임시 보정 사용을 허용한 플래그이며 실제 끝점 검증 완료를 의미하지 않는다.
- AUTO/ROS 주행 재도입, 추가 조향 텔레메트리 및 별도 보정 모드는 아직 범위 밖이다.
  후속 요청에서 필요한 기능부터 결정한다. MCU RC 신호 유실 타이머는 없으며 기존 수신기
  페일세이프에 의존하는 상태다.
- 지정되지 않은 테스트를 임의로 실행하지 않는 작업 지침을 유지한다.

정리 전 확장 구현은 [백업 안내](../backups/rc-cleanup-20261107-154207/README.md)에 있다.
ZIP에는 정리 시점 원본 18개와 패치가 있으며 `removed-files/`에는 옮긴 문서·테스트가 있다.
그 백업은 **영점 단일화와 현재 1540/1420 µs 조정 이전 버전**이므로 최신 파일 위에
그대로 덮어쓰지 않는다. 현재 RC 동작의 기준은 Git `main`의 활성 소스다.
