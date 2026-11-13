# Raspberry Pi 작업 인계

[문서 안내](README.md) · [호스트 도구](can-cli-windows.md) · [펌웨어 구조](firmware-architecture.ko.md)

인계 대상은 소스·설정·문서와 Python 의존성 정보다. 커밋·미커밋 파일·빌드 환경·장비 적용 버전을
각각 식별하여 다른 컴퓨터에서도 같은 작업 상태를 구성한다.

## 인계 대상 식별

원본 PC에서 `git status --short`와 `git rev-parse HEAD`로 커밋과 미커밋 파일을 함께 기록한다.
커밋 ID만으로 작업 트리의 수정·추가 파일까지 전달됐다고 볼 수 없다.
전달할 파일은 소스·설정·문서·`pyproject.toml`·`uv.lock`이며, `.pio`·`.venv`는 대상 환경에서 다시 만든다.
과거 복구가 목적일 때만 [2026-11-07 백업](../backups/rc-cleanup-20261107-154207/README.md)을 별도로 참고한다.

인계받은 쪽에서는 전달된 커밋과 미커밋 파일을 대조한다. `git pull`은 원격에 있는 커밋을
가져오며, 원본 PC의 미커밋 파일은 별도로 전달해야 한다.

## 환경과 프로필

1. 저장소 루트에서 `uv sync`로 Python 환경을 준비한다. Python 요구사항은 `pyproject.toml`이 기준이다.
2. PlatformIO 작업은 [네 빌드 프로필](firmware-architecture.ko.md)을 선택한다. 기본은 `teensy32_control`이다.
3. `.pio/build/<환경>/firmware.hex`의 소스 버전과 프로필을 식별한다.
4. 소스 전달, 빌드, 업로드, 실제 장치 확인 상태를 각각 기록한다.

Pi에서 포트 이름은 `/dev/ttyACM0` 등 실제 장치와 권한에 맞춘다. 다른 프로세스가 같은 포트를 사용하고
있지 않은지 확인한다. 패킷 수신 예시는 다음과 같으며, 프로그램은 명령을 쓰지 않는다.

```bash
uv run python src/packet_monitor.py --port /dev/ttyACM0
```

## 현재 동작을 함께 전달할 문서

| 인계 내용 | 기준 문서 / 파일 |
| --- | --- |
| MANUAL/PIVOT 구동, AUTO 차단, STOP·POWERDOWN | [RC 제어](rc-steering.ko.md) |
| 실측 서보 보정과 운동학 설정 | [steering_config.hpp](../include/steering_config.hpp) |
| CAN 500 kbit/s, A/B 채널, PID196/201 | [CAN 송수신](can-rx-fifo.md) |
| 상태·오류·리셋·디버그 패킷 | [USB 규격](usb-protocol.ko.md) |
| 정상 제어와 benchmark의 조건 차이 | [호스트 도구](can-cli-windows.md) |
| RC 유실·정지 전달·fault 보고의 남은 제약 | [사고 검토와 적용 범위](rc-loss-of-control-review-2026-11-09.ko.md) |

현재 검증 상태는 [문서 안내](README.md)를 참고한다. 백업의 테스트 결과와 실행 명령은
당시 기록이며 인계한 소스의 검증 결과로 사용할 수 없다.
