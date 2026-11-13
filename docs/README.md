# 문서 안내

[프로젝트 개요](../README.md)

문서는 **2026-11-13 소스 기준**의 기능·설정·인터페이스를 설명한다.
처음 읽을 때는 펌웨어 구조 → RC 제어 → CAN 송수신 → USB 프로토콜 순서로 살펴본다.
PC 관측과 장비 인계는 해당 사용 안내를 참고한다.

## 목적별 탐색

| 목적 | 기준 문서 | 주요 코드 근거 |
| --- | --- | --- |
| 프로필 선택, 모듈 관계, 실행 순서 | [펌웨어 구조](firmware-architecture.ko.md) | [platformio.ini](../platformio.ini), [main.cpp](../src/main.cpp) |
| RC 모드, 핀, PWM 보정, 모션 제한 | [RC 제어와 조향](rc-steering.ko.md) | [rc_io.cpp](../src/rc_io.cpp), [control.cpp](../src/control.cpp), [steering_config.hpp](../include/steering_config.hpp) |
| CAN 매핑, 송수신 주기, 유효성 | [CAN 송수신](can-rx-fifo.md) | [can_control.cpp](../src/can_control.cpp), [flexcan0.cpp](../src/flexcan0.cpp) |
| USB 바이트 규격, 오류, 모드 전환 | [USB 프로토콜](usb-protocol.ko.md) | [serial_protocol.cpp](../src/serial_protocol.cpp), [can_debug.cpp](../src/can_debug.cpp) |
| PC 관측, CLI, 단발 명령 | [호스트 도구](can-cli-windows.md) | [can_cli.py](../src/can_cli.py), [packet_monitor.py](../src/packet_monitor.py), [control.py](../src/control.py) |
| 누적 시간·응답 누락·0x84 해석 | [타이밍 진단](timing-diagnostics.md) | [timing_diagnostics.cpp](../src/timing_diagnostics.cpp) |
| 다른 PC/Pi로 전달 | [인계 절차](raspberry-pi-handoff.ko.md) | [pyproject.toml](../pyproject.toml), [uv.lock](../uv.lock) |

## 구현의 기준

- 펌웨어 프로필과 기능 포함 여부는 `platformio.ini`와 `build_features.hpp`가 정의한다.
- RC 모드·전원·구동 권한은 `rc_io.cpp`와 `control.cpp`가 처리한다.
- 차량 기하·서보 보정·모션 제한은 `steering_config.hpp`에 있다.
- USB 필드와 전송 조건은 프로토콜 문서, CAN 응답의 유효성은 CAN 문서가 기준이다.
- 헤더별 역할은 [include 안내](../include/README), 라이브러리 배치는 [lib 안내](../lib/README)를 참고한다.

## 검증 상태와 해석 범위

문서의 동작 설명은 소스에서 확인할 수 있는 처리 규칙이다. 현재 작업 트리에 대한 빌드·테스트·
실물 통신 검증 결과는 없으며, 장비에 업로드된 바이너리와의 일치 여부도 미확인이다.
활성 `test/` 디렉터리는 없다. 소프트웨어의 구동 상태·조향 도달 추정은 실제 정지·서보 도달을
확인하는 피드백이 아니다. 각 기능의 제약은 해당 문서에서 설명한다.

## 사고 분석과 보관 자료

아래 자료는 날짜가 명시된 관측·설계·복구 기록이다. 현재 사용법은 위 기준 문서를 따른다.

| 기록 | 내용 |
| --- | --- |
| [2026-11-09 조종 기능 상실 검토](rc-loss-of-control-review-2026-11-09.ko.md) | 사고 관측, 원인 후보, 코드 근거와 미확인 조건 |
| [2026-11-07 RC 정리 백업](../backups/rc-cleanup-20261107-154207/README.md) | ZIP·manifest·patch와 복구 절차 |
| [초기 조향 설계 한국어](../backups/rc-cleanup-20261107-154207/removed-files/docs/independent-steering.ko.md) / [영어](../backups/rc-cleanup-20261107-154207/removed-files/docs/independent-steering.md) | 당시 설계와 검증 기록 |
| [변경 그룹 메모](../backups/rc-cleanup-20261107-154207/removed-files/docs/change-groups.md) | 당시 커밋 분리·staging 메모 |

보관 자료의 테스트 결과·설치 코어·제조사 문서 검토는 각각 기록된 시점의 근거다.
백업의 실행 안내나 바이너리를 현재 소스의 검증 결과로 사용하지 않는다.
