# RC 정리 백업 — 2026-11-07

[현재 문서 안내](../../docs/README.md) · [현재 RC 보정](../../docs/rc-steering.ko.md)

이 디렉터리는 당시 정리 전 파일을 보관한다. ZIP·manifest·patch·보관 테스트는 복구 증거이며
현재 빌드 입력이 아니다. 이번 문서 정리에서는 그 원본 payload를 수정하거나 실행하지 않았다.
`removed-files/docs/`에는 당시 본문과 현재 차이 안내를 함께 두었다.

Base commit: `025b28119e39c45647f3f8efedb0928a8c3ef94b`

The ZIP contains exact pre-cleanup modified/untracked file bytes under `files/`, a SHA-256 manifest, and binary Git patches. Ignored build/cache files are excluded.

To inspect, extract into a separate directory. For full restoration, use the recorded base commit in a separate checkout and overlay `files/` at the repository root; honor any manifest entries with exists=false as deletions. This preserves untracked files and byte-level line endings that a Git patch alone may omit. Do not blindly overwrite later work.

## 2026-11-07 정리 결과

RC MANUAL 전용으로 축소했다. AUTO에서는 구동을 차단하며, ROS 각속도 운동학과
스키드 확장, 런타임 모드 전환 API, 조향 세부 계측, 선택적 0x83 출력 및
추가 0x82 조향 상태 비트를 제거했다. 기존 CAN·Windows CLI·공통 타이밍 커밋은 유지했다.

- `before-cleanup.zip`: 정리 직전 미커밋 파일 18개의 원본과 패치. 공유 파일에서
  제거한 코드도 이 원본에 포함된다. 당시 파일 내용과 ZIP 내용의 일치를 확인한 기록이다. 이번 문서 작업에서 그 무결성 검증을 재실행하지 않았다.
- `removed-files/docs/`: 초기 조향 설계 문서와 변경 그룹 메모 원본.
- `removed-files/test/`: 추가 조향/모션 테스트 원본. 당시 활성 경로 밖으로 이동했으며 현재 활성 `test/`는 제거됐다.
- `cleanup-summary.json`: HEAD로 되돌린 파일, 이동한 파일과 당시 유지한 파일 목록.

RC의 ±70° 임시 보정, PWM 방향 반전, 최대 180°/s 동시 도달, 가속/감속
200/400 RPM/s 및 5° 감속 정책 제거를 당시 코드에 유지했다. 이후 임시 보정은
2026-11-09 측정값으로 교체됐으므로 현재 보정값은 위 RC 문서를 따른다.
당시 검토 범위는 소스·참조·백업 무결성 확인이며 테스트·빌드·업로드·커밋은 수행하지 않았다.
