# RC cleanup backup

Base commit: `025b28119e39c45647f3f8efedb0928a8c3ef94b`

The ZIP contains exact pre-cleanup modified/untracked file bytes under `files/`, a SHA-256 manifest, and binary Git patches. Ignored build/cache files are excluded.

To inspect, extract into a separate directory. For full restoration, use the recorded base commit in a separate checkout and overlay `files/` at the repository root; honor any manifest entries with exists=false as deletions. This preserves untracked files and byte-level line endings that a Git patch alone may omit. Do not blindly overwrite later work.

## 정리 결과

RC MANUAL 전용으로 축소했다. AUTO에서는 구동을 차단하며, ROS 각속도 운동학과
스키드 확장, 런타임 모드 전환 API, 조향 세부 계측, 선택적 0x83 출력 및
추가 0x82 조향 상태 비트를 제거했다. 기존 CAN·Windows CLI·공통 타이밍 커밋은 유지했다.

- `before-cleanup.zip`: 정리 직전 미커밋 파일 18개의 원본과 패치. 공유 파일에서
  제거한 코드도 이 원본에 포함된다. 파일 내용과 ZIP 내용의 일치를 확인했다.
- `removed-files/docs/`: 초기 조향 설계 문서와 변경 그룹 메모 원본.
- `removed-files/test/`: 추가 조향/모션 테스트 원본. 현재 테스트 경로 밖으로 이동했다.
- `cleanup-summary.json`: HEAD로 되돌린 파일, 이동한 파일과 현재 유지한 파일 목록.

RC의 ±70° 임시 보정, PWM 방향 반전, 최대 180°/s 동시 도달, 가속/감속
200/400 RPM/s 및 5° 감속 정책 제거는 현재 코드에 유지했다.
검토 범위는 소스·참조·백업 무결성 확인이며 테스트·빌드·업로드·커밋은 수행하지 않았다.
