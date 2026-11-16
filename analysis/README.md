# 시뮬레이션 데이터 안내

이 디렉토리는 조향각·휠 RPM 지령을 계산하는 오프라인 시뮬레이션의 코드와 결과를 담는다.
이 문서는 실험 조건, 계산 대상, 기록 항목과 파일 구성을 설명한다.

문서 기준일: 2026-11-16.

## 현재 펌웨어와의 관계

이 디렉터리에 저장된 실험 1~5는 **선속도·곡률(`v, κ`) 기반** 계산과 구현의 기록이다.
현재 펌웨어의 MANUAL 입력·보간 변수는 선속도·가상 중앙 휠 조향각(`v, θ`)으로 변경되었다.
실험 1~5의 그림·CSV·JSON은 다시 계산하지 않았으며, 새 `v, θ` 구현의 검증 결과가 아니다.
새 구현의 step·ramp 재생은 [실험 6](linear_motion_schedule/theta_output/README.ko.md)에 별도로 저장했다.

기존 실험 코드와 곡률 기반 펌웨어는 커밋 `7f61e4f`에 함께 보존되어 있다.
실험 1~5의 재실행 명령은 해당 커밋의 별도 체크아웃을 대상으로 한다. 해당 Python 진입점은
곡률 명령 API가 없으면 중단한다. 곡률 값을 중앙 조향각으로 잘못 전달하지 않도록 하는 확인이다.
실험 6은 중앙 조향각을 직접 입력한다. RC 필터·PWM 역변환·노이즈를 포함한 실험은 새 구현으로 재실행하지 않았다.

## 실험 목록과 저장된 결과의 설정

| 실험 | 실행 코드 | 결과 디렉토리 | 실행 조합 | RPM 크기 증가 / 감소 한계 |
|---|---|---|---|---|
| 1. 단순 선형 보간 | [simulate.py](linear_motion_schedule/simulate.py) | [output](linear_motion_schedule/output/) | 9개 시나리오 | 300 / 300 RPM/s. 계산에는 적용하지 않고 비교 기준으로 사용 |
| 2. 당시 곡률 기반 알고리즘 재생 | [compare_controller.py](linear_motion_schedule/compare_controller.py) | [controller_output](linear_motion_schedule/controller_output/) | 9개 시나리오 × step·ramp = 18개 | 300 / 300 RPM/s. C++ 스케줄러에 적용 |
| 3. 진행률 이력 재사용 검사 | [check_fraction_history.py](linear_motion_schedule/check_fraction_history.py) | [fraction_output](linear_motion_schedule/fraction_output/) | 9개 시나리오 × step·ramp = 18개 | 200 / 400 RPM/s. 해당 결과 생성 당시의 설정 |
| 4. 정지 후 입력 순서 비교 | [compare_sequence.py](linear_motion_schedule/compare_sequence.py) | [sequence_output](linear_motion_schedule/sequence_output/) | 2개 순서 × step·ramp = 4개 | 300 / 300 RPM/s. C++ 스케줄러에 적용 |
| 5. RC 노이즈 입력 | [compare_rc_noise.py](linear_motion_schedule/compare_rc_noise.py) | [noise_output](linear_motion_schedule/noise_output/) | 노이즈 3 × 입력 2 × 속도 2 × 곡률 2 × 순서 2 = 48개 | 300 / 300 RPM/s. C++ 스케줄러에 적용 |
| 6. 중앙 조향각 기반 알고리즘 재생 | [compare_theta.py](linear_motion_schedule/compare_theta.py) | [theta_output](linear_motion_schedule/theta_output/) | 9개 시나리오 × step·ramp = 18개 | 300 / 300 RPM/s. v·θ C++ 스케줄러에 적용 |

실험 1·2는 2026-11-15에 300/300 RPM/s 설정으로 갱신했다.
실험 3은 이 설정 변경 이후 재실행하지 않은 결과다.
실험 4·5는 2026-11-16에 실행했다. 결과 디렉토리 전체가 동일한 설정으로 실행된 데이터 묶음은 아니다.

각 `summary.json`의 `source_sha256`에는 결과 생성에 사용한 헤더의 SHA-256이 기록되어 있다.
저장된 `steering_config.hpp` 해시는 실험 1·2에서 `d3fb3315…`, 실험 3에서 `a92832de…`이다.
실험 1·2는 설정 수치도 `config`에 저장하며, 실험 3의 JSON에는 설정 수치 객체 없이 소스 해시가 저장되어 있다.

## 공통 입력과 좌표의 의미

- `v`: 후륜 축 중앙을 기준으로 한 선속도. 단위는 m/s이며, 전진은 양수, 후진은 음수다.
- `κ` 또는 `k`: 곡률. 단위는 1/m이다. 일반 주행 운동학에서 차체 각속도는 `ω = vκ`이다.
- `θ`: 가상 전륜 중앙 휠 조향각. 실험 6의 C++ 입력은 rad, CSV·그래프는 degree이다. `κ = tan(θ)/L`이다.
- 양의 곡률은 전진 시 좌회전에 해당한다. 같은 곡률에서 후진하면 차체 각속도 부호는 반대가 된다.
- `LF`, `RF`, `LR`, `RR`: 각각 좌측 전륜, 우측 전륜, 좌측 후륜, 우측 후륜이다.
- 조향각은 좌우 전륜의 차량 기준 각도 지령이다. 서보축 각도 측정값이나 PWM 펄스폭이 아니다.
- RPM은 운동학에서 계산한 휠 회전수 지령이다.

공통 시나리오는 [simulate.py의 CASES](linear_motion_schedule/simulate.py)에 정의되어 있다.

| 번호 | 내용 | 시작 → 목표 v [m/s] | 시작 → 목표 κ [1/m] | 선형 입력 전환 시간 [s] |
|---|---|---|---|---:|
| 01 | 직진 가속 | 0.2 → 1.2 | 0 → 0 | 1.00 |
| 02 | 일정 선속도에서 선회 | 0.8 → 0.8 | 0 → 2 | 1.00 |
| 03 | 선속도·곡률 동시 증가 | 0.3 → 1.0 | 0 → 2 | 1.00 |
| 04 | 감속하며 선회 | 1.0 → 0.3 | 0 → 2 | 1.00 |
| 05 | 좌회전에서 우회전으로 전환 | 0.6 → 0.6 | 2 → −2 | 1.00 |
| 06 | 정지 상태 조향 | 0 → 0 | −2 → 2 | 1.00 |
| 07 | 후진 속도 크기·곡률 증가 | −0.3 → −1.0 | 0 → 2 | 1.00 |
| 08 | 일정 곡률에서 전진·후진 전환 | 0.6 → −0.6 | 1 → 1 | 1.00 |
| 09 | 짧은 전환 시간의 선회 입력 | 0.8 → 0.8 | 0 → 2 | 0.25 |

입력 모드의 정의는 다음과 같다.

- **Step:** 시각 0에 초기 출력을 기록하고, 첫 10ms 갱신부터 최종 목표를 입력·유지한다. 위 표의 선형 입력 전환 시간은 step에 적용하지 않는다.
- **Ramp:** 위 표의 시간 동안 요청 `v`, `κ` 자체를 각각 선형으로 바꾸고, 이후에는 최종 목표를 유지한다.
- 실험 1은 선형 입력을 운동학으로 직접 변환한다. 실험 2·3은 step 또는 ramp 요청을 스케줄러에 입력한다.

저장된 결과의 차체 치수는 축거 0.420m, 전·후륜 윤거 각각 0.439m, 휠 반경 0.065m이다.
공통 설정은 조향 범위 ±70°, 조향각속도 한계 180°/s, 휠 RPM 상한 200RPM이다.
입력 선속도 유효범위는 ±2m/s이다. 실험 1에서는 이 제한들을 비교·초과 판정에 사용하며 출력을 제한하지 않는다.
9개 시나리오에는 PIVOT 진입·탈출 입력이 없다.

## 1. 단순 선형 보간

### 계산 내용

시작 상태에서 목표 상태까지 다음 입력을 생성한다.

```text
s = t / T
v(t) = v0 + (v1 − v0) × s
κ(t) = κ0 + (κ1 − κ0) × s
```

이 입력을 일반 주행 운동학 식에 대입해 좌우 전륜 조향각과 네 휠 RPM을 계산한다.
차체 치수와 제한 설정은 현재 저장소 헤더에서 읽지만, `MotionController`는 실행하지 않는다.
RPM 상한, 가감속 제한, 조향각속도 제한, 상태 전환 절차를 출력에 적용하지 않는다.

저장된 결과의 샘플 간격은 10ms이다. 전환 구간의 양 끝을 포함하며,
1초 시나리오는 101개, 0.25초 시나리오는 26개 샘플이다.

### 기록과 그림

- 입력 선속도·곡률, 좌우 전륜 조향각, 네 휠 RPM.
- 운동학 식을 해석적으로 미분한 조향각속도와 RPM 변화율.
- 인접 샘플 간 조향각·RPM 차이. CSV의 첫 샘플 차이는 이전 샘플이 없어 `NaN`이다.
- 설정 한계에 대한 초과 여부 및 샘플 지점에서의 최대값.

그림의 제한선은 비교 기준이다. 제한선을 넘는 계산값도 그대로 표시한다.
전환 구간의 양 끝에서 변화율은 구간 안쪽 단측 값이며, 전환 전후 유지 구간은 포함하지 않는다.

| 파일 | 내용 |
|---|---|
| `<시나리오>.png`, `.svg` | 입력, 조향각, RPM, 변화율, 샘플 간 변화량을 표시한 상세 그림 |
| `<시나리오>.csv` | 상세 시계열 |
| `overview_01.png` | 01·02·03번 비교 |
| `overview_02.png` | 04·05·09번 비교 |
| `overview_03.png` | 06·07·08번 비교 |
| [summary.csv](linear_motion_schedule/output/summary.csv) | 시나리오별 조건·최대값·초과 항목 |
| [summary.json](linear_motion_schedule/output/summary.json) | 설정, 소스 해시, 휠별 수치, 초과 판정 |

## 2. 당시 곡률 기반 알고리즘 재생

### 계산 내용

[replay_controller.cpp](linear_motion_schedule/replay_controller.cpp)가
[vehicle_motion.hpp](../include/vehicle_motion.hpp)의 `MotionController`를 PC에서 직접 호출한다.
Python 스크립트는 C++ 실행 파일을 컴파일·실행하고 결과를 집계·시각화한다.

각 시나리오에서 초기 명령을 10초 동안 유지한 다음, 그 결과를 시각 0의 초기 출력으로 사용한다.
이후 10ms 간격으로 5초간 갱신하며, 초기 샘플을 포함해 501개를 기록한다.
초기 준비 구간은 결과 시계열에서 제외한다.

호출되는 C++ 코드에는 목표 RPM 공통 축소, 조향·RPM 변화량 검사, 후보 진행률 반감,
제동 및 상태 전환 처리가 포함된다.
실험은 원본 헤더 실행과 횟수 계측을 추가한 복사본 실행을 수행하며,
출력·상태 등 기본 14개 CSV 열의 일치 여부를 확인한다. 그 결과는 JSON에 기록한다.

### 기록과 그림

- 요청 `v`, `κ`와 생성 지령에서 역산한 `v`, `κ`.
- 전륜 조향각, 네 휠 RPM, 갱신 종료 시 `transition`, `limited`, `feasible`.
- 후보 시도·거부 횟수, `evaluate()` 호출 횟수, 후보 채택 여부.
- 10ms 출력 차분으로 계산한 조향각속도와 RPM 크기 변화율.

생성 선속도는 후륜 RPM 평균에서, 생성 곡률은 좌측 전륜 조향각에서 역산한다.
스케줄러의 비공개 `scheduled` 상태를 직접 읽거나 변경하지 않는다.
상세 그림은 왼쪽에 step, 오른쪽에 ramp를 배치하고, 기존 단순 선형 보간 결과를 색상 점선으로 함께 표시한다.

| 파일 | 내용 |
|---|---|
| `<시나리오>_step.csv`, `_ramp.csv` | 입력 방식별 전체 시계열과 계측값 |
| `<시나리오>_comparison.png`, `.svg` | step·ramp 상세 비교 |
| `overview_step_01.png` ~ `_03.png` | step 결과를 01~03, 04~06, 07~09번으로 묶은 그림 |
| `overview_ramp_01.png` ~ `_03.png` | 같은 구성의 ramp 그림 |
| `overview_contact.png` | 여섯 overview 그림의 축소 모음. 실행 스크립트와 별도로 생성한 파일 |
| [summary.csv](linear_motion_schedule/controller_output/summary.csv) | 도달 시간, 최종값, 최대값, 재시도 집계 |
| [summary.json](linear_motion_schedule/controller_output/summary.json) | 설정, 컴파일러, 소스 해시, 상태 순서, 재시도 분포 |

## 3. 진행률 이력 재사용 검사

### 검사 내용

기존 스케줄러의 출력 경로에서 첫 후보가 거부된 갱신을 대상으로 다음을 기록한다.

1. 이번 첫 후보 진행률과 직전 채택 진행률의 증가·동일·감소 관계.
2. 이번 첫 후보 진행률과 직전 첫 후보 진행률의 증가·동일·감소 관계.
3. 같은 갱신 직전 상태와 현재 목표에 직전 채택 진행률을 적용했을 때의 제한 검사 결과.
4. 같은 조건에 직전 첫 후보 진행률을 적용했을 때의 제한 검사 결과.

대안 후보는 갱신 직전 controller의 복사본에서 계산한다.
원래 출력에 반영하지 않으므로, 이전 진행률을 계속 채택하는 정책의 전체 궤적을 계산한 실험은 아니다.
초기 준비 및 관측 시간은 실험 2와 같고, 직전 갱신에 채택 후보가 없는 경우 이력 없음으로 처리한다.
첫 갱신의 직전 이력은 초기 명령 유지 구간의 마지막 채택값이다.

원본과 계측본의 기본 14개 출력 열을 비교하며, 복사본에서도 첫 후보 거부가 재현되는지 검사한다.

### 기록과 그림

| 파일 | 내용 |
|---|---|
| `<시나리오>_step.csv`, `_ramp.csv` | 기본 시계열, 현재·직전 진행률, 재사용 후보 검사 결과 |
| [failure_events.csv](linear_motion_schedule/fraction_output/failure_events.csv) | 첫 후보 거부가 발생한 갱신만 모은 데이터 |
| `fractions_step.png`, `fractions_ramp.png` | 첫 후보·채택 진행률, 직전 채택 진행률 재사용 검사 결과 |
| [summary.json](linear_motion_schedule/fraction_output/summary.json) | 증가·동일·감소 개수, 재사용 검사 통과·실패 개수, 소스 해시 |

`initial_fraction`은 이번 첫 후보, `accepted_fraction`은 이번 채택값이다.
`previous_initial`, `previous_accepted`는 각각 직전 첫 후보와 직전 채택값이다.
`reuse_accepted_ok`, `reuse_initial_ok`는 통과 `1`, 실패 `0`, 미검사 `−1`이다.
`reuse_angle_ratio`, `reuse_rpm_ratio`는 재사용 후보의 변화량을 해당 허용 변화량으로 나눈 최대 비율이다.
`first_angle_ratio`, `first_rpm_ratio`는 첫 후보에 대해 계산한 같은 값이다.

## 4. 정지 후 입력 순서 비교

[replay_sequence.cpp](linear_motion_schedule/replay_sequence.cpp)가 실행 당시 곡률 기반 C++ 스케줄러에
다음 두 입력 순서를 적용한다. 시작 명령은 선속도·곡률 모두 0이다.

| 순서 | 0.5초에 시작하는 입력 | 2.0초에 시작하는 입력 |
|---|---|---|
| 곡률 우선 | 곡률 0 → 2/m | 선속도 0 → 0.8m/s |
| 선속도 우선 | 선속도 0 → 0.8m/s | 곡률 0 → 2/m |

Step은 각 시작 시각에 즉시 변경하고, ramp는 각 시작 시각부터 1초 동안 변경한다.
10초 초기 준비 후 DRIVING에서 시작하여 10ms 간격으로 5초를 기록한다.
시동·STOP 모드 해제·PIVOT 전환이나 RC 잡음 입력은 포함하지 않는다.

- 원본과 계측 복사본의 기본 출력·상태 14개 열을 비교한다.
- 전륜 조향각, 네 휠 RPM, 생성 선속도·곡률, 차분 변화율, 상태와 후보 시도 횟수를 기록한다.
- 첫 후보의 제한 초과량과 거부 조건, 첫 진행률·채택 진행률을 함께 기록한다.
- 시간순 그림과 스로틀 입력 시작 시점을 맞춘 속도 비교 그림을 제공한다.
- 속도 지표는 스로틀 입력 후 최초 비영 지령, 90% 도달, 목표 속도 유지 도달로 나눈다.
- 최종 전체 출력 도달 기준은 해석적 최종 목표 대비 RPM 1e-3RPM, 조향각 1e-4° 이내다.
  실험 2의 마지막 샘플 기준 도달 판정과 정의·허용오차가 다르다.

조건, 집계 구간, CSV 열과 결과 표는 [실험 4 설명](linear_motion_schedule/sequence_output/README.ko.md)에 있다.

## 5. RC 노이즈 입력

실험 4의 두 순서에 대해 명목 목표 선속도 0.3/0.8m/s, 곡률 1/2m⁻¹,
step/각 1초 ramp와 세 노이즈 수준을 조합한다. 첫 입력은 0.5초, 두 번째는 2초다.

| 노이즈 | 손 떨림의 정상상태 표준편차 | 측정 오차의 균등분포 범위 |
|---|---:|---:|
| 약 | 2µs | ±1µs |
| 중 | 5µs | ±3µs |
| 강 | 10µs | ±5µs |

수치는 실측 RC 규격이 아닌 합성 입력의 가정이다. 손 떨림은 상관시간 0.15초인
Gaussian AR(1)이며, 측정 오차는 프레임별 독립 난수다. 프레임 간격은 20ms로 가정한다.
동일 목표·입력 방식의 두 순서와 세 노이즈 수준에 같은 표준 난수열을 사용한다.
조건당 하나의 난수열이며 여러 seed에 걸친 통계 실험은 아니다.

명목 v·κ를 PWM으로 역변환하여 노이즈를 더한 뒤, 실제 소스에서 추출한
10샘플 정수 이동평균·±50µs 데드밴드·MANUAL 변환을 거쳐 실행 당시 곡률 기반 스케줄러에 넣는다.
노이즈를 포함한 중립 준비 10초 후 DRIVING에서 5초를 10ms 간격으로 기록한다.
ISR 에지 동작·채널 위상·장치 응답은 모델에 포함하지 않는다.

원본 실행과 계측본의 첫 28개 CSV 열을 비교한다. 명목 PWM, 잡음 성분, 측정·필터 PWM,
변환된 요청, 생성 조향·RPM, 상태·재시도 및 첫 후보 초과량을 기록한다.
노이즈로 목표가 계속 변하므로 최종 정착 시간 대신 90% 최초 도달,
목표 ±5% 안에서 200ms를 유지하는 첫 구간, 마지막 1초의 평균·표준편차 등을 기록한다.

실험 4는 v·κ를 직접 입력했고, 실험 5는 RC 필터·정수화·프레임 갱신도 포함한다.
두 실험의 차이 전체를 노이즈만의 영향으로 간주할 수 없다.
모든 조건·수치와 그래프 목록은 [실험 5 설명](linear_motion_schedule/noise_output/README.ko.md)에 있다.

## 6. 중앙 조향각 기반 알고리즘 재생

현재 `vehicle_motion.hpp`를 PC에서 컴파일하여 실행한다. 실험 2와 같은 9개 시나리오의
시작·목표 곡률을 `θ=atan(Lκ)`로 환산하여 같은 시작·종료 자세를 사용한다.
step은 첫 갱신부터 최종 `v, θ`를 입력한다. ramp는 1초(9번 0.25초) 동안 `v, θ`를 선형 변경한다.
따라서 이전 곡률 선형 ramp와 중간 요청 경로가 다르다.

- 초기 명령 10초 유지 후 DRIVING에서 시작하고, 10ms 간격으로 5초 기록한다.
- 중앙 조향각·선속도의 요청과 생성값, 전륜 각도, 네 휠 RPM, 변화율, 재시도, 상태를 기록한다.
- 원본 헤더 실행과 계측 복사본의 첫 16개 출력 열을 비교한다.
- 도달 시각은 마지막 샘플 대신 별도로 계산한 최종 목표와 비교한다. 목표에는 공통 200RPM 제한을 적용한다.
  모든 조향각 오차 1e-4도, RPM 오차 1e-3 이내에 이후 계속 머무는 첫 시각이며 미도달은 null이다.
- RC 필터·노이즈, 실제 서보·구동기 응답, CAN·PWM 전송은 포함하지 않는다.

[실험 6 보고서](linear_motion_schedule/theta_output/README.ko.md)에 전체 표·그래프·재시도 분포를,
[summary.csv](linear_motion_schedule/theta_output/summary.csv)에 수치표를 저장한다.
시나리오별 상세 그림은 왼쪽 step, 오른쪽 ramp이며 마지막 행은 펌웨어 상태 변수다.
실험 2와 도달 판정 기준이 다르므로 도달 시간을 동일 지표로 직접 비교하지 않는다.

## 지표를 읽는 방법

### 변화율

실험 1의 변화율은 해석적 미분값이고, 실험 2의 변화율 그림은 인접 출력 차분이다.
따라서 두 그림의 변화율 값은 계산 정의가 다르다.

| 항목 | 의미 | 단위 |
|---|---|---|
| 조향각속도 | 조향각의 시간 변화율 | °/s |
| 부호 있는 RPM 변화율 | `dn/dt` | RPM/s |
| RPM 크기 변화율 | `d|n|/dt` 또는 인접 RPM 절댓값의 차분 | RPM/s |
| 샘플 간 변화량 | 이번 출력 − 직전 출력 | ° 또는 RPM |

RPM 크기 변화율이 양수이면 회전수 크기가 증가하고, 음수이면 감소한다.
실험 1에서는 RPM 0에서 출발 방향의 단측 값을 사용한다.
전후진 전환의 영점에서는 RPM 절댓값의 양측 미분이 정의되지 않을 수 있다.
요약 최대값은 기록한 샘플에서 구한 값이다.

### 후보 시도·재시도·evaluate 호출

- `attempts`: 한 갱신에서 실제 검사한 후보 수.
- 추가 재시도 수: `max(attempts − 1, 0)`.
- `rejected`: 거부된 후보 수.
- `evaluations`: 목표 계산과 중간 후보 계산을 포함한 `evaluate()` 호출 수.
- `accepted`: 후보 채택 여부. `0`에는 후보 탐색 자체가 없었던 갱신도 포함된다.
- `failed_search_updates`: 후보 탐색을 수행했지만 채택하지 못한 갱신 수.

예를 들어 목표 계산 1회, 첫 중간 후보 계산 1회, 반감 후 후보 계산 1회를 수행하면
`evaluations=3`, `attempts=2`, 추가 재시도 수는 `1`이다.
후보가 최종 목표 자체이면 이미 계산한 목표 출력을 재사용할 수 있다.

실험 2의 `retry_histogram`은 전환 완료까지 후보 탐색을 수행한 갱신을 센다.
`{"0": 58, "1": 31}`이라는 형식은 추가 재시도 0회인 갱신 58개와 1회인 갱신 31개를 뜻한다.
실험 3의 `all_vs_previous_*` 집계에는 5초 관측 중 목표 도달 후 유지 갱신도 포함된다.

### 상태와 도달 시간

CSV의 `transition`은 `STARTUP=0`, `DRIVING=1`, `BRAKING=2`, `STEERING=3`, `SETTLING=4`이다.
상태 순서 목록은 갱신 종료 시 기록된 값의 연속 중복을 제거한 것이다.
한 갱신 안에서 진입했다가 다른 상태로 넘어간 값은 이 목록에 나타나지 않을 수 있다.
`DRIVING`은 상태 변수 값이며, RPM이 0이 아니라는 의미는 아니다.

실험 2의 `reached_s`는 관측 마지막 출력과 비교해 모든 휠 RPM 차이가 1e-4RPM 이내,
모든 조향각 차이가 1e-5° 이내에 이후 계속 머무는 첫 샘플 시각이다.
비교 대상은 원래 요청값이 아니라 최종 생성 출력이다.

## 실행 환경과 재실행

Python은 NumPy·Matplotlib을 사용한다. C++ 재생은 PATH의 `g++`를 사용한다.
저장된 실험 2·3의 컴파일러는 MinGW GCC 6.3.0이며,
`-std=c++11 -O2 -msse2 -mfpmath=sse -ffp-contract=off` 옵션을 사용했다.
그래프는 `C:/Windows/Fonts/malgun.ttf`가 있으면 해당 글꼴을 사용한다.

저장소 루트에서 각 실험을 개별 실행한다.

```powershell
# 1. 단순 선형 보간
uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/simulate.py

# 2. 당시 곡률 기반 알고리즘 재생
uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/compare_controller.py

# 3. 진행률 이력 재사용 검사
uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/check_fraction_history.py

# 4. 정지 후 입력 순서 비교
uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/compare_sequence.py

# 5. RC 노이즈 입력
uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/compare_rc_noise.py

# 6. 중앙 조향각 기반 재생 (현재 체크아웃)
uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/compare_theta.py
```

각 실행은 실행 시점의 저장소 설정을 사용하고 해당 결과 디렉토리에 같은 이름의 파일을 쓴다.
따라서 `7f61e4f`의 300/300RPM/s 설정으로 실험 3을 다시 실행하면 저장된 200/400RPM/s 결과도 새 설정의 결과로 덮어쓴다.
단순 선형 보간은 `--case`, `--custom`, `--dt`, `--output` 인자를 지원한다.

`controller_output/build/`, `fraction_output/build/`, `sequence_output/build/`, `noise_output/build/`, `theta_output/build/`에는 생성된 계측 헤더, 실행 파일 및 원본 비교 CSV가 위치한다.
이 경로와 실행 파일·Python 캐시는 하위 `.gitignore`의 제외 대상이다.

여섯 실험 모두 실제 장치와 통신하지 않는다. 결과에는 실제 서보·모터 응답,
CAN 전송 및 정수화, 서보 PWM 출력 프레임, 부하·미끄러짐, 센서 측정값이 포함되지 않는다.
실험 5에만 가정한 RC 수신 프레임 간격과 정수 PWM 입력 처리가 포함된다.
PC에서 계산한 지령과 호출 횟수이며, MCU 실행시간이나 최악 실행시간을 측정한 데이터는 아니다.
