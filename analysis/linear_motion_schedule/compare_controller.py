"""Replay all nine assigned scenarios through the actual C++ MotionController.

Builds a host executable and an instrumented copy, compares their output, then
plots results. No firmware files, hardware, serial links, or project dependencies
are modified. Host replay is not an MCU execution-time measurement.
"""
from pathlib import Path
from datetime import datetime, timedelta, timezone
import csv
import hashlib
import json
import subprocess

import numpy as np
from simulate import CASES, COLORS, ROOT, WHEELS, load_config, simulate, plt, font_manager, require_curvature_scheduler

HERE = Path(__file__).resolve().parent
OUT = HERE / "controller_output"
BUILD = OUT / "build"
DT = 0.01
STATES = {0: "STARTUP", 1: "DRIVING", 2: "BRAKING", 3: "STEERING", 4: "SETTLING"}


def prepare():
    require_curvature_scheduler()
    BUILD.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "include/vehicle_motion.hpp").read_text(encoding="utf-8")
    # Count existing branches; do not change the scheduling arithmetic.
    edits = [
        ("        MotionConfig config;", "        mutable unsigned replay_evaluations = 0;\n        unsigned replay_attempts = 0, replay_rejected = 0;\n        bool replay_accepted = false;\n        MotionConfig config;"),
        ("            const float dt = initialized", "            replay_evaluations = replay_attempts = replay_rejected = 0;\n            replay_accepted = false;\n            const float dt = initialized"),
        ("            auto out = inverse(cmd", "            ++replay_evaluations;\n            auto out = inverse(cmd"),
        ("                RcCommand next = goal;", "                ++replay_attempts;\n                RcCommand next = goal;"),
        ("                    scheduled = next;", "                    replay_accepted = true;\n                    scheduled = next;"),
        ("                fraction *= 0.5f;", "                ++replay_rejected;\n                fraction *= 0.5f;"),
    ]
    for old, new in edits:
        if source.count(old) != 1:
            raise RuntimeError(f"Instrumentation anchor changed: {old}")
        source = source.replace(old, new)
    (BUILD / "instrumented_vehicle_motion.hpp").write_text(source, encoding="utf-8")
    for instrumented in (False, True):
        name = "instrumented" if instrumented else "original"
        command = ["g++", "-std=c++11", "-O2", "-msse2", "-mfpmath=sse", "-ffp-contract=off", "-Wall", "-Wextra", "-I", str(ROOT / "include"),
                   "-I", str(BUILD), str(HERE / "replay_controller.cpp"), "-o", str(BUILD / f"{name}.exe")]
        if instrumented:
            command.append("-DREPLAY_INSTRUMENTED")
        subprocess.run(command, check=True, capture_output=True, text=True)


def read_csv(path):
    return np.genfromtxt(path, delimiter=",", names=True, encoding="utf-8")


def replay(case, mode):
    for name in ("original", "instrumented"):
        dest = OUT / f"{case.key}_{mode}.csv" if name == "instrumented" else BUILD / f"{case.key}_{mode}.csv"
        subprocess.run([str(BUILD / f"{name}.exe"), mode, str(dest),
                        *map(str, (case.v0, case.v1, case.k0, case.k1, case.duration))], check=True)
    d = read_csv(OUT / f"{case.key}_{mode}.csv")
    original = read_csv(BUILD / f"{case.key}_{mode}.csv")
    for column in d.dtype.names[:14]:
        if not np.array_equal(d[column], original[column]):
            raise RuntimeError(f"Instrumentation changed {case.key}/{mode}/{column}")
    t = d["time_s"]
    angles = np.array([d[f"{w}_angle_deg"] for w in WHEELS[:2]])
    rpms = np.array([d[f"{w}_rpm"] for w in WHEELS])
    ar = np.diff(angles) / DT
    rr = np.diff(rpms) / DT
    growth = np.diff(np.abs(rpms)) / DT
    retry = np.maximum(d["attempts"] - 1, 0)
    held = (d["attempts"] > 0) & (d["accepted"] == 0)
    # First sample after which final outputs remain within these tolerances.
    different = (np.max(np.abs(rpms - rpms[:, -1:]), axis=0) > 1e-4) | (np.max(np.abs(angles - angles[:, -1:]), axis=0) > 1e-5)
    last_different = np.flatnonzero(different)
    reached = int(last_different[-1] + 1) if len(last_different) else 0
    end_time = max(float(t[reached]), case.duration if mode == "ramp" else 0)
    active = (t > 0) & (t <= end_time + 1e-8)
    drive = active & (d["attempts"] > 0)
    transition_sequence = [STATES[int(d["transition"][0])]]
    for state in d["transition"][1:]:
        if STATES[int(state)] != transition_sequence[-1]:
            transition_sequence.append(STATES[int(state)])
    histogram = {str(int(n)): int(np.sum(retry[drive] == n)) for n in np.unique(retry[drive])}
    summary = dict(case=case.key, title=case.title, input_mode=mode, reached_s=float(t[reached]),
                   final_v=float(d["generated_v"][-1]), final_k=float(d["generated_k"][-1]),
                   max_abs_rpm=float(np.max(np.abs(rpms))), max_abs_steering_rate_deg_s=float(np.max(np.abs(ar))),
                   max_rpm_increase_rpm_s=float(max(0, np.max(growth))),
                   max_rpm_decrease_rpm_s=float(max(0, -np.min(growth))),
                   max_attempts=int(np.max(d["attempts"])), max_recalculations=int(np.max(retry)),
                   max_evaluations=int(np.max(d["evaluations"])),
                   failed_search_updates=int(np.sum(held)), transition_drive_updates=int(np.sum(drive)),
                   retried_updates=int(np.sum(drive & (retry > 0))),
                   retry_histogram=histogram, transitions=transition_sequence,
                   source_output_match=True, all_feasible=bool(np.all(d["feasible"] == 1)))
    return dict(raw=d, t=t, angles=angles, rpms=rpms, angle_rates=ar, rpm_rates=rr,
                growth=growth, retries=retry, summary=summary)


def draw(ax, metric, data, case, cfg, horizon, reference=None):
    d, t = data["raw"], data["t"]
    titles = {"v": ("선속도", "m/s"), "k": ("곡률", "1/m"), "angles": ("전륜 조향각", "deg"),
              "rpms": ("휠 RPM", "RPM"), "angle_rates": ("샘플 간 조향각속도", "deg/s"),
              "growth": ("샘플 간 RPM 크기 변화율", "RPM/s"), "retries": ("갱신별 추가 후보 시도", "회")}
    if metric in ("v", "k"):
        ax.plot(t, d[f"request_{metric}"], color="#777777", ls="--", lw=1.2, label="요청")
        ax.plot(t, d[f"generated_{metric}"], color=COLORS[0], lw=1.7, label="생성")
    elif metric == "retries":
        ax.step(t, data[metric], where="post", color="#555555", lw=1, label="재시도")
        ax.set_ylim(-0.1, max(2.2, np.max(data[metric]) + 0.3))
        ax.yaxis.get_major_locator().set_params(integer=True)
    else:
        values = data[metric]
        x = t[1:] if metric in ("angle_rates", "growth") else t
        for i, value in enumerate(values):
            if reference is not None:
                ax.plot(reference["t"], reference[metric][i], color=COLORS[i], ls=":", lw=1, alpha=0.55)
            ax.plot(x, value, color=COLORS[i], lw=1.5, ls="-" if i < 2 else "--", label=WHEELS[i])
        limits = {"angles": (-70, 70), "rpms": (-cfg["rpm_limit"], cfg["rpm_limit"]),
                  "angle_rates": (-cfg["steering_rate"], cfg["steering_rate"]),
                  "growth": (-cfg["deceleration_rpm_s"], cfg["acceleration_rpm_s"])}
        for limit in limits[metric]:
            ax.axhline(limit, color="#777777", ls="--", lw=0.8, alpha=0.7)
    ax.set(title=titles[metric][0], ylabel=titles[metric][1], xlabel="시간 [s]", xlim=(0, horizon))
    ax.grid(alpha=0.2)
    ax.legend(fontsize=7, ncol=4, loc="best")


def main():
    prepare()
    cfg = load_config()
    font = Path("C:/Windows/Fonts/malgun.ttf")
    if font.exists():
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=font).get_name()
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 9})
    results, summaries = {}, []
    for case in CASES:
        reference, _ = simulate(case, cfg, DT)
        for mode in ("step", "ramp"):
            result = replay(case, mode)
            results[case.key, mode] = result
            summaries.append(result["summary"])
        horizon = max(case.duration, *(results[case.key, m]["summary"]["reached_s"] for m in ("step", "ramp"))) + 0.15
        fig, axes = plt.subplots(7, 2, figsize=(14, 20), layout="constrained")
        for col, mode in enumerate(("step", "ramp")):
            for row, metric in enumerate(("v", "k", "angles", "rpms", "angle_rates", "growth", "retries")):
                draw(axes[row, col], metric, results[case.key, mode], case, cfg, horizon, reference if metric in ("angles", "rpms", "angle_rates", "growth") else None)
            axes[0, col].set_title(("목표 즉시 입력 (step)" if mode == "step" else f"{case.duration:g}초 선형 입력 (ramp)") + " / 선속도")
        fig.suptitle(f"{case.key[:2]}. {case.title} — 현재 C++ 스케줄러\nv {case.v0:g}→{case.v1:g} m/s, κ {case.k0:g}→{case.k1:g} 1/m", fontsize=17)
        fig.supxlabel("색상 점선: 기존 미제한 선형 보간 | 회색 파선: 제한 | 변화율=10 ms 샘플 차분 | 실물 응답·MCU 실행시간 아님", fontsize=10)
        fig.savefig(OUT / f"{case.key}_comparison.png", dpi=135)
        fig.savefig(OUT / f"{case.key}_comparison.svg")
        plt.close(fig)
    for mode in ("step", "ramp"):
        for page in range(3):
            fig, axes = plt.subplots(3, 5, figsize=(20, 10.8), layout="constrained")
            for row, case in enumerate(CASES[page*3:page*3+3]):
                data = results[case.key, mode]
                horizon = max(case.duration, data["summary"]["reached_s"]) + 0.15
                for col, metric in enumerate(("angles", "rpms", "angle_rates", "growth", "retries")):
                    draw(axes[row, col], metric, data, case, cfg, horizon)
                axes[row, 0].set_title(f"{case.key[:2]}. {case.title}\nv {case.v0:g}→{case.v1:g}, κ {case.k0:g}→{case.k1:g}\n전륜 조향각", fontsize=9)
            fig.suptitle(f"현재 C++ 스케줄러 / {'목표 즉시 입력' if mode == 'step' else '기존 선형 입력 추종'} ({page+1}/3)", fontsize=18)
            fig.supxlabel("10 ms 주기 PC 재생 | 회색 파선=설정 한계 | 마지막 열=첫 후보 이후 추가 시도 횟수", fontsize=10)
            fig.savefig(OUT / f"overview_{mode}_{page+1:02d}.png", dpi=140)
            plt.close(fig)
    run_date = datetime.now(timezone(timedelta(hours=9))).date().isoformat()
    metadata = dict(run_date_kst=run_date, config=cfg, dt_s=DT, observation_s=5, warmup_s=10, floating_point_flags="-msse2 -mfpmath=sse -ffp-contract=off",
                    compiler=subprocess.run(["g++", "--version"], check=True, capture_output=True, text=True).stdout.splitlines()[0],
                    source_sha256={name: hashlib.sha256((ROOT / "include" / name).read_bytes()).hexdigest()
                                   for name in ("vehicle_motion.hpp", "vehicle_kinematics.hpp", "steering_config.hpp")},
                    instrumented_output_matches_original=True, results=summaries)
    (OUT / "summary.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    rows = [{k: v for k, v in s.items() if k not in ("retry_histogram", "transitions")} for s in summaries]
    with (OUT / "summary.csv").open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    lines = ["# 현재 MCU 스케줄러: 9가지 시나리오 비교", "", f"{run_date}. 실제 `include/vehicle_motion.hpp`를 PC에서 컴파일해 호출한 오프라인 결과이다.", "",
             "## 조건과 범위", "",
             f"- RPM 크기 증가율 {cfg['acceleration_rpm_s']:g}RPM/s, 감소율 {cfg['deceleration_rpm_s']:g}RPM/s, RPM 상한 {cfg['rpm_limit']:g}, 조향각속도 한계 {cfg['steering_rate']:g}도/s.",
             "- 10ms 고정 갱신. 초기 명령을 10초간 유지해 DRIVING에 도달한 뒤, 전환 구간 5초를 기록했다. 초기 기동 대기는 그래프에서 제외했다.",
             "- step: t=0 초기 상태를 기록하고 첫 10ms 갱신부터 최종 목표를 유지한다. MCU 자체에는 1초 전환시간 인자가 없다.",
             "- ramp: 이전 실험과 동일하게 v, κ 요청을 1초(9번은 0.25초) 동안 선형 변경하고 이후 최종 목표를 유지한다.",
             "- 조향각·RPM은 생성 지령이다. CAN 정수화·전송 간격, PWM 프레임, 모터 지연·부하·측정값은 포함하지 않았다.",
             "- 원본 헤더 실행 결과와 계측 복사본의 모든 출력·상태를 18개 실행 전체에서 동일하게 확인했다. 원본 펌웨어 파일은 수정하지 않았다.",
             "- float 연산은 PC g++에서 SSE(-msse2 -mfpmath=sse -ffp-contract=off)로 수행했다. 기본 32비트 MinGW의 x87 확장 정밀도 영향을 피했다. Teensy의 최종 비트 패턴이나 실행시간/WCET를 검증한 결과가 아니다.",
             "- 변화율은 인접 출력의 차이/0.01초이다. 이전 그림의 해석적 순간 변화율과는 정의가 다르다. 아주 작은 차이는 float와 CSV 반올림에서 생길 수 있다.",
             "- 생성 v는 후륜 RPM 평균, 생성 κ는 좌측 전륜 각도에서 역산했다. private scheduled 변수를 변경하지 않았다.",
             "- 도달 시간은 최종 출력 대비 RPM 1e-4, 조향각 1e-5도 이내에 이후 계속 머무는 첫 샘플이다. 곡률이 주어졌을 때 RPM 상한으로 낮아진 최종 속도를 포함한다.", "",
             "## 재연산 횟수의 의미", "",
             "- 후보 시도 수: `advance_drive()`의 실제 반복 횟수, 최대 12회.",
             "- 추가 재시도 수: 후보 시도 수 - 1. 첫 후보 거부 후 fraction을 절반으로 줄여 다시 시도한 횟수, 최대 11회.",
             "- evaluate 횟수: 목표 계산 1회와 실제 중간 후보 운동학 계산을 포함한다. 첫 후보가 목표 자체인 경우는 목표 계산을 재사용한다.",
             "- 모든 후보가 거부되면 이전 지령 전체를 유지한다. 이 횟수는 failed_search_updates에 기록했다.",
             "- BRAKING/STEERING/SETTLING 갱신은 이 후보 탐색을 수행하지 않을 수 있다. 재시도 0은 반드시 DRIVING이었다는 뜻이 아니다.",
             "- 표의 재시도 갱신 수/히스토그램은 전환 완료까지의 DRIVING 탐색 갱신만 집계한다. 완료 후 유지 구간을 분모에 넣지 않았다. 최대값·탐색 실패 횟수는 전체 5초 기준이다.", "",
             "## 전체 비교", "", "| 예시 | 입력 | 도달 [s] | 최종 v | 최대 RPM | 최대 조향속도 | 최대 RPM 증가율 | 최대 추가 시도 | 재시도 갱신/탐색 갱신 | 탐색 실패 |", "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for s in summaries:
        lines.append(f"| {s['title']} | {s['input_mode']} | {s['reached_s']:.2f} | {s['final_v']:.4f} | {s['max_abs_rpm']:.2f} | {s['max_abs_steering_rate_deg_s']:.2f} | {s['max_rpm_increase_rpm_s']:.2f} | {s['max_recalculations']} | {s['retried_updates']}/{s['transition_drive_updates']} | {s['failed_search_updates']} |")
    measured = lambda index, mode: results[CASES[index - 1].key, mode]["summary"]
    rate_excess = max(0, *(s['max_rpm_increase_rpm_s'] - cfg['acceleration_rpm_s'] for s in summaries),
                      *(s['max_rpm_decrease_rpm_s'] - cfg['deceleration_rpm_s'] for s in summaries))
    lines += ["", "## 주요 관찰", "",
              f"- 이번 {len(summaries)}개 조건에서는 추가 후보 시도 최대 {max(s['max_recalculations'] for s in summaries)}회(총 후보 {max(s['max_attempts'] for s in summaries)}회), evaluate 최대 {max(s['max_evaluations'] for s in summaries)}회였다. 탐색 소진 후 유지한 갱신은 {sum(s['failed_search_updates'] for s in summaries)}회다. 다른 입력·주기에서의 최악 횟수나 MCU 실행시간을 보장하는 결과는 아니다.",
              f"- 3·7번 step의 최종 속도 크기는 각각 {abs(measured(3, 'step')['final_v']):.6f}, {abs(measured(7, 'step')['final_v']):.6f}m/s다. 목표 RPM 공통 축소와 초기 fraction 계산은 추가 재시도에 포함되지 않는다. 재시도 0회가 제한을 적용하지 않았다는 뜻은 아니다.",
              f"- 9번 ramp 입력은 {CASES[8].duration:g}초에 끝나며 생성 지령은 {measured(9, 'ramp')['reached_s']:.2f}초에 도달한다. step은 전환시간을 받지 않으므로 2번과 9번은 동일한 입력이다.",
              f"- 8번의 관측 상태: step `{' → '.join(measured(8, 'step')['transitions'])}`, ramp `{' → '.join(measured(8, 'ramp')['transitions'])}`. STEERING처럼 같은 update 안에서 진입·탈출한 상태는 CSV 종료 상태에 따로 나타나지 않을 수 있다.",
              f"- 재시도가 있는 구간은 진행량을 절반으로 줄이므로 변화율이 매끄럽지 않거나 보수적으로 낮아질 수 있다. 6번 정지 조향의 도달 시간은 step {measured(6, 'step')['reached_s']:.2f}초, ramp {measured(6, 'ramp')['reached_s']:.2f}초다.",
              f"- 기록된 RPM 크기 변화율의 한계 초과 최대치는 {rate_excess:.9g}RPM/s다. float 시간 간격·출력 산술·CSV 표현에 따른 미세 차이도 포함한다.", "",
              "## 비교 그림", ""]
    for mode in ("step", "ramp"):
        for page in range(1, 4):
            lines.append(f"![{mode} {page}](overview_{mode}_{page:02d}.png)")
    lines += ["", "## 상세 그림과 데이터", ""]
    for case in CASES:
        lines.append(f"- {case.title}: [상세 PNG]({case.key}_comparison.png) · [SVG]({case.key}_comparison.svg) · [step CSV]({case.key}_step.csv) · [ramp CSV]({case.key}_ramp.csv)")
    lines += ["", "## 재시도 분포와 상태 변화", "", "분포는 `추가 시도 횟수: 갱신 개수`이다.", ""]
    for s in summaries:
        lines.append(f"- {s['title']} / {s['input_mode']}: `{s['retry_histogram']}`, 상태 `{' → '.join(s['transitions'])}`, evaluate 최대 {s['max_evaluations']}회.")
    lines += ["", "[요약 CSV](summary.csv) · [상세 수치·소스 해시 JSON](summary.json)", "", "재실행 (저장소 루트):", "", "```powershell",
              "uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/compare_controller.py", "```", ""]
    (OUT / "README.ko.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(summaries, ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
