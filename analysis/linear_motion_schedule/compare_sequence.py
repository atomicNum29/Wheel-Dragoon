"""Replay the two requested input orders using the unmodified current controller.

Run: uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/compare_sequence.py
Instrumentation is confined to a build copy and checked against original output.
"""
from pathlib import Path
from datetime import datetime, timedelta, timezone
import csv
import hashlib
import json
import subprocess

import numpy as np
from simulate import ROOT, COLORS, WHEELS, load_config, plt, font_manager

HERE = Path(__file__).resolve().parent
OUT = HERE / "sequence_output"
BUILD = OUT / "build"
DT = 0.01
ORDERS = {"steer_first": "정지 → 곡률 → 선속도", "speed_first": "정지 → 선속도 → 곡률"}
STATES = {0: "STARTUP", 1: "DRIVING", 2: "BRAKING", 3: "STEERING", 4: "SETTLING"}


def replace_once(source, old, new):
    if source.count(old) != 1:
        raise RuntimeError(f"Instrumentation anchor changed: {old}")
    return source.replace(old, new)


def prepare(build=BUILD, harness=HERE / "replay_sequence.cpp"):
    build.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "include/vehicle_motion.hpp").read_text(encoding="utf-8")
    fields = """        mutable unsigned replay_evaluations = 0;
        unsigned replay_attempts = 0, replay_rejected = 0, replay_mask = 0;
        bool replay_accepted = false;
        float replay_initial = -1, replay_final = -1;
        double replay_angle_excess = -1, replay_rpm_excess = -1, replay_peak_excess = -1;
"""
    # Observe float operands used by the original comparisons; report their
    # difference in double. Neither candidate arithmetic nor decisions change.
    probe = """                if (attempt == 0) {
                    replay_angle_excess = replay_rpm_excess = replay_peak_excess = -1e30;
                    if (!outputs.feasible) replay_mask |= 1;
                    for (int i = 0; i < STEERING_COUNT; ++i) {
                        const float delta = fabsf(outputs.angle[i] - angle[i]);
                        const float allowance = config.steering_rate * dt;
                        replay_angle_excess = fmax(replay_angle_excess, double(delta) - double(allowance));
                        if (!isfinite(outputs.angle[i]) || delta > allowance) replay_mask |= 2;
                    }
                    for (int i = 0; i < WHEEL_COUNT; ++i) {
                        const float delta = fabsf(outputs.rpm[i] - rpm[i]);
                        const float allowance = rpm_step_rate(rpm[i], outputs.rpm[i]) * dt;
                        replay_rpm_excess = fmax(replay_rpm_excess, double(delta) - double(allowance));
                        replay_peak_excess = fmax(replay_peak_excess, double(fabsf(outputs.rpm[i])) - double(config.rpm_limit));
                        if (!isfinite(outputs.rpm[i]) || fabsf(outputs.rpm[i]) > config.rpm_limit) replay_mask |= 4;
                        if (delta > allowance) replay_mask |= 8;
                    }
                }
"""
    edits = [
        ("        MotionConfig config;", fields + "        MotionConfig config;"),
        ("            const float dt = initialized", "            replay_evaluations = replay_attempts = replay_rejected = replay_mask = 0;\n            replay_accepted = false;\n            replay_initial = replay_final = -1;\n            replay_angle_excess = replay_rpm_excess = replay_peak_excess = -1;\n            const float dt = initialized"),
        ("            auto out = inverse(cmd", "            ++replay_evaluations;\n            auto out = inverse(cmd"),
        ("            for (uint8_t attempt = 0;", "            replay_initial = fraction;\n            for (uint8_t attempt = 0;"),
        ("                RcCommand next = goal;", "                ++replay_attempts;\n                RcCommand next = goal;"),
        ("                if (step_allowed(outputs, dt))", probe + "                if (step_allowed(outputs, dt))"),
        ("                    scheduled = next;", "                    replay_accepted = true;\n                    replay_final = fraction;\n                    scheduled = next;"),
        ("                fraction *= 0.5f;", "                ++replay_rejected;\n                fraction *= 0.5f;"),
    ]
    for old, new in edits:
        source = replace_once(source, old, new)
    (build / "instrumented_vehicle_motion.hpp").write_text(source, encoding="utf-8")
    flags = ["-std=c++11", "-O2", "-msse2", "-mfpmath=sse", "-ffp-contract=off", "-Wall", "-Wextra"]
    for variant in ("original", "instrumented"):
        cmd = ["g++", *flags, "-I", str(ROOT / "include"), "-I", str(build), str(harness), "-o", str(build / f"{variant}.exe")]
        if variant == "instrumented":
            cmd.append("-DREPLAY_INSTRUMENTED")
        result = subprocess.run(cmd, check=True, capture_output=True, text=True)
        (build / f"{variant}_compile.txt").write_text(result.stdout + result.stderr, encoding="utf-8")
    return flags


def first_time(t, condition, start=0):
    indexes = np.flatnonzero((t >= start - 1e-9) & condition)
    return float(t[indexes[0]]) if len(indexes) else None


def stable_time(t, close, start=0):
    stable = np.logical_and.accumulate(close[::-1])[::-1]
    return first_time(t, stable, start)


def delay(time, onset):
    return None if time is None else time-onset


def number(value):
    return "미도달" if value is None else f"{value:.2f}"


def run(order, mode, cfg):
    name = f"{order}_{mode}"
    for variant in ("original", "instrumented"):
        path = (BUILD if variant == "original" else OUT) / f"{name}.csv"
        subprocess.run([str(BUILD / f"{variant}.exe"), order, mode, str(path)], check=True)
    d = np.genfromtxt(OUT / f"{name}.csv", delimiter=",", names=True)
    original = np.genfromtxt(BUILD / f"{name}.csv", delimiter=",", names=True)
    for field in d.dtype.names[:14]:
        if not np.array_equal(d[field], original[field]):
            raise RuntimeError(f"Instrumentation altered {name}/{field}")
    if not np.array_equal(d["first_reject_mask"] > 0, d["rejected"] > 0):
        raise RuntimeError(f"Rejection observations do not match branches: {name}")
    t = d["time_s"]
    v_onset = 2.0 if order == "steer_first" else 0.5
    k_onset = 0.5 if order == "steer_first" else 2.0
    rpm = np.array([d[f"{w}_rpm"] for w in WHEELS])
    angle = np.array([d[f"{w}_angle_deg"] for w in WHEELS[:2]])
    # Analytic MANUAL target, independent of the observation's final sample.
    v, k = 0.8, 2.0
    a = np.degrees(np.arctan2(k * cfg["wheelbase"], 1 + k * np.array([-1, 1]) * cfg["front_track"] / 2))
    C = 60 / (2 * np.pi * cfg["radius"])
    target = C * v * np.array([np.hypot(1-k*cfg["front_track"]/2, k*cfg["wheelbase"]), np.hypot(1+k*cfg["front_track"]/2, k*cfg["wheelbase"]), 1-k*cfg["rear_track"]/2, 1+k*cfg["rear_track"]/2])
    target *= min(1, cfg["rpm_limit"] / np.max(np.abs(target)))
    close = (np.max(np.abs(rpm-target[:, None]), axis=0) <= 1e-3) & (np.max(np.abs(angle-a[:, None]), axis=0) <= 1e-4)
    reached = stable_time(t, close, 2.0)
    speed_reached = stable_time(t, np.abs(d["generated_v"]-v) <= 1e-5, v_onset)
    curve_reached = stable_time(t, np.abs(d["generated_k"]-k) <= 1e-5, k_onset)
    active_end = max(3.0 if mode == "ramp" else 2.0, reached if reached is not None else 5.0)
    active = (t >= .5) & (t <= active_end+1e-9) & (d["attempts"] > 0)
    retries = np.maximum(d["attempts"]-1, 0)
    rate = np.diff(rpm)/DT
    arate = np.diff(angle)/DT
    failed = d["rejected"] > 0
    acceleration = (t >= v_onset) & (t < (5 if order == "steer_first" else 2))
    acceleration_rejections = failed & acceleration
    acceleration_peak = float(np.max(target)) if order == "steer_first" else C*v
    states = [STATES[int(d["transition"][0])]]
    for s in d["transition"][1:]:
        if STATES[int(s)] != states[-1]:
            states.append(STATES[int(s)])
    summary = dict(order=order, input_mode=mode, speed_onset_s=v_onset, curvature_onset_s=k_onset,
                   first_nonzero_speed_delay_s=delay(first_time(t, d["generated_v"] > 1e-6, v_onset), v_onset),
                   speed_90_delay_s=delay(first_time(t, d["generated_v"] >= .9*v, v_onset), v_onset),
                   speed_reached_delay_s=delay(speed_reached, v_onset),
                   curvature_reached_delay_s=delay(curve_reached, k_onset),
                   acceleration_phase_peak_target_rpm=acceleration_peak,
                   ideal_continuous_rpm_slew_duration_s=acceleration_peak/cfg["acceleration_rpm_s"],
                   acceleration_phase_rejected_updates=int(np.sum(acceleration_rejections)),
                   acceleration_phase_max_first_rpm_excess=float(np.max(d["first_rpm_excess"][acceleration_rejections])) if np.any(acceleration_rejections) else None,
                   acceleration_phase_reject_masks=sorted(int(n) for n in np.unique(d["first_reject_mask"][acceleration_rejections])),
                   final_outputs_reached_s=reached, final_v=float(d["generated_v"][-1]), final_k=float(d["generated_k"][-1]),
                   speed_before_second_input=float(d["generated_v"][np.flatnonzero(t < 2)[-1]]),
                   curvature_before_second_input=float(d["generated_k"][np.flatnonzero(t < 2)[-1]]),
                   max_attempts=int(np.max(d["attempts"])), max_evaluations=int(np.max(d["evaluations"])),
                   failed_search_updates=int(np.sum((d["attempts"] > 0) & (d["accepted"] == 0))),
                   active_window_end_s=active_end,
                   active_retry_histogram={str(int(n)): int(np.sum(active & (retries == n))) for n in np.unique(retries[active])},
                   first_rejection_mask_counts={str(int(n)): int(np.sum(d["first_reject_mask"] == n)) for n in np.unique(d["first_reject_mask"][failed])},
                   max_first_rpm_excess=float(np.max(d["first_rpm_excess"][failed])) if np.any(failed) else None,
                   max_first_angle_excess_rad=float(np.max(d["first_angle_excess_rad"][failed])) if np.any(failed) else None,
                   max_abs_rpm=float(np.max(np.abs(rpm))), max_abs_rpm_rate=float(np.max(np.abs(rate))),
                   max_abs_angle_rate_deg_s=float(np.max(np.abs(arate))),
                   transitions=states, all_feasible=bool(np.all(d["feasible"] == 1)), source_output_match=True)
    return dict(raw=d, t=t, rpm=rpm, angle=angle, rate=rate, arate=arate, retries=retries, summary=summary)


def style(ax, ylabel, onset=True):
    ax.set_ylabel(ylabel)
    ax.grid(alpha=.22)
    if onset:
        for x in (.5, 2):
            ax.axvline(x, color="#999", ls=":", lw=1)
    ax.legend(fontsize=8, loc="best", ncol=2)


def plots(results, cfg):
    font = Path("C:/Windows/Fonts/malgun.ttf")
    if font.exists():
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=font).get_name()
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10})
    for mode in ("step", "ramp"):
        fig, axes = plt.subplots(8, 2, figsize=(14, 22), sharex=True, layout="constrained")
        for col, order in enumerate(ORDERS):
            r = results[order, mode]
            t, d = r["t"], r["raw"]
            for row, field, label in ((0, "v", "선속도 [m/s]"), (1, "k", "곡률 [1/m]")):
                ax = axes[row, col]
                ax.plot(t, d[f"request_{field}"], color="#777", ls="--", label="요청")
                ax.plot(t, d[f"generated_{field}"], color=COLORS[0], label="생성 지령")
                style(ax, label)
            for row, field, label in ((2, "angle", "조향각 [°]"), (3, "rpm", "RPM"), (4, "arate", "조향각속도 [°/s]"), (5, "rate", "RPM 변화율 [RPM/s]")):
                ax = axes[row, col]
                for i, y in enumerate(r[field]):
                    ax.plot(t[1:] if row >= 4 else t, y, color=COLORS[i], ls="-" if i < 2 else "--", label=WHEELS[i], lw=1.2)
                if row in (4, 5):
                    limit = cfg["steering_rate"] if row == 4 else cfg["acceleration_rpm_s"]
                    for bound in (-limit, limit):
                        ax.axhline(bound, color="#777", ls="--", lw=.8)
                style(ax, label)
            ax = axes[6, col]
            ax.step(t, r["retries"], where="post", color="#6f429c", label="첫 후보 이후 추가 시도")
            ax.set_ylim(-.1, max(2.2, max(r["retries"])+.2))
            style(ax, "추가 시도 [회]")
            ax = axes[7, col]
            ax.step(t, d["transition"], where="post", color="#444", label="갱신 종료 상태")
            ax.set_yticks(list(STATES), list(STATES.values()), fontsize=8)
            ax.set_ylim(-.2, 4.2)
            style(ax, "상태")
            ax.set_xlabel("시간 [s]")
            ax.set_xlim(0, 4)
            axes[0, col].set_title(ORDERS[order])
        fig.suptitle(f"입력 순서 비교 — {'즉시 입력 (step)' if mode == 'step' else '각 입력 1초 ramp'}\n첫 입력 0.5 s / 두 번째 입력 2.0 s / 최종 v=0.8 m/s, κ=2 /m", fontsize=17)
        fig.supxlabel("현재 C++ 스케줄러 · 10 ms 갱신 · 시작 시 DRIVING · 곡선은 생성 지령이며 실측 응답이 아님", fontsize=10)
        fig.savefig(OUT / f"{mode}_comparison.png", dpi=140)
        fig.savefig(OUT / f"{mode}_comparison.svg")
        plt.close(fig)
    fig, axes = plt.subplots(2, 3, figsize=(16, 9), layout="constrained")
    for row, mode in enumerate(("step", "ramp")):
        for i, order in enumerate(ORDERS):
            r = results[order, mode]
            t, d = r["t"], r["raw"]
            color = COLORS[i]
            axes[row, 0].plot(t, d["generated_v"], color=color, label=ORDERS[order])
            axes[row, 1].plot(t, d["generated_k"], color=color, label=ORDERS[order])
            rel = t-r["summary"]["speed_onset_s"]
            axes[row, 2].plot(rel, d["generated_v"], color=color, label=ORDERS[order])
        reference = results["steer_first", mode]
        rel = reference["t"]-2.0
        axes[row, 2].plot(rel, reference["raw"]["request_v"], color="#777", ls="--", label="공통 선속도 요청")
        for col, ylabel in enumerate(("생성 선속도 [m/s]", "생성 곡률 [1/m]", "생성 선속도 [m/s]")):
            ax = axes[row, col]
            style(ax, ylabel, onset=col != 2)
            ax.set_xlim((-.1, 1.5) if col == 2 else (0, 3.5))
            ax.set_xlabel("스로틀 입력 시작 후 시간 [s]" if col == 2 else "시간 [s]")
        axes[row, 0].set_title("step" if mode == "step" else "1초 ramp")
        axes[row, 1].set_title("조향 진행")
        axes[row, 2].set_title("스로틀 입력 시점을 맞춘 비교")
    fig.suptitle("정지 후 조작 순서에 따른 생성 지령 비교", fontsize=18)
    fig.supxlabel("실제 모터·서보 응답, RC 흔들림, CAN 정수화·전송 간격은 포함하지 않음", fontsize=10)
    fig.savefig(OUT / "overview.png", dpi=150)
    fig.savefig(OUT / "overview.svg")
    plt.close(fig)


def main():
    flags = prepare()
    cfg = load_config()
    results = {(order, mode): run(order, mode, cfg) for mode in ("step", "ramp") for order in ORDERS}
    summaries = [r["summary"] for r in results.values()]
    plots(results, cfg)
    run_date = datetime.now(timezone(timedelta(hours=9))).date().isoformat()
    metadata = dict(run_date_kst=run_date, config=cfg, dt_s=DT, warmup_s=10, observation_s=5,
                    first_input_s=.5, second_input_s=2, ramp_duration_s=1, target_v=.8, target_k=2,
                    initial_command=[0, 0], initial_transition="DRIVING", compiler_flags=flags,
                    compiler=subprocess.run(["g++", "--version"], check=True, capture_output=True, text=True).stdout.splitlines()[0],
                    source_sha256={name: hashlib.sha256((ROOT / "include" / name).read_bytes()).hexdigest() for name in ("vehicle_motion.hpp", "vehicle_kinematics.hpp", "steering_config.hpp")},
                    harness_sha256=hashlib.sha256((HERE / "replay_sequence.cpp").read_bytes()).hexdigest(),
                    instrumented_output_matches_original=True, results=summaries)
    (OUT / "summary.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    scalar = [{k: v for k, v in s.items() if not isinstance(v, (dict, list))} for s in summaries]
    with (OUT / "summary.csv").open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=scalar[0])
        writer.writeheader()
        writer.writerows(scalar)
    events = []
    for (order, mode), r in results.items():
        for row in r["raw"][r["raw"]["rejected"] > 0]:
            events.append(dict(order=order, input_mode=mode, **{k: float(row[k]) for k in r["raw"].dtype.names}))
    with (OUT / "first_rejections.csv").open("w", encoding="utf-8-sig", newline="") as f:
        fields = ["order", "input_mode", *next(iter(results.values()))["raw"].dtype.names]
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(events)
    lines = ["# 정지 후 입력 순서 비교", "", f"실행일(KST): {run_date}", "",
             "## 조건", "",
             "- 현재 C++ MotionController를 PC에서 직접 호출한다. 펌웨어 소스는 수정하지 않는다.",
             "- 두 순서: 정지 → 곡률 입력 → 선속도 입력 / 정지 → 선속도 입력 → 곡률 입력.",
             "- 최종 요청 v=0.8m/s, κ=2/m. 첫 입력 0.5초, 두 번째 입력 2.0초. 두 입력 시작 간격은 1.5초다.",
             "- step은 입력 시작 시 즉시 목표값으로 변경한다. ramp는 각 입력을 1초에 걸쳐 선형 변경한다.",
             "- 10ms 고정 갱신. 0속도·0곡률을 10초 유지하여 DRIVING에 도달한 뒤 5초 관측한다. 시동·STOP 모드 해제 실험이 아니다.",
             f"- RPM 상한 {cfg['rpm_limit']:g}, 증가·감소율 상한 {cfg['acceleration_rpm_s']:g}/{cfg['deceleration_rpm_s']:g}RPM/s, 조향각속도 상한 {cfg['steering_rate']:g}°/s.",
             "- RC 잡음, 실제 서보·모터 지연, 정지 마찰·부하·슬립, CAN 전송·정수화, PWM 출력은 모델에 포함하지 않는다.",
             "- 원본과 계측 복사본의 기본 14개 열을 네 실행에서 모두 일치 확인했다. 분기·스케줄링 산술은 바꾸지 않았다.",
             "- PC SSE float 결과다. MCU 실행시간이나 실제 장치 응답을 측정한 결과가 아니다.", "",
             "## 결과", "",
             "| 입력 | 순서 | 최초 비영 속도 지령 지연 [s] | 0.72m/s 최초 도달 지연 [s] | 0.8m/s 유지 도달 지연 [s] | 최종 전체 지령 도달 [s] | 최대 추가 시도 | 탐색 실패 |",
             "|---|---|---:|---:|---:|---:|---:|---:|"]
    for s in summaries:
        lines.append(f"| {s['input_mode']} | {ORDERS[s['order']]} | {number(s['first_nonzero_speed_delay_s'])} | {number(s['speed_90_delay_s'])} | {number(s['speed_reached_delay_s'])} | {number(s['final_outputs_reached_s'])} | {s['max_attempts']-1} | {s['failed_search_updates']} |")
    lines += ["", "속도 지연은 선속도 입력 시작 기준이다.",
              "최초 비영 지령 기준은 생성 v>1e-6m/s이며 실제 바퀴가 움직인 시각이 아니다.",
              "속도 유지 도달은 요청 0.8m/s와 오차 1e-5m/s 이내에 이후 계속 머무는 첫 샘플이다.",
              "최종 전체 지령 도달은 두 번째 입력 이후 해석적으로 계산한 최종 운동학 목표와 모든 RPM 오차 1e-3RPM, 조향각 오차 1e-4° 이내에 이후 계속 머무는 첫 시각이다.", "",
              "## 가속 구간의 분리 집계", "",
              "곡률 우선은 2~5초, 선속도 우선은 0.5초 이상 2초 미만을 가속 구간으로 집계한다. 이 구간에는 해당 곡률을 일정하게 유지한다.", "",
              "| 입력 | 순서 | 해당 가속 목표의 최대 휠 RPM | RPM 증가율만 적용한 연속시간 최소 소요 [s] | 첫 후보 거부 갱신 | 거부 mask | 첫 후보 RPM 변화량 최대 초과 [RPM] |",
              "|---|---|---:|---:|---:|---|---:|"]
    for s in summaries:
        excess = s['acceleration_phase_max_first_rpm_excess']
        lines.append(f"| {s['input_mode']} | {ORDERS[s['order']]} | {s['acceleration_phase_peak_target_rpm']:.6f} | {s['ideal_continuous_rpm_slew_duration_s']:.6f} | {s['acceleration_phase_rejected_updates']} | {s['acceleration_phase_reject_masks']} | {'해당 없음' if excess is None else f'{excess:.9g}'} |")
    lines += ["", "연속시간 최소 소요는 최대 목표 RPM / 설정 증가율이다. 입력 ramp, 재시도, 10ms 샘플 시각, 실제 장치 지연을 반영한 도달 시간은 아니다.",
              "곡률 고정 시 각 휠 RPM은 선속도에 비례하지만, 같은 차체 속도라도 곡률에 따라 최대 휠 RPM이 달라진다.",
              "따라서 두 출발의 시간 차이 전체를 재시도 탓으로 해석할 수 없다. 첫 후보 거부가 있으면 다음 후보 진행률은 절반이 된다.", "",
              "## 상태 및 재시도", "",
              "재시도 히스토그램은 0.5초부터 최종 입력·출력 전환이 모두 끝날 때까지 후보 탐색을 수행한 갱신을 센다. 중간 유지 구간도 포함한다.",
              "최대값, 탐색 실패 및 첫 후보 거부 집계는 전체 5초 관측 기준이다.", ""]
    for s in summaries:
        lines.append(f"- {s['input_mode']} / {ORDERS[s['order']]}: 상태 `{' → '.join(s['transitions'])}`, 재시도 분포 `{s['active_retry_histogram']}`, 최대 evaluate {s['max_evaluations']}회, 첫 후보 거부 mask 분포 `{s['first_rejection_mask_counts']}`.")
    lines += ["", "`first_reject_mask`: bit0=실행 불가능, bit1=조향 변화량·유한값, bit2=RPM 크기·유한값, bit3=RPM 변화량 제한.",
              "`first_angle_excess_rad`, `first_rpm_excess`, `first_peak_excess`는 첫 후보에서 실제 float 비교 피연산자들의 차이를 double로 기록한 최대값이다. 양수는 초과, 음수는 여유다. 후보가 없으면 -1이다.",
              "`initial_fraction`, `accepted_fraction`은 첫 후보·채택 후보 진행률이며 후보가 없거나 미채택이면 -1이다.",
              "첫 후보의 작은 초과와 큰 초과를 구분할 수 있도록 `first_rejections.csv`에 거부 갱신 전체를 기록했다. 작은 양수만으로 실차 지연의 원인을 확정하지 않는다.", "",
              "## 파일", "",
              "- [overview.png](overview.png): 시간순 선속도·곡률 및 스로틀 입력 시작 시점을 맞춘 속도 비교.",
              "- [step_comparison.png](step_comparison.png), [ramp_comparison.png](ramp_comparison.png): 입력·생성 지령·조향각·네 휠 RPM·변화율·재시도·상태. 그림은 0~4초, CSV는 0~5초.",
              "- 각 그림의 SVG, 네 실행의 CSV, summary.csv/json, first_rejections.csv.",
              "- build/: 원본·계측 헤더 실행 비교 자료와 실행 파일. Git 제외 대상.", "",
              "## 재실행", "", "```powershell",
              "uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/compare_sequence.py", "```", ""]
    (OUT / "README.ko.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(summaries, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
