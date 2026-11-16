"""48 seeded RC pulse-noise replays; firmware files and hardware are untouched.

Run: uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/compare_rc_noise.py
"""
from pathlib import Path
from datetime import datetime, timedelta, timezone
import csv
import hashlib
import json
import re
import subprocess

import numpy as np
from simulate import ROOT, COLORS, WHEELS, load_config, plt, font_manager
from compare_sequence import prepare, first_time, delay, number, ORDERS, STATES

HERE = Path(__file__).resolve().parent
OUT = HERE / "noise_output"
BUILD = OUT / "build"
LEVELS = {"weak": ("약", 2., 1.), "medium": ("중", 5., 3.), "strong": ("강", 10., 5.)}
SPEEDS = (.3, .8)
CURVATURES = (1., 2.)
DT = .01
FRAME_DT = .02
TAU = .15
BASE_SEED = 20261116


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def rc_adapter():
    """Copy the firmware's pure arithmetic into a host-only adapter, verbatim."""
    BUILD.mkdir(parents=True, exist_ok=True)
    header = (ROOT / "include/rc_io.hpp").read_text(encoding="utf-8")
    source = (ROOT / "src/rc_io.cpp").read_text(encoding="utf-8")
    control = (ROOT / "src/control.cpp").read_text(encoding="utf-8")
    constants = header.split("namespace rc_io {\n", 1)[1].split("struct PwmInput", 1)[0]
    count = re.search(r"constexpr int FILTER_SAMPLES = \d+;", source).group()
    start = source.index("class PulseFilter {")
    filter_class = source[start:source.index("\n};", start)+3]
    start = source.index("int32_t offset_with_deadband(")
    deadband = source[start:source.index("\n}\n", start)+2]
    start = control.index("        const float v_velocity =")
    end = control.index("        update_vehicle_output({v_velocity, curvature}, now);", start)
    mapping = control[start:end]
    adapter = ("// Extracted from rc_io.hpp, rc_io.cpp and control.cpp; generated for host replay.\n"
               "namespace rc_io {\n"+constants+count+"\n"+filter_class+"\n"+deadband+"\n}\n"
               "inline vehicle::RcCommand replay_rc_command(const vehicle::MotionController &motion, unsigned int v_pwm, unsigned int w_pwm) {\n"
               +mapping+"    return vehicle::RcCommand(v_velocity, curvature);\n}\n")
    (BUILD / "rc_replay_adapter.hpp").write_text(adapter, encoding="utf-8")
    def val(name):
        return float(re.search(rf"\b{name}\s*=\s*([0-9.]+)", constants+count).group(1))
    rc = {n: val(n) for n in ("PWM_CENTER_US", "PWM_DEADBAND_US", "PWM_MIN_US", "PWM_MAX_US", "THROTTLE_US_PER_MPS", "FILTER_SAMPLES")}
    rc["PWM_ACTIVE_SPAN_US"] = rc["PWM_MAX_US"]-rc["PWM_CENTER_US"]-rc["PWM_DEADBAND_US"]
    return rc


def generate_input(key, order, mode, vi, ki, level, cfg, rc):
    ms = np.arange(0, 15001, 20, dtype=int)
    t = ms/1000-10
    v_onset, k_onset = ((2., .5) if order == "steer_first" else (.5, 2.))
    def progress(onset):
        return np.clip(t-onset, 0, 1) if mode == "ramp" else (t >= onset).astype(float)
    v, k = SPEEDS[vi]*progress(v_onset), CURVATURES[ki]*progress(k_onset)
    angle = np.arctan2(k*cfg["wheelbase"], 1-k*cfg["front_track"]*.5)
    center, dead = rc["PWM_CENTER_US"], rc["PWM_DEADBAND_US"]
    ideal = np.column_stack((np.where(v == 0, center, center+dead+v*rc["THROTTLE_US_PER_MPS"]),
                             np.where(k == 0, center, center+dead+angle/np.radians(cfg["rc_maximum_inner_angle"])*rc["PWM_ACTIVE_SPAN_US"])))
    # Common random numbers across both orders AND the three levels. Each target
    # pair/input mode has its own seed; the two channels have independent draws.
    seed = BASE_SEED+100*vi+10*ki+(mode == "ramp")
    rng = np.random.Generator(np.random.PCG64(seed))
    innovations = rng.standard_normal((len(ms), 2))
    unit_hand = np.empty_like(innovations)
    unit_hand[0] = innovations[0]
    rho = np.exp(-FRAME_DT/TAU)
    for i in range(1, len(ms)):
        unit_hand[i] = rho*unit_hand[i-1]+np.sqrt(1-rho*rho)*innovations[i]
    _, sigma, bound = LEVELS[level]
    hand = sigma*unit_hand
    measurement = bound*rng.uniform(-1, 1, (len(ms), 2))
    transmitted = np.clip(ideal+hand, rc["PWM_MIN_US"], rc["PWM_MAX_US"])
    measured = np.floor(transmitted+measurement+.5).astype(int)
    rows = np.column_stack((ms, v, k, ideal, hand, measurement, measured))
    path = OUT / "inputs" / f"{key}.csv"
    np.savetxt(path, rows, delimiter=",", comments="", fmt=["%d"]+["%.17g"]*8+["%d"]*2,
               header="timestamp_ms,nominal_v,nominal_k,ideal_v_pwm_us,ideal_k_pwm_us,hand_v_us,hand_k_us,measurement_v_us,measurement_k_us,measured_v_pwm_us,measured_k_pwm_us")
    return path, dict(seed=int(seed), transmitted_clipped_frames=int(np.sum(np.any(transmitted != ideal+hand, axis=1))),
                     input_sha256=digest(path))


def longest_run(condition):
    padded = np.r_[False, condition, False].astype(int)
    edges = np.diff(padded)
    lengths = np.flatnonzero(edges == -1)-np.flatnonzero(edges == 1)
    return int(max(lengths, default=0))


def replay(key, order, mode, vi, ki, level, input_path, generation):
    for variant in ("original", "instrumented"):
        path = (BUILD if variant == "original" else OUT) / f"{key}.csv"
        subprocess.run([str(BUILD / f"{variant}.exe"), str(input_path), str(path)], check=True)
    d = np.genfromtxt(OUT / f"{key}.csv", delimiter=",", names=True)
    original = np.genfromtxt(BUILD / f"{key}.csv", delimiter=",", names=True)
    # Inputs, filtered requests, outputs, flags and dropped-frame counters.
    for field in d.dtype.names[:28]:
        if not np.array_equal(d[field], original[field]):
            raise RuntimeError(f"Instrumentation changed {key}/{field}")
    if not np.array_equal(d["first_reject_mask"] > 0, d["rejected"] > 0):
        raise RuntimeError(f"First rejection mismatch: {key}")
    t = d["time_s"]
    v_onset, k_onset = ((2., .5) if order == "steer_first" else (.5, 2.))
    rpm = np.array([d[f"{w}_rpm"] for w in WHEELS])
    angles = np.array([d[f"{w}_angle_deg"] for w in WHEELS[:2]])
    retry = np.maximum(d["attempts"]-1, 0)
    active = (t >= .5) & (d["attempts"] > 0)
    tail = t >= 4
    stalled = (t >= v_onset) & (d["request_v"] >= .05*SPEEDS[vi]) & (np.abs(d["generated_v"]) <= 1e-6)
    # Not "settled": merely the first 200 ms window continuously within ±5%.
    in_band = (t >= v_onset) & (np.abs(d["generated_v"]-SPEEDS[vi]) <= .05*SPEEDS[vi])
    windows = np.convolve(in_band.astype(int), np.ones(21, dtype=int), mode="valid") == 21
    window_start = first_time(t[:len(windows)], windows, v_onset)
    states = [STATES[int(d["transition"][0])]]
    for value in d["transition"][1:]:
        if STATES[int(value)] != states[-1]:
            states.append(STATES[int(value)])
    rejected = d["rejected"] > 0
    summary = dict(case=key, order=order, mode=mode, noise=level, target_v=SPEEDS[vi], target_k=CURVATURES[ki], **generation,
                   first_request_delay_s=delay(first_time(t, d["request_v"] > 1e-6, v_onset), v_onset),
                   first_generated_delay_s=delay(first_time(t, d["generated_v"] > 1e-6, v_onset), v_onset),
                   speed_90_first_delay_s=delay(first_time(t, d["generated_v"] >= .9*SPEEDS[vi], v_onset), v_onset),
                   first_200ms_in_5pct_start_delay_s=delay(window_start, v_onset),
                   tail_mean_v=float(np.mean(d["generated_v"][tail])), tail_std_v=float(np.std(d["generated_v"][tail])),
                   tail_mean_k=float(np.mean(d["generated_k"][tail])), tail_std_k=float(np.std(d["generated_k"][tail])),
                   tail_tracking_v_rmse=float(np.sqrt(np.mean((d["generated_v"][tail]-d["request_v"][tail])**2))),
                   tail_tracking_k_rmse=float(np.sqrt(np.mean((d["generated_k"][tail]-d["request_k"][tail])**2))),
                   max_abs_rpm=float(np.max(np.abs(rpm))), max_abs_rpm_rate=float(np.max(np.abs(np.diff(rpm)/DT))),
                   max_abs_steering_rate_deg_s=float(np.max(np.abs(np.diff(angles)/DT))),
                   max_attempts=int(np.max(d["attempts"])), max_evaluations=int(np.max(d["evaluations"])),
                   retried_updates=int(np.sum(active & (retry > 0))),
                   retry_histogram={str(int(n)): int(np.sum(active & (retry == n))) for n in np.unique(retry[active])},
                   failed_search_updates=int(np.sum((d["attempts"] > 0) & (d["accepted"] == 0))),
                   max_zero_output_positive_request_run_s=longest_run(stalled)*DT,
                   first_rejection_mask_counts={str(int(n)): int(np.sum(d["first_reject_mask"] == n)) for n in np.unique(d["first_reject_mask"][rejected])},
                   rpm_limited_updates=int(np.sum(d["limited"] > 0)), all_feasible=bool(np.all(d["feasible"] == 1)),
                   dropped_v_frames=int(d["dropped_v_frames"][-1]), dropped_k_frames=int(d["dropped_k_frames"][-1]),
                   transitions=states, original_output_match=True)
    return dict(raw=d, t=t, rpm=rpm, angle=angles, retry=retry, summary=summary)


def setup_font():
    font = Path("C:/Windows/Fonts/malgun.ttf")
    if font.exists():
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=font).get_name()
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 9})


def pretty_axes(ax, label, aligned=False):
    ax.set_ylabel(label)
    ax.grid(alpha=.2)
    ax.set_xlim((-.1, 2.8) if aligned else (0, 5))
    ax.set_xlabel("스로틀 시작 후 [s]" if aligned else "시간 [s]")
    if not aligned:
        for x in (.5, 2):
            ax.axvline(x, color="#999", ls=":", lw=.8)


def key_for(level, mode, vi, ki, order):
    return f"{level}_{mode}_v{SPEEDS[vi]:g}_k{CURVATURES[ki]:g}_{order}"


def make_plots(results, cfg):
    setup_font()
    for level in LEVELS:
        for mode in ("step", "ramp"):
            fig, axes = plt.subplots(4, 4, figsize=(21, 14), layout="constrained")
            for row, (vi, ki) in enumerate(( (v, k) for v in range(2) for k in range(2) )):
                for oi, order in enumerate(ORDERS):
                    r = results[key_for(level, mode, vi, ki, order)]
                    t, d = r["t"], r["raw"]
                    rel = t-(2 if order == "steer_first" else .5)
                    label = ORDERS[order]
                    axes[row, 0].plot(t, d["generated_v"], color=COLORS[oi], label=label)
                    axes[row, 1].plot(t, d["generated_k"], color=COLORS[oi], label=label)
                    axes[row, 2].plot(rel, d["generated_v"], color=COLORS[oi], label=label)
                    axes[row, 3].step(rel, r["retry"], where="post", color=COLORS[oi], alpha=.75, label=label)
                for col, ylabel in enumerate(("생성 v [m/s]", "생성 κ [1/m]", "생성 v [m/s]", "추가 후보 시도 [회]")):
                    pretty_axes(axes[row, col], ylabel, aligned=col >= 2)
                axes[row, 0].set_title(f"목표 v={SPEEDS[vi]:g} m/s, κ={CURVATURES[ki]:g} /m")
                axes[row, 2].axhline(SPEEDS[vi], color="#999", ls="--", lw=.8)
                axes[row, 3].set_ylim(-.2, max(2.2, *(max(results[key_for(level, mode, vi, ki, o)]["retry"])+.2 for o in ORDERS)))
            axes[0, 0].legend(fontsize=8)
            axes[0, 1].set_title("곡률 생성 지령")
            axes[0, 2].set_title("스로틀 입력 시점 정렬")
            axes[0, 3].set_title("스로틀 시점 정렬 / 재시도")
            fig.suptitle(f"{LEVELS[level][0]} 노이즈 / {mode} — 목표값 4조합 × 두 순서\n손 떨림 σ={LEVELS[level][1]:g}µs, 측정 오차 ±{LEVELS[level][2]:g}µs", fontsize=17)
            fig.supxlabel("합성 노이즈 · RC 20ms / 10샘플 이동평균 / 데드밴드 ±50µs · MCU 재생 10ms · 실측 운동 아님", fontsize=10)
            fig.savefig(OUT / f"overview_{level}_{mode}.png", dpi=125)
            plt.close(fig)
            for vi in range(2):
                for ki in range(2):
                    fig, axes = plt.subplots(8, 2, figsize=(14, 21), sharex=True, layout="constrained")
                    for col, order in enumerate(ORDERS):
                        r = results[key_for(level, mode, vi, ki, order)]
                        t, d = r["t"], r["raw"]
                        for row, field, label in ((0, "v", "선속도 [m/s]"), (1, "k", "곡률 [1/m]")):
                            ax = axes[row, col]
                            ax.plot(t, d[f"nominal_{field}"], color="#999", ls="--", lw=1, label="명목 입력")
                            ax.plot(t, d[f"request_{field}"], color="#D55E00", lw=1, alpha=.7, label="필터·변환 후 요청")
                            ax.plot(t, d[f"generated_{field}"], color=COLORS[0], lw=1.3, label="생성 지령")
                            pretty_axes(ax, label)
                            ax.legend(fontsize=7, ncol=3)
                        for row, values, label in ((2, r["angle"], "조향각 [°]"), (3, r["rpm"], "휠 RPM"), (4, np.diff(r["angle"])/DT, "조향각속도 [°/s]"), (5, np.diff(r["rpm"])/DT, "RPM 변화율 [RPM/s]")):
                            ax = axes[row, col]
                            for i, y in enumerate(values):
                                ax.plot(t[1:] if row >= 4 else t, y, color=COLORS[i], ls="-" if i < 2 else "--", label=WHEELS[i], lw=.9)
                            if row in (4, 5):
                                limit = cfg["steering_rate"] if row == 4 else cfg["acceleration_rpm_s"]
                                for bound in (-limit, limit):
                                    ax.axhline(bound, color="#999", ls="--", lw=.7)
                            pretty_axes(ax, label)
                            ax.legend(fontsize=7, ncol=4)
                        axes[6, col].step(t, r["retry"], where="post", color="#774499")
                        pretty_axes(axes[6, col], "추가 시도 [회]")
                        axes[6, col].set_ylim(-.1, max(2.2, max(r["retry"])+.2))
                        axes[7, col].step(t, d["transition"], where="post", color="#444")
                        axes[7, col].set_yticks(list(STATES), list(STATES.values()), fontsize=7)
                        axes[7, col].set_ylim(-.2, 4.2)
                        pretty_axes(axes[7, col], "갱신 종료 상태")
                        axes[0, col].set_title(ORDERS[order])
                    fig.suptitle(f"{LEVELS[level][0]} 노이즈 / {mode} / v={SPEEDS[vi]:g} m/s, κ={CURVATURES[ki]:g} /m\n첫 입력 0.5초, 두 번째 입력 2초", fontsize=16)
                    fig.supxlabel("현재 C++ 스케줄러의 생성 지령 · 두 순서의 잡음 난수열 동일 · 실제 모터/서보 응답 모델 없음", fontsize=10)
                    fig.savefig(OUT / f"detail_{level}_{mode}_v{SPEEDS[vi]:g}_k{CURVATURES[ki]:g}.png", dpi=110)
                    plt.close(fig)
            print(f"Plotted {level}/{mode}", flush=True)
    noise_examples(results)


def noise_examples(results):
    # Show the actual pulse-noise stages for a shared representative target.
    fig, axes = plt.subplots(3, 2, figsize=(15, 10), layout="constrained")
    limit = 5.
    for level in LEVELS:
        r = results[key_for(level, "step", 1, 1, "steer_first")]
        d = r["raw"]
        visible = r["t"] >= 3.2
        for field in ("v", "k"):
            for stage in ("measured", "filtered"):
                limit = max(limit, float(np.max(np.abs(d[f"{stage}_{field}_pwm_us"][visible]-d[f"ideal_{field}_pwm_us"][visible]))))
    limit = np.ceil(limit*1.1/5)*5
    for row, level in enumerate(LEVELS):
        r = results[key_for(level, "step", 1, 1, "steer_first")]
        d, t = r["raw"], r["t"]
        for col, field in enumerate(("v", "k")):
            ax = axes[row, col]
            ideal = d[f"ideal_{field}_pwm_us"]
            ax.plot(t, d[f"measured_{field}_pwm_us"]-ideal, color="#999", lw=.8, label="측정 PWM - 명목 PWM")
            ax.plot(t, d[f"filtered_{field}_pwm_us"]-ideal, color=COLORS[0], lw=1.2, label="이동평균 PWM - 명목 PWM")
            ax.set_xlim(3.2, 5)
            ax.set_ylim(-limit, limit)
            ax.set(title=f"{LEVELS[level][0]} / {'스로틀' if field == 'v' else '조향'}", xlabel="시간 [s]", ylabel="PWM 차이 [µs]")
            ax.grid(alpha=.2)
            ax.legend(fontsize=8)
    fig.suptitle("정상 입력 유지 구간의 노이즈 예시 — v=0.8m/s, κ=2/m, step, 조향 우선", fontsize=15)
    fig.savefig(OUT / "noise_examples.png", dpi=140)
    plt.close(fig)


def save_reports(results, cfg, rc, flags):
    summaries = [r["summary"] for r in results.values()]
    sources = ["include/vehicle_motion.hpp", "include/vehicle_kinematics.hpp", "include/steering_config.hpp", "include/rc_io.hpp", "src/rc_io.cpp", "src/control.cpp"]
    metadata = dict(run_date_kst=datetime.now(timezone(timedelta(hours=9))).date().isoformat(), config=cfg, rc_config=rc,
                    levels={k: dict(hand_stationary_sigma_us=v[1], measurement_uniform_bound_us=v[2]) for k,v in LEVELS.items()},
                    dt_s=DT, frame_dt_s=FRAME_DT, hand_correlation_time_s=TAU, rng="numpy.PCG64", base_seed=BASE_SEED,
                    seeds_shared_across_orders_and_levels=True, realizations_per_condition=1, warmup_s=10, observation_s=5,
                    first_input_s=.5, second_input_s=2., ramp_duration_s=1., compiler_flags=flags,
                    compiler=subprocess.run(["g++", "--version"], check=True, capture_output=True, text=True).stdout.splitlines()[0],
                    source_sha256={n:digest(ROOT/n) for n in sources},
                    replay_sha256={n:digest(HERE/n) for n in ("compare_rc_noise.py", "replay_rc_noise.cpp", "compare_sequence.py")},
                    adapter_sha256=digest(BUILD/"rc_replay_adapter.hpp"),
                    original_outputs_match=True, results=summaries)
    (OUT/"summary.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    flat = [{k:v for k,v in s.items() if not isinstance(v,(list,dict))} for s in summaries]
    with (OUT/"summary.csv").open("w", encoding="utf-8-sig", newline="") as f:
        writer=csv.DictWriter(f, fieldnames=flat[0]); writer.writeheader(); writer.writerows(flat)
    lines=["# RC 노이즈 입력 48개 시뮬레이션", "", f"실행일(KST): {metadata['run_date_kst']}", "",
           "## 조건과 범위", "",
           "3개 노이즈 × step/ramp × 선속도 0.3/0.8m/s × 곡률 1/2m⁻¹ × 두 입력 순서 = 48개다.",
           "두 순서는 정지 → 곡률 → 선속도 및 정지 → 선속도 → 곡률이다. 첫 입력 0.5초, 두 번째 입력 2초이며 ramp는 각 1초다.",
           "10초 동안 중립 입력과 노이즈를 공급하여 DRIVING에서 관측을 시작한다. 그 뒤 5초를 10ms 간격으로 기록한다.", "",
           "| 노이즈 | 손 떨림 정상상태 표준편차 | PWM 측정 오차 균등분포 |", "|---|---:|---:|"]
    for name, sigma, bound in LEVELS.values():
        lines.append(f"| {name} | {sigma:g}µs | −{bound:g}~+{bound:g}µs |")
    lines += ["", "수치는 사용자와 정한 합성 모델의 가정이며 RC 장비 실측값이나 규격이 아니다.",
              "- RC 프레임 20ms, 두 채널 동시 갱신·프레임 사이 값 유지로 가정한다. 실제 채널 위상·지터는 재현하지 않는다.",
              "- 손 떨림: 독립 채널의 영평균 Gaussian AR(1), 상관시간 0.15초, ρ=exp(−0.02/0.15). 초기값도 정상상태 분포에서 생성한다.",
              "- 측정 오차: 프레임·채널별 독립 균등 난수. 손 떨림과 독립이다. 합산 후 1µs 정수로 반올림한다. 이 해상도 역시 실측 보장이 아니다.",
              "- 명목 v·κ 입력을 현재 RC 매핑의 역함수로 PWM으로 만든다. 0은 1500µs, 양의 입력은 1550µs를 넘는 활성 구간이다. ramp는 PWM 자체가 아닌 v·κ 기준이다.",
              "- 손 떨림 합산 후 송신 펄스를 1000~2000µs로 제한하고 측정 오차를 더한다. 수신 측 2000µs 초과 표본은 기존 코드의 스칼라 조건처럼 필터에 넣지 않는다. ISR 에지 상태는 모델링하지 않는다.",
              "- 실제 소스의 PulseFilter(10샘플 정수 이동평균), offset_with_deadband(±50µs), MANUAL 명령 변환 부분을 build 어댑터로 추출해 실행한다.",
              "- 추출한 입력 계산 → 원본 MotionController 순서다. Arduino ISR, CAN 출력·정수화, PWM 출력, 부하·마찰·슬립·실측 조향은 포함하지 않는다.",
              "- seed=20261116+100×선속도 인덱스+10×곡률 인덱스+ramp 여부. 동일 목표·입력 방식에서는 두 순서와 세 노이즈 수준에 공통 표준 난수열을 사용한다.",
              "- 각 조건은 난수열 하나만 실행한 결과다. 다중 seed 분포·실차 발생확률을 추정하지 않는다.",
              "- 원본 실행과 계측 복사본의 첫 28개 CSV 열이 48개 모두 일치하는지 확인한다. 펌웨어 소스와 스케줄링 판단을 변경하지 않는다.",
              "- 이전 무노이즈 실험은 v·κ를 직접 입력했다. 이번에는 RC 프레임·이동평균·정수화도 추가되었으므로 이전 결과와의 차이 전체를 노이즈 영향으로 해석할 수 없다.", "",
              "## 지표", "",
              "속도 시간 지표는 명목 스로틀 입력 시작 기준이다. 최초 요청·출력은 v>1e-6m/s인 첫 샘플이다.",
              "90% 시간은 명목 목표의 90%를 처음 넘은 시각이며, 이후 유지·최종 정착을 뜻하지 않는다.",
              "±5% 구간 지표는 명목 속도 목표의 ±5% 안에 200ms(21개 샘플) 연속 머무는 첫 구간의 시작 시각이다. 이 역시 영구 정착 판정은 아니다.",
              "tail 통계는 4~5초 표본 기준이다. tracking RMSE는 명목 입력이 아닌 필터·변환 후 요청과 생성 지령의 차이다.",
              "재시도 분포는 0.5~5초의 후보 탐색 갱신을 집계한다. 탐색 실패는 시도했으나 채택하지 못한 갱신이다.",
              "양의 요청/0출력 최장 구간은 스로틀 시작 이후 요청≥명목 목표의 5%, |생성 v|≤1e-6인 연속 표본 수×10ms이다. 실차 정지 시간은 아니다.",
              "상태는 갱신 종료값으로, 한 갱신 안에서 거친 모든 중간 상태를 추적한 것은 아니다.", "",
              "## 결과 집계", "",
              f"- 총 {len(summaries)}개 중 원본 출력과 계측본 일치 {sum(s['original_output_match'] for s in summaries)}개, 모든 명령 feasible {sum(s['all_feasible'] for s in summaries)}개.",
              f"- 관측된 상태 순서: {sorted(set(tuple(s['transitions']) for s in summaries))}.",
              f"- 최대 후보 수 {max(s['max_attempts'] for s in summaries)}, 최대 evaluate {max(s['max_evaluations'] for s in summaries)}, 탐색 실패 합계 {sum(s['failed_search_updates'] for s in summaries)}회.",
              f"- 양의 요청/0출력 정의에 해당하는 최장 표본 구간 {max(s['max_zero_output_positive_request_run_s'] for s in summaries):g}초.",
              f"- 5초 관측 안에 목표 속도 ±5%를 200ms 유지한 구간이 없는 조건 {sum(s['first_200ms_in_5pct_start_delay_s'] is None for s in summaries)}개. 최초 90% 도달과 다른 지표다.", "",
              "| 노이즈 | 입력 | 8개 조건의 90% 최초 도달 범위 [s] | 8개 조건 재시도 갱신 합계 |", "|---|---|---:|---:|"]
    for level in LEVELS:
        for mode in ("step", "ramp"):
            group=[s for s in summaries if s['noise']==level and s['mode']==mode]
            reached=[s['speed_90_first_delay_s'] for s in group if s['speed_90_first_delay_s'] is not None]
            span=f"{min(reached):.2f}~{max(reached):.2f}" if reached else "미도달"
            lines.append(f"| {LEVELS[level][0]} | {mode} | {span} | {sum(s['retried_updates'] for s in group)} |")
    lines += ["", "위 범위는 목표값·순서가 서로 다른 8개 조건의 최솟값과 최댓값이다. 확률분포·신뢰구간이 아니다.", "",
              "## 전체 결과", "",
              "| 노이즈 | 입력 | v | κ | 순서 | 최초 생성 [s] | 90% 최초 [s] | ±5% 200ms 시작 [s] | 최대 추가 시도 | 탐색 실패 |",
              "|---|---|---:|---:|---|---:|---:|---:|---:|---:|"]
    for s in summaries:
        lines.append(f"| {LEVELS[s['noise']][0]} | {s['mode']} | {s['target_v']:g} | {s['target_k']:g} | {'조향 우선' if s['order']=='steer_first' else '속도 우선'} | {number(s['first_generated_delay_s'])} | {number(s['speed_90_first_delay_s'])} | {number(s['first_200ms_in_5pct_start_delay_s'])} | {s['max_attempts']-1} | {s['failed_search_updates']} |")
    lines += ["", "## 그래프 목록", "",
              "| 노이즈 | Step 전체 비교 | Ramp 전체 비교 |", "|---|---|---|"]
    for level in LEVELS:
        lines.append(f"| {LEVELS[level][0]} | [Step](overview_{level}_step.png) | [Ramp](overview_{level}_ramp.png) |")
    lines += ["", "[PWM 노이즈와 필터 출력 예시](noise_examples.png)", "",
              "| 노이즈 | v | κ | Step 상세 | Ramp 상세 |", "|---|---:|---:|---|---|"]
    for level in LEVELS:
        for v in SPEEDS:
            for k in CURVATURES:
                lines.append(f"| {LEVELS[level][0]} | {v:g} | {k:g} | [Step](detail_{level}_step_v{v:g}_k{k:g}.png) | [Ramp](detail_{level}_ramp_v{v:g}_k{k:g}.png) |")
    lines += ["", "## 파일", "",
              "- overview_<노이즈>_<입력>.png: 여섯 장. 각 장은 목표 4조합×두 순서의 선속도·곡률·정렬 속도·재시도 비교다.",
              "- detail_<노이즈>_<입력>_v<값>_k<값>.png: 24장. 두 순서의 명목/요청/생성 v·κ, 두 조향각, 네 RPM, 차분 변화율, 재시도·상태.",
              "- noise_examples.png: v=0.8, κ=2, step, 조향 우선에서 입력 유지 구간 PWM 노이즈와 필터 결과.",
              "- 각 실행 CSV 48개, inputs/ 입력 프레임 CSV 48개(10초 준비 구간 포함), summary.csv/json.",
              "- first_reject_mask와 초과량, 진행률 필드는 [앞선 순서 실험](../sequence_output/README.ko.md)의 정의와 같다.",
              "- build/는 추출 어댑터·계측 헤더·컴파일 결과·원본 비교 CSV이며 Git 제외 대상이다.", "",
              "## 재실행", "", "```powershell",
              "uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/compare_rc_noise.py", "```", ""]
    (OUT/"README.ko.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(dict(runs=len(summaries), max_attempts=max(s['max_attempts'] for s in summaries),
                          failed_search_updates=sum(s['failed_search_updates'] for s in summaries),
                          transitions=sorted(set(tuple(s['transitions']) for s in summaries)),
                          max_positive_request_zero_output_s=max(s['max_zero_output_positive_request_run_s'] for s in summaries)), ensure_ascii=False), flush=True)


def main():
    (OUT/"inputs").mkdir(parents=True, exist_ok=True)
    cfg = load_config()
    rc = rc_adapter()
    flags = prepare(BUILD, HERE/"replay_rc_noise.cpp")
    results = {}
    for level in LEVELS:
        for mode in ("step", "ramp"):
            for vi in range(2):
                for ki in range(2):
                    for order in ORDERS:
                        key = key_for(level, mode, vi, ki, order)
                        path, generation = generate_input(key, order, mode, vi, ki, level, cfg, rc)
                        results[key] = replay(key, order, mode, vi, ki, level, path, generation)
            print(f"Replayed {level}/{mode}", flush=True)
    save_reports(results, cfg, rc, flags)
    make_plots(results, cfg)


if __name__ == "__main__":
    main()
