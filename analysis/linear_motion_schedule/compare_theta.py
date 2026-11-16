"""Replay the current v/theta firmware for nine step/ramp scenarios (host only)."""
from pathlib import Path
from dataclasses import asdict
from datetime import datetime, timezone, timedelta
import csv
import hashlib
import json
import re
import subprocess

import numpy as np
from simulate import CASES, ROOT, COLORS, WHEELS, plt, font_manager

HERE = Path(__file__).resolve().parent
OUT = HERE / "theta_output"
BUILD = OUT / "build"
DT = .01
STATES = {0: "STARTUP", 1: "DRIVING", 2: "BRAKING", 3: "STEERING", 4: "SETTLING"}
FLAGS = ["-std=c++11", "-O2", "-msse2", "-mfpmath=sse", "-ffp-contract=off", "-Wall", "-Wextra"]
METRICS = ("v", "theta", "angles", "rpms", "angle_rates", "growth", "retries", "state")


def prepare():
    kinematics = (ROOT / "include/vehicle_kinematics.hpp").read_text(encoding="utf-8")
    if not re.search(r"float\s+steering_angle\s*;", kinematics):
        raise RuntimeError("Expected v/center-angle RcCommand API")
    config = (ROOT / "include/steering_config.hpp").read_text(encoding="utf-8")
    cfg = {}
    for key in ("wheelbase", "front_track", "rear_track", "radius", "rpm_limit", "acceleration_rpm_s", "deceleration_rpm_s", "steering_rate"):
        match = re.search(rf"\b{key}\s*=\s*([0-9.]+)(?:f)?\s*([;*])", kinematics + config)
        if not match or (key == "steering_rate" and not re.search(r"steering_rate\s*=\s*[0-9.]+\s*\*\s*deg", config)):
            raise RuntimeError(f"Review config parser: {key}")
        cfg[key] = float(match[1])
    BUILD.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "include/vehicle_motion.hpp").read_text(encoding="utf-8")
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
    for name in ("original", "instrumented"):
        command = ["g++", *FLAGS, "-I", str(ROOT / "include"), "-I", str(BUILD), str(HERE / "replay_theta.cpp"), "-o", str(BUILD / f"{name}.exe")]
        if name == "instrumented":
            command.append("-DREPLAY_INSTRUMENTED")
        subprocess.run(command, check=True)
    return cfg


def replay(case, mode, cfg):
    theta = np.arctan(cfg["wheelbase"] * np.array([case.k0, case.k1]))
    paths = {}
    for name in ("original", "instrumented"):
        paths[name] = (OUT if name == "instrumented" else BUILD) / f"{case.key}_{mode}.csv"
        subprocess.run([str(BUILD / f"{name}.exe"), mode, str(paths[name]), *map(str, (case.v0, case.v1, *theta, case.duration))], check=True)
    d, original = (np.genfromtxt(paths[name], delimiter=",", names=True) for name in ("instrumented", "original"))
    for field in d.dtype.names[:16]:
        if not np.array_equal(d[field], original[field]):
            raise RuntimeError(f"Instrumentation changed {case.key}/{mode}/{field}")
    if not all(np.all(np.isfinite(d[field])) for field in d.dtype.names):
        raise RuntimeError(f"Nonfinite replay output: {case.key}/{mode}")
    angles = np.array([d[f"{w}_angle_deg"] for w in WHEELS[:2]])
    rpms = np.array([d[f"{w}_rpm"] for w in WHEELS])
    ar, growth = np.diff(angles) / DT, np.diff(np.abs(rpms)) / DT
    retry = np.maximum(d["attempts"] - 1, 0)
    # Compare against the independent final target, never assume last sample converged.
    outside = (d["target_angle_error_deg"] > 1e-4) | (d["target_rpm_error"] > 1e-3)
    bad = np.flatnonzero(outside)
    reached_index = int(bad[-1] + 1) if len(bad) else 0
    reached = float(d["time_s"][reached_index]) if reached_index < len(d) else None
    end = max(reached if reached is not None else 5., case.duration if mode == "ramp" else 0.)
    active = (d["time_s"] > 0) & (d["time_s"] <= end + 1e-8) & (d["attempts"] > 0)
    transitions = [STATES[int(d["transition"][0])]]
    for value in d["transition"][1:]:
        if STATES[int(value)] != transitions[-1]:
            transitions.append(STATES[int(value)])
    summary = dict(case=case.key, title=case.title.replace("곡률", "중앙 조향각"), mode=mode,
                   theta0_deg=float(np.degrees(theta[0])), theta1_deg=float(np.degrees(theta[1])),
                   v0=case.v0, v1=case.v1, ramp_duration_s=case.duration, reached_s=reached,
                   final_v=float(d["generated_v"][-1]), final_theta_deg=float(d["generated_theta_deg"][-1]),
                   max_abs_rpm=float(np.max(np.abs(rpms))), max_steering_rate_deg_s=float(np.max(np.abs(ar))),
                   max_rpm_increase_rpm_s=float(max(0, np.max(growth))), max_rpm_decrease_rpm_s=float(max(0, -np.min(growth))),
                   max_attempts=int(np.max(d["attempts"])), max_retries=int(np.max(retry)), max_evaluations=int(np.max(d["evaluations"])),
                   failed_search_updates=int(np.sum((d["attempts"] > 0) & (d["accepted"] == 0))),
                   active_search_updates=int(np.sum(active)), retried_updates=int(np.sum(active & (retry > 0))),
                   retry_histogram={str(int(n)): int(np.sum(active & (retry == n))) for n in np.unique(retry[active])},
                   transitions=transitions, all_feasible=bool(np.all(d["feasible"] == 1)), source_output_match=True)
    return dict(raw=d, angles=angles, rpms=rpms, angle_rates=ar, growth=growth, retries=retry, summary=summary)


def draw(ax, metric, data, cfg, horizon):
    d, t = data["raw"], data["raw"]["time_s"]
    titles = dict(v=("선속도", "m/s"), theta=("가상 중앙 휠 조향각", "deg"), angles=("실제 전륜 조향각 지령", "deg"),
                  rpms=("휠 RPM 지령", "RPM"), angle_rates=("전륜 조향각속도 지령", "deg/s"), growth=("RPM 크기 변화율", "RPM/s"),
                  retries=("추가 후보 시도", "회"), state=("펌웨어 상태 변수", ""))
    if metric in ("v", "theta"):
        key = "theta_deg" if metric == "theta" else metric
        ax.plot(t, d[f"request_{key}"], "--", color="#888888", label="요청")
        ax.plot(t, d[f"generated_{key}"], color=COLORS[0], label="생성")
    elif metric in ("retries", "state"):
        ax.step(t, data["retries"] if metric == "retries" else d["transition"], where="post", color="#555555")
        if metric == "state":
            ax.set_yticks(list(STATES), list(STATES.values()), fontsize=7)
            ax.set_ylim(-.2, 4.2)
        else:
            ax.set_ylim(-.1, max(2.2, np.max(data[metric]) + .3))
            ax.yaxis.get_major_locator().set_params(integer=True)
    else:
        x = t[1:] if metric in ("angle_rates", "growth") else t
        for i, values in enumerate(data[metric]):
            ax.plot(x, values, color=COLORS[i], ls="-" if i < 2 else "--", lw=1.3, label=WHEELS[i])
        bounds = dict(angles=(-70, 70), rpms=(-cfg["rpm_limit"], cfg["rpm_limit"]),
                      angle_rates=(-cfg["steering_rate"], cfg["steering_rate"]), growth=(-cfg["deceleration_rpm_s"], cfg["acceleration_rpm_s"]))
        for limit in bounds[metric]:
            ax.axhline(limit, color="#888888", ls="--", lw=.8)
    ax.set(title=titles[metric][0], ylabel=titles[metric][1], xlabel="시간 [s]", xlim=(0, horizon))
    ax.grid(alpha=.2)
    if metric not in ("retries", "state"):
        ax.legend(fontsize=7, ncol=4)


def main():
    cfg = prepare()
    font = Path("C:/Windows/Fonts/malgun.ttf")
    if font.exists():
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=font).get_name()
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 9})
    results, summaries = {}, []
    for case in CASES:
        for mode in ("step", "ramp"):
            results[case.key, mode] = replay(case, mode, cfg)
            summaries.append(results[case.key, mode]["summary"])
        horizon = min(5, max(case.duration, *(results[case.key, m]["summary"]["reached_s"] or 5 for m in ("step", "ramp"))) + .25)
        fig, axes = plt.subplots(8, 2, figsize=(14, 23), layout="constrained")
        for col, mode in enumerate(("step", "ramp")):
            for row, metric in enumerate(METRICS):
                draw(axes[row, col], metric, results[case.key, mode], cfg, horizon)
            axes[0, col].set_title(f"{mode} / 선속도")
        s = results[case.key, "step"]["summary"]
        fig.suptitle(f"{case.key[:2]}. {s['title']} — v·θ 스케줄러\nv {case.v0:g}→{case.v1:g} m/s, θ {s['theta0_deg']:.2f}→{s['theta1_deg']:.2f}° | ramp {case.duration:g}s", fontsize=16)
        fig.supxlabel("10 ms PC 재생 · 변화율은 샘플 차분 · 회색 파선은 요청/한계 · 실물 응답 및 MCU 실행시간 아님")
        fig.savefig(OUT / f"{case.key}_comparison.png", dpi=125)
        fig.savefig(OUT / f"{case.key}_comparison.svg")
        plt.close(fig)
        print(f"Completed {case.key}", flush=True)
    for mode in ("step", "ramp"):
        for page in range(3):
            fig, axes = plt.subplots(3, 5, figsize=(20, 10), layout="constrained")
            for row, case in enumerate(CASES[page*3:page*3+3]):
                data = results[case.key, mode]
                horizon = min(5, max(case.duration, data["summary"]["reached_s"] or 5) + .25)
                for col, metric in enumerate(("angles", "rpms", "angle_rates", "growth", "retries")):
                    draw(axes[row, col], metric, data, cfg, horizon)
                axes[row, 0].set_title(f"{case.key[:2]}. {data['summary']['title']}", fontsize=9)
            fig.suptitle(f"v·θ 스케줄러 / {mode} ({page+1}/3)", fontsize=18)
            fig.savefig(OUT / f"overview_{mode}_{page+1:02d}.png", dpi=120)
            plt.close(fig)
    paths = [ROOT / "include" / name for name in ("vehicle_motion.hpp", "vehicle_kinematics.hpp", "steering_config.hpp")]
    paths += [Path(__file__), HERE / "replay_theta.cpp", HERE / "simulate.py"]
    metadata = dict(run_date_kst=datetime.now(timezone(timedelta(hours=9))).date().isoformat(), config=cfg,
                    dt_s=DT, warmup_s=10, observation_s=5, compiler_flags=FLAGS,
                    compiler=subprocess.check_output(["g++", "--version"], text=True).splitlines()[0],
                    source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},
                    cases=[asdict(c) for c in CASES], angle_endpoint_conversion="theta=atan(wheelbase*kappa)",
                    results=summaries)
    (OUT / "summary.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    rows = [{k: v for k, v in s.items() if k not in ("retry_histogram", "transitions")} for s in summaries]
    with (OUT / "summary.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0])
        writer.writeheader()
        writer.writerows(rows)
    lines = ["# v·θ 스케줄러의 step·ramp 응답", "", f"실행일: {metadata['run_date_kst']}", "",
             "현재 C++ 모션 헤더를 PC에서 직접 호출한 9개 시나리오 × 2개 입력 방식, 총 18개 실행이다.",
             "기존 실험의 시작·목표 곡률만 θ=atan(Lκ)로 환산했다. ramp 중간 경로는 v·θ에 선형이므로 이전 v·κ ramp와 다르다.",
             "초기 명령 10초 유지 후 DRIVING부터 10ms 간격으로 5초 기록했다. step은 첫 10ms 갱신부터 최종 목표를 준다.",
             "ramp는 1초(9번은 0.25초) 동안 v·θ를 선형 변경한다. RC 필터·PWM 입력·노이즈는 포함하지 않는다.",
             "선속도·중앙각 생성값은 후륜 RPM 평균·좌측 전륜 지령각에서 역산한다. private 상태는 변경하지 않는다.",
             "조향각과 RPM은 생성 지령이며 서보·구동기 동역학, 실제 추종 지연, CAN 전송, 정수화는 포함하지 않는다.",
             "원본 헤더와 계측 복사본의 첫 16개 출력 열이 전체 실행에서 일치했다. 계측은 카운터만 추가한다.",
             "PC float 연산에 SSE 옵션을 사용했다. Teensy 빌드·실물 주행·MCU 실행시간 검증은 아니다.", "",
             "## 지표 정의", "",
             "- 도달: 별도로 계산한 최종 목표(200RPM 공통 상한 적용)와 모든 조향각 1e-4도, RPM 1e-3 이내에 이후 계속 머무는 첫 시각. 5초 안에 도달하지 않으면 null.",
             "- 변화율: 인접 지령 차이/0.01초. RPM은 크기의 차분이다. float·CSV 표현에 의한 미세한 한계 초과가 포함될 수 있다.",
             "- 재시도: 첫 후보 이후 추가 후보 수(max(attempts-1,0)). evaluate는 최종 목표와 중간 후보 운동학 호출 수다.",
             "- 재시도 분포는 도달 또는 ramp 종료 중 늦은 시점까지의 탐색 갱신을 집계한다. 최대값·실패 수는 5초 전체다.",
             "- 탐색 실패: 후보 탐색을 했으나 채택하지 못해 이전 출력을 유지한 갱신. 상태 그래프는 update 종료 시점의 펌웨어 enum이며 DRIVING도 RPM 0일 수 있다.",
             "- 기존 곡률 실험의 도달 기준은 마지막 출력 기준으로 달랐으므로 도달 시간을 그대로 같은 지표로 비교하지 않는다.", "",
             "## 결과", "", "| 사례 | 입력 | 도달(s) | 최종 v | 조향속도 최대(°/s) | RPM 증가/감소 최대(RPM/s) | 추가 시도 최대 | evaluate 최대 | 재시도/탐색 갱신 | 실패 |",
             "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for s in summaries:
        lines.append(f"| {s['case'][:2]} {s['title']} | {s['mode']} | {s['reached_s']} | {s['final_v']:.5f} | {s['max_steering_rate_deg_s']:.6f} | {s['max_rpm_increase_rpm_s']:.6f}/{s['max_rpm_decrease_rpm_s']:.6f} | {s['max_retries']} | {s['max_evaluations']} | {s['retried_updates']}/{s['active_search_updates']} | {s['failed_search_updates']} |")
    lines += ["", "## 상태와 재시도 분포", ""]
    for s in summaries:
        lines.append(f"- {s['case'][:2]} {s['mode']}: `{' → '.join(s['transitions'])}`, 추가 시도 횟수별 갱신 수 `{s['retry_histogram']}`.")
    lines += ["", "## 그래프와 원자료", "", "[요약 CSV](summary.csv) · [설정·소스 해시·결과 JSON](summary.json)", ""]
    for mode in ("step", "ramp"):
        for page in range(1, 4):
            lines.append(f"![{mode} {page}](overview_{mode}_{page:02d}.png)")
    for c in CASES:
        lines.append(f"- {c.key[:2]}: [상세 PNG]({c.key}_comparison.png) · [SVG]({c.key}_comparison.svg) · [step CSV]({c.key}_step.csv) · [ramp CSV]({c.key}_ramp.csv)")
    lines += ["", "재실행(저장소 루트):", "", "```powershell", "uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/compare_theta.py", "```", ""]
    (OUT / "README.ko.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(summaries, ensure_ascii=True))


if __name__ == "__main__":
    main()
