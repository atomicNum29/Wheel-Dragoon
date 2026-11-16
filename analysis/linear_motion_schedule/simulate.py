"""Linear v/curvature schedules through the repository's MANUAL kinematics.

Offline illustration only: no serial I/O, firmware execution, saturation,
motion-state transitions, actuator dynamics, or steering feedback.
Run: uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/simulate.py
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from dataclasses import asdict, dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
WHEELS = ("LF", "RF", "LR", "RR")
COLORS = ("#0072B2", "#D55E00", "#009E73", "#8C56A2")


@dataclass(frozen=True)
class Case:
    key: str
    title: str
    v0: float
    v1: float
    k0: float
    k1: float
    duration: float = 1.0


CASES = [
    Case("01_straight_acceleration", "직진 가속", 0.2, 1.2, 0, 0),
    Case("02_constant_speed_turn", "일정 선속도에서 선회", 0.8, 0.8, 0, 2),
    Case("03_speed_and_curvature", "선속도·곡률 동시 증가", 0.3, 1.0, 0, 2),
    Case("04_slowdown_into_turn", "감속하며 선회", 1.0, 0.3, 0, 2),
    Case("05_left_to_right", "좌회전에서 우회전으로", 0.6, 0.6, 2, -2),
    Case("06_stationary_steering", "정지 상태 조향", 0, 0, -2, 2),
    Case("07_reverse_acceleration", "후진 속도·곡률 증가", -0.3, -1.0, 0, 2),
    Case("08_direction_reversal", "일정 곡률에서 전후진 전환", 0.6, -0.6, 1, 1),
    Case("09_fast_steering", "빠른 조향 요청", 0.8, 0.8, 0, 2, 0.25),
]


def require_curvature_scheduler():
    """Do not feed historical curvature commands to the center-angle API."""
    source = (ROOT / "include/vehicle_kinematics.hpp").read_text(encoding="utf-8")
    command = re.search(r"struct RcCommand\s*\{(.*?)\n\};", source, re.S)
    if command is None or not re.search(r"\bfloat\s+curvature\s*;", command.group(1)):
        raise RuntimeError(
            "These archived experiments require the v/curvature firmware API. "
            "Use a separate checkout of 7f61e4f; see analysis/README.md. "
            "The current v/center-angle API requires adapted experiments."
        )


def load_config():
    require_curvature_scheduler()
    sources = {name: (ROOT / "include" / name).read_text(encoding="utf-8")
               for name in ("vehicle_kinematics.hpp", "steering_config.hpp")}

    def number(name, source):
        match = re.search(rf"\b{re.escape(name)}\s*=\s*([0-9.]+)(?:f)?\s*[;*]", sources[source])
        if match is None:
            raise ValueError(f"Cannot read {name} from {source}; review the source format.")
        return float(match.group(1))

    cfg = {key: number(key, "vehicle_kinematics.hpp")
           for key in ("wheelbase", "front_track", "rear_track", "radius", "MAX_LINEAR_SPEED_MPS")}
    cfg.update({key: number(key, "steering_config.hpp") for key in
                ("rpm_limit", "acceleration_rpm_s", "deceleration_rpm_s", "steering_rate", "rc_maximum_inner_angle")})
    # These two source constants are written in degrees multiplied by `deg`.
    for name in ("steering_rate", "rc_maximum_inner_angle"):
        if not re.search(rf"\b{name}\s*=\s*[0-9.]+\s*\*\s*deg", sources["steering_config.hpp"]):
            raise ValueError(f"Expected degrees * deg for {name}")
    cfg["source_sha256"] = {name: hashlib.sha256((ROOT / "include" / name).read_bytes()).hexdigest()
                            for name in sources}
    return cfg


def simulate(case, cfg, dt):
    count = max(1, math.ceil(case.duration / dt))
    t = np.linspace(0, case.duration, count + 1)
    p = t / case.duration
    v = case.v0 + (case.v1 - case.v0) * p
    k = case.k0 + (case.k1 - case.k0) * p
    vd, kd = (case.v1 - case.v0) / case.duration, (case.k1 - case.k0) / case.duration
    L, tf, tr, r = (cfg[key] for key in ("wheelbase", "front_track", "rear_track", "radius"))
    C = 60 / (2 * np.pi * r)
    angles, angle_rates, rpms, rpm_rates = [], [], [], []
    for y in (tf / 2, -tf / 2):
        x, lateral = 1 - y * k, L * k
        q = np.hypot(x, lateral)
        angles.append(np.degrees(np.arctan2(lateral, x)))
        angle_rates.append(np.degrees(L * kd / q**2))
        qd = (-y * x + L * lateral) * kd / q
        rpms.append(C * v * q)
        rpm_rates.append(C * (vd * q + v * qd))
    for y in (tr / 2, -tr / 2):
        q = 1 - y * k
        rpms.append(C * v * q)
        rpm_rates.append(C * (vd * q - v * y * kd))
    angles, angle_rates, rpms, rpm_rates = map(np.asarray, (angles, angle_rates, rpms, rpm_rates))
    # At zero RPM use the departure-side derivative of |RPM|. At a reversal
    # cusp this is the right-hand limit; the two-sided derivative is undefined.
    growth = np.where(np.abs(rpms) > 1e-12, np.sign(rpms) * rpm_rates, np.abs(rpm_rates))
    data = dict(t=t, v=v, k=k, angles=angles, angle_rates=angle_rates,
                rpms=rpms, rpm_rates=rpm_rates, growth=growth,
                delta_angles=np.diff(angles, axis=1), delta_rpm=np.diff(rpms, axis=1))
    summary = dict(case=asdict(case), sample_count=len(t), effective_dt_s=float(t[1] - t[0]),
                   max_abs_angle_deg=float(np.max(np.abs(angles))),
                   max_abs_steering_rate_deg_s=float(np.max(np.abs(angle_rates))),
                   max_abs_rpm=float(np.max(np.abs(rpms))),
                   max_rpm_magnitude_increase_rpm_s=float(max(0, np.max(growth))),
                   max_rpm_magnitude_decrease_rpm_s=float(max(0, -np.min(growth))))
    summary["exceeds"] = dict(
        steering_angle=summary["max_abs_angle_deg"] > cfg["rc_maximum_inner_angle"] + 1e-9,
        steering_rate=summary["max_abs_steering_rate_deg_s"] > cfg["steering_rate"] + 1e-9,
        rpm=summary["max_abs_rpm"] > cfg["rpm_limit"] + 1e-9,
        rpm_acceleration=summary["max_rpm_magnitude_increase_rpm_s"] > cfg["acceleration_rpm_s"] + 1e-9,
        rpm_deceleration=summary["max_rpm_magnitude_decrease_rpm_s"] > cfg["deceleration_rpm_s"] + 1e-9,
        linear_speed=bool(np.any(np.abs(v) > cfg["MAX_LINEAR_SPEED_MPS"] + 1e-9)))
    summary["wheels"] = {name: dict(start_rpm=float(rpms[i, 0]), end_rpm=float(rpms[i, -1]),
                                         min_rpm=float(np.min(rpms[i])), max_rpm=float(np.max(rpms[i])),
                                         min_signed_rate_rpm_s=float(np.min(rpm_rates[i])),
                                         max_signed_rate_rpm_s=float(np.max(rpm_rates[i])))
                          for i, name in enumerate(WHEELS)}
    return data, summary


def plot_metric(ax, metric, data, cfg, title=True):
    specs = {
        "angles": (data["t"], data["angles"], WHEELS[:2], "生成 조향각", "deg"),
        "rpms": (data["t"], data["rpms"], WHEELS, "휠 RPM 지령", "RPM"),
        "angle_rates": (data["t"], data["angle_rates"], WHEELS[:2], "조향각 변화율", "deg/s"),
        "growth": (data["t"], data["growth"], WHEELS, "RPM 크기 변화율", "d|RPM|/dt [RPM/s]"),
        "delta_angles": (data["t"][1:], data["delta_angles"], WHEELS[:2], "샘플 간 조향각 변화량", "deg / step"),
        "delta_rpm": (data["t"][1:], data["delta_rpm"], WHEELS, "샘플 간 RPM 변화량", "RPM / step"),
    }
    x, values, names, heading, unit = specs[metric]
    heading = heading.replace("生成", "생성")
    for i, (name, values_i) in enumerate(zip(names, values)):
        ax.plot(x, values_i, color=COLORS[i], lw=1.9, label=name,
                linestyle="-" if i < 2 else "--")
    if metric in ("angles", "rpms", "angle_rates"):
        key = {"angles": "rc_maximum_inner_angle", "rpms": "rpm_limit", "angle_rates": "steering_rate"}[metric]
        for sign in (-1, 1):
            ax.axhline(sign * cfg[key], color="#777777", lw=0.9, ls=":")
    if metric == "growth":
        ax.axhline(cfg["acceleration_rpm_s"], color="#777777", lw=0.9, ls=":")
        ax.axhline(-cfg["deceleration_rpm_s"], color="#777777", lw=0.9, ls=":")
    ax.axhline(0, color="#C9CED4", lw=0.6, zorder=0)
    ax.set(xlabel="시간 [s]", ylabel=unit, xlim=(0, data["t"][-1]))
    if title:
        ax.set_title(heading, fontsize=11, loc="left")
    ax.grid(True, alpha=0.18)
    ax.legend(fontsize=8, ncol=len(names), loc="best", framealpha=0.8)


def case_label(case):
    return f"v: {case.v0:g} → {case.v1:g} m/s   |   κ: {case.k0:g} → {case.k1:g} 1/m   |   T={case.duration:g} s"


def save_case(case, data, summary, cfg, out):
    fig, axes = plt.subplots(4, 2, figsize=(13.5, 15), layout="constrained")
    for ax, key, title, unit in ((axes[0, 0], "v", "선형 입력: 선속도", "m/s"),
                                  (axes[0, 1], "k", "선형 입력: 곡률", "1/m")):
        ax.plot(data["t"], data[key], color="#0072B2" if key == "v" else "#E69F00", lw=2)
        ax.set(title=title, xlabel="시간 [s]", ylabel=unit, xlim=(0, case.duration))
        ax.grid(True, alpha=0.18)
    for ax, metric in zip(axes[1:].flat, ("angles", "rpms", "angle_rates", "growth", "delta_angles", "delta_rpm")):
        plot_metric(ax, metric, data, cfg)
    fig.suptitle(f"{case.key[:2]}. {case.title}\n{case_label(case)}", fontsize=17)
    fig.supxlabel(f"단순 운동학 지령 · MCU 스케줄러/실물 모델 아님 · 제한 미적용 · 점선=기준 한계 · Δt={summary['effective_dt_s']*1000:g} ms", fontsize=10)
    fig.savefig(out / f"{case.key}.png", dpi=160)
    fig.savefig(out / f"{case.key}.svg")
    plt.close(fig)
    columns = {"time_s": data["t"], "linear_velocity_mps": data["v"], "curvature_per_m": data["k"]}
    for i, name in enumerate(WHEELS[:2]):
        columns[f"{name}_steering_deg"] = data["angles"][i]
        columns[f"{name}_steering_rate_deg_s"] = data["angle_rates"][i]
        columns[f"{name}_delta_steering_deg"] = np.r_[np.nan, data["delta_angles"][i]]
    for i, name in enumerate(WHEELS):
        columns[f"{name}_rpm"] = data["rpms"][i]
        columns[f"{name}_rpm_rate_rpm_s"] = data["rpm_rates"][i]
        columns[f"{name}_rpm_magnitude_rate_rpm_s"] = data["growth"][i]
        columns[f"{name}_delta_rpm"] = np.r_[np.nan, data["delta_rpm"][i]]
    with (out / f"{case.key}.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(columns)
        writer.writerows(zip(*columns.values()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dt", type=float, default=0.01, help="Maximum sample interval, seconds")
    parser.add_argument("--case", action="append", choices=[c.key for c in CASES])
    parser.add_argument("--custom", type=float, nargs=5, metavar=("V0", "V1", "K0", "K1", "SECONDS"))
    parser.add_argument("--output", type=Path, default=Path(__file__).parent / "output")
    args = parser.parse_args()
    if not math.isfinite(args.dt) or args.dt <= 0:
        parser.error("dt must be finite and positive")
    if args.custom and args.case:
        parser.error("Choose --custom or --case")
    cases = [Case("custom", "사용자 지정 전환", *args.custom)] if args.custom else [c for c in CASES if not args.case or c.key in args.case]
    for c in cases:
        if not all(math.isfinite(x) for x in (c.v0, c.v1, c.k0, c.k1, c.duration)) or c.duration <= 0:
            parser.error("All schedule values must be finite and duration must be positive")
        if math.ceil(c.duration / args.dt) > 1_000_000:
            parser.error("Too many samples; increase --dt")
    font = Path("C:/Windows/Fonts/malgun.ttf")
    if font.exists():
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=font).get_name()
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 10, "figure.facecolor": "white"})
    cfg = load_config()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    results = {}
    for c in cases:
        data, summary = simulate(c, cfg, args.dt)
        save_case(c, data, summary, cfg, out)
        results[c.key] = (c, data, summary)
    pages = []
    groups = ([CASES[i].key for i in group] for group in ((0, 1, 2), (3, 4, 8), (5, 6, 7)))
    for index, keys in enumerate(groups, 1):
        selected = [results[key] for key in keys if key in results]
        if not selected:
            continue
        fig, axes = plt.subplots(len(selected), 4, figsize=(18, 3.7 * len(selected) + 1), squeeze=False, layout="constrained")
        for row, (case, data, _) in enumerate(selected):
            for col, metric in enumerate(("angles", "rpms", "angle_rates", "growth")):
                plot_metric(axes[row, col], metric, data, cfg)
            axes[row, 0].set_title(f"{case.key[:2]}. {case.title}\nv {case.v0:g}→{case.v1:g}, κ {case.k0:g}→{case.k1:g}, {case.duration:g}s\n생성 조향각", fontsize=10, loc="left")
        fig.suptitle(f"선형 선속도·곡률 입력의 출력 비교 ({index}/3)", fontsize=19)
        fig.supxlabel(f"모두 미제한 운동학 지령 | 점선: ±{cfg['rc_maximum_inner_angle']:g}°, ±{cfg['rpm_limit']:g} RPM, ±{cfg['steering_rate']:g}°/s, RPM 크기 증가 +{cfg['acceleration_rpm_s']:g} / 감소 -{cfg['deceleration_rpm_s']:g} RPM/s", fontsize=11)
        filename = f"overview_{index:02d}.png"
        fig.savefig(out / filename, dpi=160)
        plt.close(fig)
        pages.append(filename)
    summaries = [entry[2] for entry in results.values()]
    (out / "summary.json").write_text(json.dumps(dict(config=cfg, requested_dt_s=args.dt, results=summaries), ensure_ascii=False, indent=2), encoding="utf-8")
    rows = []
    for s in summaries:
        rows.append({**s["case"], **{k: v for k, v in s.items() if k not in ("case", "wheels", "exceeds")},
                     "exceeded_limits": ";".join(k for k, v in s["exceeds"].items() if v)})
    with (out / "summary.csv").open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    run_date = datetime.now(timezone(timedelta(hours=9))).date().isoformat()
    lines = ["# 선형 선속도·곡률 스케줄 시뮬레이션", "",
             f"{run_date}. 사용자가 요청한 오프라인 Python 시뮬레이션이다. MCU 코드 실행·통신·업로드는 하지 않는다.", "",
             "## 모델과 해석", "",
             "- 후륜 축 중앙 기준의 선속도와 곡률을 시작값에서 목표값까지 시간에 대해 선형 보간한다.",
             f"- 저장소 설정을 읽었다: 축거 {cfg['wheelbase']} m, 전륜 윤거 {cfg['front_track']} m, 후륜 윤거 {cfg['rear_track']} m, 휠 반경 {cfg['radius']} m.",
             "- 일반 MANUAL 운동학을 적용한다. PIVOT, BRAKING/SETTLING, RPM 제한·가감속 제한·조향 변화율 제한은 적용하지 않는다.",
             "- 점선은 현재 펌웨어 설정과 비교하기 위한 기준선이다. 상한 초과값을 숨기거나 클램프하지 않는다.",
             f"- 비교 기준: RPM 크기 증가율 {cfg['acceleration_rpm_s']:g}RPM/s, 감소율 {cfg['deceleration_rpm_s']:g}RPM/s, RPM 상한 {cfg['rpm_limit']:g}.",
             "- 조향각은 보정에서 사용하는 차량 좌우 전륜 각도 지령이다. 실제 서보축 각도·PWM·지연·부하·타이어 미끄러짐을 시뮬레이션한 것이 아니다.",
             "- 1/s가 아니라 곡률 단위는 1/m이다. RPM 변화율 단위는 RPM/s이다.",
             "- 변화율은 운동학 식을 해석적으로 미분했다. 그래프와 요약 최대값은 샘플 지점 값이며, 구간 전체 최대값의 별도 최적화 결과는 아니다.",
             "- 전환 구간 양 끝의 변화율은 구간 안쪽 단측 값이다. 전환 전후의 유지 구간이나 jerk는 포함하지 않는다.",
             "- RPM 크기 변화율 d|RPM|/dt: 양수는 가속, 음수는 감속이다. RPM 0에서는 출발 방향의 단측 값을 사용한다. 전후진 전환의 영점에서는 양측 미분이 정의되지 않는다.",
             "- Δ 조향각·Δ RPM 패널은 인접 샘플의 실제 차이다. CSV 첫 샘플의 Δ는 이전 샘플이 없어 NaN이다.",
             "- 역방향·정지 예시에서도 속도로 나누지 않고 곡률로 조향각을 직접 계산한다.",
             "- 전후진 전환 예시는 순수 수학 스케줄이다. 실제 MCU의 반전 감속·대기 절차를 재현하지 않는다.", "",
             "## 그림", ""]
    for page in pages:
        lines.append(f"![비교 그림]({page})")
    lines += ["", "| 예시 | v [m/s] | κ [1/m] | 시간 [s] | 상세 그림 | 데이터 |", "|---|---|---|---:|---|---|"]
    for c, _, _ in results.values():
        lines.append(f"| {c.title} | {c.v0:g} → {c.v1:g} | {c.k0:g} → {c.k1:g} | {c.duration:g} | [PNG]({c.key}.png) / [SVG]({c.key}.svg) | [CSV]({c.key}.csv) |")
    lines += ["", "[요약 수치 CSV](summary.csv) · [설정·소스 해시·휠별 상세 JSON](summary.json)", "",
              "## 다시 실행", "", "저장소 루트에서:", "", "```powershell",
              "uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/simulate.py",
              "# 사용자 지정: v0 v1 k0 k1 전환시간(초)",
              "uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/simulate.py --custom 0.3 1.0 0 2 1 --output analysis/linear_motion_schedule/custom_output",
              "```", "", "외부 plotting 의존성은 uv 임시 환경을 사용하며 저장소 pyproject.toml·uv.lock을 변경하지 않는다."]
    (out / "README.ko.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(dict(output=str(out), cases=len(cases), summary=rows), ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
