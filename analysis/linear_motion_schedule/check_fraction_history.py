"""Check fraction-history hypotheses on the existing 18 controller replays.

Counterfactual candidates are checked on a copy of the pre-update controller;
they never replace the original trajectory. No firmware or hardware is changed.
"""
from pathlib import Path
from datetime import datetime, timedelta, timezone
import csv
import hashlib
import json
import subprocess

import numpy as np
from simulate import CASES, ROOT, plt, font_manager, load_config

HERE = Path(__file__).resolve().parent
OUT = HERE / "fraction_output"
BUILD = OUT / "build"


PROBE = r"""
        struct ReplayProbe {
            bool allowed;
            float angle_ratio, rpm_step_ratio, peak_rpm;
        };
        ReplayProbe replay_probe(RcCommand goal, float fraction, float dt) {
            float lo[STEERING_COUNT], hi[STEERING_COUNT];
            for (int i = 0; i < STEERING_COUNT; ++i) {
                lo[i] = calibration[i].minimum_angle;
                hi[i] = calibration[i].maximum_angle;
            }
            bool is_limited = false;
            auto outputs = evaluate(goal, lo, hi, is_limited);
            if (fraction < 1) {
                RcCommand next = goal;
                next.linear_velocity = scheduled.linear_velocity + (goal.linear_velocity - scheduled.linear_velocity) * fraction;
                next.curvature = scheduled.curvature + (goal.curvature - scheduled.curvature) * fraction;
                next.angular_velocity = scheduled.angular_velocity + (goal.angular_velocity - scheduled.angular_velocity) * fraction;
                outputs = evaluate(next, lo, hi, is_limited);
            }
            ReplayProbe result = {step_allowed(outputs, dt), 0, 0, 0};
            for (int i = 0; i < STEERING_COUNT; ++i)
                result.angle_ratio = fmaxf(result.angle_ratio, fabsf(outputs.angle[i] - angle[i]) / (config.steering_rate * dt));
            for (int i = 0; i < WHEEL_COUNT; ++i) {
                result.rpm_step_ratio = fmaxf(result.rpm_step_ratio, fabsf(outputs.rpm[i] - rpm[i]) / (rpm_step_rate(rpm[i], outputs.rpm[i]) * dt));
                result.peak_rpm = fmaxf(result.peak_rpm, fabsf(outputs.rpm[i]));
            }
            return result;
        }
"""


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError(f"Source anchor changed: {old}")
    return text.replace(old, new)


def prepare():
    BUILD.mkdir(parents=True, exist_ok=True)
    source = (ROOT / "include/vehicle_motion.hpp").read_text(encoding="utf-8")
    edits = [
        ("        MotionConfig config;", "        mutable unsigned replay_evaluations = 0;\n        unsigned replay_attempts = 0, replay_rejected = 0;\n        bool replay_accepted = false;\n        float replay_initial = -1, replay_final = -1;\n" + PROBE + "\n        MotionConfig config;"),
        ("            const float dt = initialized", "            replay_evaluations = replay_attempts = replay_rejected = 0;\n            replay_accepted = false;\n            replay_initial = replay_final = -1;\n            const float dt = initialized"),
        ("            auto out = inverse(cmd", "            ++replay_evaluations;\n            auto out = inverse(cmd"),
        ("            for (uint8_t attempt = 0;", "            replay_initial = fraction;\n            for (uint8_t attempt = 0;"),
        ("                RcCommand next = goal;", "                ++replay_attempts;\n                RcCommand next = goal;"),
        ("                    scheduled = next;", "                    replay_accepted = true;\n                    replay_final = fraction;\n                    scheduled = next;"),
        ("                fraction *= 0.5f;", "                ++replay_rejected;\n                fraction *= 0.5f;"),
    ]
    # Inject probe after instrumenting the original functions, so its similar
    # expressions cannot accidentally become instrumentation anchors.
    probe = PROBE
    edits[0] = (edits[0][0], edits[0][1].replace(probe, "__REPLAY_PROBE__"))
    for old, new in edits:
        source = replace_once(source, old, new)
    source = replace_once(source, "__REPLAY_PROBE__", probe)
    (BUILD / "instrumented_vehicle_motion.hpp").write_text(source, encoding="utf-8")
    harness = (HERE / "replay_controller.cpp").read_text(encoding="utf-8")
    harness = replace_once(harness, "    for (uint32_t tick = 0;", "    float previous_initial = motion.replay_initial, previous_final = motion.replay_final;\n    bool previous_valid = motion.replay_accepted;\n    for (uint32_t tick = 0;")
    harness = replace_once(harness, "evaluations,accepted\\n", "evaluations,accepted,initial_fraction,accepted_fraction,previous_valid,previous_initial,previous_accepted,reuse_accepted_ok,reuse_initial_ok,reuse_angle_ratio,reuse_rpm_ratio,first_angle_ratio,first_rpm_ratio\\n")
    harness = replace_once(harness, "        if (tick) motion.update", "        auto before = motion;\n        if (tick) motion.update")
    harness = replace_once(harness, "        std::fprintf(out, \"%.9g", r"""        int reuse_ok = -1, reuse_initial_ok = -1;
        float reuse_angle = -1, reuse_rpm = -1, first_angle = -1, first_rpm = -1;
        if (tick && motion.replay_rejected > 0 && before.transition == vehicle::Transition::DRIVING) {
            const float update_dt = 10u * vehicle::SECONDS_PER_MILLISECOND;
            auto probe_first = before.replay_probe(vehicle::RcCommand(v, k), motion.replay_initial, update_dt);
            if (probe_first.allowed) return 5; // Counterfactual must reproduce first rejection.
            first_angle = probe_first.angle_ratio; first_rpm = probe_first.rpm_step_ratio;
            if (previous_valid) {
                auto probe = before.replay_probe(vehicle::RcCommand(v, k), previous_final, update_dt);
                reuse_ok = probe.allowed; reuse_angle = probe.angle_ratio; reuse_rpm = probe.rpm_step_ratio;
                reuse_initial_ok = before.replay_probe(vehicle::RcCommand(v, k), previous_initial, update_dt).allowed;
            }
        }
        std::fprintf(out, "%.9g""")
    harness = replace_once(harness, "%u,%u,%u,%u,%u,%u,%u\\n\",", "%u,%u,%u,%u,%u,%u,%u,%.9g,%.9g,%u,%.9g,%.9g,%d,%d,%.9g,%.9g,%.9g,%.9g\\n\",")
    harness = replace_once(harness, "            attempts, rejected, evaluations, accepted);", "            attempts, rejected, evaluations, accepted,\n            double(tick ? motion.replay_initial : -1), double(tick ? motion.replay_final : -1), unsigned(previous_valid),\n            double(previous_initial), double(previous_final), reuse_ok, reuse_initial_ok,\n            double(reuse_angle), double(reuse_rpm), double(first_angle), double(first_rpm));\n        if (tick) {\n            previous_valid = motion.replay_accepted;\n            previous_initial = motion.replay_initial; previous_final = motion.replay_final;\n        }")
    (BUILD / "fraction_replay.cpp").write_text(harness, encoding="utf-8")
    for name, file in (("original", HERE / "replay_controller.cpp"), ("fraction", BUILD / "fraction_replay.cpp")):
        args = ["g++", "-std=c++11", "-O2", "-msse2", "-mfpmath=sse", "-ffp-contract=off", "-Wall", "-Wextra",
                "-I", str(ROOT / "include"), "-I", str(BUILD), str(file), "-o", str(BUILD / f"{name}.exe")]
        if name == "fraction":
            args.append("-DREPLAY_INSTRUMENTED")
        subprocess.run(args, check=True, capture_output=True, text=True)


def trends(d, mask, previous):
    difference = d["initial_fraction"] - d[previous]
    return dict(increased=int(np.sum(mask & (difference > 0))),
                unchanged=int(np.sum(mask & (difference == 0))),
                decreased=int(np.sum(mask & (difference < 0))))


def main():
    prepare()
    summaries, events, traces = [], [], {}
    for case in CASES:
        for mode in ("step", "ramp"):
            paths = {}
            for name in ("original", "fraction"):
                path = (BUILD if name == "original" else OUT) / f"{case.key}_{mode}.csv"
                paths[name] = path
                subprocess.run([str(BUILD / f"{name}.exe"), mode, str(path),
                                *map(str, (case.v0, case.v1, case.k0, case.k1, case.duration))], check=True)
            d = np.genfromtxt(paths["fraction"], delimiter=",", names=True)
            original = np.genfromtxt(paths["original"], delimiter=",", names=True)
            for field in original.dtype.names[:14]:
                if not np.array_equal(d[field], original[field]):
                    raise RuntimeError(f"Original output changed: {case.key}/{mode}/{field}")
            failed = d["rejected"] > 0
            eligible = (d["attempts"] > 0) & (d["previous_valid"] == 1)
            available = failed & (d["reuse_accepted_ok"] >= 0)
            summary = dict(case=case.key, title=case.title, mode=mode,
                           first_failures=int(np.sum(failed)), unavailable=int(np.sum(failed & ~available)),
                           reuse_pass=int(np.sum(failed & (d["reuse_accepted_ok"] == 1))),
                           reuse_fail=int(np.sum(failed & (d["reuse_accepted_ok"] == 0))),
                           previous_initial_reuse_pass=int(np.sum(failed & (d["reuse_initial_ok"] == 1))),
                           failed_vs_previous_accepted=trends(d, available, "previous_accepted"),
                           failed_vs_previous_initial=trends(d, available, "previous_initial"),
                           all_vs_previous_accepted=trends(d, eligible, "previous_accepted"),
                           all_vs_previous_initial=trends(d, eligible, "previous_initial"),
                           max_attempts=int(np.max(d["attempts"])), original_outputs_match=True)
            summaries.append(summary)
            traces[case.key, mode] = d
            for row in d[failed]:
                events.append(dict(case=case.key, mode=mode, **{field: float(row[field]) for field in d.dtype.names}))
    totals = {key: sum(s[key] for s in summaries) for key in ("first_failures", "reuse_pass", "reuse_fail", "unavailable", "previous_initial_reuse_pass")}
    for field in ("failed_vs_previous_accepted", "failed_vs_previous_initial", "all_vs_previous_accepted", "all_vs_previous_initial"):
        totals[field] = {key: sum(s[field][key] for s in summaries) for key in ("increased", "unchanged", "decreased")}
    successful = [e for e in events if e["reuse_accepted_ok"] == 1]
    totals["reused_pass_vs_original_accepted"] = {
        "larger": sum(e["previous_accepted"] > e["accepted_fraction"] for e in successful),
        "equal": sum(e["previous_accepted"] == e["accepted_fraction"] for e in successful),
        "smaller": sum(e["previous_accepted"] < e["accepted_fraction"] for e in successful),
    }
    run_date = datetime.now(timezone(timedelta(hours=9))).date().isoformat()
    cfg = load_config()
    metadata = dict(run_date_kst=run_date, config=cfg, dt_s=0.01, observation_s=5, totals=totals, results=summaries,
                    source_sha256={name: hashlib.sha256((ROOT / "include" / name).read_bytes()).hexdigest()
                                   for name in ("vehicle_motion.hpp", "vehicle_kinematics.hpp", "steering_config.hpp")},
                    compiler=subprocess.run(["g++", "--version"], capture_output=True, text=True, check=True).stdout.splitlines()[0],
                    flags="-std=c++11 -O2 -msse2 -mfpmath=sse -ffp-contract=off")
    (OUT / "summary.json").write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    with (OUT / "failure_events.csv").open("w", encoding="utf-8-sig", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["case", "mode", *next(iter(traces.values())).dtype.names])
        writer.writeheader()
        writer.writerows(events)
    font = Path("C:/Windows/Fonts/malgun.ttf")
    if font.exists():
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=font).get_name()
    plt.rcParams.update({"axes.unicode_minus": False, "font.size": 9})
    for mode in ("step", "ramp"):
        fig, axes = plt.subplots(3, 3, figsize=(16, 11), layout="constrained")
        for ax, case in zip(axes.flat, CASES):
            d = traces[case.key, mode]
            # Include transition and a short stable tail, omit long steady holds.
            changing = (np.abs(d["generated_v"] - d["generated_v"][-1]) > 1e-5) | (np.abs(d["generated_k"] - d["generated_k"][-1]) > 1e-5)
            end = d["time_s"][np.flatnonzero(changing)[-1]] + 0.2 if np.any(changing) else 1.2
            mask = (d["time_s"] > 0) & (d["time_s"] <= end) & (d["attempts"] > 0)
            # Keep gaps for BRAKING/SETTLING instead of connecting across them.
            for field, label, color in (("initial_fraction", "첫 후보", "#0072B2"), ("accepted_fraction", "채택", "#D55E00")):
                ax.plot(d["time_s"], np.where(mask, d[field], np.nan), lw=1.2, label=label, color=color)
            for status, label, marker, color in ((1, "직전값 통과", "o", "#009E73"), (0, "직전값 실패", "x", "#CC3311")):
                selected = mask & (d["reuse_accepted_ok"] == status)
                ax.scatter(d["time_s"][selected], d["previous_accepted"][selected], s=16, marker=marker, color=color, label=label, zorder=4)
            ax.set(title=f"{case.key[:2]}. {case.title}", xlabel="시간 [s]", ylabel="잔여 목표에 대한 진행 비율 α", xlim=(0, end), ylim=(0, 1.04))
            ax.grid(alpha=0.2)
            ax.legend(fontsize=7, ncol=2)
        fig.suptitle(f"진행률 이력 검증 / {mode} 입력", fontsize=18)
        fig.supxlabel("점 표시는 첫 후보 거부 시에만: 직전 채택 비율을 현재 상태·목표에 대입한 검사 결과 (원래 출력에는 미반영)", fontsize=10)
        fig.savefig(OUT / f"fractions_{mode}.png", dpi=140)
        plt.close(fig)
    lines = ["# 진행률 이력 재사용 검증", "", f"{run_date}. 기존 9개 시나리오 × step/ramp, 10ms 갱신, 각 5초 관측.", "",
             f"RPM 크기 증가율 {cfg['acceleration_rpm_s']:g}RPM/s, 감소율 {cfg['deceleration_rpm_s']:g}RPM/s 설정.", "",
             "## 검사 정의", "",
             "- 첫 후보 진행률 α_initial, 실제 채택 진행률 α_accepted를 각각 기록했다.",
             "- 첫 후보 거부가 발생하면 현재 α_initial을 (A) 직전 α_accepted, (B) 직전 α_initial과 각각 비교했다.",
             "- 동일한 갱신 직전 controller 복사본에서, 현재 요청에 직전 α_accepted 또는 직전 α_initial을 대입하고 원래 evaluate/step_allowed로 검사했다.",
             "- α를 제외한 현재 상태·요청·dt·RPM 목표 제한은 그대로다. 성공은 해당 후보가 현재 코드의 전체 검사를 통과한다는 뜻이다.",
             "- 이 대안은 채택하지 않는다. 장기적으로 이전 비율을 계속 사용하는 정책을 실행한 실험이 아니라, 기존 출력 경로에서 한 갱신씩 대안을 검사한 결과다.",
             "- 직전 갱신이 후보를 채택하지 않은 경우 재사용 이력 없음으로 취급한다. 첫 갱신의 직전 값은 초기 상태를 유지한 준비 구간의 마지막 채택값(1)이다.",
             "- 증가/동일/감소는 기록된 float 값의 직접 비교다. 제한 비율은 1 초과가 위반을 뜻하나 출력 반올림으로 경계값이 1로 보일 수 있다.",
             "- 원본 C++ 출력·상태와 계측 복사본이 18개 실행 전체에서 동일함을 확인했다. 첫 후보 거부도 복사본 검사에서 다시 거부됨을 확인했다.",
             "- PC g++ SSE float 재생이다. 실제 MCU 실행시간·하드웨어 응답·모든 입력에 대한 보장으로 해석하지 않는다.", "",
             "## 집계", "",
             f"- 첫 후보 거부 {totals['first_failures']}개 중 직전 채택 비율 재사용은 {totals['reuse_pass']}개 통과, {totals['reuse_fail']}개 실패했다.",
             f"- 직전 채택 비율과 비교한 거부 갱신의 증가/동일/감소 개수: {totals['failed_vs_previous_accepted']}.",
             f"- 직전 첫 후보 비율과 비교한 거부 갱신의 증가/동일/감소 개수: {totals['failed_vs_previous_initial']}.",
             f"- 재사용 통과 후보를 원래 채택 비율과 비교한 개수: {totals['reused_pass_vs_original_accepted']}.",
             f"- 직전 첫 후보 비율 재사용 통과: {totals['previous_initial_reuse_pass']}개.",
             "- 전체 집계(all_vs_previous_*)에는 목표 도달 후 유지 갱신도 포함된다. 거부 갱신 집계(failed_vs_previous_*)와 분모가 다르다.",
             "- 비율 α는 물리적인 속도가 아니라 매 갱신의 잔여 목표 차이에 대한 비율이다. 같은 α라도 목표·현재 상태가 바뀌면 실제 변화량이 달라진다.",
             "- 각 거부 갱신의 시각, 후보 비율, 허용 변화량에 대한 비율은 failure_events.csv에 기록한다.", "",
             "## 전체 결과", "", "```json", json.dumps(totals, ensure_ascii=False, indent=2), "```", "",
             "## 시나리오별 결과", "", "| 예시 | 입력 | 첫 후보 거부 | 직전 채택 대비 증가/동일/감소 | 직전 첫 후보 대비 증가/동일/감소 | 직전 채택 재사용 통과/실패/없음 |", "|---|---|---:|---|---|---|"]
    for s in summaries:
        a = s["failed_vs_previous_accepted"]
        b = s["failed_vs_previous_initial"]
        lines.append(f"| {s['title']} | {s['mode']} | {s['first_failures']} | {a['increased']}/{a['unchanged']}/{a['decreased']} | {b['increased']}/{b['unchanged']}/{b['decreased']} | {s['reuse_pass']}/{s['reuse_fail']}/{s['unavailable']} |")
    lines += ["", "## 그래프", "", "![Step 진행률](fractions_step.png)", "![Ramp 진행률](fractions_ramp.png)", "",
              "[거부 갱신 상세 CSV](failure_events.csv) · [요약 JSON](summary.json)", "",
              "각 시나리오의 전체 시계열은 `<시나리오>_<step 또는 ramp>.csv`이다. reuse_accepted_ok는 통과 1, 실패 0, 미검사 -1이다.", "",
              "재실행:", "", "```powershell", "uv run --no-project --with numpy --with matplotlib python analysis/linear_motion_schedule/check_fraction_history.py", "```", ""]
    (OUT / "README.ko.md").write_text("\n".join(lines), encoding="utf-8")
    print(json.dumps(dict(totals=totals, results=summaries), ensure_ascii=True, indent=2))


if __name__ == "__main__":
    main()
