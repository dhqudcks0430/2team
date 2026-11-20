"""구로 v2 스모크 A안(도로망·순환만) 노선안 평가 — 실험 주간 5일(10/26~30), affected, 06:00 시작.

    python3 -I run_eval.py <2team-routegen-v2 저장소>

- 우리 안·피더안(11·13대): 버스 통행 가능 도로망 review1119/stage1c/nets/guro/road.json으로 만든 묶음.
- 지자체안(1월 실운행, 11대): review1119/stage1c/sweep/guro/muni06/plan.json, 망 road_muni.json
  (같은 도로망 + 지자체 노선을 잇는 OSM 연결 11개). 같은 대수 11대에서만 비교한다.
- 묶음: zip_v4/region/guro_0610_v4/bundles/affected_<날짜>를 복사하고 travel_times.json만 바꾼 뒤 run_inputs를
  시뮬레이터 함수(build_run_inputs)로 다시 만든다. 실행은 experiments/guro_exp/run_matrix.py(변환기 → M05 → M07).
- 테스트 주간은 열지 않는다.
"""
import json
import shutil
import statistics as stx
import subprocess
import sys
from pathlib import Path

REPO = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(REPO / "src"))
from busstrike.sim.arrivals import build_run_inputs  # noqa: E402

HERE = Path(__file__).resolve().parent
W = HERE.parents[1]
SRC = W / "zip_v4" / "region" / "guro_0610_v4" / "bundles"
DATES = ["20261026", "20261027", "20261028", "20261029", "20261030"]
NETS = {"road": W / "review1119" / "stage1c" / "nets" / "guro" / "road.json",
        "road_muni": W / "review1119" / "stage1c" / "sweep" / "guro" / "muni06" / "road_muni.json"}
PLANS = W / "routegen_v2" / "smoke" / "A" / "guro" / "plans"
RUNS = {"road": [("ours", 11, PLANS / "plan-01-affected-f11.json"), ("ours", 13, PLANS / "plan-01-affected-f13.json"),
                 ("sp", 11, PLANS / "plan-02-affected-f11.json"), ("sp", 13, PLANS / "plan-02-affected-f13.json")],
        "road_muni": [("muni", 11, W / "review1119" / "stage1c" / "sweep" / "guro" / "muni06" / "plan.json")]}


def make_bundles(net: str) -> Path:
    out = HERE / "bundles" / net / "bundles"
    for d in DATES:
        s, t = SRC / f"affected_{d}", out / f"affected_{d}"
        t.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(s / "demand_bins.csv", t / "demand_bins.csv")
        shutil.copyfile(NETS[net], t / "travel_times.json")
        for f in (11, 13):
            shutil.copyfile(s / f"scenario_f{f}.json", t / f"scenario_f{f}.json")
            ri = json.loads((s / f"run_inputs_f{f}.json").read_text(encoding="utf-8"))
            build_run_inputs(ri["input_bundle_id"] + f"-{net}-stage1c", t / f"scenario_f{f}.json",
                             t / "demand_bins.csv", t / "travel_times.json", ri["seed"], t / f"run_inputs_f{f}.json")
    return out


rows = []
for net, plans in RUNS.items():
    b = make_bundles(net)
    cmd = [sys.executable, str(REPO / "experiments" / "guro_exp" / "run_matrix.py"), "--bundles", str(b),
           "--out", str(HERE / "runs" / net), "--src", str(REPO / "src"), "--jobs", "5", "--data-label", f"zip_v4/guro+{net}"]
    for lab, f, p in plans:
        cmd += ["--plan", f"{lab}:{f}={p}"]
    subprocess.run(cmd, check=False)
    for lab, f, p in plans:
        per = []
        for d in DATES:
            rd = HERE / "runs" / net / lab / f"affected_{d}" / f"f{f}"
            met = json.loads((rd / "run" / "metrics.json").read_text(encoding="utf-8"))
            ver = json.loads((rd / "verifier_report.json").read_text(encoding="utf-8"))
            m1, m2 = met["M1"], met["M2"]
            per.append({"date": d, "served": m1["served"], "total": m1["total"], "rate": m1["rate"],
                        "unserved_by_reason": m2["unserved_by_reason"], "m07_pass": ver["pass"],
                        "m07_failed": ver["failed_checks"]})
        plan = json.loads(Path(p).read_text(encoding="utf-8"))
        rows.append({"label": lab, "fleet": f, "net": net, "routes": len(plan["routes"]),
                     "served_mean": stx.mean(x["served"] for x in per), "total_mean": stx.mean(x["total"] for x in per),
                     "rate_mean": stx.mean(x["rate"] for x in per),
                     "rate_min": min(x["rate"] for x in per), "rate_max": max(x["rate"] for x in per),
                     "unserved_mean": {k: stx.mean(x["unserved_by_reason"].get(k, 0) for x in per)
                                       for k in ("no_connection", "service_end", "wait", "journey")},
                     "m07_all_pass": all(x["m07_pass"] for x in per), "days": per})
(HERE / "results.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
print("| 안 | 대수 | 노선 | 하루 평균 충족 | 대상 | 충족률(5일 평균) | 날짜별 범위 | M07 |")
print("| --- | --- | --- | --- | --- | --- | --- | --- |")
for r in rows:
    print(f"| {r['label']} | {r['fleet']} | {r['routes']} | {r['served_mean']:.1f} | {r['total_mean']:.1f} | "
          f"{100 * r['rate_mean']:.2f}% | {100 * r['rate_min']:.2f}~{100 * r['rate_max']:.2f}% | "
          f"{'5/5 pass' if r['m07_all_pass'] else 'fail 있음'} |")
for r in rows:
    print(r["label"], r["fleet"], "미충족 사유(평균)", {k: round(v, 1) for k, v in r["unserved_mean"].items()})
