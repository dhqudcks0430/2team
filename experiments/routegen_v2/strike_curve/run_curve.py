"""파업 비율 곡선 실험(구로) — 생성 = 실험 주간, 평가 = 테스트 주간, 세 안(우리·지자체·범용 AI) 같은 조건.

    python3 -I run_curve.py <2team-routegen-v2 저장소> [--workers 4] [--only p050_d0,control] [--rates 0.1,...] [--draws 5]

시나리오
- control: 파업 없음 — 걸어서 역에 못 가는 모든 버스 승객(flow_total)이 대상(strike.scenario_rows mode="all").
- pRRR_dD: 파업 비율 RRR/100, 무작위 뽑기 D(strike.draw_strike, seed 20261121). 파업 노선 승객 중 운행 중인 노선(파업 안 한
  시내·미확인·마을, 경기)으로 같은 정류장 → 역을 갈 수 있으면 대상에서 뺀다.
안
- ours: routegen v2(버스 통행 가능 도로망 stage1c, 순환만, 필수 정류장)를 그 시나리오의 실험 주간 수요로 생성.
  대수 = 고정 11대(1월 실운행) + 비례 max(1, round(11 × 비율)).
- muni: 지자체 셔틀안(1월, 11대) stage1c/sweep/guro/muni06(plan.json, road_muni.json = 도로망 + 무정차 이음 OSM 연결).
- aiXX: 범용 AI안(gpt-6.2-sol, run05-10_new 6개, 11대). 지자체안과 같은 도구(eval_municipal --links osm)로 무정차 이음을 이음.
  지자체안·AI안은 시나리오와 무관하게 고정(파업 비율이 바뀌어도 같은 노선)이다.
평가: 테스트 주간 11/02~06 각 날짜, 변환기 → M05 → M07(run_matrix). 큰 실행 파일(events·realization·passenger_outcomes)은
M07 검증 뒤 지운다. 결과: summary/<시나리오>.json(날짜·안별 지표), 끝에 summary_all.json.
"""
import argparse
import json
import shutil
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

HERE = Path(__file__).resolve().parent
W = HERE.parents[1]
EXP = W / "zip_v4" / "region" / "guro_0610_v4"
TEST = W / "zip_v4" / "region" / "guro_0610_v4_test"
ROAD = W / "review1119" / "stage1c" / "nets" / "guro"
MUNI = W / "review1119" / "stage1c" / "sweep" / "guro" / "muni06"
FAC = W / "routegen_v2" / "facilities" / "facilities_seoul.csv"
TEST_DATES = ["20261102", "20261103", "20261104", "20261105", "20261106"]
FIXED = 11
AIS = [f"ai{i:02d}" for i in range(5, 11)]


def sid_of(rate, draw):
    return "control" if rate is None else f"p{round(rate * 100):03d}_d{draw}"


def run_one(repo: str, rate, draw: int) -> dict:
    sys.path.insert(0, str(Path(repo) / "src"))
    from busstrike.planning.routegen import data as D
    from busstrike.planning.routegen import strike as K
    from busstrike.sim import io as sio
    from busstrike.sim.arrivals import build_run_inputs

    sid = sid_of(rate, draw)
    log = {"sid": sid, "rate": rate, "draw": draw, "t0": time.time()}
    done = HERE / "summary" / f"{sid}.json"
    if done.exists():
        return json.loads(done.read_text(encoding="utf-8"))
    prop = FIXED if rate is None else max(1, round(FIXED * rate))
    fleets = sorted({FIXED, prop})
    # 1. 생성(실험 주간)
    gen = HERE / "gen" / sid
    if b0_old := (HERE / "bundles" / sid):
        if b0_old.exists():
            shutil.rmtree(b0_old)
    cached = (gen / "manifest.json").exists()     # manifest는 생성이 끝날 때만 쓰인다
    cmd = [sys.executable, "-m", "busstrike.planning.routegen", "--profile", "v2", "--network-dir", str(ROAD),
           "--root", str(EXP), "--out", str(gen), "--facilities", str(FAC), "--fleets", ",".join(map(str, fleets))]
    cmd += ["--demand-mode", "all"] if rate is None else ["--strike-rate", str(rate), "--strike-draw", str(draw)]
    if cached:
        log["gen_rc"], log["gen_cached"] = 0, True
    else:
        g = subprocess.run(cmd, cwd=repo, env={"PYTHONPATH": str(Path(repo) / "src"), "PATH": "/usr/bin:/bin"},
                           capture_output=True, text=True)
        (HERE / "gen").mkdir(exist_ok=True)
        (HERE / "gen" / f"{sid}.log").write_text(g.stdout + g.stderr, encoding="utf-8")
        log["gen_rc"] = g.returncode
    if log["gen_rc"] != 0:
        log["error"] = "generation failed"
        return log
    man = json.loads((gen / "manifest.json").read_text(encoding="utf-8"))
    ours = {p["fleet"]: p for p in man["plans"] if p["label"] == "ours"}
    log["ours_no_candidates"] = sorted(f for f, p in ours.items() if p.get("no_candidates"))
    # 2. 평가 묶음(테스트 주간). 수요 칸은 날짜마다 한 번 만들고 망별 묶음에서 링크로 쓴다
    for _ in range(180):                                 # 범용 AI안 도로망(eval_municipal)이 다 생길 때까지 기다린다(최대 30분)
        if all((HERE / "ai" / a / "road_muni.json").exists() and (HERE / "ai" / a / "plan.json").exists() for a in AIS):
            break
        time.sleep(10)
    else:
        log["error"] = "AI 도로망 없음"
        return log
    st = None if rate is None else K.draw_strike(TEST, rate, draw)
    flows = []
    _, kst = K.scenario_rows(TEST, st, "strike" if st else "all", flows_out=flows)
    log["test_demand"] = kst
    b0 = HERE / "bundles" / sid
    nets = {"road": ROAD / "road.json", "road_muni": MUNI / "road_muni.json"}
    nets.update({a: HERE / "ai" / a / "road_muni.json" for a in AIS})
    plans = {"road": [("ours", f, Path(ours[f]["file"])) for f in fleets if f in ours and not ours[f].get("no_candidates")],
             "road_muni": [("muni", FIXED, MUNI / "plan.json")]}
    plans.update({a: [(a, FIXED, HERE / "ai" / a / "plan.json")] for a in AIS})
    passengers = {}
    for d in TEST_DATES:
        src = TEST / "bundles" / f"affected_{d}"
        base = json.loads((src / f"scenario_f{FIXED}.json").read_text(encoding="utf-8"))
        dem = b0 / "_demand" / f"{d}.csv"
        dem.parent.mkdir(parents=True, exist_ok=True)
        sc11 = D.read_scenario(src, FIXED)
        passengers[d] = K.write_demand_bins(flows, d, sc11, dem)
        for net, path in nets.items():
            t = b0 / net / "bundles" / f"strike_{d}"
            t.mkdir(parents=True, exist_ok=True)
            for name, target in (("demand_bins.csv", dem), ("travel_times.json", path)):
                # 시뮬레이터 입력 함수는 입력 파일이 run_inputs 폴더 안에 실제로 있어야 한다(링크 거절) — 복사한다
                shutil.copyfile(target, t / name)
            for f in sorted({fl for _, fl, _ in plans[net]}):
                sc = json.loads(json.dumps(base))
                tmpl = sc["fleet"]["vehicles"][0]
                sc["fleet"]["vehicles"] = [dict(tmpl, vehicle_id=f"V{i:03d}") for i in range(1, f + 1)]
                sc["fleet"]["authorized_count"] = f
                sc["scenario_id"] = f"{base['scenario_id']}-{sid}-f{f}"
                sio.write_bytes(t / f"scenario_f{f}.json", sio.json_bytes(sc))
                build_run_inputs(f"guro-test-{sid}-{d}-{net}-f{f}", t / f"scenario_f{f}.json", t / "demand_bins.csv",
                                 t / "travel_times.json", 42, t / f"run_inputs_f{f}.json")
    log["passengers"] = passengers
    # 3. 실행(망별 run_matrix) → 지표 → 큰 파일 정리
    rows = []
    for net, pl in plans.items():
        if not pl:
            continue
        out = HERE / "runs" / sid / net
        cmd = [sys.executable, str(Path(repo) / "experiments" / "guro_exp" / "run_matrix.py"), "--bundles",
               str(b0 / net / "bundles"), "--out", str(out), "--src", str(Path(repo) / "src"), "--jobs", "2",
               "--data-label", f"guro-test-{sid}-{net}"]
        for lab, f, p in pl:
            cmd += ["--plan", f"{lab}:{f}={p}"]
        r = subprocess.run(cmd, capture_output=True, text=True)
        (out / "run_matrix.log").write_text(r.stdout + r.stderr, encoding="utf-8")
        for lab, f, p in pl:
            for d in TEST_DATES:
                rd = out / lab / f"strike_{d}" / f"f{f}"
                row = {"sid": sid, "rate": rate, "draw": draw, "plan": lab, "fleet": f, "net": net, "date": d}
                try:
                    met = json.loads((rd / "run" / "metrics.json").read_text(encoding="utf-8"))
                    ver = json.loads((rd / "verifier_report.json").read_text(encoding="utf-8"))
                    row.update(served=met["M1"]["served"], total=met["M1"]["total"], rate_served=met["M1"]["rate"],
                               unserved=met["M2"]["unserved_by_reason"], refusals=met["M2"].get("refusal_events"),
                               wait=met.get("M3"), m07_pass=ver["pass"], m07_failed=ver["failed_checks"])
                except (OSError, KeyError, ValueError) as e:
                    row["error"] = f"{type(e).__name__}: {e}"
                rows.append(row)
                for big in ("events.jsonl", "realization.jsonl", "passenger_outcomes.csv"):
                    fp = rd / "run" / big
                    if fp.is_file():
                        fp.unlink()
    for f in fleets:                                    # 우리 안 후보가 없던 대수: 충족 0으로 기록
        if f in ours and ours[f].get("no_candidates"):
            for d in TEST_DATES:
                rows.append({"sid": sid, "rate": rate, "draw": draw, "plan": "ours", "fleet": f, "net": "road",
                             "date": d, "served": 0, "total": passengers[d], "rate_served": 0.0 if passengers[d] else None,
                             "no_candidates": True})
    assert b0.is_relative_to(HERE / "bundles")
    shutil.rmtree(b0)
    log["rows"] = rows
    log["seconds"] = round(time.time() - log["t0"], 1)
    done.parent.mkdir(exist_ok=True)
    done.write_text(json.dumps(log, ensure_ascii=False, indent=1), encoding="utf-8")
    return log


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("repo")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--rates", default="0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0")
    ap.add_argument("--draws", type=int, default=5)
    ap.add_argument("--only", default=None)
    a = ap.parse_args()
    rates = [float(x) for x in a.rates.split(",")]
    # 파업 100%는 뽑기와 무관하게 같은 집합이라 한 번만
    jobs = [(None, 0)] + [(r, d) for d in range(a.draws) for r in rates if not (r >= 1.0 and d > 0)]
    if a.only:
        keep = set(a.only.split(","))
        jobs = [j for j in jobs if sid_of(*j) in keep]
    if a.workers <= 1:
        # 병렬 풀 없이 순서대로(이 환경에서 병렬 풀 보조 프로세스가 64MB 메모리 한도 묶음에서 강제 종료됨).
        # 병렬은 이 스크립트를 --only 묶음별로 여러 번 따로 띄워서 한다.
        for r, d in jobs:
            try:
                res = run_one(a.repo, r, d)
                ok = sum(1 for x in res.get("rows", []) if x.get("m07_pass"))
                print(f"{sid_of(r, d)}: {res.get('error', 'ok')} · 실행 {len(res.get('rows', []))} · M07 pass {ok} · "
                      f"{res.get('seconds')}s", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"{sid_of(r, d)}: 예외 {type(e).__name__}: {e}", flush=True)
        jobs = []
    with ProcessPoolExecutor(max(1, a.workers)) as ex:
        futs = {ex.submit(run_one, a.repo, r, d): sid_of(r, d) for r, d in jobs}
        for fu in as_completed(futs):
            try:
                res = fu.result()
                ok = sum(1 for x in res.get("rows", []) if x.get("m07_pass"))
                print(f"{futs[fu]}: {res.get('error', 'ok')} · 실행 {len(res.get('rows', []))} · M07 pass {ok} · "
                      f"{res.get('seconds')}s", flush=True)
            except Exception as e:  # noqa: BLE001
                print(f"{futs[fu]}: 예외 {type(e).__name__}: {e}", flush=True)
    rows = []
    for p in sorted((HERE / "summary").glob("*.json")):
        rows += json.loads(p.read_text(encoding="utf-8")).get("rows", [])
    (HERE / "summary_all.json").write_text(json.dumps(rows, ensure_ascii=False), encoding="utf-8")
    print("rows", len(rows))


if __name__ == "__main__":
    main()
