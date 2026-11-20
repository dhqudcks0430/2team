"""routegen v2 스모크 노선안을 /lab 지도에서 보려고 만드는 실험실 폴더(routegen_v2/lab/<구>_v2smoke).

    python3 -I build_smoke_lab.py

- 배경 층(정류장·역·수요·버스 도로·경계·파업 노선·마을/경기 노선)은 lab_all/<구>/layers에서 복사한다.
- 노선 모양: lab_all/<구>/layers/segpaths.json(구간 → 도로 모양). 없는 구간은 정류장 직선으로 잇는다(개수를 meta.note에 적는다).
- 망은 묶음 원래 망(실험실 '원래 망')이다. 변형 affected, 흡수 '기존 흡수 판정'.
  mode를 sweep으로 두면 화면이 도로 망·정밀 흡수로 시작해 안이 안 뜬다. 그래서 mode = v2smoke(화면 기본 = 원래 망·흡수 제외 없음).
  이 모드에서는 안 고르기 칸이 숨어 처음 맞는 안(우리 안)만 보인다.
- 시뮬레이션: 스모크 10/28 하루(runs/<구>/<안>/affected_20261028/f<대수>). 5일 평균이 아니다.
- 지자체안: 노선만 보인다(pending). lab_all의 지자체안 숫자는 조건(정밀 흡수·5일 평균)이 달라 넣지 않는다.
- 필수 정류장은 노선 설명(hubs) 끝에 '필수 …'로, 못 덮은 필수 정류장은 meta.note에 적는다.
"""
import json
import shutil
import sys
from pathlib import Path

COND = sys.argv[1]                       # 조건 폴더 이름(A|B)
LABEL = {"A": "도로망 · 순회형 우회율 1.6", "B": "도로망 · 순회형 우회율 3.0"}.get(COND, COND)
HERE = Path(__file__).resolve().parent / COND
LAB_REPO = Path("/mnt/c/Users/ddonggojin/orca/projects/2team-lab")
W = Path(__file__).resolve().parents[2]
OUT = HERE.parent.parent / "lab"
GU = {"guro": "구로구", "gangseo": "강서구", "gangdong": "강동구"}
BUNDLE = "affected_20261028"
NAMES = {"ours": "우리 안 · v2(순회형·필수 정류장)", "sp": "피더안 · v2(최단경로 피더·필수 전용)"}


def sim(run_dir: Path):
    m = run_dir / "run" / "metrics.json"
    if not m.exists():
        return None
    met = json.loads(m.read_text(encoding="utf-8"))
    m1, m2 = met["M1"], met.get("M2", {})
    by = m2.get("unserved_by_reason") or m2.get("by_reason") or {}
    ver = json.loads((run_dir / "verifier_report.json").read_text(encoding="utf-8"))
    return {"days": 1, "rate": round(100 * m1["rate"], 2), "served": m1["served"], "total": m1["total"],
            "unserved": {k: by.get(k, 0) for k in ("no_connection", "service_end", "wait", "journey")},
            "refusals": m2.get("refusal_events") or m2.get("refusals"), "m07_pass": ver.get("pass")}


def concat(parts):
    out = []
    for p in parts:
        out.extend(p if not out else p[1:] if out[-1] == p[0] else p)
    return out


for slug, gu in GU.items():
    if not (HERE / slug / "manifest.json").exists() or not (HERE / "runs" / slug / "runs.json").exists():
        print(slug, "생성·시뮬레이션이 아직 없다 — 건너뜀")
        continue
    src = W / "lab_all" / slug / "layers"
    out = OUT / f"{slug}_v2{COND}"
    (out / "layers").mkdir(parents=True, exist_ok=True)
    for n in ("stops", "stations", "demand", "bus_roads", "boundaries", "strike_routes", "absorbing_routes"):
        shutil.copyfile(src / f"{n}.json", out / "layers" / f"{n}.json")
    seg = json.loads((src / "segpaths.json").read_text(encoding="utf-8"))
    rn = None
    osm = W / "osm" / f"{slug}_roads.json"
    if osm.exists() and "--osm" in sys.argv:
        # 노선 모양을 도로망(stage1c)과 같은 규칙으로 다시 만든 RoadNet 경로로 그린다(3b 실험실과 같은 방식)
        sys.path[:0] = [str(LAB_REPO / "src"), str(LAB_REPO / "experiments/routegen_lab")]
        from roadnet import RoadNet
        rn = RoadNet(osm, W / "zip_v4" / "region" / f"{slug}_0610_v4")
        rn.build()
        same = json.dumps(rn.road_doc, sort_keys=True) == json.dumps(
            json.loads((W / "review1119" / "stage1c" / "nets" / slug / "road.json").read_text(encoding="utf-8")),
            sort_keys=True)
        print(slug, "stage1c 도로망과 같은가:", same, flush=True)
        if not same:
            rn = None
    stops = json.loads((src / "stops.json").read_text(encoding="utf-8"))
    st_name = {}
    for s in json.loads((src / "stations.json").read_text(encoding="utf-8")):
        for x in (s["access"], s["egress"]):
            if x:
                st_name.setdefault(x, s["name"])
    rg = HERE / slug
    man = json.loads((rg / "manifest.json").read_text(encoding="utf-8"))
    must_names = {}
    must_csv = rg / "must_stops" / "must_stops.csv"
    if must_csv.exists():
        for line in must_csv.read_text(encoding="utf-8").splitlines()[1:]:
            sid, _, _, _, name, _ = line.split(",")
            must_names.setdefault(sid, []).append(name)
    straight = 0
    results, infeasible = [], {}

    def coords(seq):
        parts = []
        for a, b in zip(seq, seq[1:]):
            c = rn.path_any(a, b) if rn is not None else seg.get(f"{a}>{b}")
            if c is None:
                c = [stops[a][:2], stops[b][:2]] if a in stops and b in stops else []
                straight_local[0] += 1
            parts.append(c)
        return concat([p for p in parts if p])

    straight_local = [0]
    cand = {c["route_id"]: c for c in json.loads((rg / "candidates" / "affected.json").read_text(encoding="utf-8"))["candidates"]}
    for p in sorted(man["plans"], key=lambda x: (x["label"] != "ours", x["fleet"])):
        rep = json.loads(Path(p["plan_report"]).read_text(encoding="utf-8"))
        plan = json.loads(Path(p["file"]).read_text(encoding="utf-8"))
        routes = []
        for r in plan["routes"]:
            info = rep["routes"][r["route_id"]]
            hubs = list(dict.fromkeys(st_name[s] for s in r["stops"] if s in st_name))
            label = ["순회형" if info["shape"] == "loop" else "편도"] + hubs
            label += [f"필수 {'/'.join(must_names.get(s, [s]))}" for s in info["must"]]
            kind = "loop" if info["shape"] == "loop" else info["kind"]
            routes.append({"id": r["route_id"], "stops": r["stops"], "vehicles": r["vehicles"], "kind": kind,
                           "origin": info["origin"], "cycle_s": info["cycle_s"], "hubs": label,
                           "approx": round(p["routes"][r["route_id"]]["approx_served_by_hour"] and
                                           sum(p["routes"][r["route_id"]]["approx_served_by_hour"]), 1),
                           # 순환: 끝 → 기점 닫는 구간(변환기 공차 경로)까지 이어 그려 한 바퀴 고리로 보이게 한다
                           "turn": info["turn"], "close_stops": cand[info["candidate"]]["return_stops"],
                           "coords": coords(list(r["stops"]) + cand[info["candidate"]]["return_stops"][1:])})
        if rep["must"].get("enabled") and rep["must"]["infeasible"]:
            infeasible[f"{p['label']}-f{p['fleet']}"] = rep["must"]["infeasible"]
        exp_ = sim(HERE / "runs" / slug / p["label"] / BUNDLE / f"f{p['fleet']}")
        results.append({"net": "road", "variant": "affected", "run_variant": "affected", "config": p["label"],
                        "config_name": NAMES[p["label"]], "absorb": "bundle", "fleet": p["fleet"], "cover": False,
                        "covered_groups": [], "approx": p["approx_served"], "pool_cover": None,
                        "demand_total": p["demand_total_5day_avg"], "routes": routes, "schedule_date": None,
                        "schedule": None, "exp": exp_, "test": None})
    (out / "layers" / "results.json").write_text(json.dumps(results, ensure_ascii=False), encoding="utf-8")
    muni = json.loads((src / "municipal.json").read_text(encoding="utf-8"))
    if muni:
        muni = {k: muni[k] for k in ("fleet", "plan_vehicles", "routes", "dropped_stops", "dropped_routes",
                                     "added_segments") if k in muni}
        muni["pending"] = True
    (out / "layers" / "municipal.json").write_text(json.dumps(muni, ensure_ascii=False), encoding="utf-8")
    meta = json.loads((src / "meta.json").read_text(encoding="utf-8"))
    must = man.get("must_stops") or {}
    note = (f"routegen v2 스모크 {COND}({LABEL}) · 순환 노선만 · 코드 {str(man['commit'])[:7]} · 망 = 버스 통행 가능 도로망(stage1c) · affected · 시뮬레이션 10/28 하루(5일 평균 아님) · "
            f"필수 정류장 {len(must.get('stops', []))}곳"
            + (f" · 못 덮음 {infeasible}" if infeasible else "")
            + (f" · 도로 모양 없는 구간 {straight_local[0]}개는 직선" if straight_local[0] else "")
            + " · 지자체안은 노선만(숫자는 조건이 달라 뺌)")
    meta.update({"gu": f"{gu} · v2 스모크 {COND}({LABEL})", "gu_key": f"{slug}_v2{COND}", "fleets": sorted({r['fleet'] for r in results}),
                 "variants": ["affected"], "nets": ["road"], "mode": "v2smoke", "results": True,
                 "region_root": str(W / "zip_v4" / "region" / f"{slug}_0610_v4"),
                 "configs": {k: {"name": v} for k, v in NAMES.items()}, "note": note})
    (out / "layers" / "meta.json").write_text(json.dumps(meta, ensure_ascii=False), encoding="utf-8")
    print(slug, [(r["config"], r["fleet"], len(r["routes"]), r["exp"] and r["exp"]["rate"]) for r in results], note)
