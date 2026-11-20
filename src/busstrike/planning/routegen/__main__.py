"""후보 생성 + per_gu 배분 → 비교 입력 plan.json.

    python -m busstrike.planning.routegen --root <구 실험 묶음 폴더> --out <저장소 밖 폴더> [--fleets 11,13]
        [--tau 0.2 --radius-m 3000 --c0-min 40 --l-max 20,30 --max-stops 8|none --max-detour 1.6 ...]

맨 앞에서 실험 묶음인지 확인한다(`data.check_root(split="exp")`: 망 ID `<구>-buf2km-0610-v2`, README 실험 주간
10/26~30 문구, 묶음 날짜). 다르면 멈춘다. 테스트 묶음으로는 후보를 만들지 않는다.
`--fleets`·`--home-gu`를 주지 않으면 README의 대수("대수 11·13대")와 구 이름을 쓴다.

출력(`--out`)
- bundle_check.json: 잔여 수요 → 시뮬레이터 정수화 결과가 묶음 demand_bins.csv와 같은지(다르면 중단)
- candidates/<변형>.json: 후보 목록(route_id, kind, stops, C, 덮는 수요, origin …), 허브·군집·소요 시간
- plans/<plan_id>.json: 비교 입력(experiments/plans/schema.md) — 변형 4개 × 차량 조건 × {ours, sp}
- manifest.json: 안별 배분·근사 충족(κ=1)·초기해별 국소 탐색 결과·이동 종류별 시도/수락·파라미터·코드 커밋

배분 순서: 피더안(sp 풀)을 먼저 배분하고, 그 배분을 우리 안(ours 풀) 초기해 하나로 넣는다.
그래서 같은 후보 풀에서 우리 안의 근사 ≥ 피더안의 근사다(아니면 멈춘다). 필수 정류장이 있으면 비교는
사전식 (덮은 필수 정류장 수, 근사)다.

프로필(설계 v2 5절; 명시한 인자가 프로필 값보다 앞선다)
- `--profile v1`: 순회형 끔, 필수 정류장 끔, 고정 한도(편도 20·30분, 정차 8) → 기존 v1과 바이트 단위로 같은 plan.json.
- `--profile v2`(기본): 순환 운행만(편도 노선은 배분 후보·결과에 없다), 필수 정류장 켬(`--facilities` 필요),
  길이 규칙 N=2, 정차 상한 없음, 우회율 1.6(고리 전체 ÷ 2×폭), 닫는 공차 ≤ 300초, 길 건너 짝 100m.
  v1은 기존 결과와 바이트 비교(회귀)용이다. v1 안은 편도 + 빈 차 복귀라 운행안으로 쓰지 않는다.

v2 추가 출력
- plans/<plan_id>.plan_report.json: 노선별 형태(편도/순회형)·시간당 운행, 필수 정류장별 덮는 노선·시간당 운행,
  must_stop_infeasible과 정류장 목록, 설정 전체와 해시, 코드 커밋. plan.json 형식은 바꾸지 않는다.
- must_stops/: 이 묶음의 필수 정류장 계산 결과(`must_stops.for_bundle`).
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path

from busstrike.sim import io as sio

from . import allocate as A
from . import data as D
from . import generate as G
from . import must_stops as MS

PLAN_LABELS = {"ours": "01", "sp": "02"}   # plan_id에 출처를 드러내지 않는다(schema.md §1)


def _git(*args: str) -> str | None:
    try:
        p = subprocess.run(["git", *args], cwd=Path(__file__).resolve().parent, capture_output=True, text=True,
                           check=False)
    except OSError:
        return None
    return p.stdout.strip() if p.returncode == 0 else None


def _ints(text: str) -> tuple[int, ...]:
    return tuple(int(x) for x in text.split(","))


def _opt_int(text: str) -> int | None:
    return None if text.lower() in ("none", "inf", "0") else int(text)


UNSET = object()   # 프로필 값으로 채울 인자의 기본값(문자열이 아니어야 argparse가 type으로 바꾸지 않는다)

# 프로필 기본값. UNSET인 인자만 프로필 값으로 채운다.
PROFILES: dict[str, dict] = {
    "v1": {"loops": False, "must_stops": False, "length_n": None, "l_max": (20, 30), "max_stops": 8,
           "max_detour": 1.6, "endpoints": "all", "pair_radius_m": 50.0, "loop_detour": "direction",
           "close_max_s": None},
    # v2: 순환 운행만. 짝 100m·고리 전체 우회율은 지자체 셔틀안(2026-01, 25개 구) 모양을 근거로 정했다(설계 문서 7절).
    "v2": {"loops": True, "must_stops": True, "length_n": 2, "l_max": (20, 30), "max_stops": None,
           "max_detour": 1.6, "endpoints": "all", "pair_radius_m": 100.0, "loop_detour": "loop_total",
           "close_max_s": 300},
}


def _gtfs_depots(root: Path) -> frozenset[str]:
    """기점 후보 = 묶음 GTFS 운행편마다 첫 정류장(stop_sequence 최소)."""
    first: dict[str, tuple[int, str]] = {}
    for r in D._read_csv(root / "gtfs" / "stop_times.txt"):
        seq = int(r["stop_sequence"])
        cur = first.get(r["trip_id"])
        if cur is None or seq < cur[0]:
            first[r["trip_id"]] = (seq, r["stop_id"])
    return frozenset(s for _, s in first.values())


def _sha(obj) -> str:
    return hashlib.sha256(json.dumps(obj, ensure_ascii=False, sort_keys=True, default=list).encode()).hexdigest()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="python -m busstrike.planning.routegen")
    ap.add_argument("--root", required=True, type=Path, help="구 실험 묶음 폴더(README.md, network/, demand/, bundles/)")
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--fleets", default=None, type=_ints, help="기본: README의 대수")
    ap.add_argument("--home-gu", default=None, help="cross_gu 표시 기준 구. 기본: README의 구 이름")
    ap.add_argument("--variants", default=",".join(D.VARIANTS))
    d = G.Params()
    ap.add_argument("--tau", type=float, default=d.tau)
    ap.add_argument("--tau-step", type=float, default=d.tau_step)
    ap.add_argument("--radius-m", type=float, default=d.radius_m)
    ap.add_argument("--c0-min", type=int, default=d.c0_s // 60)
    ap.add_argument("--l-max", type=_ints, default=UNSET, help="편도 상한(분) 목록. 기본: 프로필(20,30)")
    ap.add_argument("--sectors", type=_ints, default=d.sectors)
    ap.add_argument("--max-stops", type=_opt_int, default=UNSET,
                    help="N_max(순회형은 방향마다). none = 검사하지 않음. 기본: 프로필(v1 8, v2 none)")
    ap.add_argument("--max-detour", type=float, default=UNSET, help="우회율 상한. 기본: 프로필(1.6)")
    ap.add_argument("--sp-top", type=int, default=d.sp_top)
    ap.add_argument("--jaccard", type=float, default=d.jaccard)
    ap.add_argument("--revision", type=int, default=1)
    ap.add_argument("--condition", default="base", help="실행 조건 이름(manifest에 적는다)")
    # v2
    ap.add_argument("--profile", choices=sorted(PROFILES), default="v2")
    ap.add_argument("--loops", action=argparse.BooleanOptionalAction, default=UNSET, help="순회형 후보(기본: 프로필)")
    ap.add_argument("--must-stops", action=argparse.BooleanOptionalAction, default=UNSET,
                    help="필수 정류장 후보·배분 제약(기본: 프로필)")
    ap.add_argument("--length-n", type=_opt_int, default=UNSET,
                    help="길이 규칙 N(운행 창 안에 N회). none = 편도 --l-max 고정 한도. 기본: 프로필(v1 none, v2 2)")
    ap.add_argument("--endpoints", choices=G.ENDPOINT_MODES, default=UNSET, help="끝점 후보 집합. 기본: 프로필(all)")
    ap.add_argument("--facilities", type=Path, default=None, help="facilities_seoul.csv(저장소 밖)")
    ap.add_argument("--must-radius-m", type=float, default=MS.MustConfig.radius_m)
    ap.add_argument("--must-kinds", default=None, help="쉼표 목록(kind 또는 type). 기본: 전부")
    ap.add_argument("--must-no-gg", action="store_true", help="필수 정류장 판정에서 경기 노선을 계속 운행으로 보지 않는다")
    ap.add_argument("--must-max-wait-min", type=int, default=20, help="필수 정류장 대기 한도(분) → 시간당 60/W회")
    ap.add_argument("--must-insert-radius-m", type=float, default=d.must_insert_radius_m)
    ap.add_argument("--pair-radius-m", type=float, default=UNSET, help="길 건너 짝 거리(이름 같음). 기본: 프로필(v2 100)")
    ap.add_argument("--return-detour", type=float, default=None,
                    help="순회형 오는 쪽 우회율 상한(--loop-detour direction일 때만). 기본: --max-detour")
    ap.add_argument("--loop-max-detour", type=float, default=None,
                    help="순회형 우회율 상한(망 경로 기준). 기본: --max-detour(1.6)")
    ap.add_argument("--loop-detour", choices=("direction", "loop_total"), default=UNSET,
                    help="순회형 우회율 재는 법. 기본: 프로필(v2 loop_total)")
    ap.add_argument("--close-max-s", type=_opt_int, default=UNSET,
                    help="순회형 닫는 공차(끝 → 기점) 상한(초). none = 없음. 기본: 프로필(v2 300)")
    ap.add_argument("--od-csv", type=Path, default=None,
                    help="정류장 → 정류장 수요(date,hour,origin_stop,dest_stop,flow). 주면 demand_flows 대신 쓴다")
    ap.add_argument("--scenario-variant", default="affected", help="--od-csv일 때 시나리오를 읽을 묶음 변형")
    args = ap.parse_args(argv)
    prof = PROFILES[args.profile]
    for k in ("loops", "must_stops", "endpoints", "length_n", "l_max", "max_stops", "max_detour", "pair_radius_m",
              "loop_detour", "close_max_s"):
        if getattr(args, k) is UNSET:
            setattr(args, k, prof[k])
    if args.profile == "v2" and not args.loops:
        raise SystemExit("v2는 순환 운행만 만든다 — 편도 노선안(--no-loops)은 없다")
    if args.must_stops and args.facilities is None:
        raise SystemExit("필수 정류장이 켜져 있다(--profile v2) — --facilities <facilities_seoul.csv>를 주거나 --no-must-stops")

    root, out = args.root.resolve(), args.out.resolve()
    root_check = D.check_root(root, "exp")
    if args.fleets is None:
        if not root_check["fleets"]:
            raise SystemExit("README에서 대수를 못 읽었다 — --fleets를 준다")
        args.fleets = tuple(root_check["fleets"])
    home_gu = args.home_gu or root_check["gu"]
    if not home_gu:
        raise SystemExit("README에서 구 이름을 못 읽었다 — --home-gu를 준다")
    (out / "candidates").mkdir(parents=True, exist_ok=True)
    (out / "plans").mkdir(parents=True, exist_ok=True)
    started = time.time()

    net = D.load_network(root)
    if args.od_csv is None:
        rows = D.load_flows(root)
        check = D.check_bundles(root, rows, args.fleets[0])
    else:
        rows, check = None, [{"skipped": "od-csv", "od_csv": str(args.od_csv)}]
    (out / "bundle_check.json").write_text(json.dumps(check, ensure_ascii=False, indent=1), encoding="utf-8")
    bad = [c["bundle"] for c in check if not c.get("match", True)]
    if bad:
        print(f"잔여 수요가 묶음 demand_bins.csv와 다르다: {bad}", file=sys.stderr)
        return 1

    must_res = None
    must_list: tuple[str, ...] = ()
    if args.must_stops:
        mcfg = MS.MustConfig(args.must_radius_m, tuple(args.must_kinds.split(",")) if args.must_kinds else None,
                             not args.must_no_gg)
        must_res = MS.for_bundle(args.facilities, root, home_gu, mcfg)
        MS.write(out / "must_stops", must_res)
        must_list = tuple(must_res["must_stops"])
    depots = _gtfs_depots(root) if args.endpoints == "stations+depots" else frozenset()

    params = G.Params(c0_s=args.c0_min * 60, radius_m=args.radius_m, tau=args.tau, tau_step=args.tau_step,
                      sectors=args.sectors, l_max_min=args.l_max, max_stops=args.max_stops,
                      max_detour=args.max_detour, sp_top=args.sp_top, jaccard=args.jaccard, home_gu=home_gu,
                      loops=args.loops, length_n=args.length_n, endpoints=args.endpoints, must_stops=must_list,
                      must_insert_radius_m=args.must_insert_radius_m, pair_radius_m=args.pair_radius_m,
                      return_detour=args.return_detour, loop_detour=args.loop_detour, close_max_s=args.close_max_s,
                      loop_only=args.loops, loop_max_detour=args.loop_max_detour)
    ap_ = A.AllocParams()
    settings = {"profile": args.profile, "params": asdict(params) | {"f_min": params.f_min},
                "alloc_params": asdict(ap_), "must_max_wait_s": args.must_max_wait_min * 60,
                "must_config": must_res["config"] if must_res else None,
                "facilities": str(args.facilities) if args.facilities else None,
                "facilities_sha256": sio.sha256_file(args.facilities) if args.facilities else None,
                "od_csv": str(args.od_csv) if args.od_csv else None, "fleets": list(args.fleets),
                "loop_only": params.loop_only,
                "variants": args.variants, "root": str(root), "network_snapshot_id": net.tt.network_snapshot_id}
    settings_sha = _sha(settings)
    manifest = {"condition": args.condition, "fleets": list(args.fleets), "root_check": root_check, "root": str(root), "readme_title": (root / "README.md").read_text(encoding="utf-8").splitlines()[0],
                "network_snapshot_id": net.tt.network_snapshot_id,
                "commit": _git("rev-parse", "HEAD"),
                "branch": _git("branch", "--show-current"),
                "tracked_changes": bool(_git("status", "--porcelain", "--untracked-files=no")),
                "code_sha256": {p.name: sio.sha256_file(p) for p in sorted(Path(__file__).resolve().parent.glob("*.py"))},
                "python": sys.version.split()[0],
                "params": asdict(params) | {"f_min": params.f_min}, "alloc_params": asdict(ap_),
                "bundle_check": {"bundles": len(check), "all_match": not bad}, "variants": {}, "plans": [],
                "profile": args.profile, "settings_sha256": settings_sha,
                "must_stops": ({"stops": list(must_list), "counts": must_res["counts"], "gu_code": must_res["gu_code"]}
                               if must_res else None)}

    variants = args.variants.split(",") if args.od_csv is None else ["od"]
    od_rows = D.load_od_csv(args.od_csv) if args.od_csv else None
    for variant in variants:
        t0 = time.time()
        if od_rows is None:
            demand = D.build_demand(rows, variant)
            sv = variant
        else:
            demand = D.build_demand_od(od_rows, net)
            sv = args.scenario_variant
        scen = {f: D.read_scenario(root / "bundles" / f"{sv}_{D.EXPECTED_DATES[0] if od_rows else demand.dates[0]}", f)
                for f in args.fleets}
        scen0 = scen[args.fleets[0]]
        policy = scen0.policy
        if policy.max_wait_s != ap_.wait_s or any(v.capacity != ap_.seat for v in scen0.vehicles):
            raise SystemExit("시나리오 정책(대기 한도·좌석)이 배분 파라미터와 다르다")
        res = G.generate(net, demand, scen0, params, depots)
        pools = G.pools(res.candidates, params.jaccard)
        t_gen = time.time() - t0
        vinfo = {
            "demand_total_5day_avg": round(demand.total(), 2),
            "hubs": {k: {"name": net.stations[k].name, "F": round(v, 2), "catchment": res.catchment_sizes[k]}
                     for k, v in res.hubs.items()},
            "overlaps": res.overlaps, "clusters": res.clusters, "dropped": res.dropped, "reasons": res.reasons,
            "reason_sets": res.reason_sets,
            "seconds": res.seconds | {"generate_total": round(t_gen, 2)},
            "candidates": len(res.candidates),
            "by_origin_kind": {}, "pools": {k: {"kept": len(v[0]), "removed_jaccard": v[1]} for k, v in pools.items()},
        }
        if params.loops or params.must_stops:
            vinfo["loop_stats"] = res.loop_stats
            vinfo["must_stats"] = res.must_stats
            vinfo["by_shape"] = {sh: sum(1 for c in res.candidates if c.shape == sh) for sh in ("oneway", "loop")}
        for c in res.candidates:
            key = f"{c.origin}/{c.kind}"
            vinfo["by_origin_kind"][key] = vinfo["by_origin_kind"].get(key, 0) + 1
        cand_doc = {"variant": variant, "params": manifest["params"], **vinfo,
                    "pool_members": {k: [c.route_id for c in v[0]] for k, v in pools.items()},
                    "candidates": [c.record() for c in res.candidates]}
        (out / "candidates" / f"{variant}.json").write_text(json.dumps(cand_doc, ensure_ascii=False, indent=1),
                                                             encoding="utf-8")
        by_id = {c.route_id: c for c in res.candidates}
        vinfo["alloc_seconds"] = {}
        # ours 풀은 sp 풀의 정차열을 모두 가져야 한다(같은 후보 풀 비교). 빠진 정차열은 sp 후보를 그대로 더한다.
        ours_by_stops = {c.stops: c.route_id for c in pools["ours"][0]}
        added = [c for c in pools["sp"][0] if c.stops not in ours_by_stops]
        ours_pool = sorted(pools["ours"][0] + added, key=lambda c: c.route_id)
        sp_to_ours = {c.route_id: ours_by_stops.get(c.stops, c.route_id) for c in pools["sp"][0]}
        vinfo["sp_routes_added_to_ours"] = [c.route_id for c in added]
        def _must_of(pool):
            if not must_list:
                return None
            return {s: tuple(c.route_id for c in pool if s in c.must) for s in must_list}
        models = {lab: A.Model([(c.route_id, c.cycle_s, c.cover_keys) for c in pool], demand.od, ap_,
                               _must_of(pool), args.must_max_wait_min * 60 if must_list else None)
                  for lab, pool in (("sp", pools["sp"][0]), ("ours", ours_pool))}
        for fleet in args.fleets:
            allocs = {}
            for label in ("sp", "ours"):
                model = models[label]
                starts = None
                if label == "ours":
                    starts = {"sp_plan": {sp_to_ours[r]: k for r, k in allocs["sp"]["n"].items()}}
                t1 = time.time()
                alloc = allocs[label] = A.allocate(model, fleet, starts)
                vinfo["alloc_seconds"][f"{label}_f{fleet}"] = round(time.time() - t1, 2)
                if label == "ours" and (model.covered(alloc["n"]), alloc["value"]) < (
                        models["sp"].covered(allocs["sp"]["n"]), allocs["sp"]["value"] - 1e-6):
                    raise SystemExit(f"{variant} f{fleet}: 우리 안 근사 {alloc['value']:.3f} < 피더안 {allocs['sp']['value']:.3f}")
                det = model.detail(alloc["n"])
                plan_id = f"plan-{PLAN_LABELS[label]}-{variant}-f{fleet}"
                routes = []
                route_map = {}
                for i, cid in enumerate(sorted(alloc["n"]), start=1):
                    rid = f"r{i:02d}"
                    route_map[rid] = cid
                    routes.append({"route_id": rid, "stops": list(by_id[cid].stops), "vehicles": alloc["n"][cid]})
                plan = {"plan_id": plan_id, "revision": args.revision,
                        "network_snapshot_id": net.tt.network_snapshot_id, "routes": routes}
                path = out / "plans" / f"{plan_id}.json"
                sio.write_bytes(path, sio.json_bytes(plan))
                manifest["plans"].append({
                    "plan_id": plan_id, "label": label, "variant": variant, "fleet": fleet, "file": str(path),
                    "sha256": sio.sha256_file(path),
                    "routes": {rid: {"candidate": cid, "vehicles": alloc["n"][cid], "kind": by_id[cid].kind,
                                     "origin": by_id[cid].origin, "cycle_s": by_id[cid].cycle_s,
                                     "approx_served_by_hour": [round(x, 4) for x in det["by_route"][cid]]}
                               for rid, cid in route_map.items()},
                    "approx_served": round(det["total"], 4), "approx_demand_covered": round(det["demand_covered"], 4),
                    "approx_cap_cut": round(det["cap_cut"], 4),
                    "value_greedy": round(alloc["value_greedy"], 4), "value_filled": round(alloc["value_filled"], 4),
                    "greedy": alloc["greedy"], "local_search": alloc["local_search"],
                    "chosen_start": alloc["chosen"], "move_stats": alloc["move_stats"],
                    "starts": {k: {"initial": round(v["initial"], 4), "final": round(v["final"], 4),
                                   "start": v["start"], "n": v["n"], "moves": v["moves"], "stats": v["stats"]}
                               for k, v in alloc["inits"].items()},
                    "pool_size": len(model.ids),
                    "demand_total_5day_avg": round(demand.total(), 4)})
                rep_routes = {}
                for rid, cid in route_map.items():
                    c = by_id[cid]
                    rep_routes[rid] = {"candidate": cid, "shape": c.shape, "kind": c.kind, "origin": c.origin,
                                       "vehicles": alloc["n"][cid], "cycle_s": c.cycle_s,
                                       "trips_per_hour": round(alloc["n"][cid] * 3600 / c.cycle_s, 4),
                                       "headway_s": round(c.cycle_s / alloc["n"][cid], 1),
                                       "n_stops": len(c.stops), "turn": c.turn, "detour": round(c.detour, 4),
                                       "must": list(c.must), "loop": c.loop_info}
                must_rep = {"enabled": False}
                if alloc.get("must"):
                    inv = {cid: rid for rid, cid in route_map.items()}
                    m = alloc["must"]
                    must_rep = {"enabled": True, "must_wait_s": m["must_wait_s"],
                                "need_trips_per_hour": m["need_trips_per_hour"],
                                "start_vehicles": m["start_vehicles"],
                                "stops": {s: dict(v, routes=[inv[r] for r in v["routes"]]) for s, v in m["stops"].items()},
                                "must_stop_infeasible": m["must_stop_infeasible"], "infeasible": m["infeasible"],
                                "milp": "없음(표준 라이브러리만, 솔버 미설치)"}
                report = {"plan_id": plan_id, "label": label, "variant": variant, "fleet": fleet,
                          "plan_sha256": sio.sha256_file(path), "profile": args.profile,
                          "shape_counts": {sh: sum(1 for r in rep_routes.values() if r["shape"] == sh)
                                           for sh in ("oneway", "loop")},
                          "routes": rep_routes, "must": must_rep,
                          "approx_served": round(det["total"], 4),
                          "settings": settings, "settings_sha256": settings_sha,
                          "commit": manifest["commit"], "branch": manifest["branch"],
                          "tracked_changes": manifest["tracked_changes"], "code_sha256": manifest["code_sha256"]}
                (out / "plans" / f"{plan_id}.plan_report.json").write_text(
                    json.dumps(report, ensure_ascii=False, indent=1), encoding="utf-8")
                manifest["plans"][-1]["plan_report"] = str(out / "plans" / f"{plan_id}.plan_report.json")
                if alloc.get("must"):
                    manifest["plans"][-1]["must_stop_infeasible"] = alloc["must"]["must_stop_infeasible"]
        manifest["variants"][variant] = vinfo
        print(f"{variant}: 허브 {len(res.hubs)} · 군집 {len(res.clusters)} · 후보 {len(res.candidates)} "
              f"(ours {len(pools['ours'][0])}, sp {len(pools['sp'][0])}) · {time.time() - t0:.1f}s", flush=True)
    manifest["wall_seconds"] = round(time.time() - started, 2)
    (out / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=1), encoding="utf-8")
    for p in manifest["plans"]:
        print(f"  {p['plan_id']}: 노선 {len(p['routes'])} · 대수 "
              f"{'/'.join(str(r['vehicles']) for r in p['routes'].values())} · 근사 충족 {p['approx_served']:.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
