"""routegen v2 단위 테스트: 필수 정류장 규칙, 순회형 노선, 필수 정류장 배분 제약, 결정론.

합성 망·합성 묶음만 쓴다(실제 시설·묶음 자료는 저장소에 넣지 않는다).
"""

from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

from busstrike.planning.routegen import allocate as A
from busstrike.planning.routegen import generate as G
from busstrike.planning.routegen import must_stops as MS
from busstrike.planning.routegen.cycle import route_cycle, single_route_plan
from busstrike.planning.routegen.data import Demand, Network, Station, Stop
from busstrike.planning.routegen.graph import Graph
from busstrike.planning.routegen.loop import LoopMaker, loop_violations, opposite_pairs
from busstrike.sim import io as sio
from busstrike.sim.adapters.comparison_plan import check_plan
from busstrike.sim.traveltime import TravelTimeTable
from tests.planning.test_routegen import DLON, LAT, _segments, scenario

# ---------------------------------------------------------------- 합성 왕복 도로
# 북쪽 N1–N4 → 역 access H, 역 egress E → 남쪽 S4–S1(N_i와 같은 이름, 약 22m 건너편), S1 → N1 짧은 연결.


def _st(sid: str, name: str, x: float, y: float = 0.0) -> Stop:
    return Stop(sid, name, LAT + y * DLON, 126.85 + x * DLON, "구로구")


def street_network(extra: list[tuple[str, str]] = ()) -> Network:
    stops = {f"N{i}": _st(f"N{i}", f"P{i}", i) for i in range(1, 5)}
    stops.update({f"S{i}": _st(f"S{i}", f"P{i}", i, -0.05) for i in range(1, 5)})
    stops["H"] = _st("H", "역앞", 5)
    stops["E"] = _st("E", "역건너", 5, -0.05)
    pairs = [("N1", "N2"), ("N2", "N3"), ("N3", "N4"), ("N4", "H"), ("H", "E"), ("E", "S4"), ("S4", "S3"),
             ("S3", "S2"), ("S2", "S1"), ("S1", "N1")] + list(extra)
    tt = TravelTimeTable.from_dict({"schema_version": 1, "network_snapshot_id": "syn-1", "valid_from_s": 21600,
                                    "valid_until_s": 43200, "segments": _segments(stops, pairs)})
    st = {"K": Station("K", "역", stops["H"].lat, stops["H"].lon + 0.0005, "H", "E")}
    return Network(tt, stops, st)


def street_demand() -> Demand:
    d = Demand("affected", ("20261026",))
    for s, v in (("N1", 30.0), ("N2", 20.0), ("N3", 10.0)):
        d.pair[("access", s, "K")] = [v, v, v, v]
        d.od[(s, "H")] = [v, v, v, v]
    for s, v in (("S2", 25.0), ("S1", 15.0)):
        d.pair[("egress", s, "K")] = [v, v, v, v]
        d.od[("E", s)] = [v, v, v, v]
    return d


class LoopTest(unittest.TestCase):
    def setUp(self):
        self.net = street_network()
        self.g = Graph(self.net, 10, 7200)
        self.od = {k: sum(v) for k, v in street_demand().od.items()}

    def test_pairs_same_name_within_radius(self):
        p = opposite_pairs(self.net, 50)
        self.assertEqual(p["N2"], "S2")
        self.assertEqual(p["S3"], "N3")
        self.assertNotIn("H", p)                       # 이름이 다르면 짝이 아니다
        self.assertEqual(opposite_pairs(self.net, 10), {})   # 22m > 10m

    def test_loop_uses_opposite_side_and_station_partner(self):
        mk = LoopMaker(self.net, self.g, self.od, 1.6, None, 7020)
        b = mk.build(("N1", "N2", "N3", "N4", "H"))
        self.assertIsNotNone(b)
        self.assertEqual(b.stops, ("N1", "N2", "N3", "N4", "H", "E", "S4", "S3", "S2", "S1"))
        self.assertEqual(b.turn, 4)
        self.assertEqual(len(set(b.stops)), len(b.stops))          # S2: 같은 ID 두 번 없음
        self.assertEqual(b.stops.count("H"), 1)                    # 회차 정류장은 한 번
        self.assertEqual(b.skipped, [])

    def test_loop_passes_converter_and_deadhead_is_short(self):
        mk = LoopMaker(self.net, self.g, self.od, 1.6, None, 7020)
        b = mk.build(("N1", "N2", "N3", "N4", "H"))
        sc = scenario(2)
        rep = check_plan(single_route_plan(b.stops, self.net.tt, sc), self.net.tt, sc)
        self.assertTrue(rep["ok"], rep["failures"])
        self.assertTrue(all(v == "pass" for v in rep["checks"].values()), rep["checks"])
        self.assertEqual(rep["routes"][0]["return_stops"], ["S1", "N1"])   # 공차 복귀 = 짧은 연결 하나
        cyc = route_cycle(b.stops, self.net.tt, sc)
        self.assertIsNotNone(cyc)

    def test_loop_cover_counts_both_directions_but_not_past_anchor(self):
        mk = LoopMaker(self.net, self.g, self.od, 1.6, None, 7020)
        b = mk.build(("N1", "N2", "N3", "N4", "H"))
        keys = set(G.covered_keys(b.stops, street_demand().od))
        self.assertEqual(keys, {("N1", "H"), ("N2", "H"), ("N3", "H"), ("E", "S2"), ("E", "S1")})
        # A23: 오는 쪽 S1에서 타서 기점을 넘어 N2로 가는 수요는 세지 않는다
        self.assertNotIn(("S1", "N2"), G.covered_keys(b.stops, {("S1", "N2"): [1, 1, 1, 1]}))

    def test_missing_pair_falls_back_to_shortest_path_with_demand(self):
        net = street_network()
        stops = dict(net.stops)
        stops["S3"] = _st("S3", "다른이름", 3, -0.05)              # N3의 짝이 없다
        net = Network(net.tt, stops, net.stations)
        g = Graph(net, 10, 7200)
        mk = LoopMaker(net, g, self.od, 1.6, None, 7020)
        b = mk.build(("N1", "N2", "N3", "N4", "H"), pool=["S3"])
        self.assertIn("N3", b.skipped)
        self.assertIn("S3", b.stops)                               # 최단경로로 지나며 들어온다
        self.assertEqual(b.stops[-1], "S1")

    def test_return_detour_limit(self):
        stops = ("N1", "N2", "N3", "N4", "H", "E", "S4", "S3", "S2", "S1")
        self.assertEqual(loop_violations(self.g, stops, 4, 1.6, None, 7020), [])
        self.assertIn("no_return", loop_violations(self.g, stops[:5], 4, 1.6, None, 7020))
        self.assertIn("length", loop_violations(self.g, stops, 4, 1.6, None, 60))
        self.assertIn("max_stops", loop_violations(self.g, stops, 4, 1.6, 4, 7020))
        self.assertIn("repeat", loop_violations(self.g, stops + ("N1",), 4, 1.6, None, 7020))


# ---------------------------------------------------------------- 필수 정류장 규칙

def _write(path: Path, header: list[str], rows: list[list]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


STEP = 0.003


def must_bundle(tmp: Path) -> Path:
    """정류장(116 구로 · 117 다른 구), GTFS 노선 R1(affected) · R2(unaffected), 마을 M1 · 경기 G1."""
    root = tmp / "b"
    pts = {"116000001": (0, 0), "116000002": (0, 1.0), "116000003": (0, 2.0), "116000004": (0, 3.0),
           "117000001": (0, 4.0), "116000005": (0, 5.0), "116000006": (0, 6.0)}     # 남북으로 약 333m 간격
    _write(root / "network" / "stops.csv", ["stop_id", "stop_name", "stop_lat", "stop_lon", "gu"],
           [[s, s, LAT + y * STEP, 126.85 + x, "다른구" if s.startswith("117") else "구로구"]
            for s, (x, y) in pts.items()])
    _write(root / "gtfs" / "routes.txt", ["route_id", "route_short_name", "route_type"], [["R1", "1", 3], ["R2", "2", 3]])
    _write(root / "gtfs" / "trips.txt", ["route_id", "service_id", "trip_id"], [["R1", "W", "t1"], ["R2", "W", "t2"]])
    _write(root / "gtfs" / "stop_times.txt", ["trip_id", "arrival_time", "departure_time", "stop_id", "stop_sequence"],
           [["t1", "07:00:00", "07:00:00", "116000001", 1], ["t1", "07:05:00", "07:05:00", "116000002", 2],
            ["t2", "07:00:00", "07:00:00", "116000003", 1]])
    _write(root / "registry" / "strike_route_registry.csv", ["route_id", "strike_state"],
           [["R1", "affected"], ["R2", "unaffected"]])
    _write(root / "network" / "absorbing_routes.csv", ["route_id", "route_name", "mode", "order_known", "stops",
                                                       "trips_0610"],
           [["M1", "마을", "maeul", "True", "116000005", "10"], ["G1", "경기", "gg", "True", "116000006", "10"]])
    return root


def fac(fid: str, lat_off: float, lon_off: float = 0.0) -> MS.Facility:
    return MS.Facility(fid, "medical", "병원", fid, "구로구", LAT + lat_off, 126.85 + lon_off)


class MustRuleTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = must_bundle(Path(self.tmp.name))
        self.stops = MS.load_stops(self.root)

    def tearDown(self):
        self.tmp.cleanup()

    def run_rule(self, facilities, include_gg=True):
        run = MS.running_stops(self.root, include_gg)["stops"]
        return MS.compute(facilities, self.stops, run, "116", MS.MustConfig(include_gg=include_gg))

    def test_running_stops(self):
        run = MS.running_stops(self.root)["stops"]
        self.assertEqual(run, {"116000003", "116000005", "116000006"})   # R2 + 마을 + 경기, R1(affected)는 아님
        self.assertNotIn("116000006", MS.running_stops(self.root, include_gg=False)["stops"])

    def test_served_by_running_route_is_not_must(self):
        res = self.run_rule([fac("f1", 2.0 * STEP)])                   # 116000003 바로 위(R2가 선다)
        self.assertEqual(res["by_facility"]["f1"]["reason"], "served_by_running")
        self.assertEqual(res["must_stops"], [])

    def test_no_stop_in_radius_is_out_of_network(self):
        res = self.run_rule([fac("f2", 0.0, 0.01)])                    # 약 880m 떨어짐
        self.assertEqual(res["by_facility"]["f2"]["reason"], "out_of_network")

    def test_nearest_candidate_and_tie_by_id(self):
        res = self.run_rule([fac("f3", 0.0004)])                        # 116000001(44m)만 반경 안, R1(affected)만 선다
        self.assertEqual(res["by_facility"]["f3"], {"reason": "must", "stop_id": "116000001", "distance_m": 44.5})
        near2 = self.run_rule([fac("f7", STEP - 0.0012)])               # 116000002(133m)가 116000001(200m 밖)보다 가깝다
        self.assertEqual(near2["by_facility"]["f7"]["stop_id"], "116000002")
        tie = self.run_rule([fac("f4", STEP / 2 * 1.0)])               # 둘 다 약 167m, 같은 거리
        self.assertEqual(tie["by_facility"]["f4"]["stop_id"], "116000001")

    def test_other_gu_prefix(self):
        res = self.run_rule([fac("f5", 4.0 * STEP)])                   # 117000001이 가장 가깝다
        self.assertEqual(res["by_facility"]["f5"]["reason"], "other_gu")

    def test_gg_exclusion_setting(self):
        f = [fac("f6", 6.0 * STEP)]                                     # 116000006: 경기 G1만 선다
        self.assertEqual(self.run_rule(f)["by_facility"]["f6"]["reason"], "served_by_running")
        self.assertEqual(self.run_rule(f, include_gg=False)["by_facility"]["f6"]["reason"], "must")

    def test_gu_code_from_stops(self):
        self.assertEqual(MS.gu_code_of(self.stops, "구로구"), "116")


# ---------------------------------------------------------------- 배분 제약

def must_model(must, cycles=None, demand=None):
    cycles = cycles or {"rA": 3600, "rB": 2400, "rC": 1200, "rD": 1200}
    keys = {"rA": (("a", "x"),), "rB": (("b", "x"),), "rC": (("c", "x"),), "rD": (("d", "x"),)}
    od = demand or {("a", "x"): [5, 5, 5, 5], ("b", "x"): [5, 5, 5, 5], ("c", "x"): [200, 200, 200, 200],
                    ("d", "x"): [150, 150, 150, 150]}
    return A.Model([(r, cycles[r], keys[r]) for r in sorted(cycles)], od, A.AllocParams(), must)


class MustAllocTest(unittest.TestCase):
    def test_constraint_holds_for_all_starts_and_local_search(self):
        m = must_model({"m1": ("rA",), "m2": ("rB", "rC")})
        res = A.allocate(m, 8)
        self.assertEqual(sum(res["n"].values()), 8)
        self.assertEqual(res["must"]["infeasible"], [])
        self.assertFalse(res["must"]["must_stop_infeasible"])
        for name, run in res["inits"].items():
            self.assertEqual(run["covered"], 2, name)
            self.assertGreaterEqual(run["n"].get("rA", 0), 3, name)          # 3600/1200 = 3대 이상
        # 시간당 운행 ≥ 3
        self.assertTrue(all(v["trips_per_hour"] >= 3 - 1e-9 for v in res["must"]["stops"].values()))

    def test_unconstrained_would_drop_must_route(self):
        free = A.allocate(must_model(None), 4)
        self.assertNotIn("rA", free["n"])                                     # 제약 없으면 rA는 안 연다
        held = A.allocate(must_model({"m1": ("rA",)}), 4)
        self.assertEqual(held["n"].get("rA"), 3)                              # 제약이 있으면 3대(시간당 3회)

    def test_infeasible_reports_and_maximizes_covered(self):
        m = must_model({"m1": ("rA",), "m2": ("rB",), "m3": ()})
        res = A.allocate(m, 2)                                                # rA 3대·rB 2대 필요, 2대뿐
        self.assertTrue(res["must"]["must_stop_infeasible"])
        self.assertEqual(res["must"]["infeasible"], ["m1", "m3"])            # m2(rB 2대)만 덮는다
        self.assertEqual(res["n"], {"rB": 2})

    def test_local_search_rejects_breaking_moves(self):
        m = must_model({"m1": ("rA",)})
        n, _ = A.local_search(m, 5, {"rA": 3, "rC": 2})
        self.assertGreaterEqual(n.get("rA", 0), 3)

    def test_milp_not_available_is_documented(self):
        self.assertIn("MILP", A.__doc__)


# ---------------------------------------------------------------- 생성 + 배분 결정론

def _plan_bytes(net, demand, sc, p) -> bytes:
    res = G.generate(net, demand, sc, p)
    pool = G.pools(res.candidates, p.jaccard)["ours"][0]
    must = {s: tuple(c.route_id for c in pool if s in c.must) for s in p.must_stops} or None
    model = A.Model([(c.route_id, c.cycle_s, c.cover_keys) for c in pool], demand.od, A.AllocParams(), must)
    alloc = A.allocate(model, len(sc.vehicles))
    by_id = {c.route_id: c for c in pool}
    routes = [{"route_id": f"r{i:02d}", "stops": list(by_id[c].stops), "vehicles": alloc["n"][c]}
              for i, c in enumerate(sorted(alloc["n"]), start=1)]
    return sio.json_bytes({"plan_id": "p", "revision": 1, "network_snapshot_id": "syn-1", "routes": routes})


class GenerateV2Test(unittest.TestCase):
    def params(self, **kw):
        base = dict(c0_s=144000, radius_m=5000, length_n=2, max_stops=None, loops=True, home_gu="구로구")
        base.update(kw)
        return G.Params(**base)

    def test_loops_and_must_candidates_and_determinism(self):
        net, dem, sc = street_network(), street_demand(), scenario(4)
        p = self.params(must_stops=("N3",))
        res = G.generate(net, dem, sc, p)
        shapes = {c.shape for c in res.candidates}
        self.assertEqual(shapes, {"oneway", "loop"})
        self.assertTrue(any("N3" in c.must for c in res.candidates))
        self.assertTrue(all(c.cycle_s <= (36000 - 21600) // 2 for c in res.candidates))   # 길이 규칙 N=2
        for c in res.candidates:                                                         # 변환기 통과
            self.assertTrue(check_plan(single_route_plan(c.stops, net.tt, sc), net.tt, sc)["ok"], c.stops)
        self.assertEqual(_plan_bytes(net, dem, sc, p), _plan_bytes(net, dem, sc, p))

    def test_must_feeder_ignores_detour_but_insert_respects_it(self):
        # M: 역에서 직선 220m, 망으로는 N1을 돌아 크게 우회(우회율 > 1.6)하는 필수 정류장
        net = street_network()
        stops = dict(net.stops)
        stops["M"] = _st("M", "필수", 5, 0.5)
        tt = TravelTimeTable.from_dict({"schema_version": 1, "network_snapshot_id": "syn-1", "valid_from_s": 21600,
                                        "valid_until_s": 43200, "segments": _segments(stops, [
                                            ("N1", "N2"), ("N2", "N3"), ("N3", "N4"), ("N4", "H"), ("H", "E"),
                                            ("E", "S4"), ("S4", "S3"), ("S3", "S2"), ("S2", "S1"), ("S1", "N1"),
                                            ("M", "N1"), ("N1", "M")])})
        net = Network(tt, stops, net.stations)
        res = G.generate(net, street_demand(), scenario(4), self.params(must_stops=("M",)))
        feeders = [c for c in res.candidates if c.origin == "must_feeder"]
        self.assertTrue(feeders)
        self.assertTrue(all(c.stops[0] == "M" for c in feeders if c.shape == "oneway"))
        self.assertGreater(max(c.detour for c in feeders), 1.6)
        self.assertTrue(all(c.detour <= 1.6 + 1e-9 for c in res.candidates if c.origin == "must_insert"))

    def test_endpoint_filter(self):
        net, dem, sc = street_network(), street_demand(), scenario(4)
        res = G.generate(net, dem, sc, self.params(endpoints="stations"))
        st = {"H", "E"}
        for c in res.candidates:
            self.assertTrue(set(G.terminals(c)) <= st, (c.stops, c.shape))

    def test_v1_defaults_make_no_loops(self):
        net, dem, sc = street_network(), street_demand(), scenario(4)
        res = G.generate(net, dem, sc, G.Params(c0_s=144000, radius_m=5000, home_gu="구로구"))
        self.assertEqual({c.shape for c in res.candidates}, {"oneway"})
        self.assertEqual(res.loop_stats, {})
        self.assertEqual(res.must_stats, {})


if __name__ == "__main__":
    unittest.main()
