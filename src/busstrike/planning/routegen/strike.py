"""파업 시나리오 수요 — 파업 노선 집합 S를 받아 셔틀 대상 수요(정류장 → 정류장)를 만든다.

생성기를 특정 파업(2026-01 전면 파업)에 묶지 않고, 어떤 파업 비율·노선 조합에서도 같은 규칙으로 돌게 한다.

정의(구 묶음 하나, 날짜·시간·종류·정류장·역 행마다)
- 대상 행: 걸어서 못 감(`walkable == False`), 역 지정 정류장 있음.
- 파업 몫: flow_S = flow × frac_S(타는 정류장, 시간, 날짜)
  - flow = 묶음의 파업 노선(등록부 affected 전체) 승객(`demand_flows.flow`).
  - frac_S = Σ_{r∈S} 승차 ÷ Σ_{r∈affected} 승차(`demand/stop_route_boardings`, 타는 정류장 = access는 출발 정류장,
    egress는 역 정류장). S = affected 전체면 1이라 묶음 수요와 같다. 그 칸에 affected 승차 기록이 없으면 |S| ÷ |affected|.
- 흡수(운행 중인 노선으로 갈 수 있으면 셔틀 대상이 아니다): 다음 중 하나면 그 행을 뺀다.
  - 운행 중인 묶음 GTFS 노선(S 밖의 모든 노선: 파업 안 한 시내·미확인·마을) 중 한 운행편이 타는 정류장 → 내리는 정류장
    순서로 지난다.
  - 경기버스가 닿는다(`gg_reach`, 경기버스는 파업하지 않는다).
  정원(흡수량 상한)은 보지 않는다. 모든 파업 비율에 같은 규칙을 쓴다(그래서 100%에서도 파업 안 한 시내버스로 갈 수 있는
  승객이 빠져 묶음의 affected 변형보다 조금 작다).
- 대조군(파업 없음, mode="all"): flow_total(모든 버스 노선 승객, 걸어서 못 감)을 흡수 없이 그대로 쓴다.

결과는 `data.OdRow` 목록(정류장 → 정류장: access는 정류장 → 역 access 정류장, egress는 역 egress 정류장 → 정류장)이고
`data.build_demand_od`로 생성기 수요가 된다. 시뮬레이터 수요(demand_bins)는 `scenario_flows`의 DemandFlow로 만든다.
"""

from __future__ import annotations

import csv
import gzip
import random
from dataclasses import dataclass
from pathlib import Path

from .data import OdRow, _read_csv


@dataclass(frozen=True)
class StrikeSet:
    routes: frozenset[str]          # 파업 노선(묶음 GTFS route_id)
    eligible: tuple[str, ...]       # 파업 대상 후보(등록부 affected) — 비율의 분모
    rate: float                     # 명목 파업 비율(|routes| ÷ |eligible|)
    draw: int | None = None         # 무작위 뽑기 번호(재현용)
    seed: int | None = None


def eligible_routes(root: Path) -> tuple[str, ...]:
    """파업 대상 후보 = 묶음 등록부 strike_state == affected 이면서 묶음 GTFS에 있는 노선(ID 순)."""
    root = Path(root)
    gtfs = {r["route_id"] for r in _read_csv(root / "gtfs" / "routes.txt")}
    return tuple(sorted(r["route_id"] for r in _read_csv(root / "registry" / "strike_route_registry.csv")
                        if r["strike_state"] == "affected" and r["route_id"] in gtfs))


def draw_strike(root: Path, rate: float, draw: int, seed: int = 20261121) -> StrikeSet:
    """파업 비율 rate(0~1)로 후보 중 round(rate × n)개를 무작위로 뽑는다. (seed, rate, draw)가 같으면 같은 집합."""
    elig = eligible_routes(root)
    k = round(rate * len(elig))
    rng = random.Random(f"{seed}:{rate:.4f}:{draw}")
    chosen = frozenset(rng.sample(elig, k)) if k else frozenset()
    return StrikeSet(chosen, elig, rate, draw, seed)


def _route_orders(root: Path) -> dict[str, list[tuple[str, ...]]]:
    """노선 → 운행편별 정류장 순서(묶음 GTFS). 같은 순서는 한 번만."""
    root = Path(root)
    trip_route = {r["trip_id"]: r["route_id"] for r in _read_csv(root / "gtfs" / "trips.txt")}
    seqs: dict[str, list[tuple[int, str]]] = {}
    with open(root / "gtfs" / "stop_times.txt", encoding="utf-8", newline="") as f:
        for r in csv.DictReader(f):
            seqs.setdefault(r["trip_id"], []).append((int(r["stop_sequence"]), r["stop_id"]))
    out: dict[str, set[tuple[str, ...]]] = {}
    for t, s in seqs.items():
        rid = trip_route.get(t)
        if rid:
            out.setdefault(rid, set()).add(tuple(x for _, x in sorted(s)))
    return {r: sorted(v) for r, v in out.items()}


class ReachIndex:
    """(타는 정류장, 내리는 정류장) → 그 순서로 지나는 노선 집합."""

    def __init__(self, root: Path):
        self.orders = _route_orders(root)
        self._pos: dict[str, dict[str, list[tuple[int, int]]]] = {}   # 정류장 → 노선 → [(운행편 번호, 위치)]
        for rid, trips in self.orders.items():
            for ti, seq in enumerate(trips):
                for i, s in enumerate(seq):
                    self._pos.setdefault(s, {}).setdefault(rid, []).append((ti, i))
        self._cache: dict[tuple[str, str], frozenset[str]] = {}

    def routes(self, a: str, b: str) -> frozenset[str]:
        key = (a, b)
        got = self._cache.get(key)
        if got is not None:
            return got
        pa, pb = self._pos.get(a, {}), self._pos.get(b, {})
        hit = set()
        for rid in pa.keys() & pb.keys():
            ib = {}
            for ti, i in pb[rid]:
                ib.setdefault(ti, []).append(i)
            if any(any(j > i for j in ib.get(ti, ())) for ti, i in pa[rid]):
                hit.add(rid)
        got = self._cache[key] = frozenset(hit)
        return got


def _boardings(root: Path) -> dict[tuple[str, str, int], dict[str, float]]:
    """(날짜, 정류장, 시간) → {노선: 승차}(affected 노선만)."""
    out: dict[tuple[str, str, int], dict[str, float]] = {}
    with gzip.open(Path(root) / "demand" / "stop_route_boardings.csv.gz", "rt", encoding="utf-8", newline="") as f:
        for r in csv.DictReader(f):
            if r["strike_state"] != "affected":
                continue
            b = float(r["boardings"])
            if b > 0:
                d = out.setdefault((r["date"], r["stop_id"], int(r["hour"])), {})
                d[r["route_id"]] = d.get(r["route_id"], 0.0) + b
    return out


def scenario_rows(root: Path, strike: StrikeSet | None, mode: str = "strike",
                  reach: ReachIndex | None = None, flows_out: list | None = None) -> tuple[list[OdRow], dict]:
    """셔틀 대상 수요(정류장 → 정류장)와 집계. mode = strike(파업 S) | all(대조군: 파업 없음, 모든 버스 승객).

    flows_out(목록)을 주면 시뮬레이터 실현기 입력용 (날짜, 종류, 정류장, 역 정류장, 시간, 유량)도 거기에 넣는다.
    """
    root = Path(root)
    if mode not in ("strike", "all"):
        raise ValueError(f"mode는 strike 또는 all: {mode}")
    reach = reach or ReachIndex(root)
    all_gtfs = set(reach.orders)
    running = all_gtfs - set(strike.routes) if strike else all_gtfs
    board = _boardings(root) if mode == "strike" else {}
    fallback = (len(strike.routes) / len(strike.eligible)) if (strike and strike.eligible) else 0.0
    rows: list[OdRow] = []
    stats = {"rows_in": 0, "flow_bundle": 0.0, "flow_strike_share": 0.0, "absorbed_running_gtfs": 0.0,
             "absorbed_gg": 0.0, "flow_out": 0.0, "fallback_cells": 0}
    with gzip.open(root / "demand" / "demand_flows.csv.gz", "rt", encoding="utf-8", newline="") as f:
        for r in csv.DictReader(f):
            if r["walkable"] != "False" or not r["station_stop_id"]:
                continue
            stats["rows_in"] += 1
            kind, s, ss = r["kind"], r["stop_id"], r["station_stop_id"]
            o, d = (s, ss) if kind == "access" else (ss, s)
            if mode == "all":
                v = float(r["flow_total"])
                if v > 0:
                    rows.append(OdRow(r["date"], int(r["hour"]), o, d, v))
                    if flows_out is not None:
                        flows_out.append((r["date"], kind, s, ss, int(r["hour"]), v))
                    stats["flow_out"] += v
                continue
            base = float(r["flow"])
            stats["flow_bundle"] += base
            if base <= 0 or not strike.routes:
                continue
            cell = board.get((r["date"], o, int(r["hour"])))
            if cell:
                tot = sum(cell.values())
                frac = sum(v for k, v in cell.items() if k in strike.routes) / tot
            else:
                frac = fallback
                stats["fallback_cells"] += 1
            v = base * frac
            if v <= 0:
                continue
            stats["flow_strike_share"] += v
            if reach.routes(o, d) & running:
                stats["absorbed_running_gtfs"] += v
                continue
            if r["gg_reach"] == "True":
                stats["absorbed_gg"] += v
                continue
            rows.append(OdRow(r["date"], int(r["hour"]), o, d, v))
            if flows_out is not None:
                flows_out.append((r["date"], kind, s, ss, int(r["hour"]), v))
            stats["flow_out"] += v
    dates = sorted({x.date for x in rows}) or [None]
    nd = max(1, len({x.date for x in rows}))
    stats = {k: (round(v / nd, 2) if isinstance(v, float) else v) for k, v in stats.items()}
    stats["per_day"] = True
    stats["dates"] = dates
    return rows, stats


def write_od_csv(path: Path, rows: list[OdRow]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["date", "hour", "origin_stop", "dest_stop", "flow"])
        for x in sorted(rows, key=lambda z: (z.date, z.hour, z.origin_stop, z.dest_stop)):
            w.writerow([x.date, x.hour, x.origin_stop, x.dest_stop, f"{x.flow:.6f}"])


def write_demand_bins(flows: list, date: str, scenario, path: Path, seed: int = 42, bin_s: int = 3600) -> int:
    """그 날짜의 시나리오 유량을 시뮬레이터 실현기(realize_bins, 묶음과 같은 seed·칸)로 demand_bins.csv에 쓴다. 승객 수를 돌려준다."""
    from busstrike.sim.arrivals import realize_bins
    from busstrike.sim.arrivals import write_demand_bins as _write
    from busstrike.sim.models import DemandFlow
    rows = [DemandFlow(k, s, ss, h * 3600, f"{v:.6f}") for d, k, s, ss, h, v in flows if d == date and v > 0]
    bins = realize_bins(rows, scenario.policy, scenario.window, seed, bin_s)
    _write(bins, Path(path))
    return sum(b.passenger_count for b in bins)
