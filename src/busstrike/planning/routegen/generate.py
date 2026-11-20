"""후보 노선 생성(설계 v1 4절). 구 하나·결정론.

1. 허브: F_k = Σ_s r(s↔k)(access+egress, 4시간 합, 5일 평균) ≥ F_min = 2 × 좌석 × (240 ÷ C₀)
2. 권역 K_k: 허브 반경 R 안에서 r(s↔k) > 0인 정류장(중복 소속 허용)
3. 겹침 병합: O_ij = Σ_{s∈K_i∩K_j}(r(s↔i)+r(s↔j)) / min(F_i, F_j) ≥ τ → union-find.
   군집이 역 3개를 넘으면 그 군집만 τ를 τ_step씩 올려 다시 나눈다. 구 경계를 넘으면 cross_gu.
4. 섹터 분할 + 최선 삽입: 패턴·m·L_max마다 섹터별 seed = argmax v_s × dist(s, 중심),
   삽입 점수 = 증가 덮개 수요 ÷ 증가 편도 시간. 제약(편도·정차 수·우회율·S2) 위반 삽입은 버린다. 끝에 2-opt 1회.
5. 추가 후보: 허브·방향마다 수요 상위 seed의 최단경로 피더(삽입 휴리스틱의 비교 기준).
6. 패턴: 회송 피더(access: 정류장들 → 역 access 정류장, egress: 역 egress 정류장 → 정류장들),
   편도 연결(역 i egress 정류장 → 정류장들 → 역 j access 정류장).
7. 후보 처리: 덮개(노선 위 o가 d보다 앞인 OD), 주기(`cycle.route_cycle`), 중복 제거(Jaccard).

v2(설정으로 켠다. 기본값은 모두 v1과 같은 동작 — `--profile v1`이 기존 plan.json을 바이트 단위로 그대로 낸다)
- loops: 편도 후보마다 순회형(기점 → 회차 → 기점, `loop.LoopMaker`) 버전을 같이 만든다. kind = loop_<편도 kind>.
- loop_only: 결과 후보(배분 풀)를 순회형만 남긴다. 편도 정차열은 순회형을 만드는 출발점으로만 쓴다.
  셔틀은 순환 운행만 한다(편도 + 빈 차 복귀는 없다) — v2 프로필은 늘 켠다.
- length_n: 길이 상한 = 운행 창 안에 N회 이상 돌 수 있는 길이. 생성 중에는 영업 시간 ≤ 창/N − 회차,
  마지막에 시뮬레이터 주기 ≤ 창/N. 주면 l_max_min(편도 20·30분) 대신 쓴다.
- must_stops: 필수 정류장(`must_stops`)마다 ① 우회율 한도 안에서 기존 후보에 끼운 버전(origin must_insert),
  ② 필수 정류장 → 가까운 역 access 정류장 전용 편도·순회형(origin must_feeder, 피더안 풀에도 넣는다).
  전용 후보의 역 = 직선거리로 가까운 must_feeder_stations개 중 망 최단 시간이 가장 짧은 역(동률이면 직선거리·역 ID).
  변환기를 못 지나는 역(역 → 필수 정류장 공차 경로 없음, RETURN)은 건너뛰고 다음 역을 쓴다.
  전용 후보는 우회율을 보지 않는다(길이 규칙·S2·정차 상한만). 필수 정류장은 대개 망에서 갈래가 하나뿐인 곳이라
  가까운 역까지도 망 경로가 직선의 3~20배다(v4 강서 실측).
- endpoints: 노선 끝점(편도 첫·끝, 순회형 기점·회차)을 끝점 후보 집합 안으로 제한(all | stations | stations+depots).
"""

from __future__ import annotations

import math
import time
from dataclasses import asdict, dataclass, field

from busstrike.sim.models import Scenario

from .cycle import route_cycle
from .data import Demand, Network, haversine_m
from .graph import Graph, Limits
from .loop import LoopMaker, loop_detour, loop_violations

ENDPOINT_MODES = ("all", "stations", "stations+depots")


@dataclass(frozen=True)
class Params:
    seat: int = 45
    c0_s: int = 2400                 # C₀ = 40분
    horizon_s: int = 4 * 3600        # 06~10시 240분
    radius_m: float = 3000.0         # R
    tau: float = 0.2
    tau_step: float = 0.05
    cluster_max: int = 3
    sectors: tuple[int, ...] = (2, 3, 4)
    l_max_min: tuple[int, ...] = (20, 30)
    max_stops: int | None = 8        # N_max. None = 정차 수를 검사하지 않음(편도 L_max·우회율만)
    max_detour: float = 1.6
    sp_top: int = 15
    jaccard: float = 0.8
    home_gu: str = "구로구"
    # v2(기본값 = v1 동작)
    loops: bool = False              # 순회형 후보를 같이 만든다
    length_n: int | None = None      # 길이 규칙 N(운행 창 안에 N회). None = l_max_min 고정 한도
    endpoints: str = "all"           # 끝점 후보 집합: all | stations | stations+depots
    must_stops: tuple[str, ...] = ()  # 필수 정류장(비면 필수 정류장 후보를 만들지 않는다)
    must_insert_radius_m: float = 600.0   # 이 거리 안에 정류장이 있는 후보에만 필수 정류장을 끼워 본다
    must_feeder_stations: int = 3    # 전용 후보: 가까운 역 몇 개까지 시도
    pair_radius_m: float = 50.0      # 길 건너 짝: 이름이 같고 이 거리 안
    return_detour: float | None = None   # 순회형 오는 쪽 우회율 상한(None = max_detour, direction 방식만)
    loop_detour: str = "direction"   # 순회형 우회율: direction(방향마다) | loop_total(고리 전체 ÷ 2×폭)
    close_max_s: int | None = None   # 순회형 닫는 공차(끝 → 기점) 상한(초). None = 없음
    loop_only: bool = False          # 결과 후보를 순회형만 남긴다(편도는 순회형을 만드는 출발점으로만 쓴다)
    loop_max_detour: float | None = None   # 순회형 우회율 상한(None = max_detour). 망 경로로 재므로 지자체 고리도 크게 나온다
    loop_insert_top: int = 30        # 순회형 오는 쪽에 끼울 수요 정류장 후보 수(값 큰 순)

    @property
    def f_min(self) -> float:
        return 2 * self.seat * (self.horizon_s / self.c0_s)


@dataclass
class Candidate:
    route_id: str
    kind: str                         # access | egress | connect
    stops: tuple[str, ...]
    origin: str                       # generated | shortest_path_feeder
    hubs: tuple[str, ...]             # 패턴의 역(access/egress: 1개, connect: i, j)
    cluster: int | None
    made_by: tuple[str, ...]          # 이 정차열을 낸 (m, L_max) 조합 또는 "sp:seed"
    oneway_s: int = 0
    length_m: int = 0
    detour: float = 0.0
    n_stops: int = 0                  # 역 지정 정류장을 뺀 정차 수
    cycle_s: int = 0
    return_stops: tuple[str, ...] = ()
    cover: float = 0.0                # 덮는 수요(4시간 합, 5일 평균)
    cover_keys: tuple[tuple[str, str], ...] = ()
    cross_gu: bool = False
    shape: str = "oneway"             # oneway | loop
    turn: int | None = None           # 순회형 회차 정류장 인덱스
    loop_info: dict | None = None     # 순회형 짝·건너뜀·끼움·공차(보고용)
    must: tuple[str, ...] = ()        # 노선이 지나는 필수 정류장

    def record(self) -> dict:
        d = asdict(self)
        d["stops"] = list(self.stops)
        d["return_stops"] = list(self.return_stops)
        d["must"] = list(self.must)
        d["cover_keys"] = [list(k) for k in self.cover_keys]
        d["cover"] = round(self.cover, 4)
        d["detour"] = round(self.detour, 4)
        return d


# ---------------------------------------------------------------- 덮개

def covered_keys(stops: tuple[str, ...], od: dict) -> list[tuple[str, str]]:
    """노선이 덮는 OD 키: 노선 위에서 출발 정류장이 도착 정류장보다 앞.

    access 키 (s, 역 access 정류장)는 s가 그 정류장보다 앞일 때, egress 키 (역 egress 정류장, s)는
    반대일 때 덮는다. 시뮬레이터가 승객을 태우는 조건(같은 영업 leg에서 o 다음에 d)과 같다.
    """
    out = []
    n = len(stops)
    for a in range(n):
        for b in range(a + 1, n):
            k = (stops[a], stops[b])
            if k in od:
                out.append(k)
    return out


def cover_value(stops: tuple[str, ...], od_total: dict[tuple[str, str], float]) -> float:
    n = len(stops)
    v = 0.0
    for a in range(n):
        sa = stops[a]
        for b in range(a + 1, n):
            v += od_total.get((sa, stops[b]), 0.0)
    return v


# ---------------------------------------------------------------- 1~3 허브·권역·병합

def hubs_of(demand: Demand, net: Network, p: Params) -> dict[str, float]:
    F = demand.station_total()
    return {k: F[k] for k in sorted(F)
            if F[k] >= p.f_min and k in net.stations
            and (net.stations[k].access_stop or net.stations[k].egress_stop)}


def catchments(demand: Demand, net: Network, hubs: dict[str, float], p: Params) -> dict[str, dict[str, float]]:
    """허브 → {정류장: r(s↔k) 4시간 합}."""
    out: dict[str, dict[str, float]] = {k: {} for k in hubs}
    for (kind, s, k), v in demand.pair.items():
        if k not in hubs or s not in net.stops:
            continue
        if net.station_stop_dist_m(k, s) <= p.radius_m:
            out[k][s] = out[k].get(s, 0.0) + sum(v)
    return {k: {s: v for s, v in sorted(m.items()) if v > 0} for k, m in out.items()}


def overlap(i: str, j: str, K: dict[str, dict[str, float]], F: dict[str, float]) -> float:
    common = set(K[i]) & set(K[j])
    return sum(K[i][s] + K[j][s] for s in common) / min(F[i], F[j])


def _union_find(nodes: list[str], edges: list[tuple[str, str]]) -> list[list[str]]:
    parent = {n: n for n in nodes}

    def find(x: str) -> str:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    for a, b in edges:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[max(ra, rb)] = min(ra, rb)
    groups: dict[str, list[str]] = {}
    for n in nodes:
        groups.setdefault(find(n), []).append(n)
    return sorted((sorted(g) for g in groups.values()), key=lambda g: g[0])


def merge_hubs(hubs: list[str], O: dict[tuple[str, str], float], tau: float, tau_step: float,
               cluster_max: int) -> list[dict]:
    """겹침 병합. 반환: [{hubs, tau}] — 군집이 cluster_max를 넘으면 그 군집만 τ를 올려 다시 나눈다."""
    def split(nodes: list[str], t: float) -> list[dict]:
        edges = [(a, b) for a in nodes for b in nodes if a < b and O[(a, b)] >= t - 1e-12]
        out = []
        for g in _union_find(nodes, edges):
            if len(g) > cluster_max:
                out.extend(split(g, round(t + tau_step, 10)))
            else:
                out.append({"hubs": g, "tau": t})
        return out
    return sorted(split(sorted(hubs), tau), key=lambda c: c["hubs"][0])


# ---------------------------------------------------------------- 4 섹터·삽입

@dataclass(frozen=True)
class Pattern:
    kind: str
    hubs: tuple[str, ...]
    start: str | None      # 고정 시작 정류장(egress·connect)
    end: str | None        # 고정 끝 정류장(access·connect)
    center: tuple[float, float]

    @property
    def station_stops(self) -> frozenset[str]:
        return frozenset(x for x in (self.start, self.end) if x)

    def anchors_with(self, seed: str) -> list[str]:
        return [x for x in (self.start, seed, self.end) if x]

    def movable(self, anchors: list[str]) -> range:
        lo = 1 if self.start else 0
        hi = len(anchors) - (1 if self.end else 0)
        return range(lo, hi)

    def insert_positions(self, anchors: list[str]) -> range:
        """새 anchor를 넣을 수 있는 자리(그 인덱스 앞에 넣음)."""
        lo = 1 if self.start else 0
        hi = len(anchors) - (1 if self.end else 0)
        return range(lo, hi + 1)


def stop_value(pattern: Pattern, s: str, demand: Demand) -> float:
    def r(kind: str, k: str) -> float:
        v = demand.pair.get((kind, s, k))
        return sum(v) if v else 0.0
    if pattern.kind == "access":
        return r("access", pattern.hubs[0])
    if pattern.kind == "egress":
        return r("egress", pattern.hubs[0])
    return r("egress", pattern.hubs[0]) + r("access", pattern.hubs[1])


def _bearing(center: tuple[float, float], lat: float, lon: float) -> float:
    dy = lat - center[0]
    dx = (lon - center[1]) * math.cos(math.radians(center[0]))
    return math.atan2(dy, dx) % (2 * math.pi)


def sectors(stops: list[str], values: dict[str, float], center: tuple[float, float], m: int,
            net: Network) -> list[list[str]]:
    """방위각 순으로 늘어놓고 가장 큰 빈 각에서 시작해, 수요가 거의 같은 연속 m조각으로 나눈다."""
    if not stops:
        return []
    ang = sorted(((_bearing(center, net.stops[s].lat, net.stops[s].lon), s) for s in stops))
    if len(ang) > 1:
        gaps = [((ang[(i + 1) % len(ang)][0] - ang[i][0]) % (2 * math.pi), i) for i in range(len(ang))]
        _, gi = max(gaps, key=lambda g: (g[0], -g[1]))
        ang = ang[gi + 1:] + ang[:gi + 1]
    total = sum(values[s] for _, s in ang)
    out: list[list[str]] = [[] for _ in range(m)]
    acc = 0.0
    for _, s in ang:
        idx = min(m - 1, int(acc / total * m)) if total > 0 else 0
        out[idx].append(s)
        acc += values[s]
    return [g for g in out if g]


class Builder:
    def __init__(self, net: Network, graph: Graph, demand: Demand, p: Params):
        self.net, self.g, self.demand, self.p = net, graph, demand, p
        self.od_total = {k: sum(v) for k, v in demand.od.items()}
        # 거절 사유 집계: (단계, kind, 사유) → 횟수. 사유는 여러 개가 함께 붙을 수 있다(편도·정차 수·우회율).
        self.reasons: dict[tuple[str, str, str], int] = {}
        # 거절 사유 조합: (단계, kind, "l_max+detour" 같은 조합) → 횟수(단독 원인을 셀 때 쓴다)
        self.reason_sets: dict[tuple[str, str, str], int] = {}

    def check(self, pattern: Pattern, anchors: list[str], lim: Limits) -> tuple[tuple[str, ...] | None, list[str]]:
        stops = self.g.expand(anchors)
        if stops is None:
            # cutoff 안 경로 없음 또는 이어 붙인 정차열에 같은 정류장(S2): 상한 없는 최단경로로 제약별 사유를 매긴다
            return None, self.g.diagnose(anchors, pattern.station_stops, lim)
        bad = self.g.violations(stops, pattern.station_stops, lim)
        return (None if bad else stops), bad

    def route_of(self, pattern: Pattern, anchors: list[str], lim: Limits, stage: str | None = None) -> tuple[str, ...] | None:
        stops, bad = self.check(pattern, anchors, lim)
        if stage is not None:
            for r in (bad + ["rejected"]) if bad else ["ok"]:   # rejected = 거절된 시도 수(사유는 겹쳐 셈)
                key = (stage, pattern.kind, r)
                self.reasons[key] = self.reasons.get(key, 0) + 1
            if bad:
                key = (stage, pattern.kind, "+".join(sorted(bad)))
                self.reason_sets[key] = self.reason_sets.get(key, 0) + 1
        return stops

    def insertion(self, pattern: Pattern, seed: str, pool: list[str], lim: Limits,
                  stage: str = "insertion") -> tuple[str, ...] | None:
        anchors = pattern.anchors_with(seed)
        stops = self.route_of(pattern, anchors, lim)
        if stops is None:
            return None
        cur_dem = cover_value(stops, self.od_total)
        cur_t = self.g.oneway_s(stops)
        while True:
            on = set(stops)
            best = None
            for s in pool:
                if s in on:
                    continue
                for pos in pattern.insert_positions(anchors):
                    trial = anchors[:pos] + [s] + anchors[pos:]
                    new = self.route_of(pattern, trial, lim, stage)
                    if new is None:
                        continue
                    d_dem = cover_value(new, self.od_total) - cur_dem
                    if d_dem <= 1e-9:
                        continue
                    d_t = self.g.oneway_s(new) - cur_t
                    score = math.inf if d_t <= 0 else d_dem / d_t
                    key = (score, d_dem, -pos, s)   # 동률: 증가 수요 큰 쪽, 앞자리, 정류장 ID
                    if best is None or key > best[0]:
                        best = (key, trial, new)
            if best is None:
                break
            _, anchors, stops = best
            cur_dem = cover_value(stops, self.od_total)
            cur_t = self.g.oneway_s(stops)
        return self.two_opt(pattern, anchors, stops, lim)

    def two_opt(self, pattern: Pattern, anchors: list[str], stops: tuple[str, ...], lim: Limits) -> tuple[str, ...]:
        """한 번 훑기: 움직일 수 있는 anchor 구간을 뒤집어 편도가 줄고 덮개가 줄지 않으면 받는다."""
        cur_t = self.g.oneway_s(stops)
        cur_dem = cover_value(stops, self.od_total)
        mv = list(pattern.movable(anchors))
        for x in range(len(mv)):
            for y in range(x + 1, len(mv)):
                i, j = mv[x], mv[y]
                trial = anchors[:i] + anchors[i:j + 1][::-1] + anchors[j + 1:]
                new = self.route_of(pattern, trial, lim)
                if new is None:
                    continue
                t = self.g.oneway_s(new)
                dem = cover_value(new, self.od_total)
                if t < cur_t and dem >= cur_dem - 1e-9:
                    anchors, stops, cur_t, cur_dem = trial, new, t, dem
        return stops


def _patterns(cluster: list[str], net: Network) -> list[Pattern]:
    out = []
    for h in cluster:
        st = net.stations[h]
        c = (st.lat, st.lon)
        if st.access_stop:
            out.append(Pattern("access", (h,), None, st.access_stop, c))
        if st.egress_stop:
            out.append(Pattern("egress", (h,), st.egress_stop, None, c))
    for i in cluster:
        for j in cluster:
            if i == j:
                continue
            si, sj = net.stations[i], net.stations[j]
            if si.egress_stop and sj.access_stop and si.egress_stop != sj.access_stop:
                c = ((si.lat + sj.lat) / 2, (si.lon + sj.lon) / 2)
                out.append(Pattern("connect", (i, j), si.egress_stop, sj.access_stop, c))
    return out


def _pool(pattern: Pattern, K: dict[str, dict[str, float]], demand: Demand) -> dict[str, float]:
    stops = set()
    for h in pattern.hubs:
        stops |= set(K[h])
    stops -= pattern.station_stops
    vals = {s: stop_value(pattern, s, demand) for s in sorted(stops)}
    return {s: v for s, v in vals.items() if v > 0}


# ---------------------------------------------------------------- 전체

@dataclass
class Result:
    params: Params
    hubs: dict[str, float]
    catchment_sizes: dict[str, int]
    overlaps: dict[str, float]
    clusters: list[dict]
    candidates: list[Candidate]
    dropped: dict[str, int] = field(default_factory=dict)
    seconds: dict[str, float] = field(default_factory=dict)
    reasons: dict[str, int] = field(default_factory=dict)   # "단계/kind/사유" → 횟수(단계 = seed_L20, insertion_L30 …)
    reason_sets: dict[str, int] = field(default_factory=dict)   # "단계/kind/사유1+사유2" → 횟수
    loop_stats: dict = field(default_factory=dict)          # v2 순회형 만들기 집계
    must_stats: dict = field(default_factory=dict)          # v2 필수 정류장 후보 집계


def length_limits(p: Params, scenario: Scenario) -> list[tuple[str, int]]:
    """생성 중 편도(영업) 시간 상한 목록 [(태그, 초)]. v1 = 편도 L_max분 고정, v2 = 창/N − 회차."""
    if p.length_n is None:
        return [(f"L{L}", L * 60) for L in p.l_max_min]
    return [(f"N{p.length_n}", cycle_limit_s(p, scenario) - scenario.policy.turnaround_s)]


def cycle_limit_s(p: Params, scenario: Scenario) -> int:
    """운행 창 안에 N회 이상 돌 수 있는 주기 상한 = (창 끝 − 창 시작) // N."""
    w = scenario.window
    return (w.end_s - w.start_s) // p.length_n


def _station_stops(net: Network, hubs: tuple[str, ...]) -> frozenset[str]:
    return frozenset(x for x in (
        [net.stations[h].access_stop for h in hubs] + [net.stations[h].egress_stop for h in hubs]) if x)


def endpoint_set(net: Network, mode: str, depots: frozenset[str] = frozenset()) -> frozenset[str] | None:
    """끝점 후보 집합. all = None(제한 없음), stations = 역 지정 정류장, stations+depots = + 기점(GTFS 노선 첫 정류장)."""
    if mode not in ENDPOINT_MODES:
        raise ValueError(f"endpoints는 {ENDPOINT_MODES} 중 하나다: {mode}")
    if mode == "all":
        return None
    st = frozenset(x for k in net.stations.values() for x in (k.access_stop, k.egress_stop) if x)
    return st if mode == "stations" else st | frozenset(depots)


def terminals(c: Candidate) -> tuple[str, str]:
    """노선 끝점: 편도 = (첫, 끝), 순회형 = (기점, 회차)."""
    if c.shape == "loop":
        return c.stops[0], c.stops[c.turn]
    return c.stops[0], c.stops[-1]


def _finish(c: Candidate, b: Builder, scenario: Scenario, p: Params,
            endpoints: frozenset[str] | None = None) -> str | None:
    """덮개·주기·지표를 채운다. 성공이면 None, 버리면 사유(conversion_failed | length_rule | endpoint)."""
    g, net = b.g, b.net
    st_stops = _station_stops(net, c.hubs)
    end_stops = frozenset(x for x in (c.stops[0], c.stops[-1]) if x in st_stops)
    if endpoints is not None and any(x not in endpoints for x in terminals(c)):
        return "endpoint"
    cyc = route_cycle(c.stops, net.tt, scenario)
    if cyc is None:
        return "conversion_failed"
    c.cycle_s, c.return_stops = cyc.period_s, cyc.return_stops
    if p.length_n is not None and c.cycle_s > cycle_limit_s(p, scenario):
        return "length_rule"
    if c.shape == "loop" and g.turns_format is not None:
        # 고리를 닫는 이음(끝 → 공차 → 기점 → 둘째 정류장)에서도 U턴하지 않는다
        ring = c.stops + tuple(cyc.return_stops[1:]) + (c.stops[1],)
        if g.has_uturn(ring):
            return "loop_uturn"
    c.oneway_s = g.oneway_s(c.stops)
    c.length_m = g.length_m(c.stops)
    if c.shape == "loop":
        c.detour = loop_detour(g, c.stops, c.turn, p.loop_detour)
        c.n_stops = sum(1 for s in c.stops if s not in st_stops)
    else:
        c.detour = g.detour(c.stops)
        c.n_stops = sum(1 for s in c.stops if s not in end_stops)
    c.cover_keys = tuple(covered_keys(c.stops, b.demand.od))
    c.cover = sum(b.od_total[k] for k in c.cover_keys)
    c.cross_gu = any(net.stops[s].gu != p.home_gu for s in c.stops)
    if p.must_stops:
        must = set(p.must_stops)
        c.must = tuple(s for s in c.stops if s in must)
    return None


def direction_key(c: Candidate) -> tuple:
    """같은 방향 = 같은 kind + 같은 고정 역 정류장(access 끝, egress 시작, connect 둘 다).

    역 ID가 아니라 정류장으로 본다(온수 1821·2752처럼 역 둘이 지정 정류장 하나를 같이 쓰면 같은 노선이다).
    순회형은 (기점, 회차) 정류장이 같으면 같은 방향이다.
    """
    if c.shape == "loop":
        return ("loop", c.stops[0], c.stops[c.turn])
    if c.kind == "access":
        return ("access", c.stops[-1])
    if c.kind == "egress":
        return ("egress", c.stops[0])
    return ("connect", c.stops[0], c.stops[-1])


def dedupe(cands: list[Candidate], threshold: float) -> tuple[list[Candidate], int]:
    """같은 방향끼리 정차 정류장 Jaccard ≥ threshold면 주기가 짧은 쪽만 남긴다.

    방향이 다른 노선(예: access A→역과 egress 역→A)은 정류장 집합이 같아도 다른 승객을 태우므로 비교하지 않는다.
    동률이면 덮개가 큰 쪽, 그다음 정차열 사전순.
    필수 정류장을 지나는 노선은 같은 필수 정류장을 지나는 노선과만 중복으로 본다(끼운 버전이 지워지지 않게).
    """
    order = sorted(cands, key=lambda c: (c.cycle_s, -c.cover, c.stops, c.origin))
    kept: list[Candidate] = []
    for c in order:
        sc = set(c.stops)
        dup = False
        dk = direction_key(c)
        for k in kept:
            if direction_key(k) != dk or k.must != c.must:
                continue
            sk = set(k.stops)
            if len(sc & sk) / len(sc | sk) >= threshold:
                dup = True
                break
        if not dup:
            kept.append(c)
    kept.sort(key=lambda c: c.route_id)
    return kept, len(cands) - len(kept)


def _reverse_pool(hubs: tuple[str, ...], K: dict[str, dict[str, float]], demand: Demand,
                  limit: int) -> list[str]:
    """순회형 오는 쪽에 끼울 수요 정류장: 노선 역들의 권역 정류장, access+egress 수요 큰 순."""
    vals: dict[str, float] = {}
    for h in hubs:
        for s in K.get(h, {}):
            v = sum(sum(demand.pair.get((kind, s, h), ())) for kind in ("access", "egress"))
            if v > 0:
                vals[s] = vals.get(s, 0.0) + v
    return sorted(vals, key=lambda s: (-vals[s], s))[:limit]


def _insert_must(g: Graph, stops: tuple[str, ...], turn: int | None, m: str
                 ) -> tuple[tuple[str, ...], int | None] | None:
    """필수 정류장 m을 정차열에 끼운다(영업 시간 증가가 가장 작은 자리, 동률이면 앞자리). S2를 깨면 그 자리는 버린다."""
    best = None
    for i in range(len(stops) - 1):
        p1, p2 = g.path(stops[i], m), g.path(m, stops[i + 1])
        if p1 is None or p2 is None:
            continue
        new = stops[:i + 1] + p1[1:] + p2[1:] + stops[i + 2:]
        if len(set(new)) != len(new):
            continue
        t = g.oneway_s(new)
        nt = turn if turn is None or i >= turn else turn + (len(new) - len(stops))
        if best is None or (t, i) < best[0]:
            best = ((t, i), new, nt)
    return (best[1], best[2]) if best else None


def generate(net: Network, demand: Demand, scenario: Scenario, p: Params = Params(),
             depots: frozenset[str] = frozenset()) -> Result:
    """depots = 기점 후보(묶음 GTFS 노선 첫 정류장). endpoints = stations+depots일 때만 쓴다."""
    t0 = time.time()
    hubs = hubs_of(demand, net, p)
    K = catchments(demand, net, hubs, p)
    hub_ids = sorted(hubs)
    O = {(a, b): overlap(a, b, K, hubs) for a in hub_ids for b in hub_ids if a < b}
    clusters = merge_hubs(hub_ids, O, p.tau, p.tau_step, p.cluster_max)
    for ci, cl in enumerate(clusters):
        cl["id"] = ci
        gus = sorted({net.station_gu(h) or "?" for h in cl["hubs"]})
        cl["gu"] = gus
        cl["cross_gu"] = gus != [p.home_gu]
    t_setup = time.time()

    limits = length_limits(p, scenario)
    graph = Graph(net, scenario.policy.dwell_base_s, cutoff_s=max(s for _, s in limits))
    b = Builder(net, graph, demand, p)
    raw: dict[tuple, Candidate] = {}
    dropped = {"seed_infeasible": 0, "conversion_failed": 0}

    def add(kind, stops, origin, hubs_, cluster, tag, shape="oneway", turn=None, loop_info=None):
        key = (origin, kind, hubs_, stops)
        if key in raw:
            raw[key].made_by = tuple(sorted(set(raw[key].made_by) | {tag}))
            return
        raw[key] = Candidate("", kind, stops, origin, hubs_, cluster, (tag,), shape=shape, turn=turn,
                             loop_info=loop_info)

    # 4. 섹터 분할 + 최선 삽입
    for cl in clusters:
        for pat in _patterns(cl["hubs"], net):
            values = _pool(pat, K, demand)
            for tag, L_s in limits:
                lim = Limits(L_s, p.max_stops, p.max_detour)
                feas = [s for s in values if b.route_of(pat, pat.anchors_with(s), lim, f"seed_{tag}") is not None]
                dropped["seed_infeasible"] += len(values) - len(feas)
                for m in p.sectors:
                    for sec in sectors(feas, values, pat.center, m, net):
                        seed = max(sec, key=lambda s: (values[s] * haversine_m(
                            pat.center[0], pat.center[1], net.stops[s].lat, net.stops[s].lon), s))
                        stops = b.insertion(pat, seed, sec, lim, f"insertion_{tag}")
                        if stops is not None:
                            add(pat.kind, stops, "generated", pat.hubs, cl["id"], f"m{m}_{tag}")
    t_gen = time.time()

    # 5. 최단경로 피더(허브·방향마다 수요 상위 sp_top seed, 같은 제약·L_max 최댓값)
    tag_max, L_max_s = max(limits, key=lambda x: x[1])
    lim = Limits(L_max_s, p.max_stops, p.max_detour)
    cl_of = {h: cl["id"] for cl in clusters for h in cl["hubs"]}
    for h in hub_ids:
        for pat in _patterns([h], net):
            values = _pool(pat, K, demand)
            ranked = sorted(values, key=lambda s: (-values[s], s))
            picked = 0
            for s in ranked:
                if picked >= p.sp_top:
                    break
                stops = b.route_of(pat, pat.anchors_with(s), lim, f"sp_{tag_max}")
                if stops is None:
                    continue
                picked += 1
                add(pat.kind, stops, "shortest_path_feeder", pat.hubs, cl_of[h], f"sp:{s}")
    t_sp = time.time()

    # v2-a. 필수 정류장 전용 편도(필수 정류장 → 가까운 역 access 정류장)
    must_stats: dict = {}
    must = [m for m in sorted(set(p.must_stops))]
    if must:
        known = net.tt.stop_ids()
        must_stats = {"must_stops": must, "not_in_network": [m for m in must if m not in known],
                      "feeder": {}, "insert": {}}
        stations = sorted((k for k, st in net.stations.items() if st.access_stop and st.access_stop in known))
        lim_free = Limits(L_max_s, p.max_stops, float("inf"))
        for m in must:
            if m not in known:
                continue
            near = sorted(stations, key=lambda k: (net.station_stop_dist_m(k, m), k))[: p.must_feeder_stations]
            ranked = []
            for k in near:
                hit = graph.tree(m).get(net.stations[k].access_stop)
                if hit is not None:
                    ranked.append((hit[0], net.station_stop_dist_m(k, m), k))
            made = None
            for _, _, k in sorted(ranked):
                pat = Pattern("access", (k,), None, net.stations[k].access_stop,
                              (net.stations[k].lat, net.stations[k].lon))
                stops = b.route_of(pat, [m, net.stations[k].access_stop], lim_free, f"must_{tag_max}")
                if stops is not None and route_cycle(stops, net.tt, scenario) is None:
                    must_stats.setdefault("feeder_no_return", {}).setdefault(m, []).append(k)
                    continue                                   # 역에서 필수 정류장으로 돌아올 망 경로가 없다(RETURN)
                if stops is not None:
                    add("access", stops, "must_feeder", (k,), cl_of.get(k), f"must:{m}")
                    made = k
                    break
            must_stats["feeder"][m] = made
    t_mf = time.time()

    # v2-b. 순회형: 편도 후보마다(생성·피더·필수 전용) 순회형 버전
    loop_stats: dict = {}
    loop_cap = p.max_detour if p.loop_max_detour is None else p.loop_max_detour
    if p.loops:
        maker = LoopMaker(net, graph, b.od_total, loop_cap, p.max_stops, L_max_s, p.pair_radius_m,
                          p.loop_insert_top, p.return_detour, p.loop_detour, p.close_max_s)
        for key in sorted(raw, key=lambda k: (k[0], k[1], k[2], k[3])):
            c = raw[key]
            if c.shape != "oneway":
                continue
            built = maker.build(c.stops, _reverse_pool(c.hubs, K, demand, p.loop_insert_top),
                                detour_free=c.origin == "must_feeder")
            if built is None:
                continue
            info = {"from": list(c.stops[:1]) + [c.stops[-1]], "targets": built.targets, "paired": built.paired,
                    "skipped": built.skipped, "inserted": built.inserted, "deadhead_s": built.deadhead_s}
            add(f"loop_{c.kind}", built.stops, c.origin, c.hubs, c.cluster, "loop", "loop", built.turn, info)
        loop_stats = dict(maker.stats, pairs_known=len(maker.pairs))
    t_loop = time.time()

    # v2-c. 필수 정류장을 기존 후보(편도·순회형)에 끼운 버전
    if must:
        lim_ins = lim
        for key in sorted(raw, key=lambda k: (k[0], k[1], k[2], k[3])):
            c = raw[key]
            if c.origin == "must_insert" or (p.loop_only and c.shape != "loop"):
                continue
            for m in must:
                if m not in known or m in c.stops:
                    continue
                if min(net.stop_dist_m(m, s) for s in c.stops) > p.must_insert_radius_m:
                    continue
                got = _insert_must(graph, c.stops, c.turn, m)
                ok = False
                if got is not None:
                    new, nt = got
                    if c.shape == "loop":
                        ok = not loop_violations(graph, new, nt, loop_cap, p.max_stops, L_max_s, p.return_detour,
                                                 p.loop_detour)
                    else:
                        ok = not graph.violations(new, _station_stops(net, c.hubs), lim_ins)
                st = must_stats["insert"].setdefault(m, {"tried": 0, "ok": 0})
                st["tried"] += 1
                if ok:
                    st["ok"] += 1
                    add(c.kind, new, "must_insert", c.hubs, c.cluster, f"must:{m}", c.shape, nt,
                        dict(c.loop_info or {}, must_inserted=m) if c.shape == "loop" else None)
    t_mi = time.time()

    # 7. 덮개·주기
    endpoints = endpoint_set(net, p.endpoints, depots)
    cands = []
    for key in sorted(raw, key=lambda k: (k[0], k[1], k[2], k[3])):
        c = raw[key]
        why = _finish(c, b, scenario, p, endpoints)
        if why is None:
            cands.append(c)
        else:
            dropped[why] = dropped.get(why, 0) + 1
    cands = [c for c in cands if c.cover > 0 or c.must]
    if p.loop_only:
        seeds = sum(1 for c in cands if c.shape != "loop")
        cands = [c for c in cands if c.shape == "loop"]
        loop_stats["oneway_seeds_not_allocated"] = seeds
    for i, c in enumerate(cands, start=1):
        c.route_id = f"c{i:04d}"
    t_fin = time.time()
    seconds = {"setup": round(t_setup - t0, 2), "insertion": round(t_gen - t_setup, 2),
               "shortest_path_feeders": round(t_sp - t_gen, 2), "cover_cycle": round(t_fin - t_mi, 2)}
    if must:
        seconds.update(must_feeders=round(t_mf - t_sp, 2), must_insert=round(t_mi - t_loop, 2))
    if p.loops:
        seconds["loops"] = round(t_loop - t_mf, 2)
    return Result(p, hubs, {k: len(v) for k, v in K.items()}, {f"{a}-{b_}": round(v, 4) for (a, b_), v in O.items()},
                  clusters, cands, dropped, seconds,
                  {"/".join(k): v for k, v in sorted(b.reasons.items())},
                  {"/".join(k): v for k, v in sorted(b.reason_sets.items())}, loop_stats, must_stats)


SP_ORIGINS = ("shortest_path_feeder", "must_feeder")


def pools(cands: list[Candidate], threshold: float) -> dict[str, tuple[list[Candidate], int]]:
    """배분 후보 풀. ours = 전체(중복 제거), sp = 최단경로 피더 + 필수 정류장 전용(순회형 버전 포함, 중복 제거)."""
    return {"ours": dedupe(cands, threshold),
            "sp": dedupe([c for c in cands if c.origin in SP_ORIGINS], threshold)}
