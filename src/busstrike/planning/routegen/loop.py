"""순회형(왕복) 노선 — 기점 → 회차 → 기점을 영업 구간 하나로(설계 v2 2절).

비교 입력 plan.json은 영업 정차열 하나만 받고, 변환기가 끝 정류장 → 첫 정류장 공차 leg를 붙인다
(`comparison_plan.to_engine_plan`). 그래서 순회형도 형식을 바꾸지 않고 "가는 쪽 + 오는 쪽" 정차열 하나로 쓴다.

만드는 법(편도 후보 S = s_1 … s_n에서)
1. 가는 쪽 = S 그대로. 회차 정류장 = s_n(한 번만 쓴다).
2. s_n이 역 access 지정 정류장이면 그 역 egress 지정 정류장(다르면)을 오는 쪽 첫 목표로 둔다(역에서 내린 뒤 다시 태움).
3. 오는 쪽 목표 = s_{n-1} … s_2의 길 건너 짝, 마지막은 s_1의 짝(s_1이 역 egress 지정 정류장이면 그 역 access 지정 정류장).
   짝 = 이름이 같고 50m 안인 다른 정류장(가장 가까운 것, 동률이면 ID 사전순).
4. 목표를 순서대로 망 최단경로로 잇는다. 이미 쓴 정류장(S2: 같은 ID 두 번 금지)을 지나는 목표는 건너뛴다.
5. 짝을 못 찾거나 건너뛴 목표가 있으면 오는 쪽에 수요 정류장을 끼운다(증가 덮개 ÷ 증가 시간, 제약 안에서).
6. 끝: 오는 쪽 정류장 중 첫 정류장까지 공차 복귀(망 최단경로)가 가장 짧은 곳에서 자른다(동률이면 뒤쪽).

승객은 기점(앵커)을 넘어 탈 수 없다(시뮬레이터 가정 A23: 영업 leg 하나 안에서 o 다음에 d). 덮개 계산
(`generate.covered_keys`)이 이미 그 조건이라 근사에서도 기점을 넘는 수요는 세지 않는다.

제약(순회형)
- 우회율은 방향마다: 가는 쪽(s_1 … 회차), 오는 쪽(회차 … 끝) 각각 ≤ max_detour.
- 정차 상한(설정 시)도 방향마다.
- 길이: 영업 구간 시간(`Graph.oneway_s`) ≤ 길이 상한. 최종 주기 상한은 `generate._finish`가 시뮬레이터 주기로 본다.
"""

from __future__ import annotations

from dataclasses import dataclass

from .data import Network, haversine_m
from .graph import Graph

PAIR_RADIUS_M = 50.0


def opposite_pairs(net: Network, radius_m: float = PAIR_RADIUS_M) -> dict[str, str]:
    """정류장 → 길 건너 짝(이름이 같고 radius_m 안, 가장 가까운 것, 동률이면 ID). 망(tt)에 있는 정류장만."""
    known = net.tt.stop_ids()
    by_name: dict[str, list[str]] = {}
    for sid in sorted(net.stops):
        if sid in known:
            by_name.setdefault(net.stops[sid].name, []).append(sid)
    out: dict[str, str] = {}
    for ids in by_name.values():
        if len(ids) < 2:
            continue
        for a in ids:
            sa = net.stops[a]
            best = None
            for b in ids:
                if b == a:
                    continue
                sb = net.stops[b]
                d = haversine_m(sa.lat, sa.lon, sb.lat, sb.lon)
                if d <= radius_m and (best is None or (d, b) < best):
                    best = (d, b)
            if best:
                out[a] = best[1]
    return out


def station_partners(net: Network) -> tuple[dict[str, str], dict[str, str]]:
    """(access 지정 정류장 → 같은 역 egress 지정 정류장, egress → access). 같은 정류장이면 넣지 않는다."""
    a2e: dict[str, str] = {}
    e2a: dict[str, str] = {}
    for k in sorted(net.stations):
        st = net.stations[k]
        if st.access_stop and st.egress_stop and st.access_stop != st.egress_stop:
            a2e.setdefault(st.access_stop, st.egress_stop)
            e2a.setdefault(st.egress_stop, st.access_stop)
    return a2e, e2a


@dataclass
class LoopBuild:
    stops: tuple[str, ...]
    turn: int                      # 회차 정류장 인덱스(가는 쪽 마지막)
    targets: int                   # 오는 쪽 목표 수
    paired: int                    # 짝으로 이은 목표 수
    skipped: list[str]             # 짝이 없거나(S2·경로 없음) 건너뛴 가는 쪽 정류장
    inserted: list[str]            # 수요로 끼운 정류장
    deadhead_s: int                # 끝 → 첫 정류장 공차(망 최단경로, 06~09시 최댓값 가중치)


def loop_violations(g: Graph, stops: tuple[str, ...], turn: int, max_detour: float, max_stops: int | None,
                    length_s: int, return_detour: float | None = None) -> list[str]:
    """return_detour = 오는 쪽 우회율 상한(None = max_detour와 같음)."""
    bad = []
    if len(set(stops)) != len(stops):
        bad.append("repeat")
    if turn <= 0 or turn >= len(stops) - 1:
        bad.append("no_return")
        return bad
    if g.oneway_s(stops) > length_s:
        bad.append("length")
    fwd, back = stops[:turn + 1], stops[turn:]
    if g.detour(fwd) > max_detour or g.detour(back) > (max_detour if return_detour is None else return_detour):
        bad.append("detour")
    if max_stops is not None and (len(fwd) > max_stops or len(back) > max_stops):
        bad.append("max_stops")
    return bad


class LoopMaker:
    def __init__(self, net: Network, g: Graph, od_total: dict[tuple[str, str], float], max_detour: float,
                 max_stops: int | None, length_s: int, pair_radius_m: float = PAIR_RADIUS_M,
                 insert_top: int = 30, return_detour: float | None = None):
        self.net, self.g, self.od_total = net, g, od_total
        self.max_detour, self.max_stops, self.length_s = max_detour, max_stops, length_s
        self.return_detour = return_detour
        self._detour_free = False
        self.pairs = opposite_pairs(net, pair_radius_m)
        self.a2e, self.e2a = station_partners(net)
        self.insert_top = insert_top
        self.stats = {"built": 0, "failed": 0, "targets": 0, "paired": 0, "skipped": 0, "inserted": 0}

    # ------------------------------------------------------------ 이어 붙이기
    def _extend(self, seq: list[str], targets: list[str]) -> tuple[list[str], list[str], int]:
        """seq 끝에서 targets를 순서대로 잇는다. 돌려주는 값: (정차열, 건너뛴 목표, 이은 목표 수)."""
        seq = list(seq)
        used = set(seq)
        skipped = []
        joined = 0
        for t in targets:
            if t in used:
                skipped.append(t)
                continue
            p = self.g.path(seq[-1], t)
            if p is None or any(x in used for x in p[1:]):
                skipped.append(t)
                continue
            seq.extend(p[1:])
            used.update(p[1:])
            joined += 1
        return seq, skipped, joined

    def _cover(self, stops: tuple[str, ...]) -> float:
        n = len(stops)
        v = 0.0
        for a in range(n):
            for b in range(a + 1, n):
                v += self.od_total.get((stops[a], stops[b]), 0.0)
        return v

    def _ok(self, stops: tuple[str, ...], turn: int) -> bool:
        if self._detour_free:
            return not loop_violations(self.g, stops, turn, float("inf"), self.max_stops, self.length_s,
                                       float("inf"))
        return not loop_violations(self.g, stops, turn, self.max_detour, self.max_stops, self.length_s,
                                   self.return_detour)

    def _deadhead(self, a: str, b: str) -> int | None:
        if a == b:
            return 0
        hit = self.g.tree(a, capped=False).get(b)
        return hit[0] if hit else None

    def _insert(self, fwd: list[str], back_targets: list[str], pool: list[str]) -> tuple[list[str], list[str]]:
        """오는 쪽 목표 사이에 수요 정류장을 끼운다(증가 덮개 ÷ 증가 영업 시간이 가장 큰 것부터)."""
        turn = len(fwd) - 1
        targets = list(back_targets)
        seq, _, _ = self._extend(fwd, targets)
        cur = tuple(seq)
        if not self._ok(cur, turn):
            return targets, []
        cur_dem, cur_t = self._cover(cur), self.g.oneway_s(cur)
        added = []
        cand = [s for s in pool if s not in set(cur)][: self.insert_top]
        while True:
            best = None
            on = set(cur)
            for s in cand:
                if s in on:
                    continue
                for pos in range(len(targets) + 1):
                    trial = targets[:pos] + [s] + targets[pos:]
                    seq, skipped, _ = self._extend(fwd, trial)
                    if s in skipped:
                        continue
                    new = tuple(seq)
                    if not self._ok(new, turn):
                        continue
                    d_dem = self._cover(new) - cur_dem
                    if d_dem <= 1e-9:
                        continue
                    d_t = self.g.oneway_s(new) - cur_t
                    score = float("inf") if d_t <= 0 else d_dem / d_t
                    key = (score, d_dem, -pos, s)
                    if best is None or key > best[0]:
                        best = (key, trial, new)
            if best is None:
                break
            _, targets, cur = best
            added.append(best[0][3])
            cur_dem, cur_t = self._cover(cur), self.g.oneway_s(cur)
        return targets, added

    # ------------------------------------------------------------ 만들기
    def build(self, oneway: tuple[str, ...], pool: list[str] = (), detour_free: bool = False) -> LoopBuild | None:
        """편도 정차열 → 순회형. 제약을 못 지키면 None. pool = 오는 쪽에 끼울 수 있는 수요 정류장(값 큰 순).

        detour_free = 우회율을 보지 않는다(필수 정류장 전용 후보: 길이 규칙·S2만).
        """
        self._detour_free = detour_free
        try:
            return self._build(oneway, pool)
        finally:
            self._detour_free = False

    def _build(self, oneway: tuple[str, ...], pool: list[str]) -> LoopBuild | None:
        if len(oneway) < 2:
            self.stats["failed"] += 1
            return None
        fwd = list(oneway)
        first, turn_stop = fwd[0], fwd[-1]
        targets: list[str] = []
        if turn_stop in self.a2e:
            targets.append(self.a2e[turn_stop])
        for s in reversed(fwd[1:-1]):
            if s in self.pairs:
                targets.append(self.pairs[s])
        end = self.e2a.get(first) or self.pairs.get(first)
        if end:
            targets.append(end)
        missing = [s for s in fwd[:-1] if not (s in self.pairs or (s == first and end))]
        seq, skipped, joined = self._extend(fwd, targets)
        inserted: list[str] = []
        if (missing or skipped) and pool:
            targets, inserted = self._insert(fwd, targets, list(pool))
            seq, skipped, joined = self._extend(fwd, targets)
        turn = len(fwd) - 1
        if len(seq) <= len(fwd):
            self.stats["failed"] += 1
            return None
        # 끝 자르기: 첫 정류장까지 공차가 가장 짧은 오는 쪽 정류장(동률이면 뒤쪽)
        best = None
        for j in range(turn + 1, len(seq)):
            dh = self._deadhead(seq[j], first)
            if dh is None:
                continue
            key = (dh, -j)
            if best is None or key < best[0]:
                best = (key, j)
        if best is None:
            self.stats["failed"] += 1
            return None
        j = best[1]
        stops = tuple(seq[: j + 1])
        if not self._ok(stops, turn):
            self.stats["failed"] += 1
            return None
        kept = set(stops)
        inserted = [s for s in inserted if s in kept]
        self.stats["built"] += 1
        self.stats["targets"] += len(targets)
        self.stats["paired"] += joined
        self.stats["skipped"] += len(skipped) + len(missing)
        self.stats["inserted"] += len(inserted)
        return LoopBuild(stops, turn, len(targets), joined, sorted(set(skipped) | set(missing)), inserted,
                         best[0][0])
