"""구 단독(per_gu) 차량 배분(설계 v1 5절).

충족 근사 v2 (시간 h = 6, 7, 8, 9시를 따로 계산해 더한다)
    served_k,h = κ · d_k,h · min(1, W · Σ_{r∋k} n_r / C_r)
    노선별 승객 = served_k,h × (n_r/C_r) / Σ_{r∋k} n_r/C_r   (배차 비율로 나눔)
    노선 수송 한도 n_r × 3600 / C_r × 좌석(시간당)을 넘는 몫은 잘라낸다.

개설: 새 노선은 n0 = ⌈C/W⌉대를 한 번에 넣는다. 비교 기준은 1대당 이득.
탐욕: 지연 평가(CELF) 우선순위 큐.
국소 탐색(v1): 매 회차 아래 이동을 모두 평가해 가장 좋은 개선 하나를 받는다. 개선이 없으면
가치 낮은 노선 닫고 재배분(close_reopt)을 시도하고, 그것도 없으면 멈춘다.
- shift1·shift2·shift3: 열린 노선 a에서 b로 k대(a는 1대 이상 남김. AllocParams.keep_n0(v2)이면 n0대 이상)
- swap: 열린 노선 a를 닫고 안 열린 후보 c를 n0[c]대로 연다. 대수가 모자라면 다른 노선에서
  1대씩(뺄 때 손실이 가장 작은 노선부터, 1대는 남김) 가져오고, 남으면 fill로 넣는다.
초기해 여러 개(탐욕 + fill, 수요 상위 노선부터 n0씩, 호출자가 준 해 — 예: 피더안 배분)에서 각각 국소 탐색을
돌리고 근사가 가장 큰 해를 고른다(동률이면 앞 초기해). 국소 탐색은 근사를 줄이지 않으므로 결과는 모든 초기해 이상이다.
차량 합은 정확히 가용 대수. 남는 대수는 1대씩 이득이 가장 큰 열린 노선에 넣는다.
난수를 쓰지 않는다. 동률은 노선 ID 순.

필수 정류장 제약(v2, `Model(must=...)`을 줄 때만)
- 덮음: 개설 노선 중 하나 이상이 필수 정류장 s에 정차하고, s가 그 노선 영업 구간의 첫·끝 정류장이 아니다
  (must[s] = 그런 후보 노선들; 호출자가 첫·끝을 뺀 정차로 만든다). 시간당 운행 수는 요구하지 않는다(모든 노선의
  최대 배차 = 대기 한도 20분 규칙으로 따라온다). `must_min_tph` > 0이면 그 시간당 운행도 요구한다(기본 0).
- 하드 제약: 덮을 수 있는 필수 정류장은 모두 덮는다.
  - 초기해(`must_start`): 아직 덮지 못한 필수 정류장을 덮는 후보 중 근사 이득(n0대로 열 때)이 가장 큰 것부터 연다.
    그 뒤 탐욕(CELF)·fill로 나머지 대수를 배분한다. 다른 초기해(수요 상위, 호출자가 준 해)도 must_start 위에서 만든다.
  - 국소 탐색은 덮음을 깨는 이동(필수 정류장을 덮는 마지막 노선을 닫는 이동 등)을 거절한다.
  - MILP(솔버가 있으면): Σ_{r ∋ s} y_r ≥ 1 (y_r = 노선 r 개설 여부, n_r ≤ budget × y_r). 이 패키지는 표준 라이브러리만
    써서 MILP 배분은 없다.
- 덮을 수 없으면(지나는 후보 없음 no_candidate, 대수 부족 budget) 그 정류장은 제약에서 빼고 보고서
  must_stop_uncovered(정류장·이유)에 남긴다. 나머지는 평소대로 배분한다.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

EPS = 1e-9


@dataclass(frozen=True)
class AllocParams:
    wait_s: int = 1200
    seat: int = 45
    kappa: float = 1.0
    keep_n0: bool = False      # v2: 열린 노선은 n0 = ⌈C/W⌉대 아래로 줄이지 않는다(모든 노선 최대 배차 = 대기 한도 W)


class Model:
    """후보 노선(route_id, 주기, 덮는 OD 키)과 시간별 OD 수요로 근사 충족을 계산한다.

    must = {필수 정류장: 그 정류장을 지나는 노선 ID들}(v2). must_wait_s = 필수 정류장 대기 한도(기본 = p.wait_s).
    """

    def __init__(self, routes: list[tuple[str, int, tuple[tuple[str, str], ...]]],
                 od_hourly: dict[tuple[str, str], list[float]], p: AllocParams = AllocParams(),
                 must: dict[str, tuple[str, ...]] | None = None, must_min_tph: float = 0.0):
        self.p = p
        self.must = {s: tuple(sorted(rs)) for s, rs in sorted((must or {}).items())}
        self.must_min_tph = must_min_tph
        self.required: frozenset[str] = frozenset()     # 하드 제약으로 지킬 필수 정류장(배분 때 정한다)
        self.ids = sorted(r for r, _, _ in routes)
        self.cycle = {r: c for r, c, _ in routes}
        keys = sorted({k for _, _, ks in routes for k in ks})
        self.key_index = {k: i for i, k in enumerate(keys)}
        self.keys = keys
        self.d = [tuple(od_hourly[k]) for k in keys]
        self.cover = {r: tuple(self.key_index[k] for k in ks) for r, _, ks in routes}
        self.n0 = {r: max(1, math.ceil(c / p.wait_s)) for r, c, _ in routes}
        self.rank = {r: i for i, r in enumerate(self.ids)}
        self.demand_of = {r: sum(sum(self.d[ki]) for ki in self.cover[r]) for r in self.ids}
        self._cache: dict[tuple, float] = {}

    def floor(self, r: str) -> int:
        """열린 노선의 최소 대수: keep_n0이면 n0(배차 ≤ 대기 한도), 아니면 1(v1)."""
        return self.n0[r] if self.p.keep_n0 else 1

    def detail(self, n: dict[str, int]) -> dict:
        """{'total', 'by_route': {r: [h6..h9 승객]}, 'demand_covered', 'cap_cut'}."""
        W, seat, kappa = self.p.wait_s, self.p.seat, self.p.kappa
        freq = {r: n[r] / self.cycle[r] for r in sorted(n) if n[r] > 0}
        f: dict[int, float] = {}
        for r, fr in freq.items():
            for ki in self.cover[r]:
                f[ki] = f.get(ki, 0.0) + fr
        by_route: dict[str, list[float]] = {}
        total = 0.0
        cut = 0.0
        for r, fr in freq.items():
            cap = fr * 3600 * seat
            loads = [0.0, 0.0, 0.0, 0.0]
            for ki in self.cover[r]:
                fk = f[ki]
                share = kappa * min(1.0, W * fk) * fr / fk
                dk = self.d[ki]
                for h in range(4):
                    loads[h] += dk[h] * share
            kept = [min(x, cap) for x in loads]
            cut += sum(loads) - sum(kept)
            by_route[r] = kept
            total += sum(kept)
        covered = sum(sum(self.d[ki]) for ki in f)
        return {"total": total, "by_route": by_route, "demand_covered": covered, "cap_cut": cut}

    def value(self, n: dict[str, int]) -> float:
        key = tuple(sorted((r, k) for r, k in n.items() if k > 0))
        got = self._cache.get(key)
        if got is None:
            got = self._cache[key] = self.detail(n)["total"]
        return got

    # ------------------------------------------------------------ 필수 정류장
    def must_tph(self, n: dict[str, int]) -> dict[str, float]:
        """필수 정류장마다 그 정류장을 덮는 개설 노선의 시간당 운행 합 = Σ n_r × 3600 / C_r."""
        return {s: sum(n.get(r, 0) * 3600 / self.cycle[r] for r in rs) for s, rs in self.must.items()}

    def covered_set(self, n: dict[str, int]) -> set[str]:
        """덮인 필수 정류장(개설 노선 중 하나 이상이 첫·끝이 아닌 정차로 지나고, must_min_tph > 0이면 그 운행도 충족)."""
        out = set()
        for s, rs in self.must.items():
            if any(n.get(r, 0) > 0 for r in rs):
                if self.must_min_tph <= 0 or sum(n.get(r, 0) * 3600 / self.cycle[r] for r in rs) >= self.must_min_tph - EPS:
                    out.add(s)
        return out

    def feasible(self, n: dict[str, int]) -> bool:
        """하드 제약(required의 모든 정류장을 덮음)을 지키는가."""
        return not self.required or self.required <= self.covered_set(n)


def _with(n: dict[str, int], r: str, k: int) -> dict[str, int]:
    m = dict(n)
    m[r] = m.get(r, 0) + k
    if m[r] <= 0:
        del m[r]
    return m


def greedy(model: Model, budget: int, start: dict[str, int] | None = None) -> dict[str, int]:
    """CELF 지연 평가 탐욕. 움직임: 닫힌 노선 개설(n0대) 또는 열린 노선 +1대. 1대당 이득 순."""
    n = dict(start or {})
    used = sum(n.values())
    base = model.value(n)
    it = 0
    heap: list[tuple[float, str, str, int, int]] = []   # (-이득/대, route_id, 종류, 대수, stamp)

    def gain(r: str, k: int) -> float:
        return (model.value(_with(n, r, k)) - base) / k

    for r in model.ids:
        k = 1 if r in n else model.n0[r]
        if used + k <= budget:
            heapq.heappush(heap, (-gain(r, k), r, "add" if r in n else "open", k, it))
    while heap:
        neg, r, kind, k, stamp = heapq.heappop(heap)
        if used + k > budget:
            continue                                   # 예산은 줄기만 하므로 영영 못 쓴다
        if (kind == "open") == (r in n):
            continue                                   # 이미 열렸거나 닫힌 노선의 낡은 움직임
        if stamp != it:
            heapq.heappush(heap, (-gain(r, k), r, kind, k, it))
            continue
        if -neg <= EPS:
            break
        n = _with(n, r, k)
        used += k
        base = model.value(n)
        it += 1
        if used + 1 <= budget:
            heapq.heappush(heap, (-gain(r, 1), r, "add", 1, it))
    return n


def fill(model: Model, budget: int, n: dict[str, int]) -> dict[str, int]:
    """남는 대수를 1대씩 이득이 가장 큰 열린 노선에 넣는다(이득 0이어도 넣는다: 변환기 S5)."""
    n = dict(n)
    while sum(n.values()) < budget:
        left = budget - sum(n.values())
        if n:
            base = model.value(n)
            r = max(sorted(n), key=lambda x: (model.value(_with(n, x, 1)) - base, -model.rank[x]))
            n = _with(n, r, 1)
        else:                                          # 열린 노선이 없으면 남은 대수로 가장 좋은 노선 하나를 연다
            r = max(model.ids, key=lambda x: (model.value({x: min(left, model.n0[x])}), -model.rank[x]))
            n = {r: min(left, model.n0[r])}
    return n


MOVES = ("shift1", "shift2", "shift3", "swap", "close_reopt")


def top_demand_start(model: Model, budget: int) -> dict[str, int]:
    """덮는 수요가 큰 노선부터 n0대씩 연다(예산을 넘는 노선은 건너뜀). 남는 대수는 fill."""
    n: dict[str, int] = {}
    used = 0
    for r in sorted(model.ids, key=lambda x: (-model.demand_of[x], x)):
        k = model.n0[r]
        if used + k <= budget:
            n[r] = k
            used += k
        if used == budget:
            break
    return fill(model, budget, n)


def swap(model: Model, budget: int, n: dict[str, int], a: str, c: str) -> dict[str, int] | None:
    """노선 교체: 열린 a를 닫고 안 열린 c를 n0[c]대로 연다. 불가능하면 None."""
    if a not in n or c in n:
        return None
    m = {x: y for x, y in n.items() if x != a}
    m[c] = model.n0[c]
    while sum(m.values()) > budget:
        donors = [x for x in sorted(m) if x != c and m[x] > model.floor(x)]
        if not donors:
            return None
        base = model.value(m)
        x = max(donors, key=lambda y: (model.value(_with(m, y, -1)) - base, -model.rank[y]))
        m = _with(m, x, -1)
    return fill(model, budget, m)


def _new_stats() -> dict[str, dict[str, int]]:
    return {k: {"tries": 0, "accepts": 0} for k in MOVES}


def local_search(model: Model, budget: int, n: dict[str, int], max_rounds: int = 1000,
                 stats: dict[str, dict[str, int]] | None = None) -> tuple[dict[str, int], list[dict]]:
    """회차마다 shift1~3·swap 중 최선 개선 하나 → (없으면) close_reopt. 더 나아지지 않을 때까지.

    stats(이동 종류별 시도·수락 수)를 받으면 거기에 더한다. 기록(받아들인 이동 목록)을 함께 돌려준다.
    """
    st = stats if stats is not None else _new_stats()
    cur = model.value(n)
    log: list[dict] = []
    for _ in range(max_rounds):
        best = None

        def consider(m: dict[str, int] | None, info: dict) -> None:
            nonlocal best
            st[info["move"]]["tries"] += 1
            if m is None:
                return
            if model.required and not model.feasible(m):
                return                                     # 필수 정류장 덮음을 깨는 이동은 받지 않는다
            v = model.value(m)
            if v > cur + EPS and (best is None or v > best[0] + EPS):
                best = (v, m, info)

        for k in (1, 2, 3):
            for a in sorted(n):
                if n[a] - k < model.floor(a):
                    continue
                for b in sorted(n):
                    if a != b:
                        consider(_with(_with(n, a, -k), b, k), {"move": f"shift{k}", "from": a, "to": b})
        closed = [c for c in model.ids if c not in n]
        for a in sorted(n):
            for c in closed:
                consider(swap(model, budget, n, a, c), {"move": "swap", "closed": a, "opened": c})
        if best is None:
            order = sorted(n, key=lambda r: (cur - model.value({x: y for x, y in n.items() if x != r}), r))
            for r in order:
                rest = {x: y for x, y in n.items() if x != r}
                consider(fill(model, budget, greedy(model, budget, rest)), {"move": "close_reopt", "closed": r})
                if best is not None:
                    break
        if best is None:
            break
        st[best[2]["move"]]["accepts"] += 1
        log.append(dict(best[2], gain=round(best[0] - cur, 6)))
        cur, n = best[0], best[1]
    return n, log


def _check(n: dict[str, int], budget: int, model: Model, name: str) -> None:
    if sum(n.values()) != budget or any(v < 1 for v in n.values()) or any(r not in model.cover for r in n):
        raise AssertionError(f"{name}: vehicle sum {sum(n.values())} != {budget} or bad route")


def must_start(model: Model, budget: int) -> tuple[dict[str, int], dict[str, str]]:
    """필수 정류장 덮기 초기해. 돌려주는 값: (배분, 못 덮은 정류장 → 이유).

    아직 덮지 못한 필수 정류장을 덮는 후보 중 n0대로 열 때 근사 이득이 가장 큰 것부터 연다(동률: 새로 덮는 정류장 수,
    노선 ID). 지나는 후보가 없으면 no_candidate, 남은 대수로 열 수 있는 후보가 없으면 budget.
    must_min_tph > 0이면 덮은 노선에 그 운행이 될 때까지 1대씩 더한다.
    """
    n: dict[str, int] = {}
    uncovered = {s: "no_candidate" for s, rs in model.must.items() if not rs}
    while True:
        done = model.covered_set(n)
        todo = [s for s in model.must if s not in done and s not in uncovered]
        if not todo:
            break
        left = budget - sum(n.values())
        base = model.value(n)
        best = None
        for r in sorted({r for s in todo for r in model.must[s]}):
            if r in n:                              # 열렸는데 덮지 못함 = 시간당 운행 부족(must_min_tph): 1대 더
                k = 1
            else:                                   # n0대, must_min_tph가 있으면 그 운행에 필요한 대수까지
                k = max(model.n0[r], math.ceil(model.must_min_tph * model.cycle[r] / 3600 - EPS))
            if k > left:
                continue
            m = _with(n, r, k)
            newly = len(model.covered_set(m) - done)
            if newly <= 0 and r not in n:
                continue
            key = (model.value(m) - base, newly, -model.rank[r])
            if best is None or key > best[0]:
                best = (key, m)
        if best is None:
            for s in todo:
                uncovered[s] = "budget"
            break
        n = best[1]
    return n, uncovered


def allocate(model: Model, budget: int, starts: dict[str, dict[str, int]] | None = None) -> dict:
    """초기해 여러 개 → 각각 국소 탐색 → 근사 최대(동률이면 앞 초기해).

    starts: 추가 초기해(이름 → 배분). 예산과 차량 합이 같고 이 모델의 노선만 써야 한다.
    model.must가 있으면 덮을 수 있는 필수 정류장 덮음을 하드 제약으로 두고 결과에 must 보고를 붙인다.
    """
    if model.must:
        return _allocate_must(model, budget, starts)
    g = greedy(model, budget)
    v_greedy = model.value(g)
    f = fill(model, budget, g)
    inits = [("greedy", f), ("top_demand", top_demand_start(model, budget))]
    inits += sorted((starts or {}).items())
    total_stats = _new_stats()
    runs: dict[str, dict] = {}
    chosen = None
    for name, n0 in inits:
        _check(n0, budget, model, f"start {name}")
        st = _new_stats()
        n, log = local_search(model, budget, dict(n0), stats=st)
        _check(n, budget, model, f"local search from {name}")
        v = model.value(n)
        runs[name] = {"start": dict(sorted(n0.items())), "initial": model.value(n0), "final": v,
                      "n": dict(sorted(n.items())), "moves": log, "stats": st}
        for k in MOVES:
            for t in ("tries", "accepts"):
                total_stats[k][t] += st[k][t]
        if chosen is None or v > runs[chosen]["final"] + EPS:
            chosen = name
    n = runs[chosen]["n"]
    return {"n": n, "value": model.value(n), "chosen": chosen, "inits": runs, "move_stats": total_stats,
            "value_greedy": v_greedy, "value_filled": model.value(f), "greedy": dict(sorted(g.items())),
            "local_search": runs[chosen]["moves"]}


def _top_demand_from(model: Model, budget: int, n: dict[str, int]) -> dict[str, int]:
    """n 위에 덮는 수요가 큰 노선부터 n0대씩 연다(예산 안). 남는 대수는 fill."""
    n = dict(n)
    used = sum(n.values())
    for r in sorted(model.ids, key=lambda x: (-model.demand_of[x], x)):
        if r in n:
            continue
        k = model.n0[r]
        if used + k <= budget:
            n[r] = k
            used += k
        if used == budget:
            break
    return fill(model, budget, n)


def _allocate_must(model: Model, budget: int, starts: dict[str, dict[str, int]] | None) -> dict:
    base, uncovered = must_start(model, budget)
    model.required = frozenset(model.covered_set(base))
    g = greedy(model, budget, base)
    v_greedy = model.value(g)
    f = fill(model, budget, g)
    inits = [("greedy", f), ("top_demand", _top_demand_from(model, budget, base))]
    for name, st0 in sorted((starts or {}).items()):        # 호출자가 준 해는 제약을 지킬 때만 쓴다
        if model.feasible(st0):
            inits.append((name, st0))
    total_stats = _new_stats()
    runs: dict[str, dict] = {}
    chosen = None
    for name, n0 in inits:
        _check(n0, budget, model, f"start {name}")
        st = _new_stats()
        n, log = local_search(model, budget, dict(n0), stats=st)
        _check(n, budget, model, f"local search from {name}")
        if not model.feasible(n):
            raise AssertionError(f"local search from {name} broke the must-stop constraint")
        v = model.value(n)
        runs[name] = {"start": dict(sorted(n0.items())), "initial": model.value(n0), "final": v,
                      "n": dict(sorted(n.items())), "moves": log, "stats": st}
        for k in MOVES:
            for t in ("tries", "accepts"):
                total_stats[k][t] += st[k][t]
        if chosen is None or v > runs[chosen]["final"] + EPS:
            chosen = name
    n = runs[chosen]["n"]
    cov = model.covered_set(n)
    tph = model.must_tph(n)
    report = {"definition": "개설 노선 중 하나 이상이 첫·끝이 아닌 정차로 지남" +
                            (f" + 시간당 {model.must_min_tph}회 이상" if model.must_min_tph > 0 else ""),
              "must_min_tph": model.must_min_tph,
              "start": dict(sorted(base.items())), "start_vehicles": sum(base.values()),
              "stops": {s: {"routes": [r for r in model.must[s] if r in n], "covered": s in cov,
                            "trips_per_hour": round(tph[s], 4), "candidates": len(model.must[s])}
                        for s in model.must},
              "must_stop_uncovered": [{"stop": s, "reason": uncovered[s]} for s in sorted(uncovered)]}
    return {"n": n, "value": model.value(n), "chosen": chosen, "inits": runs, "move_stats": total_stats,
            "value_greedy": v_greedy, "value_filled": model.value(f), "greedy": dict(sorted(g.items())),
            "local_search": runs[chosen]["moves"], "must": report}
