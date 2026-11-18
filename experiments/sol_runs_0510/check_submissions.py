"""run-XX/sol-run-XX.json 제출물 형식 검사 (inputs/stops.csv 기준, 차량 합계 11).

사용: python check_submissions.py [실험 폴더]  (기본: 이 파일이 있는 폴더)
"""
import csv
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
FLEET = 11


def check(run_dir: Path, stops: set[str]) -> list[str]:
    rid = run_dir.name
    errs = []
    if not (run_dir / f"sol-{rid}.md").exists():
        errs.append("md 없음")
    plan = json.loads((run_dir / f"sol-{rid}.json").read_text(encoding="utf-8"))
    ids = [r["route_id"] for r in plan["routes"]]
    if len(ids) != len(set(ids)):
        errs.append("route_id 중복")
    total = 0
    for r in plan["routes"]:
        s, v = r["stops"], r["vehicles"]
        if set(r) != {"route_id", "stops", "vehicles"}:
            errs.append(f"{r['route_id']}: 필드 {sorted(r)}")
        if not all(isinstance(x, str) for x in s):
            errs.append(f"{r['route_id']}: 문자열 아닌 ID")
        unknown = [x for x in s if x not in stops]
        if unknown:
            errs.append(f"{r['route_id']}: 없는 정류장 {unknown}")
        if len(set(s)) < 2:
            errs.append(f"{r['route_id']}: 정류장 2개 미만")
        if any(a == b for a, b in zip(s, s[1:])):
            errs.append(f"{r['route_id']}: 연속 중복")
        if isinstance(v, int) and not isinstance(v, bool) and v > 0:
            total += v
        else:
            errs.append(f"{r['route_id']}: 차량 {v!r}")
    if total != FLEET:
        errs.append(f"차량 합계 {total}")
    print(rid, f"노선 {len(ids)}", f"차량 {total}", "통과" if not errs else errs)
    return errs


def main() -> int:
    exp_dir = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else HERE
    with open(HERE / "inputs" / "stops.csv", encoding="utf-8-sig") as f:
        stops = {row["stop_id"] for row in csv.DictReader(f)}
    bad = 0
    for run_dir in sorted(exp_dir.glob("run-*")):
        bad += bool(check(run_dir, stops))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
