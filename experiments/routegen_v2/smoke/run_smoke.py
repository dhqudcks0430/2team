"""routegen v2 스모크: 구로·강서·강동, 10/28 하루, affected, 묶음 대수 2조건 — 생성(이미 함) → 변환기 → M05 → M07.

    python3 -I run_smoke.py <2team-routegen-v2 저장소> <조건 폴더 이름(A|B)> [구 ...]

성능 주장 없음. 생성·시뮬레이션 모두 버스 통행 가능 도로망(review1119/stage1c/nets/<구>). 표: 노선 수(편도/순회형), 필수 정류장 덮음 여부, 충족률, M07 결과.
"""
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(sys.argv[1]).resolve()
W = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent / sys.argv[2]
GUS = sys.argv[3:] or ["guro", "gangseo", "gangdong"]
BUNDLE = "affected_20261028"

TABLE = HERE / "smoke_table.json"
rows = [r for r in (json.loads(TABLE.read_text(encoding="utf-8")) if TABLE.exists() else []) if r["gu"] not in GUS]
for g in GUS:
    rg = HERE / g
    man = json.loads((rg / "manifest.json").read_text(encoding="utf-8"))
    view = HERE / "views" / g / "bundles"
    view.mkdir(parents=True, exist_ok=True)
    link = view / BUNDLE
    if not link.exists():
        # 버스 통행 가능 도로망 묶음(make_road_bundles.py). 묶음 정류장 구간망 묶음은 쓰지 않는다
        link.symlink_to(Path(__file__).resolve().parent / "road_bundles" / g / BUNDLE, target_is_directory=True)
    cmd = [sys.executable, str(REPO / "experiments" / "guro_exp" / "run_matrix.py"), "--bundles", str(view),
           "--out", str(HERE / "runs" / g), "--src", str(REPO / "src"), "--jobs", "4", "--data-label", f"zip_v4/{g}"]
    for p in sorted(man["plans"], key=lambda x: (x["label"], x["fleet"])):
        cmd += ["--plan", f"{p['label']}:{p['fleet']}={p['file']}"]
    rc = subprocess.run(cmd, check=False).returncode
    index = json.loads((HERE / "runs" / g / "runs.json").read_text(encoding="utf-8"))
    for p in sorted(man["plans"], key=lambda x: (x["label"], x["fleet"])):
        rep = json.loads(Path(p["plan_report"]).read_text(encoding="utf-8"))
        run_dir = HERE / "runs" / g / p["label"] / BUNDLE / f"f{p['fleet']}"
        met = json.loads((run_dir / "run" / "metrics.json").read_text(encoding="utf-8")) if (run_dir / "run" / "metrics.json").exists() else None
        ver = json.loads((run_dir / "verifier_report.json").read_text(encoding="utf-8")) if (run_dir / "verifier_report.json").exists() else None
        conv = json.loads((run_dir / "conversion_report.json").read_text(encoding="utf-8")) if (run_dir / "conversion_report.json").exists() else None
        m1 = (met or {}).get("M1", {})
        must = rep["must"]
        rows.append({
            "gu": g, "label": p["label"], "fleet": p["fleet"], "routes": len(rep["routes"]),
            "oneway": rep["shape_counts"]["oneway"], "loop": rep["shape_counts"]["loop"],
            "must_n": len(must.get("stops", {})) if must.get("enabled") else 0,
            "must_ok": sum(1 for v in must.get("stops", {}).values() if v["ok"]) if must.get("enabled") else 0,
            "must_infeasible": must.get("infeasible", []) if must.get("enabled") else [],
            "conversion_ok": (conv or {}).get("ok"),
            "served": m1.get("served"), "total": m1.get("total") or m1.get("eligible"),
            "rate": m1.get("rate"),
            "m07_pass": (ver or {}).get("pass"), "m07_fully": (ver or {}).get("fully_verified"),
            "m07_not_verified": (ver or {}).get("not_verified_checks"), "m07_failed": (ver or {}).get("failed_checks"),
            "approx_served": rep["approx_served"], "run_matrix_rc": rc,
        })
rows.sort(key=lambda r: (["guro", "gangseo", "gangdong"].index(r["gu"]), r["label"] != "ours", r["fleet"]))
TABLE.write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")
print("| 구 | 안 | 대수 | 노선(편도/순회형) | 필수 정류장 덮음 | 충족(M05) | M07 |")
print("| --- | --- | --- | --- | --- | --- | --- |")
for r in rows:
    must = f"{r['must_ok']}/{r['must_n']}" + (f" (못 덮음 {', '.join(r['must_infeasible'])})" if r["must_infeasible"] else "") if r["must_n"] else "필수 없음"
    rate = f"{r['served']}/{r['total']} = {r['rate'] * 100:.2f}%" if r["rate"] is not None else "—"
    m07 = ("pass" if r["m07_pass"] else f"fail {r['m07_failed']}") + (f", 미검증 {','.join(r['m07_not_verified'])}" if r["m07_not_verified"] else "")
    print(f"| {r['gu']} | {r['label']} | {r['fleet']} | {r['routes']} ({r['oneway']}/{r['loop']}) | {must} | {rate} | {m07} |")
