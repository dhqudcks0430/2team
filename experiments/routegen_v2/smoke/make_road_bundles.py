"""스모크 시뮬레이션 묶음(10/28 affected)을 버스 통행 가능 도로망으로 만든다.

    python3 -I make_road_bundles.py <2team-routegen-v2 저장소>

원본 묶음(zip_v4/region/<구>_0610_v4/bundles/affected_20261028)을 복사하고 travel_times.json만
review1119/stage1c/nets/<구>/road.json으로 바꾼 뒤, run_inputs를 시뮬레이터 함수(build_run_inputs)로 다시 만든다
(fork run_networks.bundles_for와 같음). 결과: road_bundles/<구>/affected_20261028/
"""
import json
import shutil
import sys
from pathlib import Path

REPO = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(REPO / "src"))
from busstrike.sim.arrivals import build_run_inputs  # noqa: E402

HERE = Path(__file__).resolve().parent
W = HERE.parents[1]
BUNDLE = "affected_20261028"
for g in ("guro", "gangseo", "gangdong"):
    src = W / "zip_v4" / "region" / f"{g}_0610_v4" / "bundles" / BUNDLE
    road = W / "review1119" / "stage1c" / "nets" / g / "road.json"
    t = HERE / "road_bundles" / g / BUNDLE
    t.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src / "demand_bins.csv", t / "demand_bins.csv")
    shutil.copyfile(road, t / "travel_times.json")
    fleets = sorted(int(p.stem.split("_f")[1]) for p in src.glob("scenario_f*.json"))
    for f in fleets:
        shutil.copyfile(src / f"scenario_f{f}.json", t / f"scenario_f{f}.json")
        ri = json.loads((src / f"run_inputs_f{f}.json").read_text(encoding="utf-8"))
        build_run_inputs(ri["input_bundle_id"] + "-road-stage1c", t / f"scenario_f{f}.json", t / "demand_bins.csv",
                         t / "travel_times.json", ri["seed"], t / f"run_inputs_f{f}.json")
    print(g, fleets, "→", t)
