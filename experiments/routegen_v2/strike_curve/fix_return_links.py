"""범용 AI안·지자체안 망에 공차 복귀(끝 → 첫 정류장) 이음이 없으면 더한다.

    python3 -I fix_return_links.py <2team-routegen-v2 저장소> <2team-lab 저장소> ai/ai06 ai/ai09 ...

eval_municipal은 연속 정차 사이에만 OSM 이음을 더하고, 변환기가 붙이는 공차 복귀(끝 → 첫)는 보지 않는다. 그 경로가 망에
없으면 변환기 RETURN에서 그 안 전체가 실패한다. 같은 규칙(RoadNet.osm_link: 원 OSM 도로 최단 경로, U턴만 금지, 시간 =
도로 길이 ÷ 그 도로 시간대별 실측 속도)으로 끝 → 첫 이음을 road_muni.json에 더하고 muni.json에 return_links로 남긴다.
plan.json(노선·정차)은 바꾸지 않는다.
"""
import json
import sys
from pathlib import Path

REPO, LAB = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
sys.path[:0] = [str(REPO / "src"), str(LAB / "experiments" / "routegen_lab")]
from busstrike.sim import io as sio  # noqa: E402
from busstrike.sim.adapters.comparison_plan import _return_path  # noqa: E402
from busstrike.sim.traveltime import TravelTimeTable  # noqa: E402

HERE = Path(__file__).resolve().parent
W = HERE.parents[1]
ROOT = W / "zip_v4" / "region" / "guro_0610_v4"
rn = None
for d in sys.argv[3:]:
    d = (HERE / d).resolve()
    doc = json.loads((d / "road_muni.json").read_text(encoding="utf-8"))
    tt = TravelTimeTable.from_dict(doc)
    plan = json.loads((d / "plan.json").read_text(encoding="utf-8"))
    missing = [(r["route_id"], r["stops"][-1], r["stops"][0]) for r in plan["routes"]
               if _return_path(tt, r["stops"][-1], r["stops"][0], 21600) is None]
    rec = json.loads((d / "muni.json").read_text(encoding="utf-8"))
    rec["return_links"] = []
    if missing:
        if rn is None:
            from roadnet import RoadNet
            rn = RoadNet(W / "osm" / "guro_roads.json", ROOT)
            rn.build()
        have = {(s["from_stop_id"], s["to_stop_id"]) for s in doc["segments"]}
        for rid, a, b in missing:
            got = rn.osm_link(a, b, via=[])
            if got is None:
                rec["return_links"].append({"route_id": rid, "from": a, "to": b, "unreachable": True})
                continue
            if (a, b) not in have:
                doc["segments"].append({"from_stop_id": a, "to_stop_id": b, "profile": got["profile"],
                                        "distance_m": got["length_m"]})
            rec["return_links"].append({"route_id": rid, "from": a, "to": b, "osm_m": got["length_m"],
                                        "time_s": {str(t0): t for t0, t in got["profile"]}, "time_rule": "osm_speed"})
        doc["segments"].sort(key=lambda s: (s["from_stop_id"], s["to_stop_id"]))
        (d / "road_muni.json").write_text(json.dumps(doc, separators=(",", ":")), encoding="utf-8")
    (d / "muni.json").write_text(json.dumps(rec, ensure_ascii=False, indent=1), encoding="utf-8")
    tt2 = TravelTimeTable.from_dict(json.loads((d / "road_muni.json").read_text(encoding="utf-8")))
    still = [r["route_id"] for r in plan["routes"] if _return_path(tt2, r["stops"][-1], r["stops"][0], 21600) is None]
    print(d.name, "복귀 이음 추가", len(rec["return_links"]), [x["route_id"] for x in rec["return_links"]], "· 여전히 없음", still)
