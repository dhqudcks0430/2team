# routegen v2 실험 스크립트

생성기 코드(`src/busstrike/planning/routegen/`)를 돌린 실험 스크립트다. 데이터·결과는 저장소 밖
`guro_exp_work/routegen_v2/`에 있고, 스크립트 안의 경로도 그 폴더를 가리킨다(이 PC 기준 절대 경로). 결과 파일은 올리지 않는다.

| 폴더 | 내용 |
| --- | --- |
| `smoke/` | v2 스모크(구로·강서·강동, 10/28 하루): 도로망 시뮬 묶음 만들기, 변환기 → M05 → M07, /lab 지도 폴더 만들기 |
| `eval/run_eval_guro.py` | 구로 노선안 실험 주간 5일 평가(우리 안·피더안·지자체안) |
| `strike_curve/run_curve.py` | 파업 비율 곡선(생성 = 실험 주간, 평가 = 테스트 주간; 우리 안·지자체안·범용 AI안 6개; 대조군 = 파업 없음) |
| `strike_curve/fix_return_links.py` | 범용 AI안·지자체안 망에 공차 복귀(끝 → 첫) OSM 이음이 없으면 더함(지자체안 이음과 같은 규칙) |
| `municipal_shape/measure.py` | 지자체 셔틀안(25개 구) 노선 모양 측정(순환 여부, 방향별 우회율, 길 건너 짝 거리) |

실행 환경 메모: Python 3.14의 ProcessPoolExecutor 보조 프로세스가 작업 환경의 작은 메모리 한도 묶음에서 강제 종료돼,
곡선 실험은 `--workers 1`로 묶음(`--only`)별 작업을 따로 띄워 병렬로 돌렸다.
