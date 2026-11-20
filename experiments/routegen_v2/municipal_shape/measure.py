"""지자체 셔틀안(municipal/<구>/plan.json) 노선 모양 측정: 순환 여부, 회차, 방향별 우회율(직선 합 기준),
길 건너 짝(같은 이름 정류장이 오는 쪽에 있는지, 거리), 방향별 정차 수."""
import csv, json, math, statistics, sys
from pathlib import Path
W = Path('/mnt/c/Users/ddonggojin/orca/projects/guro_exp_work')
stops = {r['stop_id']: r for r in csv.DictReader(open(W/'zip_v4/seoul/seoul_routegen_inputs_0610_v4/network/stops.csv', encoding='utf-8'))}
def hv(a, b):
    la1, lo1, la2, lo2 = map(math.radians, (float(stops[a]['stop_lat']), float(stops[a]['stop_lon']), float(stops[b]['stop_lat']), float(stops[b]['stop_lon'])))
    h = math.sin((la2-la1)/2)**2 + math.cos(la1)*math.cos(la2)*math.sin((lo2-lo1)/2)**2
    return 2*6371008.8*math.asin(math.sqrt(h))
rows = []
for d in sorted((W/'municipal').iterdir()):
    p = d/'plan.json'
    if not p.is_file(): continue
    plan = json.load(open(p, encoding='utf-8'))
    for r in plan['routes']:
        s = [x for x in r['stops'] if x in stops]
        if len(s) < 3: continue
        t = max(range(len(s)), key=lambda i: (hv(s[0], s[i]), -i))       # 회차 = 첫 정류장에서 가장 먼 정류장
        close = hv(s[-1], s[0])
        L = lambda seq: sum(hv(a, b) for a, b in zip(seq, seq[1:]))
        fwd, back = s[:t+1], s[t:]
        st = hv(s[0], s[t])
        det_f = L(fwd)/st if st > 0 else None
        det_b = L(back)/hv(s[t], s[-1]) if hv(s[t], s[-1]) > 0 else None
        loopness = L(s)/(2*st) if st > 0 else None
        # 짝: 가는 쪽 정류장마다 오는 쪽에 같은 이름 정류장(다른 ID)이 있으면 거리
        pd = []
        for a in fwd[1:-1]:
            same = [b for b in back if b != a and stops[b]['stop_name'] == stops[a]['stop_name']]
            pd.append(min(hv(a, b) for b in same) if same else None)
        rows.append({'gu': d.name, 'route': r['route_id'], 'n': len(s), 'n_fwd': len(fwd), 'n_back': len(back),
                     'close_m': round(close), 'span_m': round(st), 'detour_fwd': det_f, 'detour_back': det_b,
                     'loop_total': loopness, 'pair_d': pd, 'loop': close <= 300})
json.dump(rows, open(W/'routegen_v2/municipal_shape/routes.json', 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
def q(v, ps=(.5, .75, .9)):
    v = sorted(x for x in v if x is not None)
    return [round(v[int(p*(len(v)-1))], 2) for p in ps] if v else None
lp = [r for r in rows if r['loop']]
print('구', len({r['gu'] for r in rows}), '노선', len(rows), '닫힌 고리(끝–첫 ≤300m)', len(lp), '열린', len(rows)-len(lp))
print('끝–첫 거리 q50/75/90', q([r['close_m'] for r in rows]))
print('[고리] 가는 쪽 우회율 q50/75/90', q([r['detour_fwd'] for r in lp]), ' 오는 쪽', q([r['detour_back'] for r in lp]), ' 전체 길이/(2×폭)', q([r['loop_total'] for r in lp]))
for lim in (1.6, 2.0):
    print(f'  오는 쪽 ≤{lim}', sum(1 for r in lp if r['detour_back'] and r['detour_back'] <= lim), '/', len(lp), f' 전체/(2×폭) ≤{lim}', sum(1 for r in lp if r['loop_total'] and r['loop_total'] <= lim))
pds = [x for r in lp for x in r['pair_d']]
have = [x for x in pds if x is not None]
print('[고리] 가는 쪽 중간 정류장', len(pds), '오는 쪽에 같은 이름 있음', len(have), f'({len(have)/max(1,len(pds)):.0%})', '거리 q50/75/90', q(have))
for lim in (50, 100, 150):
    print(f'  같은 이름 ≤{lim}m', sum(1 for x in have if x <= lim))
print('[고리] 방향별 정차 수 가는/오는 q50/90', q([r['n_fwd'] for r in lp], (.5, .9)), q([r['n_back'] for r in lp], (.5, .9)))
