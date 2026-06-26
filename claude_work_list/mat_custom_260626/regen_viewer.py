#!/usr/bin/env python3
"""mat_viewer_senario_260514c 1.html 의 var DATA 라인을 변경된 mat 셋으로 재생성."""
import os, glob, json
import numpy as np
import scipy.io as sio
from pyproj import Transformer

MAP_DIR = '/home/sim/mcar/src/localization/gps_system_localizer/mapfiles/senario'
VIEWER  = os.path.join(MAP_DIR, 'mat_viewer_senario_260514c 1.html')
DATA_LINE = 820  # 1-based

t = Transformer.from_crs('EPSG:5179', 'EPSG:4326', always_xy=True)

# 1) 모든 mat 로드
db = {}  # lid -> (filepath, dict)
for f in sorted(glob.glob(os.path.join(MAP_DIR, 'link_*.mat'))):
    d = sio.loadmat(f)
    lid = int(d['LINK_ID'][0, 0])
    db[lid] = (f, d)

print(f'loaded {len(db)} mat files')


def s(d, k, default=0):
    if k not in d:
        return default
    v = d[k]
    if isinstance(v, np.ndarray):
        if v.dtype.kind in 'iuf':
            return int(v.flatten()[0]) if v.dtype.kind != 'f' else float(v.flatten()[0])
        return str(v[0]) if v.size else default
    return v


# 2) features 빌드
features = []
for lid in sorted(db.keys()):
    fpath, d = db[lid]
    e = d['east'].flatten()
    n = d['north'].flatten()
    lon, lat = t.transform(e, n)
    coords = [[round(float(lo), 7), round(float(la), 7)] for lo, la in zip(lon, lat)]
    sta = d['station'].flatten()
    nx = int(d['NEXT_LINK_ID'][0, 0])

    gap = 0.0
    has_gap = 0
    no_next = 0
    if nx == 0:
        no_next = 1
    elif nx in db:
        nd = db[nx][1]
        ne = nd['east'].flatten()[0]
        nn_ = nd['north'].flatten()[0]
        gap = float(((e[-1] - ne) ** 2 + (n[-1] - nn_) ** 2) ** 0.5)
        has_gap = 1 if gap > 0.01 else 0
    else:
        no_next = 1

    lid_string = ''
    if 'LINK_ID_string' in d:
        v = d['LINK_ID_string']
        if isinstance(v, np.ndarray) and v.size:
            try:
                lid_string = str(v[0])
            except Exception:
                lid_string = ''

    props = {
        "file": os.path.basename(fpath),
        "start_lon": coords[0][0],
        "start_lat": coords[0][1],
        "end_lon": coords[-1][0],
        "end_lat": coords[-1][1],
        "station_start": round(float(sta[0]), 2),
        "station_end": round(float(sta[-1]), 2),
        "LINK_ID": lid,
        "NEXT_LINK_ID": nx,
        "RIGHT_LINK_ID": int(d['RIGHT_LINK_ID'][0, 0]),
        "LEFT_LINK_ID": int(d['LEFT_LINK_ID'][0, 0]),
        "have_to_LangeChange_right": int(d['have_to_LangeChange_right'][0, 0]),
        "have_to_LangeChange_left": int(d['have_to_LangeChange_left'][0, 0]),
        "right_LaneChange_avail": int(d['right_LaneChange_avail'][0, 0]),
        "left_LaneChange_avail": int(d['left_LaneChange_avail'][0, 0]),
        "is_stop_line": int(d['is_stop_line'][0, 0]),
        "Speed_Limit": int(d['Speed_Limit'][0, 0]),
        "look_at_signalGroupID": int(d['look_at_signalGroupID'][0, 0]),
        "look_at_IntersectionID": int(d['look_at_IntersectionID'][0, 0]),
        "guard_zone": int(d['guard_zone'][0, 0]),
        "MANUAVER": int(d['MANUAVER'][0, 0]),
        "LINK_ID_string": lid_string,
        "point_count": len(coords),
        "length_m": round(float(sta[-1]), 2),
        "gap_next_m": round(gap, 2),
        "has_gap_next": has_gap,
        "no_next_link": no_next,
    }
    features.append({
        "type": "Feature",
        "geometry": {"type": "LineString", "coordinates": coords},
        "properties": props,
    })

new_data = {"type": "FeatureCollection", "features": features}
new_line = "var DATA = " + json.dumps(new_data, separators=(", ", ": "), ensure_ascii=False) + ";"

# 3) line 820 만 교체
with open(VIEWER, 'r', encoding='utf-8') as fp:
    lines = fp.readlines()

print(f'before: line {DATA_LINE} len = {len(lines[DATA_LINE-1])}')
lines[DATA_LINE-1] = new_line + '\n'
print(f'after:  line {DATA_LINE} len = {len(lines[DATA_LINE-1])}')

with open(VIEWER, 'w', encoding='utf-8') as fp:
    fp.writelines(lines)

print(f'wrote: {VIEWER}')
print(f'features = {len(features)}')

# 통계: 변경된 링크들 확인
target_ids = [870, 877, 871, 3870, 3877, 3871, 1204, 1205]
print('\n[변경/신규 링크 정보 in 뷰어]')
for lid in target_ids:
    for feat in features:
        if feat['properties']['LINK_ID'] == lid:
            p = feat['properties']
            print(f"  link_{lid}: pc={p['point_count']}, len={p['length_m']}, stop={p['is_stop_line']}, next={p['NEXT_LINK_ID']}, R={p['RIGHT_LINK_ID']}, L={p['LEFT_LINK_ID']}, gap_next={p['gap_next_m']}")
            break

DEL = [786, 785, 792, 798, 791, 790, 567, 781, 764, 793, 789, 795]
present_deleted = [lid for lid in DEL if any(f['properties']['LINK_ID']==lid for f in features)]
print(f'\n[삭제된 ID 중 뷰어에 잔재]: {present_deleted}')
