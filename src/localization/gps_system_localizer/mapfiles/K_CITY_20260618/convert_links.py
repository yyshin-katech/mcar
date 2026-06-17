#!/usr/bin/env python3
"""
A2_LINK shp → mat 변환
EPSG:32652 → EPSG:5179, 2m 간격 보간
"""
import numpy as np
import scipy.io
import shapefile
from pyproj import Transformer

SHP_PATH = '/home/sim/shp_file/A2_LINK'
OUT_DIR  = '/home/sim/shp_file'

# 링크 ID → (파일번호, LINK_ID, NEXT_LINK_ID, is_stop_line, intersection_id, signal_group_id)
LINK_MAP = {
    'A2256W000085': dict(file_no=61,  link_id=61,  next_id=62, is_stop=0, iid=0,    sgid=0),
    'A2256W000229': dict(file_no=86,  link_id=86,  next_id=87, is_stop=0, iid=0,    sgid=0),
    'A2256W000231': dict(file_no=87,  link_id=87,  next_id=61, is_stop=1, iid=1500, sgid=10),
    'A2256W000327': dict(file_no=88,  link_id=88,  next_id=86, is_stop=1, iid=100,  sgid=16),
}
SPEED_LIMIT = 30
INTERVAL    = 2.0   # m

transformer = Transformer.from_crs('EPSG:32652', 'EPSG:5179', always_xy=True)


def interpolate_2m(pts_5179: np.ndarray):
    """
    pts_5179: (N,2) east/north
    반환: east(1,M), north(1,M), station(1,M)
    """
    east_in  = pts_5179[:, 0]
    north_in = pts_5179[:, 1]

    # 누적 거리 계산
    diffs = np.hypot(np.diff(east_in), np.diff(north_in))
    cum   = np.concatenate([[0.0], np.cumsum(diffs)])
    total = cum[-1]

    # 2m 간격 샘플 위치 (시작+끝 보장)
    sample_s = np.arange(0.0, total, INTERVAL)
    if sample_s[-1] < total - 1e-6:
        sample_s = np.append(sample_s, total)

    east_out  = np.interp(sample_s, cum, east_in)
    north_out = np.interp(sample_s, cum, north_in)

    return east_out.reshape(1, -1), north_out.reshape(1, -1), sample_s.reshape(1, -1)


def main():
    sf = shapefile.Reader(SHP_PATH)
    fields = [f[0] for f in sf.fields[1:]]
    id_idx = fields.index('ID')

    for sr in sf.shapeRecords():
        shp_id = sr.record[id_idx]
        if shp_id not in LINK_MAP:
            continue

        cfg = LINK_MAP[shp_id]
        pts_raw = np.array(sr.shape.points)  # (N,2) in EPSG:32652

        # 좌표 변환
        ex, ny = transformer.transform(pts_raw[:, 0], pts_raw[:, 1])
        pts_5179 = np.column_stack([ex, ny])

        east, north, station = interpolate_2m(pts_5179)

        mat_data = {
            'east':                     east,
            'north':                    north,
            'LINK_ID':                  np.array([[cfg['link_id']]], dtype=np.float64),
            'NEXT_LINK_ID':             np.array([[cfg['next_id']]], dtype=np.float64),
            'have_to_LangeChange_right': np.array([[0]], dtype=np.float64),
            'have_to_LangeChange_left':  np.array([[0]], dtype=np.float64),
            'left_LaneChange_avail':     np.array([[0]], dtype=np.float64),
            'right_LaneChange_avail':    np.array([[0]], dtype=np.float64),
            'RIGHT_LINK_ID':             np.array([[0]], dtype=np.float64),
            'LEFT_LINK_ID':              np.array([[0]], dtype=np.float64),
            'is_stop_line':              np.array([[cfg['is_stop']]], dtype=np.float64),
            'Speed_Limit':               np.array([[SPEED_LIMIT]], dtype=np.float64),
            'look_at_signalGroupID':     np.array([[cfg['sgid']]], dtype=np.float64),
            'look_at_IntersectionID':    np.array([[cfg['iid']]], dtype=np.float64),
            'station':                   station,
        }

        out_path = f"{OUT_DIR}/link_{cfg['file_no']}.mat"
        scipy.io.savemat(out_path, mat_data)
        print(f"[OK] {shp_id} → link_{cfg['file_no']}.mat  "
              f"pts={east.shape[1]}  total={station[0,-1]:.2f}m  "
              f"is_stop={cfg['is_stop']}  iid={cfg['iid']}  sgid={cfg['sgid']}")


if __name__ == '__main__':
    main()
