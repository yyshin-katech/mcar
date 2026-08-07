#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""extract_route.py — global_nav_hmi 주행 예정 경로 오프라인 추출기.

A2_LINK.shp(senario_shp_20260623, WGS_1984_UTM_Zone_52N = EPSG:32652)에서
위상정렬 완료된 중앙 대표선 링크열(OLD/NEW)의 지오메트리를 EPSG:5179 절대좌표
시퀀스로 이어붙여 route JSON을 1회 생성한다. 런타임 재추출 없음(확정 결정 2) —
web_hmi_threejs_bridge.py 가 이 JSON을 읽어 /hmi/threejs/route 로 latch 발행한다.

의존: pyshp(shapefile), pyproj(Transformer) — bridge 와 동일 라이브러리.
좌표변환: Transformer.from_crs("EPSG:32652","EPSG:5179",always_xy=True)
          (web_hmi_threejs_bridge._publish_map_once 와 동일).

사용:
  python3 extract_route.py                 # 기본 shp → route_senario_20260623.json
  python3 extract_route.py --shp <path> --out <path>
  python3 extract_route.py --bfs           # 빈구간 gap-fill 링크 BFS 재확인(참고)

기대 산출(검증값):
  OLD 299pts end=[931884.13,1929013.76]
  NEW 246pts end=[931688.57,1928916.96]
  shared 161pts branch=[931706.40,1928787.28]
"""
import argparse
import json
import math
import os
import sys
from collections import defaultdict, deque

try:
    import shapefile  # pyshp
except ImportError:
    sys.exit("extract_route: pyshp not installed (pip install pyshp)")

try:
    from pyproj import Transformer
except ImportError:
    sys.exit("extract_route: pyproj not installed (pip install pyproj)")

# A2_LINK .prj = WGS_1984_UTM_Zone_52N. bridge 와 동일하게 EPSG:5179 로 reproject.
SRC_EPSG = "EPSG:32652"
DST_EPSG = "EPSG:5179"

# 위상정렬 완료 중앙 대표선 링크열 (00_constraints / 01_spec 확정, 재정렬 금지).
#   old/new 앞 6링크(785093..785190) 완전 동일 = 분기 전 공유 구간.
OLD = [
    "A222BF785093", "A222BF785104", "A222BF785105", "A222BF785110",
    "A222BF785189", "A222BF785190", "A222BF785059", "A222BF785065",
    "A222BF785203", "A222BF785208",
]
NEW = [
    "A222BF785093", "A222BF785104", "A222BF785105", "A222BF785110",
    "A222BF785189", "A222BF785190", "A222BF785057", "A222BF785082",
    "A222BF795017", "A222BF795018", "A222BF795117",
]
SHARED_N = 6                    # 785093..785190 (분기 전 공유 링크 수)
SHARED_LINK = "A222BF785190"    # 공유 중앙차선 마지막 링크

DEDUP_EPS = 0.005               # m — 이음매 중복점(직전 점과 <5mm) 제거 임계

# 스크립트 위치(scripts/) 기준 기본 경로 (bridge 의 mapdir 규약과 동일한 상대 이동).
_HERE = os.path.dirname(os.path.realpath(__file__))
DEFAULT_SHP = os.path.join(
    _HERE, "..", "..", "..", "localization", "gps_system_localizer",
    "src", "shp_map", "senario_shp_20260623", "A2_LINK.shp",
)
DEFAULT_OUT = os.path.join(_HERE, "route_senario_20260623.json")


def build_id_index(sf, fields):
    """ID 필드 값 -> shape index 매핑(첫 등장 우선)."""
    id_idx = fields.index("ID")
    idx = {}
    for i, rec in enumerate(sf.records()):
        idx.setdefault(rec[id_idx], i)
    return idx


def link_geom(sf, tx, shp_index):
    """링크 shape.points 를 EPSG:5179 [round(e,2),round(n,2)] 리스트로 변환."""
    shp = sf.shape(shp_index)
    out = []
    for x, y in shp.points:
        e, n = tx.transform(x, y)
        out.append([round(e, 2), round(n, 2)])
    return out


def concat(link_ids, sf, tx, id_map, seams=None):
    """링크별 geom 을 순서대로 append. 이음매 중복점(<DEDUP_EPS) skip.

    seams(list)가 주어지면 각 링크 경계의 (from_link, to_link, dist_m) 를 채워
    연결성 자가점검에 사용한다. 커넥터(측방 전이)는 dist>0 로 남는다.
    """
    pts = []
    prev_link = None
    for lid in link_ids:
        if lid not in id_map:
            raise KeyError("link id not in A2_LINK ID field: %s" % lid)
        g = link_geom(sf, tx, id_map[lid])
        if not g:
            continue
        if pts and seams is not None:
            d = math.hypot(g[0][0] - pts[-1][0], g[0][1] - pts[-1][1])
            seams.append((prev_link, lid, round(d, 3)))
        for p in g:
            if pts and abs(p[0] - pts[-1][0]) < DEDUP_EPS \
                    and abs(p[1] - pts[-1][1]) < DEDUP_EPS:
                continue  # 이음매 중복점 제거
            pts.append(p)
        prev_link = lid
    return pts


def bfs_gap(sf, fields, start_link, goal_link):
    """빈구간 gap-fill 재확인(참고/옵션).

    링크 그래프(A.ToNodeID == B.FromNodeID 이면 A→B 인접)에서 start_link →
    goal_link 최단 경로를 BFS 로 찾는다. 경로의 중간 링크(path[1:-1])가 gap-fill.
    사양 확정값: 785104 → 785110 최단경로 = [785104, A222BF785105, 785110]
                 → gap-fill = ['A222BF785105'] (유일).
    링크열이 이미 이를 포함하므로 런타임 BFS 는 불필요(경로 변경 시 재사용용).
    """
    id_idx = fields.index("ID")
    fn_idx = fields.index("FromNodeID")
    tn_idx = fields.index("ToNodeID")
    recs = sf.records()
    by_id = {r[id_idx]: r for r in recs}
    out_links = defaultdict(list)          # FromNode -> [link ...]
    for r in recs:
        out_links[r[fn_idx]].append(r[id_idx])
    if start_link not in by_id or goal_link not in by_id:
        return None
    q = deque([(start_link, [start_link])])
    seen = {start_link}
    while q:
        cur, path = q.popleft()
        if cur == goal_link:
            return path
        for nxt in out_links.get(by_id[cur][tn_idx], []):
            if nxt not in seen:
                seen.add(nxt)
                q.append((nxt, path + [nxt]))
    return None


def parse_args():
    ap = argparse.ArgumentParser(description="global_nav_hmi route extractor")
    ap.add_argument("--shp", default=DEFAULT_SHP, help="A2_LINK.shp path")
    ap.add_argument("--out", default=DEFAULT_OUT, help="output route JSON path")
    ap.add_argument("--bfs", action="store_true",
                    help="빈구간 gap-fill 링크 BFS 재확인 후 종료(참고용)")
    return ap.parse_args()


def main():
    args = parse_args()
    if not os.path.isfile(args.shp):
        sys.exit("extract_route: shp not found: %s" % args.shp)

    sf = shapefile.Reader(args.shp)
    fields = [f[0] for f in sf.fields[1:]]

    if args.bfs:
        # 빈구간(785104.ToNode → 785110.FromNode) 연결 링크 재확인.
        path = bfs_gap(sf, fields, "A222BF785104", "A222BF785110")
        print("BFS 785104->785110 path:", path)
        if path:
            print("gap-fill (path[1:-1]):", path[1:-1])
        return

    id_map = build_id_index(sf, fields)
    tx = Transformer.from_crs(SRC_EPSG, DST_EPSG, always_xy=True)

    old_seams, new_seams = [], []
    old_pts = concat(OLD, sf, tx, id_map, seams=old_seams)
    new_pts = concat(NEW, sf, tx, id_map, seams=new_seams)

    # 검증: 공유 prefix(앞 SHARED_N 링크) 좌표 시퀀스 동일.
    shared_old = concat(OLD[:SHARED_N], sf, tx, id_map)
    shared_new = concat(NEW[:SHARED_N], sf, tx, id_map)
    assert shared_old == shared_new, "shared prefix mismatch (OLD[:N] != NEW[:N])"
    shared_point_count = len(shared_old)
    branch_pt = shared_old[-1]

    # 강한 자가점검: 완성된 old/new 경로의 앞 shared_point_count 점도 완전 일치.
    assert old_pts[:shared_point_count] == new_pts[:shared_point_count], \
        "old/new front points diverge before branch"

    data = {
        "epsg": 5179,
        "branch": {
            "east": branch_pt[0],
            "north": branch_pt[1],
            "shared_link": SHARED_LINK,
            "shared_point_count": shared_point_count,
        },
        "routes": {
            "old": {"link_ids": OLD, "points": old_pts},
            "new": {"link_ids": NEW, "points": new_pts},
        },
    }
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, separators=(",", ":"))

    # 검증 로그.
    print("wrote %s (%d B)" % (args.out, os.path.getsize(args.out)))
    print("OLD %dlinks %dpts end=%s" % (len(OLD), len(old_pts), old_pts[-1]))
    print("NEW %dlinks %dpts end=%s" % (len(NEW), len(new_pts), new_pts[-1]))
    print("shared %dpts branch=[%.2f,%.2f]"
          % (shared_point_count, branch_pt[0], branch_pt[1]))
    print("OLD seams (from,to,dist_m):", old_seams)
    print("NEW seams (from,to,dist_m):", new_seams)


if __name__ == "__main__":
    main()
