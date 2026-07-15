#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""extract_map_data.py

Convert per-link .mat files in mapfiles/senario/ (EPSG:5179 Korean TM) to
two WGS84 lat/lon JSON files used by the spat_viewer web/index.html.

Each link_*.mat carries: east, north, LINK_ID, look_at_IntersectionID,
look_at_signalGroupID, is_stop_line, MANUAVER, Speed_Limit, ...
We use those keys directly — no shapefile or auxiliary CSV needed.

Outputs (relative to repo root):
  src/visualization/spat_viewer/web/data/intersections.json   (15 entries)
  src/visualization/spat_viewer/web/data/road_links.json      (308 features)

Inputs:
  src/localization/gps_system_localizer/mapfiles/senario/link_*.mat

Run:
  cd /home/katech/mcar_v13
  python3 src/visualization/spat_viewer/scripts/extract_map_data.py
"""

from __future__ import annotations

import json
import os
import sys
from collections import defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.abspath(os.path.join(HERE, '..', '..', '..', '..'))

MAT_DIR = os.path.join(
    REPO_ROOT,
    'src/localization/gps_system_localizer/mapfiles/senario',
)
OUT_DIR = os.path.join(
    REPO_ROOT,
    'src/visualization/spat_viewer/web/data',
)
OUT_INTERSECTIONS = os.path.join(OUT_DIR, 'intersections.json')
OUT_ROAD_LINKS = os.path.join(OUT_DIR, 'road_links.json')

EPSG_SOURCE = 'EPSG:5179'
EPSG_TARGET = 'EPSG:4326'


def main() -> int:
    try:
        import scipy.io as sio
    except ImportError:
        print('ERROR: scipy is required. pip install scipy', file=sys.stderr)
        return 1
    try:
        from pyproj import Transformer
    except ImportError:
        print('ERROR: pyproj is required. pip install pyproj', file=sys.stderr)
        return 1

    if not os.path.isdir(MAT_DIR):
        print(f'ERROR: mat directory not found: {MAT_DIR}', file=sys.stderr)
        return 1

    mat_files = sorted(f for f in os.listdir(MAT_DIR) if f.startswith('link_') and f.endswith('.mat'))
    if not mat_files:
        print(f'ERROR: no link_*.mat files in {MAT_DIR}', file=sys.stderr)
        return 1

    print(f'[extract] reading {len(mat_files)} link mats from {MAT_DIR}')
    os.makedirs(OUT_DIR, exist_ok=True)

    tf = Transformer.from_crs(EPSG_SOURCE, EPSG_TARGET, always_xy=True)

    links: list[dict] = []
    iid_agg: dict[int, dict] = defaultdict(lambda: {
        'sg': set(),
        'stop_endpoints': [],
        'all_endpoints': [],
        'link_ids': set(),
        'man': set(),
    })

    vertex_count = 0
    for fname in mat_files:
        m = sio.loadmat(os.path.join(MAT_DIR, fname))
        try:
            east = m['east'].ravel()
            north = m['north'].ravel()
            lid = int(m['LINK_ID'][0, 0])
            iid = int(m['look_at_IntersectionID'][0, 0])
            sg = int(m['look_at_signalGroupID'][0, 0])
            sl = int(m['is_stop_line'][0, 0])
            man = int(m['MANUAVER'][0, 0])
            spd = int(m['Speed_Limit'][0, 0])
            lid_str = str(m['LINK_ID_string'][0]) if 'LINK_ID_string' in m else ''
            nl = int(m['NEXT_LINK_ID'][0, 0]) if 'NEXT_LINK_ID' in m else 0
        except (KeyError, IndexError) as e:
            print(f'  skip {fname}: missing key {e}', file=sys.stderr)
            continue

        if len(east) < 2:
            continue

        lon_arr, lat_arr = tf.transform(east, north)
        coords = [[round(float(lat_arr[i]), 7), round(float(lon_arr[i]), 7)]
                  for i in range(len(east))]
        vertex_count += len(coords)

        link_entry = {
            'link_id': lid,
            'link_id_string': lid_str,
            'next_link_id': nl,
            'look_at_IntersectionID': iid,
            'look_at_signalGroupID': sg,
            'is_stop_line': sl,
            'MANUAVER': man,
            'Speed_Limit': spd,
            'mat_file': fname,
            'coords': coords,
        }
        links.append(link_entry)

        if iid > 0:
            e = iid_agg[iid]
            if sg > 0:
                e['sg'].add(sg)
            e['link_ids'].add(lid)
            e['man'].add(man)
            end = (coords[-1][0], coords[-1][1])
            e['all_endpoints'].append(end)
            if sl == 1:
                e['stop_endpoints'].append(end)

    # Build intersections list — stop_line endpoint avg if available, else
    # fall back to all-link endpoint avg.
    intersections: list[dict] = []
    for iid in sorted(iid_agg.keys()):
        e = iid_agg[iid]
        if e['stop_endpoints']:
            pts = e['stop_endpoints']
            source = 'stop_line_avg'
        else:
            pts = e['all_endpoints']
            source = 'link_endpoint_avg'
        lat = sum(p[0] for p in pts) / len(pts)
        lon = sum(p[1] for p in pts) / len(pts)
        intersections.append({
            'intersection_id': iid,
            'center': {'lat': round(lat, 7), 'lon': round(lon, 7)},
            'source': source,
            'signal_groups': sorted(e['sg']),
            'linked_link_ids': sorted(e['link_ids']),
            'tl_points_count': len(e['stop_endpoints']),
            'maneuvers': sorted(e['man']),
        })

    intersections_doc = {
        'schema_version': 2,
        'data_source': 'mapfiles/senario/link_*.mat',
        'epsg_source': EPSG_SOURCE,
        'epsg_target': EPSG_TARGET,
        'intersections': intersections,
    }
    road_links_doc = {
        'schema_version': 2,
        'data_source': 'mapfiles/senario/link_*.mat',
        'epsg_source': EPSG_SOURCE,
        'epsg_target': EPSG_TARGET,
        'feature_count': len(links),
        'vertex_count': vertex_count,
        'features': links,
    }

    with open(OUT_INTERSECTIONS, 'w', encoding='utf-8') as f:
        json.dump(intersections_doc, f, ensure_ascii=False, indent=2)
    with open(OUT_ROAD_LINKS, 'w', encoding='utf-8') as f:
        json.dump(road_links_doc, f, ensure_ascii=False, separators=(',', ':'))

    print(f'[extract] wrote intersections: {OUT_INTERSECTIONS} ({len(intersections)} IIDs)')
    print(f'[extract] wrote road_links:    {OUT_ROAD_LINKS} ({len(links)} links, {vertex_count} vertices)')
    print()
    print('intersection summary:')
    for it in intersections:
        sg = ','.join(str(x) for x in it['signal_groups']) or '-'
        man = ','.join(str(x) for x in it['maneuvers'])
        print(f"  IID {it['intersection_id']:>5} | sg=[{sg}] | man=[{man}] | "
              f"center=({it['center']['lat']:.5f},{it['center']['lon']:.5f}) | "
              f"src={it['source']} (stop_pts={it['tl_points_count']})")
    return 0


if __name__ == '__main__':
    sys.exit(main())
