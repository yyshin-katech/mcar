#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
senario link_*.mat 에서 LINK_ID(int) → (LINK_ID_string, NEXT_LINK_ID) 매핑을
CSV 로 추출한다. mqtt_bsm_tx_node 가 to_control_team.LINK_ID(현재 링크)로부터
NEXT_LINK_ID 체인을 따라가며 곧 지날 LINK_ID_string 시퀀스를 만들 때 사용.

출력 CSV (헤더 포함):
    link_id,link_id_string,next_link_id

사용:
    python3 build_senario_link_map.py [SENARIO_DIR] [OUT_CSV]
기본:
    SENARIO_DIR = ../../../localization/gps_system_localizer/mapfiles/senario
    OUT_CSV     = ../config/senario_link_map.csv
"""
import os
import sys
import glob
import scipy.io as sio

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_SENARIO = os.path.normpath(os.path.join(
    HERE, '..', '..', '..', 'localization',
    'gps_system_localizer', 'mapfiles', 'senario'))
DEFAULT_OUT = os.path.normpath(os.path.join(HERE, '..', 'config', 'senario_link_map.csv'))


def _scalar(mat, key, default=None):
    if key not in mat:
        return default
    try:
        return mat[key].flatten()[0]
    except Exception:
        return default


def main():
    senario_dir = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_SENARIO
    out_csv = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUT

    mat_files = sorted(glob.glob(os.path.join(senario_dir, 'link_*.mat')))
    if not mat_files:
        print('[build_senario_link_map] no link_*.mat in %s' % senario_dir)
        return 1

    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    rows = []
    for f in mat_files:
        m = sio.loadmat(f)
        lid = _scalar(m, 'LINK_ID')
        nxt = _scalar(m, 'NEXT_LINK_ID', 0)
        sval = None
        if 'LINK_ID_string' in m:
            try:
                sval = str(m['LINK_ID_string'][0])
            except Exception:
                sval = None
        if lid is None or sval is None:
            print('[build_senario_link_map] skip %s (missing LINK_ID/LINK_ID_string)'
                  % os.path.basename(f))
            continue
        rows.append((int(lid), sval.strip(), int(nxt)))

    rows.sort(key=lambda r: r[0])
    with open(out_csv, 'w') as fp:
        fp.write('link_id,link_id_string,next_link_id\n')
        for lid, sval, nxt in rows:
            fp.write('%d,%s,%d\n' % (lid, sval, nxt))

    print('[build_senario_link_map] wrote %d links -> %s' % (len(rows), out_csv))
    return 0


if __name__ == '__main__':
    sys.exit(main())
