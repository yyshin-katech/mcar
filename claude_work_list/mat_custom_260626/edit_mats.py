#!/usr/bin/env python3
"""
mat_custom_260626.md 작업 스크립트
- link_1205 wp1..13 -> 새 link_1205 (is_stop_line=1)
- link_1205 wp13..19 + 기존 link_1204 -> 새 link_1204
- link_870/877/871 wp1..34 -> 자기 자신 (is_stop_line=1), NEXT=3870/3877/3871
- link_870/877/871 wp34..149 -> 새 link_3870/3877/3871 (원본 NEXT/RIGHT/LEFT 상속)
- 경계 wp 는 양쪽 공유, station 재계산
"""
import os
import shutil
import numpy as np
import scipy.io as sio

MAP_DIR = '/home/sim/mcar/src/localization/gps_system_localizer/mapfiles/senario'


def load(lid):
    return sio.loadmat(os.path.join(MAP_DIR, f'link_{lid}.mat'))


def save(lid, d):
    sio.savemat(os.path.join(MAP_DIR, f'link_{lid}.mat'), d, do_compression=False)


def backup(lid):
    src = os.path.join(MAP_DIR, f'link_{lid}.mat')
    bak_dir = '/home/sim/mcar/claude_work_list/mat_custom_260626/backup'
    os.makedirs(bak_dir, exist_ok=True)
    if os.path.exists(src):
        shutil.copy2(src, os.path.join(bak_dir, f'link_{lid}.mat'))


def recompute_station(east, north):
    e = np.asarray(east).flatten()
    n = np.asarray(north).flatten()
    diffs = np.hypot(np.diff(e), np.diff(n))
    s = np.concatenate([[0.0], np.cumsum(diffs)])
    return s.reshape(1, -1)


def slice_with_keep(d, idx_lo, idx_hi_inclusive):
    """원본 dict d 에서 idx_lo..idx_hi_inclusive 의 east/north 만 슬라이스해서 새 dict 반환."""
    new = {k: v for k, v in d.items() if not k.startswith('__')}
    e = np.asarray(d['east']).flatten()[idx_lo:idx_hi_inclusive + 1].reshape(1, -1)
    n = np.asarray(d['north']).flatten()[idx_lo:idx_hi_inclusive + 1].reshape(1, -1)
    new['east'] = e
    new['north'] = n
    new['station'] = recompute_station(e, n)
    return new


def set_scalar(d, key, val):
    """기존 mat 의 스칼라 필드 dtype 유지하면서 값 변경."""
    orig = d.get(key)
    if isinstance(orig, np.ndarray):
        d[key] = np.array([[val]], dtype=orig.dtype)
    else:
        d[key] = np.array([[val]])


def show(lid, label=None):
    d = load(lid)
    e = d['east'].flatten()
    n = d['north'].flatten()
    s = d['station'].flatten()
    tag = f' ({label})' if label else ''
    print(f'link_{lid}{tag}: wp={len(e)}, len={s[-1]:.2f}m, NEXT={int(d["NEXT_LINK_ID"][0,0])}, '
          f'RIGHT={int(d["RIGHT_LINK_ID"][0,0])}, LEFT={int(d["LEFT_LINK_ID"][0,0])}, '
          f'is_stop={int(d["is_stop_line"][0,0])}, first=({e[0]:.3f},{n[0]:.3f}), last=({e[-1]:.3f},{n[-1]:.3f})')


# --- 1) link_1204, link_1205 분할/통합 ---
print('[1] link_1204, link_1205 분할/통합')
backup(1204); backup(1205)

orig1205 = load(1205)
orig1204 = load(1204)

# 새 link_1205 = 원본 wp1..13 (idx 0..12), 13개 포인트, is_stop=1
new1205 = slice_with_keep(orig1205, 0, 12)
set_scalar(new1205, 'is_stop_line', 1)
# NEXT_LINK_ID, RIGHT/LEFT, ID 등 모두 그대로 (원본 1205 의 NEXT=1204 유지)

# 새 link_1204 = 원본 1205 wp13..19 (idx 12..18) + 기존 1204 의 wp2.. (첫점 중복 제거)
e_pre = np.asarray(orig1205['east']).flatten()[12:19]
n_pre = np.asarray(orig1205['north']).flatten()[12:19]
e_old = np.asarray(orig1204['east']).flatten()
n_old = np.asarray(orig1204['north']).flatten()
# orig1205.wp19 (idx 18) == orig1204.wp1 (idx 0) → 중복 제거
e_new1204 = np.concatenate([e_pre, e_old[1:]]).reshape(1, -1)
n_new1204 = np.concatenate([n_pre, n_old[1:]]).reshape(1, -1)
new1204 = {k: v for k, v in orig1204.items() if not k.startswith('__')}
new1204['east'] = e_new1204
new1204['north'] = n_new1204
new1204['station'] = recompute_station(e_new1204, n_new1204)
# 1204 의 NEXT/RIGHT/LEFT, is_stop_line 등 모두 원본 유지

save(1205, new1205)
save(1204, new1204)

show(1205, '신규')
show(1204, '신규')

# --- 2) link_870/877/871 분할 + 3870/3877/3871 신규 ---
print('\n[2] link_870/877/871 분할 + 3870/3877/3871 신규')
for src, new_id in [(870, 3870), (877, 3877), (871, 3871)]:
    backup(src)

PAIR = [(870, 3870), (877, 3877), (871, 3871)]
NEXT_TAIL = {3870: 1808, 3877: 1807, 3871: 1806}
# new RIGHT/LEFT 매핑
NEW_RIGHT = {870: 877, 877: 871, 871: 0,
             3870: 3877, 3877: 3871, 3871: 0}
NEW_LEFT = {870: 0, 877: 870, 871: 877,
            3870: 0, 3877: 3870, 3871: 3877}

for src, new_id in PAIR:
    orig = load(src)

    # src_short: wp1..34 (idx 0..33), 34개
    short = slice_with_keep(orig, 0, 33)
    set_scalar(short, 'is_stop_line', 1)
    set_scalar(short, 'NEXT_LINK_ID', new_id)
    set_scalar(short, 'RIGHT_LINK_ID', NEW_RIGHT[src])
    set_scalar(short, 'LEFT_LINK_ID', NEW_LEFT[src])

    # new_id: wp34..149 (idx 33..148), 116개. 경계 wp34 공유
    tail = slice_with_keep(orig, 33, 148)
    # LINK_ID, LINK_ID_string 갱신
    set_scalar(tail, 'LINK_ID', new_id)
    tail['LINK_ID_string'] = np.array([orig.get('LINK_ID_string', np.array(['']))[0] + f'_T{new_id}'])
    set_scalar(tail, 'NEXT_LINK_ID', NEXT_TAIL[new_id])
    set_scalar(tail, 'RIGHT_LINK_ID', NEW_RIGHT[new_id])
    set_scalar(tail, 'LEFT_LINK_ID', NEW_LEFT[new_id])
    set_scalar(tail, 'is_stop_line', int(orig['is_stop_line'][0, 0]))  # 원본 유지 (=0)

    save(src, short)
    save(new_id, tail)
    show(src, '잘림')
    show(new_id, '신규')

# --- 3) 12개 링크 파일 삭제 ---
print('\n[3] 12개 링크 파일 삭제')
DEL_IDS = [786, 785, 792, 798, 791, 790, 567, 781, 764, 793, 789, 795]
for n in DEL_IDS:
    backup(n)
    p = os.path.join(MAP_DIR, f'link_{n}.mat')
    if os.path.exists(p):
        os.remove(p)
        print(f'  deleted: link_{n}.mat')
    else:
        print(f'  missing: link_{n}.mat')

print('\n[완료]')
