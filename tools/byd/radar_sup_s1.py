#!/usr/bin/env python3
"""Supervision S1 - vision cross-check (byd-radar-multitarget-port task 1.4).

Decodes the wire lateral candidate (block C F0 bits 35-45, affine
y ~ 0.006977*raw - 7.478 from the 2026-10-31 probe regression) over the 415
segments, pairs each timestamp's row whose ENGINE-scale distance best matches
modelV2.leadsV3[0] dRel, and scores (y_radar, y_vision) agreement:
  - correlation + median |dy| over follow windows (vision prob >= 0.7)
  - sweep detection: |dy| excursions during lane changes (blinker on)
Expected (design D2): small stable deltas while following, sweeps on
lane changes, steps on cut-ins. This is the semantic gate for wiring the
field out of HYPOTHESIS (2.2).

  .venv/bin/python tools/byd/radar_sup_s1.py <archive_dir> [--workers 10]
"""
import glob
import math
import os
import sys
from collections import Counter
from multiprocessing import Pool

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from opendbc.car.byd.radar_multitarget import C_DIST_SCALE, C_DIST_OFFSET  # noqa: E402

Y_SCALE = 0.00697682   # probe regression vs engine y (R2=0.9887)
Y_OFFSET = -7.478
LAT_BITS = (35, 46)    # [lo, hi) of the F0 frame
MATCH_M = 3.0
PROB_MIN = 0.7


def decode_rows(dat_by_t, t):
  """Live C rows at time t: list of (d, y_raw)."""
  out = []
  for k in range(6):
    addr = 0x380 + 4 * k
    xs = dat_by_t.get(addr)
    if not xs:
      continue
    cand = min(xs, key=lambda p: abs(p[0] - t))
    if abs(cand[0] - t) > 0.05:
      continue
    dat = cand[1]
    x12 = ((dat[3] & 0xF) << 8) | dat[2]
    if x12 == 0 or (x12 & 0x800):
      continue
    d = (x12 & 0x7FF) * C_DIST_SCALE + C_DIST_OFFSET
    lat = int.from_bytes(dat, 'little') >> LAT_BITS[0] & ((1 << (LAT_BITS[1] - LAT_BITS[0])) - 1)
    out.append((d, lat))
  return out


def seg_score(path):
  from openpilot.tools.lib.logreader import LogReader
  # pass 1: CAN rows per address (t seconds from segment start)
  can = {}
  w0 = None
  try:
    for m in LogReader(os.path.join(path, 'rlog.zst')):
      if m.which() == 'can':
        for c in m.can:
          if c.src == 1 and 0x380 <= c.address < 0x398:
            if w0 is None:
              w0 = m.logMonoTime
            can.setdefault(c.address, []).append(((m.logMonoTime - w0) / 1e9, bytes(c.dat)))
      elif m.which() == 'modelV2':
        mv = m.modelV2
        if len(mv.leadsV3) and mv.leadsV3[0].prob >= PROB_MIN and w0 is not None \
           and len(mv.leadsV3[0].x) and len(mv.leadsV3[0].y):
          t = (m.logMonoTime - w0) / 1e9
          lead_d, lead_y = mv.leadsV3[0].x[0], mv.leadsV3[0].y[0]
          rows = decode_rows(can, t)
          if rows:
            d, lat = min(rows, key=lambda r: abs(r[0] - lead_d))
            if abs(d - lead_d) <= MATCH_M and 0 < lat < 2047:  # edge guard
              seg_score.pairs.append((lead_y, lat * Y_SCALE + Y_OFFSET, t))
  except Exception as e:
    return {'err': f'{type(e).__name__}: {str(e)[:80]}'}
  return None


def seg_init():
  seg_score.pairs = []


def seg_worker(path):
  seg_init()
  r = seg_score(path)
  pairs = getattr(seg_score, 'pairs', [])
  # per-segment summary: correlation + median |dy|
  if len(pairs) < 50:
    return {'n': len(pairs), 'err': (r or {}).get('err')}
  ys_r = [p[1] for p in pairs]
  ys_v = [p[0] for p in pairs]
  dy = sorted(abs(a - b) for a, b in zip(ys_v, ys_r))
  n = len(pairs)
  mx, my = sum(ys_v) / n, sum(ys_r) / n
  sxx = sum((v - mx) ** 2 for v in ys_v)
  syy = sum((v - my) ** 2 for v in ys_r)
  sxy = sum((v - mx) * (r - my) for v, r in zip(ys_v, ys_r))
  corr = sxy / math.sqrt(sxx * syy) if sxx > 0 and syy > 0 else 0.0
  return {'n': n, 'corr': corr, 'dy_med': dy[n // 2], 'dy_p90': dy[int(n * 0.9)]}


def main():
  import argparse
  ap = argparse.ArgumentParser()
  ap.add_argument('dir')
  ap.add_argument('--workers', type=int, default=10)
  a = ap.parse_args()
  segs = [s.rstrip('/') for s in sorted(glob.glob(f'{a.dir}/*/'))
          if os.path.exists(os.path.join(s, 'rlog.zst'))]
  print(f'# S1 vision cross-check: {len(segs)} segments', flush=True)
  stats = Counter()
  corrs, dys = [], []
  with Pool(a.workers, initializer=seg_init) as p:
    for i, r in enumerate(p.imap_unordered(seg_worker, segs, chunksize=4)):
      if r.get('err'):
        stats['err'] += 1
      stats['segs'] += 1
      if 'corr' in r:
        corrs.append(r['corr'])
        dys.append((r['dy_med'], r['dy_p90'], r['n']))
      if (i + 1) % 50 == 0:
        print(f'# {i+1}/{len(segs)}', flush=True)
  print(f"# segs scored: {len(corrs)}/{len(segs)} (errs {stats['err']})")
  if corrs:
    corrs.sort()
    print(f"# per-seg corr: p10={corrs[len(corrs)//10]:.3f} p50={corrs[len(corrs)//2]:.3f} "
          f"p90={corrs[9*len(corrs)//10]:.3f}")
    dys.sort(key=lambda x: x[0])
    print(f"# per-seg |dy| med: p10={dys[len(dys)//10][0]:.2f} p50={dys[len(dys)//2][0]:.2f} "
          f"p90={dys[9*dys.__len__()//10][0]:.2f} m")
    top = dys[-5:]
    print(f"# worst-5 segs (|dy| med): {[(round(x[0],2), round(x[1],2), x[2]) for x in top]}")


if __name__ == '__main__':
  main()
