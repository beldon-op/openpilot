#!/usr/bin/env python3
"""Vision truth extraction for the offline lateral-y calibration (task 2.2).

Per segment: modelV2 leadsV3 primary/secondary (x, y, prob, v) + carState vEgo,
keyed by logMonoTime so engine labels (CAN timestamps) can be paired with a
time tolerance. Mirrors radar_multitarget_extract.py I/O conventions.

  .venv/bin/python tools/byd/radar_y_vision_extract.py \
      ~/Documents/work/op/sunnypilot/docs_site/byd_logs_2026-10-05_07/realdata \
      --workers 8 --out /tmp/y_vision.pkl
"""
import glob
import os
import pickle
import time
from multiprocessing import Pool

import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from openpilot.tools.lib.logreader import LogReader  # noqa: E402


def seg_extract(path):
  t0 = time.time()
  rows = []  # (logMonoTime, x0, y0, p0, v0, x1, y1, p1, vEgo)
  err = None
  rlog = os.path.join(path, 'rlog.zst')
  cur = {'model': None, 'vEgo': None}
  try:
    for m in LogReader(rlog):
      w = m.which()
      if w == 'modelV2':
        leads = m.modelV2.leadsV3
        x0 = float(leads[0].x[0]) if len(leads) and len(leads[0].x) else None
        y0 = float(leads[0].y[0]) if len(leads) and len(leads[0].y) else None
        p0 = float(leads[0].prob) if len(leads) else None
        v0 = float(leads[0].v[0]) if len(leads) and len(leads[0].v) else None
        x1 = float(leads[1].x[0]) if len(leads) > 1 and len(leads[1].x) else None
        y1 = float(leads[1].y[0]) if len(leads) > 1 and len(leads[1].y) else None
        p1 = float(leads[1].prob) if len(leads) > 1 else None
        cur['model'] = (x0, y0, p0, v0, x1, y1, p1)
      elif w == 'carState':
        cur['vEgo'] = float(m.carState.vEgo)
      if cur['model'] is not None and cur['vEgo'] is not None:
        x0, y0, p0, v0, x1, y1, p1 = cur['model']
        rows.append((m.logMonoTime, x0, y0, p0, v0, x1, y1, p1, cur['vEgo']))
        cur['model'] = None
  except Exception as e:  # corrupted tail: keep partial stream (precedent)
    err = f'{type(e).__name__}: {str(e)[:80]}'
  return {'seg': os.path.basename(path), 'err': err, 'rows': rows, 'dt': time.time() - t0}


def main():
  import argparse
  ap = argparse.ArgumentParser()
  ap.add_argument('dir')
  ap.add_argument('--workers', type=int, default=8)
  ap.add_argument('--out', default='/tmp/y_vision.pkl')
  a = ap.parse_args()
  segs = sorted(glob.glob(f'{a.dir}/*/'))
  segs = [s.rstrip('/') for s in segs if os.path.exists(os.path.join(s, 'rlog.zst'))]
  print(f'# {len(segs)} segments, workers={a.workers}', flush=True)
  t_start = time.time()
  with Pool(a.workers) as p:
    res = []
    for i, r in enumerate(p.imap_unordered(seg_extract, segs, chunksize=4)):
      res.append(r)
      if (i + 1) % 25 == 0:
        print(f'# {i+1}/{len(segs)} segs ({time.time()-t_start:.0f}s)', flush=True)
  out = {r['seg']: {'err': r['err'], 'rows': r['rows']} for r in res}
  with open(a.out, 'wb') as fp:
    pickle.dump(out, fp, protocol=4)
  n = sum(len(r['rows']) for r in res)
  errs = sum(1 for r in res if r['err'])
  print(f'# done: {len(res)} segs, {n} vision rows, {errs} errs, '
        f'{os.path.getsize(a.out)/1e6:.0f} MB -> {a.out} ({time.time()-t_start:.0f}s)', flush=True)


if __name__ == '__main__':
  main()
