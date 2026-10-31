#!/usr/bin/env python3
"""Archive-wide frame cache for the multitarget replay diff (task 2.7) and
the S1 supervision: per segment, the addresses the four-layer radar chain
consumes - 0x109/0x10A (b7 lead) + 0x32D (HUD, bus 2) + slot-table blocks
B/C/D (bus 1). One pickle, one pass, ~N segments.

  .venv/bin/python tools/byd/radar_multitarget_extract.py \
      ~/Documents/work/op/sunnypilot/docs_site/byd_logs_2026-10-05_07/realdata \
      --workers 10 --out /tmp/mt_replay.pkl
"""
import glob
import os
import pickle
import sys
import time
from multiprocessing import Pool

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from openpilot.tools.lib.logreader import LogReader  # noqa: E402

BUS1_ADDRS = ([0x109, 0x10A] + [0x301] +
              [a for a in range(0x340, 0x34B) if a not in (0x342, 0x345, 0x348)] +
              list(range(0x380, 0x398)) + list(range(0x3C8, 0x3E4)))
ALL_ADDRS = frozenset(BUS1_ADDRS + [0x32D])


def seg_extract(path):
  t0 = time.time()
  buf = {a: [[], []] for a in ALL_ADDRS}
  err = None
  rlog = os.path.join(path, 'rlog.zst')
  try:
    for m in LogReader(rlog):
      if m.which() != 'can':
        continue
      for c in m.can:
        if c.address in buf and (c.src == 1 or c.address == 0x32D):
          b = buf[c.address]
          b[0].append(m.logMonoTime)
          b[1].append(bytes(c.dat))
  except Exception as e:  # corrupted tail: keep partial stream (precedent)
    err = f'{type(e).__name__}: {str(e)[:80]}'
  out = {'err': err, 'seg': os.path.basename(path)}
  for a in ALL_ADDRS:
    ts, ds = buf[a]
    out[f'{a:03x}_t'] = ts
    out[f'{a:03x}_d'] = b''.join(ds) if ds else b''
  return out


def main():
  import argparse
  ap = argparse.ArgumentParser()
  ap.add_argument('dir')
  ap.add_argument('--workers', type=int, default=10)
  ap.add_argument('--out', default='/tmp/mt_replay.pkl')
  a = ap.parse_args()
  segs = sorted(glob.glob(f'{a.dir}/*/'))
  segs = [s.rstrip('/') for s in segs if os.path.exists(os.path.join(s, 'rlog.zst'))]
  print(f'# {len(segs)} segments, {len(ALL_ADDRS)} addrs, workers={a.workers}', flush=True)
  with Pool(a.workers) as p:
    res = []
    for i, r in enumerate(p.imap_unordered(seg_extract, segs, chunksize=4)):
      res.append(r)
      if (i + 1) % 25 == 0:
        print(f'# {i+1}/{len(segs)} segs ({time.time()-t_start:.0f}s)', flush=True)
  cache = {r['seg']: {k: v for k, v in r.items() if k != 'seg'} for r in res}
  with open(a.out, 'wb') as fp:
    pickle.dump(cache, fp, protocol=4)
  tot = sum(len(r['109_t']) for r in res)
  print(f'# done: {len(cache)} segs, {sum(1 for r in res if r["err"])} errs, '
        f'b7-frames={tot}, {os.path.getsize(a.out)/1e6:.0f} MB -> {a.out}', flush=True)


if __name__ == '__main__':
  t_start = time.time()
  main()
