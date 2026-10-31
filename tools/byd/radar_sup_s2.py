#!/usr/bin/env python3
"""Supervision S2 - turn compensation (byd-radar-multitarget-port task 1.5).

In a curve the vehicle-frame lateral of a target drifts by psi_dot * d
(heading rotation). Test on the engine's y output vs the carrier's steering:
regress d(y)/dt against psi_dot * d inside curve windows (|steer| >= 2 deg).
  slope ~ -1  -> engine-y is heading/antenna-frame (uncompensated)
  slope ~  0  -> engine-y is ego-frame (turn-compensated)
Either outcome CONFIRMS the field is a real lateral quantity (a static wire
quantity would show no psi coupling); the slope value pins the frame, which
the lane-band consumer needs (bands are vehicle-frame).
psi = steer / wheelbase (small-angle), wheelbase 2.765 m (values.py specs).

  .venv/bin/python tools/byd/radar_sup_s2.py /tmp/probe_table.json \
      /tmp/probe_carrier12.bin
"""
import json
import os
import struct
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from radar_probe_fields import ols  # noqa: E402

WHEELBASE = 2.765
STEER_MIN = 2.0
GEAR_RATIO = 15.0  # steering-wheel deg per road-wheel deg (rough); slopes are compared relatively


def main():
  tab = json.load(open(sys.argv[1]))
  # steering series from the carrier dump (f32 at record offset 12+8)
  raw = open(sys.argv[2], 'rb').read()
  count = struct.unpack_from('<I', raw, 8)[0]
  off = 12
  w0 = None
  steer = []
  for _ in range(count):
    ts = struct.unpack_from('<q', raw, off)[0]
    if w0 is None:
      w0 = ts
    s = struct.unpack_from('<f', raw, off + 12)[0]
    steer.append(((ts - w0) / 1e9, s))
    off += 36

  recs = tab.get('baseline_records') or []
  ys = {}  # slot -> [(t, y)]
  for fr in recs:
    for r in fr['recs']:
      y, flags, slot = r[1], r[10], r[8]
      if flags & 32768 or y != y:
        continue
      ys.setdefault(slot, []).append((fr['t'], y, r[0]))  # t, y, d

  print('# S2 turn compensation (engine y vs psi*d drift)')
  for slot, rows in sorted(ys.items()):
    # dy/dt per consecutive pair + psi_dot*d at midpoint (psi from steer diff)
    xs, dys = [], []
    for (t0, y0, d0), (t1, y1, d1) in zip(rows, rows[1:]):
      dt = t1 - t0
      if not (0.03 < dt < 0.15):
        continue
      # steer at both ends
      sa = min(steer, key=lambda p: abs(p[0] - t0))[1]
      sb = min(steer, key=lambda p: abs(p[0] - t1))[1]
      if abs(sa) < STEER_MIN and abs(sb) < STEER_MIN:
        continue  # straight driving: no coupling expected
      psi_dot = ((sb - sa) / GEAR_RATIO) / dt * 3.14159265 / 180.0  # rad/s
      dmid = (d0 + d1) / 2.0
      xs.append(psi_dot * dmid)
      dys.append((y1 - y0) / dt)
    if len(xs) < 50:
      print(f'slot {slot}: n={len(xs)} curve samples - too few')
      continue
    b, a0, r2 = ols(xs, dys)
    print(f'slot {slot}: dy/dt = {b:.3f}*(psi_dot*d) + {a0:.4f}  (R2={r2:.3f}, n={len(xs)}) '
          f'-> frame: {"antenna/heading (slope~-1)" if b < -0.5 else "ego/turn-compensated (slope~0)" if b < 0.5 else "coupled (slope~+1)"}')


if __name__ == '__main__':
  main()
