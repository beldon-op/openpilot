#!/usr/bin/env python3
"""Supervision S3 - slot consistency (byd-radar-multitarget-port task 1.5).

From the probe table's baseline engine records (per-slot d/y/vr series):
  - lock-window smoothness: within a continuous slot hold, the per-cycle
    |dd| median must sit at the decode's quantization floor (one count =
    0.062319 m at 15 Hz) - b7-form continuity, no teleports;
  - slot-switch jump identification: when the engine's slot id churns, does
    the VALUE continue (identity preserved through reassignment -> the slot
    algebra is an indexing detail, distance-gated pairing is correct) or
    teleport (identity broken -> pairing needs slot-awareness)?

  .venv/bin/python tools/byd/radar_sup_s3.py /tmp/probe_table.json
"""
import json
import os
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from opendbc.car.byd.radar_multitarget import C_DIST_SCALE, CYCLE_PERIOD_S  # noqa: E402


def main():
  tab = json.load(open(sys.argv[1]))
  recs = tab.get('baseline_records') or []
  # per-slot value series (engine output, driving region)
  series = {}   # slot -> [(t, d)]
  for fr in recs:
    for r in fr['recs']:
      d, y, vr, flags, slot = r[0], r[1], r[2], r[10], r[8]
      if flags & 32768 or d != d:
        continue
      series.setdefault(slot, []).append((fr['t'], d, y, vr))

  print('# S3 slot consistency (engine output, driving region)')
  print(f'# quantization floor per cycle: {C_DIST_SCALE:.5f} m '
        f'({C_DIST_SCALE / CYCLE_PERIOD_S:.3f} m/s per count)')
  for slot, rows in sorted(series.items()):
    rows.sort()
    ts = [r[0] for r in rows]
    dd = [abs(pb[1] - pa[1]) for pa, pb in zip(rows, rows[1:])
          if 0 < pb[0] - pa[0] <= 0.1]  # consecutive-cycle steps only
    dd.sort()
    med = dd[len(dd) // 2] if dd else float('nan')
    p95 = dd[int(len(dd) * 0.95)] if dd else float('nan')
    # teleport detection: steps > 4x quantization in one cycle
    teleports = sum(1 for v in dd if v > 4 * C_DIST_SCALE)
    # value continuity across slot-identity churn: nearest-value match when
    # this slot reappears after a gap (the pair test for switch jumps)
    gaps = [(pa, pb) for pa, pb in zip(rows, rows[1:]) if 0.1 < pb[0] - pa[0] <= 3.0]
    gap_jump_med = sorted(abs(b[1] - a[1]) for a, b in gaps)[len(gaps) // 2] if gaps else float('nan')
    print(f'slot {slot}: n={len(rows)} span={rows[-1][0]-rows[0][0]:.1f}s '
          f'per-cycle |dd| med={med:.4f}m p95={p95:.4f}m teleports(>4cnt)={teleports} '
          f'gap-jump med={gap_jump_med:.3f}m')
  # cross-slot: do two slots ever hold near-identical d at the same t (indexing
  # churn would duplicate a target across slots)?
  ts_idx = {}
  for slot, rows in series.items():
    for t, d, y, vr in rows:
      ts_idx.setdefault(round(t, 2), []).append((slot, d))
  dup = 0
  for t, items in ts_idx.items():
    ds = [d for _s, d in items]
    for i in range(len(ds)):
      for j in range(i + 1, len(ds)):
        if abs(ds[i] - ds[j]) < 0.3:
          dup += 1
  print(f'# cross-slot near-duplicate (|dd|<0.3m) frames: {dup} '
        f'({"index churn PRESENT - pairing must be distance-gated" if dup else "slots independent"})')


if __name__ == '__main__':
  main()
