#!/usr/bin/env python3
"""Affine-decode regression for the probe-solved fields (task 1.3 extension).

The vendor engine's per-slot outputs (y/vr/closing...) are hidden in the probe
table's baseline_records. If the engine decode is affine in a wire bit-run,
OLS of the engine output against the run's raw value recovers (scale, offset)
with R^2 ~ 1.0 - the same closure that PINNED the block-C distance, applied to
the HYPOTHESIS fields. Per (field, slot) we scan every candidate address and
bit-run from the probe solve and report the best affine fit.

  .venv/bin/python tools/byd/radar_probe_fields.py /tmp/probe_table.json \
      /tmp/probe_carrier12.bin
"""
import json
import struct
import sys


def load_wire(path):
  raw = open(path, 'rb').read()
  count = struct.unpack_from('<I', raw, 8)[0]
  off = 12
  w0 = None
  out = []
  for _ in range(count):
    ts, addr = struct.unpack_from('<qH', raw, off)
    dat = raw[off + 28: off + 36]
    off += 36
    if w0 is None:
      w0 = ts
    out.append(((ts - w0) / 1e9, addr, dat))
  return out


def ols(xs, ys):
  n = len(xs)
  mx, my = sum(xs) / n, sum(ys) / n
  sxx = sum((x - mx) ** 2 for x in xs)
  syy = sum((y - my) ** 2 for y in ys)
  sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=True))
  if sxx == 0 or syy == 0:
    return 0.0, my, 0.0
  b = sxy / sxx
  return b, my - b * mx, sxy * sxy / (sxx * syy)


def bits(dat, lo, hi):
  """LSB-first bit slice [lo, hi) of the 8-byte frame -> int."""
  v = int.from_bytes(dat, 'little')
  return (v >> lo) & ((1 << (hi - lo)) - 1)


def main():
  tab = json.load(open(sys.argv[1]))
  wire = load_wire(sys.argv[2])
  w0s = wire[0][0]

  recs = tab.get('baseline_records') or []
  # index wire frames per address for nearest-join
  by_addr = {}
  for t, a, dat in wire:
    by_addr.setdefault(a, []).append((t, dat))

  # engine slots: field series per slot
  fields = ('d', 'y', 'vr', 'va', 'a', 'vl', 'move', 'closing')  # 8 floats
  slots = {}
  for fr in recs:
    for r in fr['recs']:
      vals = r[:8]
      slot, flags = r[8], r[10]
      if flags & 32768:
        continue
      for f, v in zip(fields, vals, strict=True):
        if v == v:
          slots.setdefault((f, slot), []).append((fr['t'], v))

  # candidate bit-runs to test per probe group (from the 1.3 solve; the
  # ladder bases came out at bit 16 for F0/F2 byte fields and 35 for the
  # b4/b5 lateral candidate - scan a few standard runs)
  runs = [(16, 8), (16, 11), (16, 12), (17, 11), (24, 8), (32, 8), (32, 16),
          (35, 11), (36, 10), (40, 8), (48, 8), (48, 12), (56, 8)]
  groups = {'C-F0': [0x380 + 4 * k for k in range(6)],
            'C-F1': [0x381 + 4 * k for k in range(6)],
            'C-F2': [0x382 + 4 * k for k in range(6)],
            'C-F3': [0x383 + 4 * k for k in range(6)],
            'D-F0': [0x3C8, 0x3D0], 'H': [0x301]}

  print('# affine-decode regressions (engine output vs wire bit-run)')
  for (f, slot), rows in sorted(slots.items()):
    if len(rows) < 80:
      continue
    best = []
    for gname, addrs in groups.items():
      for addr in addrs:
        frames = by_addr.get(addr, [])
        if not frames:
          continue
        for (lo, ln) in runs:
          jx, jy = [], []
          for t, v in rows:
            fr = min(frames, key=lambda p: abs(p[0] - t))
            if abs(fr[0] - t) < 0.05:
              jx.append(bits(fr[1], lo, lo + ln))
              jy.append(v)
          if len(jx) < 80:
            continue
          b, a0, r2 = ols(jx, jy)
          if r2 > 0.97:
            best.append((r2, gname, addr, lo, ln, b, a0, len(jx)))
    best.sort(reverse=True)
    seen = set()
    shown = 0
    for r2, gname, addr, lo, ln, b, a0, n in best:
      key = (addr, lo, ln)
      if key in seen:
        continue
      seen.add(key)
      print(f'{f}@slot{slot}: {gname} 0x{addr:03X} @{lo}|{ln} -> '
            f'v = {b:.6g}*raw + {a0:.4g}  (R2={r2:.5f}, n={n})')
      shown += 1
      if shown >= 2:
        break


if __name__ == '__main__':
  main()
