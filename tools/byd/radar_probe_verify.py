#!/usr/bin/env python3
"""Physical cross-check of the probe-solved layouts against the carrier wire
(task 1.3 verification; settles the 0.0623-vs-1/32 scale question with data).

Joins, over the same time axis:
  - engine slot d/y/vr (probe_table.json baseline_records, t>=diff_after)
  - 0x109 b7 counts x u=0.25 (the PINNED fused-lead distance)
  - block C F0 12-bit x per address (the slot67 x)
Reports per slot: best-matching C address, the OLS slope of engine-d vs x
(the engine's decode scale), slope of b7-model vs x (the physical scale), and
the y/closing field plausibility (|y| band, closing vs dd/dt agreement).

  .venv/bin/python tools/byd/radar_probe_verify.py /tmp/probe_table.json \
      /tmp/probe_carrier12.bin
"""
import json
import math
import struct
import sys


def load_wire(path):
  raw = open(path, 'rb').read()
  count = struct.unpack_from('<I', raw, 8)[0]
  off = 12
  out = []
  for _ in range(count):
    ts, addr = struct.unpack_from('<qH', raw, off)
    dat = raw[off + 28: off + 36]
    out.append((ts, addr, dat))
    off += 36
  return out


def series(frames, addr, t0, t1):
  """(t_s, value) for one address, value = None unless frame present."""
  vals = []
  for ts, a, dat in frames:
    if a != addr or not (t0 <= ts < t1):
      continue
    vals.append((ts, dat))
  return vals


def ols(xs, ys):
  n = len(xs)
  mx, my = sum(xs) / n, sum(ys) / n
  sxx = sum((x - mx) ** 2 for x in xs)
  syy = sum((y - my) ** 2 for y in ys)
  sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys, strict=True))
  if sxx == 0 or syy == 0:
    return 0.0, my, 0.0
  b = sxy / sxx
  a = my - b * mx
  r2 = (sxy * sxy) / (sxx * syy)
  return b, a, r2


def main():
  tab = json.load(open(sys.argv[1]))
  wire = load_wire(sys.argv[2])
  # normalize the wire axis to window-relative seconds (dump = whole segment,
  # baseline_records' t axis) - raw logMonoTime is an absolute 10-05 clock
  w0 = wire[0][0]
  frames = [((ts - w0) / 1e9, a, dat) for ts, a, dat in wire]

  recs = tab.get('baseline_records') or []
  if not recs:
    sys.exit('no baseline_records in probe table (re-run with --dump-baseline)')

  # wire series on the same axis (seconds from segment start)
  b7 = [(t, dat[7]) for t, a, dat in frames if a == 0x109]
  c_f0 = {0x380 + 4 * k: [] for k in range(6)}
  for t, a, dat in frames:
    if 0x380 <= a < 0x398 and (a - 0x380) % 4 == 0:
      x = ((dat[3] & 0xF) << 8) | dat[2]
      c_f0[a].append((t, x))

  # engine slots: d/y/vr per second bucket (records are per feed; use nearest)
  slots = {}
  for fr in recs:
    for r in fr['recs']:
      d, y, vr, flags, slot = r[0], r[1], r[2], r[10], r[8]
      if flags & 32768 or d != d:
        continue
      slots.setdefault(slot, []).append((fr['t'], d, y, vr))

  print(f"# engine slots: { {s: len(v) for s, v in sorted(slots.items())} }")
  for slot, rows in sorted(slots.items()):
    ts_e = [r[0] for r in rows]
    d_e = [r[1] for r in rows]
    y_e = [r[2] for r in rows if r[2] == r[2]]
    vr_e = [r[3] for r in rows if r[3] == r[3]]
    # nearest-b1 join vs each C address + b7
    best = None
    for addr, xs in c_f0.items():
      if not xs:
        continue
      jx, jd = [], []
      for t, d in zip(ts_e, d_e, strict=True):
        # nearest x sample within 70ms
        cand = min(xs, key=lambda p: abs(p[0] - t))
        if abs(cand[0] - t) < 0.07 and cand[1] > 0 and cand[1] < 0xFFF:
          jx.append(cand[1])
          jd.append(d)
      if len(jx) < 200:
        continue
      b, a0, r2 = ols(jx, jd)
      if best is None or r2 > best[3]:
        best = (addr, b, a0, r2, len(jx))
    if best is None:
      print(f"slot {slot}: no C-address match (n rows {len(rows)})")
      continue
    addr, b_eng, a_eng, r2_eng, n = best
    # physical scale: b7-model vs the same address's x
    xs = c_f0[addr]
    jx, jb = [], []
    for t, d in zip(ts_e, d_e, strict=True):
      cand = min(b7, key=lambda p: abs(p[0] - t))
      cx = min(xs, key=lambda p: abs(p[0] - t))
      if abs(cand[0] - t) < 0.07 and abs(cx[0] - t) < 0.07 and cx[1] > 0 and cx[1] < 0xFFF:
        jx.append(cx[1])
        jb.append(cand[1] * 0.25)
    b_phys, a_phys, r2_phys = ols(jx, jb)
    yr = (min(y_e), max(y_e)) if y_e else None
    vr = (min(vr_e), max(vr_e)) if vr_e else None
    print(f"slot {slot}: rows={len(rows)} d[{min(d_e):.1f}..{max(d_e):.1f}]m "
          f"y[{yr[0]:.2f}..{yr[1]:.2f}]" if yr else f"slot {slot}: y NaN",
          end=' ')
    if vr:
      print(f"vr[{vr[0]:.2f}..{vr[1]:.2f}]", end=' ')
    print()
    print(f"  best C addr 0x{addr:03X}: engine-d = {b_eng:.6f}*x + {a_eng:.2f} "
          f"(R2={r2_eng:.4f}, n={n})  [probe-solved scale 0.062319?]")
    print(f"  physical   : b7*u     = {b_phys:.6f}*x + {a_phys:.2f} "
          f"(R2={r2_phys:.4f}, n={len(jx)})  [1/32 model = 0.03125]")


if __name__ == '__main__':
  main()
