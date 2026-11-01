#!/usr/bin/env python3
"""Wire-direct lateral-y decoder + b7 pairing gate (byd-radar-y-wire-port 2.5/2.7).

2026-11-01 verdict revision: the vendor .so spec table (rd_spec_table.json,
group 1) + engine-label verification (radar_y_stream_labels.py smoke, 427/427
records) identified the lateral code ON THE WIRE:

  carrier = C-block F0 frame (0x380 + 4*slot, the cycle's first frame);
    u_d = LSB-first bits[16,27)  -> d   = u_d * C_SCALE + C_DIST_OFF   (exact)
    u_c = LSB-first bits[35,46)  -> code = u_c - 1024                  (exact)
    y   = (base + d_off) * K * (code - c0),  base = u_d * C_SCALE
          K = 0.00050091, defaults c0 = -15, d_off = -1.671 (oracle grid)

S1 killed bits35-45 as "affine y" - correct data, wrong functional form:
the code is distance-modulated (y ∝ d * code), invisible to any affine test.

This tool decodes (t, slot, d, code, y) for every cycle of every segment from
the archive cache - pure Python, no .so - verifies against engine labels where
provided, and computes the 2.5 pairing-gate stats: |d_slot - b7*0.25|
mismatch distribution -> PAIR_GATE_M + pairing rate.

  .venv/bin/python tools/byd/radar_y_wire_decode.py /tmp/mt_replay.pkl \
      [--engine-labels /tmp/yout/y_stream_smoke_b.pkl] [--out /tmp/y_wire_obs.pkl]
"""
import argparse
import bisect
import pickle
import statistics
import sys
from collections import defaultdict

C_SCALE = 0.062319
C_DIST_OFF = -1.671          # d = u_d * C_SCALE + C_DIST_OFF (spec table sig0)
CODE_ZERO = 1024             # 11-bit offset binary
K = 0.00050091
C0_DEFAULT = -15.0
D_OFF_DEFAULT = -1.671
B7_SCALE = 0.25              # pinned b7 distance scale (owner)


def le_bits(data, start, ln):
  return (int.from_bytes(data, 'little') >> start) & ((1 << ln) - 1)


def decode_segment(d, c0=C0_DEFAULT, d_off=D_OFF_DEFAULT):
  """Assemble C-block cycles by b1 change-edges (multitarget-port discipline).
  Emit one observation per C-family F0 frame (slot, u_d, u_c), stamped at the
  F0 frame's own time - the d/y values are F0-instant data, so the b7 pairing
  stats must not carry cycle-edge staleness. y = (base + d_off)*K*(code - c0)."""
  frames = []
  for a in range(0x380, 0x3E4):
    raw = d.get(f'{a:x}_d')
    ts = d.get(f'{a:x}_t')
    if not raw or not ts:
      continue
    n = min(len(raw) // 8, len(ts))
    frames.extend((ts[i], a, raw[i * 8:(i + 1) * 8]) for i in range(n))
  b7_t = d.get('109_t', [])
  b7_d = d.get('109_d', b'')
  nb7 = min(len(b7_t), len(b7_d) // 8)
  b7_ts = [b7_t[i] for i in range(nb7)]
  b7_raw = [b7_d[i * 8:(i + 1) * 8] for i in range(nb7)]
  # presence (0x10A Presence_10A 31|3@0+ = byte3[7:5]): value 7 = forward target
  pa_t = d.get('10a_t', [])
  pa_d = d.get('10a_d', b'')
  npa = min(len(pa_t), len(pa_d) // 8)
  pres_ts = [pa_t[i] for i in range(npa)]
  pres_v = [((pa_d[i * 8 + 3] >> 5) & 7) == 7 for i in range(npa)]

  frames.sort()
  obs = []
  b1_cur = None
  for t, a, data in frames:
    b1 = data[1]
    if b1_cur is None:
      b1_cur = b1
    elif b1 != b1_cur:
      b1_cur = b1
    slot, idx = divmod(a - 0x380, 4)
    if idx == 0 and a < 0x3C8:    # C-family F0 carrier
      u_d = le_bits(data, 16, 11)
      u_c = le_bits(data, 35, 11)
      base = u_d * C_SCALE
      code = u_c - CODE_ZERO
      y = (base + d_off) * K * (code - c0)
      j = bisect.bisect_left(b7_ts, t - 40_000_000)
      b7v = None
      pres = False
      if j < len(b7_ts) and b7_ts[j] <= t + 40_000_000:
        b7v = b7_raw[j][7]        # MSG_109_B7 = byte7 (63|8@0+)
        k = bisect.bisect_left(pres_ts, t - 60_000_000)
        if k < len(pres_ts) and pres_ts[k] <= t + 60_000_000:
          pres = pres_v[k]
      obs.append(dict(t=t, slot=slot, u_d=u_d, u_c=u_c, base=base,
                      code=code, d=base + C_DIST_OFF, y=y, b7=b7v, pres=pres))
  return obs


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('cache')
  ap.add_argument('--engine-labels', default=None,
                  help='y_stream_labels.pkl for cross-verification')
  ap.add_argument('--c0', type=float, default=C0_DEFAULT)
  ap.add_argument('--d-off', type=float, default=D_OFF_DEFAULT)
  ap.add_argument('--out', default=None, help='pickle the per-segment observations')
  ap.add_argument('--limit', type=int, default=0)
  a = ap.parse_args()

  cache = pickle.load(open(a.cache, 'rb'))
  segs = sorted(cache.keys())
  if a.limit:
    segs = segs[:a.limit]

  all_obs = {}
  n_seg_obs = []
  for seg in segs:
    obs = decode_segment(cache[seg], a.c0, a.d_off)
    all_obs[seg] = obs
    n_seg_obs.append(len(obs))
  total = sum(n_seg_obs)
  print(f'[decode] {len(segs)} segments, {total} observations '
        f'(median/seg {statistics.median(n_seg_obs) if n_seg_obs else 0:.0f})', flush=True)

  # engine cross-verification
  if a.engine_labels:
    eng = pickle.load(open(a.engine_labels, 'rb'))
    dy = []
    dd = []
    dc = []
    miss = 0
    for seg, blk in eng.items():
      if seg not in all_obs:
        continue
      obs = all_obs[seg]
      obs_ts = [o['t'] for o in obs]
      for r in blk['rows']:
        if r[13] is None:
          continue
        # engine rows carry feed-anchored ts; match by (slot, d, code) triple
        slot, d_eng, code_eng = r[3], r[8], int(round(r[13][1]))
        cands = [o for o in obs if o['slot'] == slot and o['code'] == code_eng
                 and abs(o['d'] - d_eng) < 0.2]
        if not cands:
          miss += 1
          continue
        o = cands[0]
        dy.append(o['y'] - r[9])
        dd.append(o['d'] - d_eng)
        dc.append(o['code'] - code_eng)
    if dy:
      ady = sorted(abs(x) for x in dy)
      n = len(dy)
      print(f'[verify] vs engine: n={n} miss={miss} '
            f'|Δy| med={1000*ady[n//2]:.1f}mm p95={1000*ady[int(n*0.95)]:.1f}mm '
            f'|Δd| med={1000*sorted(abs(x) for x in dd)[n//2]:.1f}mm '
            f'code mismatch={sum(1 for x in dc if x)}/n', flush=True)

  # 2.5 pairing gate: per cycle, the slot whose |d_slot - b7*0.25| is minimal
  # is the b7-lead candidate; the mismatch distribution over those minima (and
  # the pairing rate per gate) defines PAIR_GATE_M (design D3).
  gates = [0.5, 0.75, 1.0, 1.5, 2.0, 2.5, 3.0]
  mism = []
  paired = {g: 0 for g in gates}
  n_cycles = 0
  for seg, obs in all_obs.items():
    by_t = defaultdict(list)
    for o in obs:
      by_t[o['t']].append(o)
    for t, group in by_t.items():
      b7v = group[0].get('b7')
      if b7v is None or not (0 < b7v <= 245) or not group[0].get('pres'):
        continue
      n_cycles += 1
      m = min(abs(o['d'] - b7v * B7_SCALE) for o in group)
      mism.append(m)
      for g in gates:
        if m <= g:
          paired[g] += 1
  if mism:
    sm = sorted(mism)
    n = len(sm)
    print(f'[2.5] b7-valid cycles: {n}', flush=True)
    print(f'[2.5] |d_slot - b7*0.25|: p50={sm[n//2]:.2f}m p75={sm[int(n*0.75)]:.2f}m '
          f'p90={sm[int(n*0.9)]:.2f}m p95={sm[int(n*0.95)]:.2f}m max={sm[-1]:.2f}m', flush=True)
    for g in gates:
      print(f'[2.5] PAIR_GATE_M={g:.2f}: 配对率 {100*paired[g]/n:.1f}%', flush=True)

  if a.out:
    pickle.dump(all_obs, open(a.out, 'wb'))
    print(f'[out] {a.out}', flush=True)


if __name__ == '__main__':
  main()
