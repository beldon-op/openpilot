#!/usr/bin/env python3
"""Wire field identity for the engine angle code (byd-radar-y-wire-port 1.3/2.1).

For each engine label record (implied code = y/(d*K) - 15, identity recovered
2026-11-01), pull the record's block-C row frames (4 x 8 B, matched by the b1
cycle counter) and scan every (bit_start, bit_len, signed) candidate for an
affine relation code = a*w + b. S1's affine-to-y verdict does not apply here:
the engine model is y = d*K*(code - c0), so the code relates to y only through
d - the correct test is code-vs-wire, then y follows in closed form.

  docker run --rm --platform linux/arm64 -v /tmp:/cache:ro \
    -v <repo>:/repo:ro -v /tmp/yout:/out pyarmor-local \
    python /repo/tools/byd/radar_y_field_identity.py \
    --labels /cache/yout/y_engine_labels.pkl --cache /cache/mt_replay.pkl
"""
import argparse
from collections import defaultdict
import os
import pickle
import sys
import time

import numpy as np

K = 0.00050091  # engine lateral model slope (see radar_y_engine_labels.py)
BLOCK_C_BASE = 0x380
ROW_FRAMES = 4


def load_labels(path, max_records):
  data = pickle.load(open(path, 'rb'))
  cols = {k: [] for k in ('seg_idx', 't', 'slot', 'counter', 'd', 'y', 'code', 'addr', 'cnt')}
  seg_names = sorted(data.keys())
  for si, seg in enumerate(seg_names):
    c = data[seg]['cols']
    n = len(c['d'])
    for k in ('t', 'slot', 'counter', 'd', 'y', 'code', 'addr', 'cnt'):
      cols[k].extend(c[k])
    cols['seg_idx'].extend([si] * n)
  out = {k: np.asarray(v) for k, v in cols.items()}
  if max_records and len(out['d']) > max_records:
    idx = np.linspace(0, len(out['d']) - 1, max_records).astype(int)
    out = {k: v[idx] for k, v in out.items()}
  return seg_names, data, out


def build_row_index(cache, seg_names, seg_idx):
  """Per segment, per block-C address: sorted ts array + frames matrix.

  b1 wraps every 16 cycles (step 0x11) so (addr, b1) keys alias across cycles;
  alignment must be time-based instead (records pair with frames in a window).
  """
  rows = []
  for si in range(len(seg_names)):
    d = cache[seg_names[si]]
    per = {}
    for off in range(24):
      addr = BLOCK_C_BASE + off
      k = f'{addr:03x}'
      ts_list, raw = d.get(k + '_t'), d.get(k + '_d', b'')
      if not ts_list or not raw:
        continue
      n = min(len(ts_list), len(raw) // 8)
      ts = np.asarray(ts_list[:n], dtype=np.int64)
      fr = np.frombuffer(raw[:n * 8], dtype=np.uint8).reshape(n, 8)
      order = np.argsort(ts)
      per[addr] = (ts[order], fr[order])
    rows.append(per)
  return rows


def row_at(per, addr, t_ns, lo_ns=-200_000_000, hi_ns=20_000_000):
  """Latest frame at addr within [t+lo, t+hi]; None if absent."""
  if addr not in per:
    return None
  ts, fr = per[addr]
  hi = np.searchsorted(ts, t_ns + hi_ns, side='right')
  lo = np.searchsorted(ts, t_ns + lo_ns, side='left')
  if hi <= lo:
    return None
  return fr[hi - 1]


def extract_fields(rowbytes, start, length, signed):
  """LSB-first bit extract (engine extractor @0xde8 convention) over n x 32 B."""
  n = rowbytes.shape[0]
  byte0 = start // 8
  shift = start % 8
  nbytes = (shift + length + 7) // 8
  acc = np.zeros(n, dtype=np.uint64)
  for i in range(nbytes):
    b = rowbytes[:, byte0 + i].astype(np.uint64)
    if i == 0:
      acc |= b >> shift
    else:
      acc |= b << (8 * i - shift)
  acc &= (1 << length) - 1
  if signed and length > 0:
    acc = acc.astype(np.int64)
    sign_bit = 1 << (length - 1)
    acc = np.where(acc & sign_bit, acc - (1 << length), acc)
  return acc.astype(np.float64)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--labels', default='/out/y_engine_labels.pkl')
  ap.add_argument('--cache', default='/cache/mt_replay.pkl')
  ap.add_argument('--max-records', type=int, default=150_000)
  ap.add_argument('--top', type=int, default=25)
  ap.add_argument('--min-len', type=int, default=4)
  ap.add_argument('--max-len', type=int, default=16)
  ap.add_argument('--out', default='/out/y_field_identity.txt')
  a = ap.parse_args()

  cache = pickle.load(open(a.cache, 'rb'))
  seg_names, label_data, rec = load_labels(a.labels, a.max_records)
  n = len(rec['d'])
  print(f'# {n} records from {len(seg_names)} segs', flush=True)

  # implied-code quantization: grid step reveals the wire LSB through K
  codes = rec['code']
  uc = np.unique(np.round(codes, 4))
  if len(uc) > 2:
    dif = np.diff(uc)
    dif = dif[dif > 1e-3]
    if len(dif):
      med = np.median(dif)
      print(f'# implied code: {len(uc)} unique values, median step {med:.5f} '
            f'(wire LSB ~= {med/K:.5f} code units per step; y-LSB at d=30m ~= {med*30:.4f} m)', flush=True)
  print(f'# code range [{codes.min():.1f}, {codes.max():.1f}], '
        f'd [{rec["d"].min():.1f}, {rec["d"].max():.1f}]', flush=True)

  # assemble objects with the PROVEN decoder formulas (block B near-field +
  # block C far fields feed ONE fused engine track table - resolve the engine
  # record's object by distance, not by slot index), then match engine records
  # against the per-segment observation list by time+distance.
  print('# assembling objects (decoder formulas)...', flush=True)
  t0 = time.time()
  B_BASE, B_N = 0x340, 2
  C_BASE, C_N = 0x380, 4
  B_ADDRS = [a for a in range(0x340, 0x34B) if a not in (0x342, 0x345, 0x348)]
  C_ADDRS = list(range(0x380, 0x398))

  def decode_d(block, f0):
    if block == 'C':
      raw = ((f0[3] & 0xF) << 8) | f0[2]
      if raw == 0 or (raw & 0x800):
        return None
      return (raw & 0x7FF) * 0.062319 - 1.671
    if block == 'B':
      raw = f0[3]
      if raw == 0 or raw >= 0xFF:
        return None
      return raw * 0.1
    return None

  def is_placeholder_C(f0):
    return f0[2] == 0xFF and f0[3] == 0xFF and f0[4] == 0xFF and f0[5] == 0xFF

  pairs_code = []   # (implied code, frame bytes, d, t)
  for si, seg in enumerate(seg_names):
    dcache = cache[seg]
    frames = []
    for ad in B_ADDRS + C_ADDRS:
      ts_l, raw = dcache.get(f'{ad:03x}_t'), dcache.get(f'{ad:03x}_d', b'')
      if not ts_l or not raw:
        continue
      for i in range(min(len(ts_l), len(raw) // 8)):
        frames.append((int(ts_l[i]), ad, raw[i * 8:(i + 1) * 8]))
    if not frames:
      continue
    frames.sort(key=lambda x: x[0])
    cycles = defaultdict(lambda: defaultdict(list))
    for t, fr_ad, dat in frames:
      b1 = dat[1]
      if fr_ad in B_ADDRS:
        block, base, nf = 'B', B_BASE, B_N
      else:
        block, base, nf = 'C', C_BASE, C_N
      cycles[b1][(block, (fr_ad - base) // nf)].append((t, dat))
    # observation list: (t_obj_median, d, bytes) for valid complete objects
    obs = []
    for b1, objs in cycles.items():
      for (block, obj), fl in objs.items():
        nf = B_N if block == 'B' else C_N
        if len(fl) < nf:
          continue
        fl2 = sorted(fl)
        f0 = fl2[0][1]
        if block == 'C' and is_placeholder_C(f0):
          continue
        dd = decode_d(block, f0)
        if dd is None or not (2.0 <= dd <= 124.0):
          continue
        obs.append((fl2[0][0], dd, b''.join(f[1] for f in fl2)))
    if not obs:
      continue
    obs.sort(key=lambda x: x[0])
    obs_t = np.asarray([o[0] for o in obs])
    m = rec['seg_idx'] == si
    for i in np.where(m)[0]:
      t_eng = int(rec['t'][i])
      lo = np.searchsorted(obs_t, t_eng - 40_000_000)
      hi = np.searchsorted(obs_t, t_eng + 40_000_000)
      if hi <= lo:
        continue
      d_eng = rec['d'][i]
      best = None
      for k in range(lo, hi):
        dj = abs(obs[k][1] - d_eng)
        if dj < 0.5 and (best is None or dj < best[0]):
          best = (dj, obs[k])
      if best is not None:
        pairs_code.append((rec['code'][i], best[1][2], best[1][1]))
        if len(pairs_code) >= a.max_records:
          break
    if len(pairs_code) >= a.max_records:
      break
  print(f'# distance-resolved pairs: {len(pairs_code)} ({time.time()-t0:.0f}s)', flush=True)
  code = np.asarray([p[0] for p in pairs_code])
  nbytes = max(len(p[1]) for p in pairs_code)
  rowbytes = np.zeros((len(pairs_code), nbytes), dtype=np.uint8)
  for i, p in enumerate(pairs_code):
    rowbytes[i, :len(p[1])] = np.frombuffer(p[1], dtype=np.uint8)
  dd = np.asarray([p[2] for p in pairs_code], dtype=np.float64)
  n = len(pairs_code)

  # candidate scan
  results = []
  t0 = time.time()
  code_c = code - code.mean()
  cnorm = np.sqrt((code_c ** 2).mean())
  for start in range(0, 256):
    for length in range(a.min_len, min(a.max_len, 256 - start) + 1):
      for signed in (0, 1):
        w = extract_fields(rowbytes, start, length, signed)
        ws = w.std()
        if ws < 1e-9:
          continue
        r = float(((w - w.mean()) * code_c).mean() / (ws * cnorm))
        results.append((abs(r), r, start, length, signed))
    if start % 64 == 63:
      print(f'# scan {start+1}/256 ({time.time()-t0:.0f}s)', flush=True)
  results.sort(reverse=True)

  # refine top candidates: affine fit + residual
  lines = []
  for ar, r, start, length, signed in results[:a.top * 3]:
    w = extract_fields(rowbytes, start, length, signed)
    A = np.vstack([w, np.ones_like(w)]).T
    (aa, bb), res, *_ = np.linalg.lstsq(A, code, rcond=None)
    resid = code - (aa * w + bb)
    lines.append((abs(r), r, start, length, signed, aa, bb,
                  float(np.median(np.abs(resid))), float(resid.std())))
  lines.sort(reverse=True)
  out = ['# top affine candidates: code_implied = a*w + b  (K=0.00050091)',
         '# |r|     r  start len sgn        a          b   med|res|   std(res)']
  for ar, r, start, length, signed, aa, bb, med, sd in lines[:a.top]:
    out.append(f'{ar:5.3f} {r:+5.3f}  {start:3d}  {length:2d}   {signed}   '
               f'{aa:10.5f}  {bb:10.3f}  {med:8.4f}  {sd:8.4f}')
  # residual quantization for the best: wire code should sit on a fixed grid
  if lines:
    ar, r, start, length, signed, aa, bb, med, sd = lines[0]
    w = extract_fields(rowbytes, start, length, signed)
    code_fit = aa * w + bb
    fr = code_fit - np.round(code_fit / max(med, 1e-9)) * med if med else code_fit
    out.append(f'# best candidate: affine-fit residual std {sd:.4f} code units '
               f'(~{sd * 30 * 0.00050091:.4f} m at d=30 m)')
  txt = '\n'.join(out) + '\n'
  open(a.out, 'w').write(txt)
  print(txt, flush=True)


if __name__ == '__main__':
  main()
