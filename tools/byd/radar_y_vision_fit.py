#!/usr/bin/env python3
"""C2 vision-anchored lateral-y fit (byd-radar-y-wire-port 2.1/2.2/2.3/2.4).

Pairs OUR decoder's real slot-table objects with modelV2 vision leads
(distance+time gates, radard-style association), then scans every wire bit
field of the paired object frames for the engine's lateral model

    y = (d + D0) * (A*w + B)          # D0 ~ 1.671 (block C offset + mount),
                                      # A/B absorb K/code-scale/c0

i.e. y/d is LINEAR in the wire field w (engine closed form recovered 2026-11-01:
y = (base + d_off)*K*(code - c0), base = d + 1.671). S1's mistakes are corrected:
the regression target is y/d (not y), and pairing is distance-gated (not slot
index). Acceptance on held-out segments: median |dy| <= 0.4 m, |bias| <= 0.15 m.

  docker run --rm --platform linux/arm64 -v /tmp:/cache:ro \
    -v <repo>:/repo:ro -v /tmp/yout:/out pyarmor-local \
    python /repo/tools/byd/radar_y_vision_fit.py \
    --cache /cache/mt_replay.pkl --vision /cache/y_vision.pkl --out /out/y_c2
"""
import argparse
import pickle
import time
from collections import defaultdict

import numpy as np

# mirror radar_multitarget.py EXACTLY: divmod stride and completion count are
# DIFFERENT for block B (stride 3, complete at 2 frames) - using 3 as the
# completion count killed every B object (2026-11-01, 20x yield bug)
B_BASE, B_STRIDE, B_NF = 0x340, 3, 2
C_BASE, C_STRIDE, C_NF = 0x380, 4, 4
B_ADDRS = [a for a in range(0x340, 0x34B) if a not in (0x342, 0x345, 0x348)]
C_ADDRS = list(range(0x380, 0x398))
PROB_MIN = 0.9
PAIR_DT_NS = 75_000_000
PAIR_DD_M = 2.5
D0_DEFAULT = 1.671


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


def is_placeholder(f0):
  return f0[2] == 0xFF and f0[3] == 0xFF and f0[4] == 0xFF and f0[5] == 0xFF


def j1850_crc8(payload: bytes) -> int:
  """SAE J1850 CRC-8 over b1..b7 (radar_multitarget.py convention)."""
  crc = 0xFF
  for byte in payload:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ 0xFF


def crc_ok(dat: bytes) -> bool:
  return len(dat) == 8 and dat[0] == j1850_crc8(dat[1:8])


def seg_objects(dcache, d_rows=False, block_filter='all'):
  """Assemble per-cycle objects -> (t, d, bytes, block) list, placeholder-free.

  F0 is identified by ADDRESS PHASE ((addr-base) % stride == 0), never by
  timestamp order - non-F0 frames carry bit27 set and would decode as garbage
  (2026-11-01: timestamp-order F0 selection starved assembly 50x). Block B
  mirrors the decoder: divmod stride 3, completion at 2 frames. With d_rows,
  a C object's bytes gain ALL block-D frames (0x3C8-0x3FF) of the same cycle,
  address-ordered - no index-mapping assumption (D is likely 14 objects x 4
  phases; the lateral code may live there).
  """
  D_ADDRS = list(range(0x3C8, 0x400)) if d_rows else []
  frames = []
  for ad in B_ADDRS + C_ADDRS + D_ADDRS:
    ts_l, raw = dcache.get(f'{ad:03x}_t'), dcache.get(f'{ad:03x}_d', b'')
    if not ts_l or not raw:
      continue
    for i in range(min(len(ts_l), len(raw) // 8)):
      frames.append((int(ts_l[i]), ad, raw[i * 8:(i + 1) * 8]))
  if not frames:
    return []
  frames.sort(key=lambda x: x[0])
  # sequential cycle segmentation on b1 CHANGE (decoder semantics): b1 cycles
  # through only 16 values, so grouping by b1 VALUE merges ~67 real cycles.
  obs = []
  cur, cur_b1 = {}, None

  def flush(cur):
    d_frames = sorted((ad, dat) for (blk, obj), phs in cur.items() if blk == 'D'
                      for (_t, ad, dat) in phs.values())
    d_byts = b''.join(dat for _ad, dat in d_frames)
    for (block, obj), phs in cur.items():
      if block == 'D':
        continue
      if block_filter != 'all' and block != block_filter:
        continue
      stride, nf = (B_STRIDE, B_NF) if block == 'B' else (C_STRIDE, C_NF)
      if len(phs) < nf:
        continue
      f0_t, f0 = phs[0][0], phs[0][2]
      if block == 'C' and is_placeholder(f0):
        continue
      dd = decode_d(block, f0)
      if dd is None or not (2.0 <= dd <= 124.0):
        continue
      byts = b''.join(phs[ph][2] for ph in range(nf))
      if block == 'C' and d_rows and d_byts:
        byts = byts + d_byts
      obs.append((f0_t, dd, byts, block))

  for t, fr_ad, dat in frames:
    if not crc_ok(dat):
      continue   # corrupt frame: skip (decoder precedent) - must not overwrite
    if cur_b1 is not None and dat[1] != cur_b1:
      flush(cur)
      cur = {}
    cur_b1 = dat[1]
    if fr_ad in B_ADDRS:
      block, base, stride = 'B', B_BASE, B_STRIDE
    elif fr_ad in C_ADDRS:
      block, base, stride = 'C', C_BASE, C_STRIDE
    else:
      block, base, stride = 'D', 0x3C8, 4
    obj, ph = divmod(fr_ad - base, stride)
    cur.setdefault((block, obj), {})[ph] = (t, fr_ad, dat)
  flush(cur)
  return obs


def extract_fields(rowbytes, start, length, signed):
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
  if signed:
    acc = acc.astype(np.int64)
    sign_bit = 1 << (length - 1)
    acc = np.where(acc & sign_bit, acc - (1 << length), acc)
  return acc.astype(np.float64)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--cache', default='/cache/mt_replay.pkl')
  ap.add_argument('--vision', default='/cache/y_vision.pkl')
  ap.add_argument('--out', default='/out/y_c2')
  ap.add_argument('--max-pairs', type=int, default=400_000)
  ap.add_argument('--scan-sample', type=int, default=40_000,
                  help='train subsample for the candidate scan (refit on full)')
  ap.add_argument('--d-rows', action='store_true',
                  help='append same-index block-D row frames to C object bytes')
  ap.add_argument('--block', choices=['all', 'B', 'C'], default='all')
  ap.add_argument('--ext-cache', default=None,
                  help='extended-address cache (0x101-0x108 etc.): append frames '
                       'in the object cycle window to the candidate bytes')
  ap.add_argument('--min-len', type=int, default=4)
  ap.add_argument('--max-len', type=int, default=16)
  ap.add_argument('--top', type=int, default=15)
  a = ap.parse_args()

  cache = pickle.load(open(a.cache, 'rb'))
  vision = pickle.load(open(a.vision, 'rb'))
  ext = pickle.load(open(a.ext_cache, 'rb')) if a.ext_cache else None
  EXT_ADDRS = [0x101 + i for i in range(8)]
  segs = sorted(set(cache.keys()) & set(vision.keys()))
  print(f'# {len(segs)} segs', flush=True)
  t0 = time.time()

  # 1) pair objects with vision leads
  P_T, P_D, P_Y, P_BYTES, P_SEG = [], [], [], [], []
  holdout = []
  for si, seg in enumerate(segs):
    # deterministic holdout split: every 5th segment by name hash
    if abs(hash(seg)) % 5 == 0:
      holdout.append(si)
    rows = vision[seg]['rows']
    if rows:
      v_t = np.asarray([r[0] for r in rows], dtype=np.int64)
      v_x = np.asarray([r[1] if r[1] is not None else np.nan for r in rows])
      v_y = np.asarray([r[2] if r[2] is not None else np.nan for r in rows])
      v_p = np.asarray([r[3] if r[3] is not None else 0.0 for r in rows])
      good = (v_p >= PROB_MIN) & np.isfinite(v_x) & np.isfinite(v_y) & (np.abs(v_y) < 3.0)
      v_t, v_x, v_y = v_t[good], v_x[good], -v_y[good]  # adapter: radar-frame y = -schema y
    else:
      v_t = np.zeros(0)
    obs = seg_objects(cache[seg], d_rows=a.d_rows, block_filter=a.block)
    obs.sort(key=lambda x: x[0])
    o_t = np.asarray([o[0] for o in obs], dtype=np.int64)
    e_ts = e_dat = None
    if ext is not None:
      e_rows = []
      for ead in EXT_ADDRS:
        ets, eraw = ext.get(seg, {}).get(f'{ead:03x}_t'), ext.get(seg, {}).get(f'{ead:03x}_d', b'')
        if not ets or not eraw:
          continue
        for i in range(min(len(ets), len(eraw) // 8)):
          e_rows.append((int(ets[i]), eraw[i * 8:(i + 1) * 8]))
      if e_rows:
        e_rows.sort(key=lambda x: x[0])
        e_ts = np.asarray([r[0] for r in e_rows], dtype=np.int64)
      else:
        e_rows = None
    for t, dd, byts, blk in obs:
      if not len(v_t):
        break
      if e_rows is not None:
        lo_e = int(np.searchsorted(e_ts, t - 90_000_000))
        hi_e = int(np.searchsorted(e_ts, t + 30_000_000))
        if hi_e > lo_e:
          byts = byts + b''.join(r[1] for r in e_rows[lo_e:hi_e])
      lo = np.searchsorted(v_t, t - PAIR_DT_NS)
      hi = np.searchsorted(v_t, t + PAIR_DT_NS)
      if hi <= lo:
        continue
      best = None
      for k in range(lo, hi):
        dj = abs(v_x[k] - dd)
        if dj <= PAIR_DD_M and (best is None or dj < best[0]):
          best = (dj, k)
      if best is None:
        continue
      P_T.append(t)
      P_D.append(dd)
      P_Y.append(v_y[best[1]])
      P_BYTES.append(byts)
      P_SEG.append(si)
      if len(P_D) >= a.max_pairs:
        break
    if len(P_D) >= a.max_pairs:
      break
  print(f'# paired samples: {len(P_D)} ({time.time()-t0:.0f}s)', flush=True)
  if not P_D:
    print('# NO PAIRS - abort', flush=True)
    return

  n = len(P_D)
  d_arr = np.asarray(P_D)
  y_arr = np.asarray(P_Y)
  seg_arr = np.asarray(P_SEG)
  holdout_mask = np.isin(seg_arr, holdout)
  train_mask = ~holdout_mask
  nb = max(len(b) for b in P_BYTES)
  rowbytes = np.zeros((n, nb), dtype=np.uint8)
  for i, b in enumerate(P_BYTES):
    rowbytes[i, :len(b)] = np.frombuffer(b, dtype=np.uint8)
  print(f'# train={train_mask.sum()} holdout={holdout_mask.sum()} '
        f'({len(holdout)} holdout segs)', flush=True)

  # 2) candidate scan on TRAIN (subsampled): z = y/(d+D0) linear in w
  z = y_arr[train_mask] / (d_arr[train_mask] + D0_DEFAULT)
  scan_idx = np.arange(len(z))
  if a.scan_sample and len(z) > a.scan_sample:
    scan_idx = np.linspace(0, len(z) - 1, a.scan_sample).astype(int)
  z_s = z[scan_idx]
  zc = z_s - z_s.mean()
  zn = np.sqrt((zc ** 2).mean())
  rb_scan = rowbytes[train_mask][scan_idx]
  results = []
  bits = nb * 8
  for start in range(0, bits):
    for length in range(a.min_len, min(a.max_len, bits - start) + 1):
      for signed in (0, 1):
        w = extract_fields(rb_scan, start, length, signed)
        ws = w.std()
        if ws < 1e-9:
          continue
        r = float(((w - w.mean()) * zc).mean() / (ws * zn))
        results.append((abs(r), r, start, length, signed))
  results.sort(reverse=True)
  print(f'# scanned {len(results)} candidates ({time.time()-t0:.0f}s)', flush=True)

  # degenerate-model guard: rank by residual improvement over the B-only
  # (constant-bearing) model, and require the slope to carry real variance -
  # A*w with A~0 just reproduces y = const*(d+1.671) via the B term.
  z_tr = y_arr[train_mask] / (d_arr[train_mask] + D0_DEFAULT)
  B0 = float(np.median(z_tr))
  resid_B0 = z_tr - B0
  std_B0 = float(resid_B0.std())
  w_all_cache = {}
  def get_w(start, length, signed):
    key = (start, length, signed)
    if key not in w_all_cache:
      w_all_cache[key] = extract_fields(rowbytes, start, length, signed)
    return w_all_cache[key]
  # baseline (no wire): y = B0*(d+1.671)
  dy_ho_b0 = y_arr[holdout_mask] - B0 * (d_arr[holdout_mask] + D0_DEFAULT) if holdout_mask.sum() else None
  base_ho = (float(np.median(np.abs(dy_ho_b0))), float(np.median(dy_ho_b0))) if dy_ho_b0 is not None else (float('nan'), float('nan'))

  scored = []
  for ar, r, start, length, signed in results[:400]:
    w_tr = extract_fields(rowbytes[train_mask], start, length, signed)
    A, B = np.polyfit(w_tr, z, 1)
    gain = std_B0 - float((z - (A * w_tr + B)).std())   # residual std improvement
    # slope must contribute: A*std(w) vs std(z)
    slope_frac = abs(A) * float(w_tr.std()) / max(std_B0, 1e-12)
    scored.append((gain, slope_frac, ar, r, start, length, signed, float(A), float(B)))
  scored.sort(reverse=True)

  lines = []
  for gain, slope_frac, ar, r, start, length, signed, A, B in scored[:60]:
    if slope_frac < 0.3:   # slope carries <30% of z spread: degenerate
      continue
    w_tr = get_w(start, length, signed)[train_mask]
    d_tr = d_arr[train_mask]
    y_pred_tr = (d_tr + D0_DEFAULT) * (A * w_tr + B)
    dy_tr = y_arr[train_mask] - y_pred_tr
    if holdout_mask.sum():
      w_ho = get_w(start, length, signed)[holdout_mask]
      y_pred_ho = (d_arr[holdout_mask] + D0_DEFAULT) * (A * w_ho + B)
      dy_ho = y_arr[holdout_mask] - y_pred_ho
      med_ho, bias_ho = float(np.median(np.abs(dy_ho))), float(np.median(dy_ho))
    else:
      med_ho = bias_ho = float('nan')
    lines.append((gain, slope_frac, ar, r, start, length, signed, A, B,
                  float(np.median(np.abs(dy_tr))), float(np.median(dy_tr)),
                  med_ho, bias_ho))
    if len(lines) >= a.top:
      break

  out = ['# C2 vision-anchored fit: y = (d+1.671)*(A*w+B), z=y/(d+1.671) ~ w',
         f'# baseline (B-only, no wire): holdout med|dy|={base_ho[0]:.3f} bias={base_ho[1]:+.3f}',
         '# gain  slp%  |r|     r  start len sgn        A          B   train med|dy|  bias '
         '   HOLDOUT med|dy|  bias   gate',
         '# gate: holdout med|dy| <= 0.4 and |bias| <= 0.15; slp% = slope variance share']
  for gain, slope_frac, ar, r, start, length, signed, A, B, mtr, btr, mho, bho in lines:
    gate = 'PASS' if (mho == mho and mho <= 0.4 and abs(bho) <= 0.15) else '-'
    out.append(f'{gain:5.3f} {slope_frac*100:4.0f}% {ar:5.3f} {r:+5.3f}  {start:3d}  {length:2d}   {signed}   '
               f'{A:11.6f}  {B:9.4f}   {mtr:8.3f} {btr:+7.3f}   '
               f'{mho:8.3f} {bho:+7.3f}   {gate}')
  txt = '\n'.join(out) + '\n'
  open(a.out + '_fit.txt', 'w').write(txt)
  print(txt, flush=True)

  with open(a.out + '_pairs.pkl', 'wb') as fp:
    pickle.dump({'d': d_arr, 'y': y_arr, 'seg': seg_arr, 'bytes': P_BYTES,
                 'holdout_segs': [segs[i] for i in holdout]}, fp, protocol=4)
  print(f'# pairs saved -> {a.out}_pairs.pkl ({time.time()-t0:.0f}s total)', flush=True)


if __name__ == '__main__':
  main()
