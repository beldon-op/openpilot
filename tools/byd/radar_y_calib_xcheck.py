#!/usr/bin/env python3
"""Vendor-calibration offline cross-check (byd-radar-calib-xcheck).

Replicates the vendor _GeometryLearning estimator (sunnypilot stream.py v74,
profile 2 = Song) over OUR y-wire verdict observations from the C2 pairing
archive (y_c2f_pairs.pkl: d=radar decoded, y=vision sign-flipped, seg, bytes,
holdout_segs).  Pure Python + numpy - no vendor .so (spec: estimator must be
replicated, not invoked).

Wire re-derivation per the 2.7 verdict (radar_y_wire_decode.py discipline):
  bytes[0:8] is the C-block F0 carrier (archive built with --block C --d-rows)
    u_d = le_bits(f0, 16, 11)  -> base = u_d * C_SCALE
    u_c = le_bits(f0, 35, 11)  -> code = u_c - CODE_ZERO
  sanity: base == d + 1.671 (exact, since archive d = base + C_DIST_OFF)
  abort when sanity failure rate > 1% (spec gate).

Vendor-verbatim estimator on the products stream (base in (12, 80) - the
static part of vendor's moving gate; context gates unavailable offline):
  products = code * base; iterative median mean-shift (window +/-1500,
  >=50 inliers, <=30 iters, convergence <0.01); c0_learn = median(code|inlier)
  plausible: |c0| <= 128; cross-check median(code | |y_v|<0.3) vs c0_learn < 8
  holdout (segment-level): yhat = (base + d_off)*K*(code - c0) vs vision y,
  gates |signed bias| < 0.15 and med|dy| < 0.4; two-window stability by
  segment-order halves (|dc0| < 2); named reason counters on every rejection.

Known deviations from vendor (recorded in the report, design.md):
  - d_off fixed at -1.671 (v_x not persisted in the archive) -> y axis only
  - moving gate degrades to base range (no v_ego/steer/yaw in archive)
  - two-window is segment-order halves (weaker than fresh-sample re-estimate)

  python3 tools/byd/radar_y_calib_xcheck.py [--pairs /tmp/yout/y_c2f_pairs.pkl]
"""
import argparse
import pickle
import time
from collections import Counter

import numpy as np

# --- verdict constants (pinned; radar_y_wire_decode.py / radar_multitarget.py)
C_SCALE = 0.062319
C_DIST_OFF = -1.671
CODE_ZERO = 1024
Y_K = 0.00050091
C0_DEFAULT = -15.0
D_OFF_DEFAULT = -1.671

# --- vendor _GeometryLearning gates (stream.py v74; verbatim numbers)
CLUSTER_WIN = 1500.0
CLUSTER_MIN = 50
CLUSTER_ITERS = 30
CLUSTER_CONV = 0.01
PLAUSIBLE_C0 = 128.0
CROSS_GATE = 8.0
BIAS_GATE = 0.15          # vendor BIAS_LAT (signed holdout bias, y axis)
NOISE_GATE = 0.4          # vendor ABS_LAT (holdout med|dy|, y axis)
STABILITY_GATE = 2.0      # |dc0| across two windows
MIN_HOLDOUT = 60
MIN_CROSS = 100
SANITY_FAIL_MAX = 0.01    # spec: abort above 1% layout failures
BASE_LO, BASE_HI = 12.0, 80.0   # static part of vendor's moving gate
CROSS_Y_MAX = 0.3


class Probe:
  def __init__(self):
    self.reasons = Counter()

  def reason(self, name):
    self.reasons[name] += 1


def le_bits(data, start, ln):
  return (int.from_bytes(data, 'little') >> start) & ((1 << ln) - 1)


def rederive(pairs, probe):
  """(base, code) from the F0 carrier + base==d+1.671 sanity gate."""
  n = len(pairs['d'])
  base = np.empty(n)
  code = np.empty(n)
  fail = []
  for i in range(n):
    f0 = pairs['bytes'][i][:8]
    u_d = le_bits(f0, 16, 11)
    u_c = le_bits(f0, 35, 11)
    base[i] = u_d * C_SCALE
    code[i] = u_c - CODE_ZERO
    if abs(base[i] - (pairs['d'][i] - C_DIST_OFF)) > 1e-6:
      if len(fail) < 5:
        fail.append(i)
      probe.reason('sanity_failed')
  rate = probe.reasons['sanity_failed'] / n
  return base, code, rate, fail


def estimate_c0(base, code, probe, label=''):
  """Vendor-verbatim cluster: iterative median mean-shift on code*base."""
  products = code * base
  center = 0.0
  next_center = 0.0
  iters = 0
  for it in range(CLUSTER_ITERS):
    iters = it + 1
    sel = products[np.abs(products - center) < CLUSTER_WIN]
    if len(sel) < CLUSTER_MIN:
      probe.reason(f'no_lane_peak{label}')
      return None
    next_center = float(np.median(sel))
    if abs(next_center - center) < CLUSTER_CONV:
      break
    center = next_center
  inlier = np.abs(products - next_center) < CLUSTER_WIN
  if inlier.sum() < CLUSTER_MIN:
    probe.reason(f'no_lane_peak{label}')
    return None
  c0 = float(np.median(code[inlier]))
  if not np.isfinite(c0) or abs(c0) > PLAUSIBLE_C0:
    probe.reason(f'out_of_bounds{label}')
    return None
  return dict(c0=c0, center=next_center, n_inlier=int(inlier.sum()),
              iters=iters, inlier=inlier)


def cross_check(code, y_v, probe):
  """Vendor cross stream: near-zero-lateral pairs' median code vs c0."""
  sel = np.abs(y_v) < CROSS_Y_MAX
  if sel.sum() < MIN_CROSS:
    probe.reason('cross_sample_starved')
    return None, int(sel.sum())
  return float(np.median(code[sel])), int(sel.sum())


def holdout_eval(base, code, y_v, c0, d_off, probe):
  """Segment-level holdout: yhat vs vision y under vendor's y-axis gates."""
  dy = y_v - (base + d_off) * Y_K * (code - c0)
  return float(np.median(dy)), float(np.median(np.abs(dy)))


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--pairs', default='/tmp/yout/y_c2f_pairs.pkl')
  ap.add_argument('--sweep', default='-45:10:0.5',
                  help='c0 sensitivity sweep start:end:step')
  ap.add_argument('--json-out', default=None)
  a = ap.parse_args()

  t0 = time.time()
  probe = Probe()
  pairs = pickle.load(open(a.pairs, 'rb'))
  n = len(pairs['d'])
  print(f'# pairs: {n} from {a.pairs}', flush=True)

  base, code, sanity_rate, fail = rederive(pairs, probe)
  print(f'# sanity base==d+1.671: failure rate {sanity_rate*100:.4f}% '
        f'({probe.reasons["sanity_failed"]}/{n})', flush=True)
  if sanity_rate > SANITY_FAIL_MAX:
    print(f'# ABORT sanity_failed (first failing pairs: {fail})', flush=True)
    return 1

  y_v = pairs['y']
  seg = pairs['seg']
  # Holdout must be the ARCHIVE's own split (spec), frozen as holdout_segs
  # names in the pickle.  The fit's segs list = sorted(cache_keys & vision_keys)
  # is rebuildable from the caches, so name -> seg index is exact (the name
  # hash only chose the split at fit time; the choice is persisted).
  ho_names = set(pairs['holdout_segs'])
  seg_names = None
  try:
    cache_keys = set(pickle.load(open('/tmp/mt_replay.pkl', 'rb')).keys())
    vision_keys = set(pickle.load(open('/tmp/y_vision.pkl', 'rb')).keys())
    seg_names = sorted(cache_keys & vision_keys)
  except OSError as e:
    print(f'# archive name lookup unavailable ({e}); fallback split', flush=True)
  if seg_names is not None and len(seg_names) > seg.max():
    name_of_seg = {i: nm for i, nm in enumerate(seg_names)}
    ho = np.array([name_of_seg.get(int(s)) in ho_names for s in seg])
  else:
    seg_ids_fb = np.sort(np.unique(seg))
    ho_ids = set(int(s) for i, s in enumerate(seg_ids_fb) if i % 5 == 4)
    ho = np.isin(seg, list(ho_ids))
    print('# fallback holdout: 1-of-5 by sorted seg index (deviation)', flush=True)
  tr = ~ho
  print(f'# holdout: {int(ho.sum())} pairs in '
        f'{len(set(seg[ho].tolist()))} segs (archive split, {len(ho_names)} names)',
        flush=True)

  # products stream: static part of vendor's moving gate
  ps = (base > BASE_LO) & (base < BASE_HI)
  print(f'# products stream: {int(ps.sum())} pairs (base in ({BASE_LO},{BASE_HI}))',
        flush=True)

  est = estimate_c0(base[ps], code[ps], probe)
  if est is None:
    print(f'# ABORT estimator rejected: {dict(probe.reasons)}', flush=True)
    return 1
  c0 = est['c0']
  print(f"# c0_learn = {c0:.3f}  (cluster center {est['center']:.1f}, "
        f"inliers {est['n_inlier']}, iters {est['iters']})  vs default {C0_DEFAULT}", flush=True)

  xmed, n_cross = cross_check(code, y_v, probe)
  cross_delta = None if xmed is None else abs(c0 - xmed)
  print(f'# cross-check: median(code | |y_v|<{CROSS_Y_MAX}) = {xmed} over '
        f'{n_cross} pairs -> |delta| = {cross_delta} (gate <{CROSS_GATE})', flush=True)

  # ungated variant (no base restriction) as robustness check
  est_all = estimate_c0(base, code, probe, label='_ungated')
  print(f"# c0_learn ungated = {est_all['c0']:.3f} (inliers {est_all['n_inlier']})"
        if est_all else '# c0_learn ungated: rejected', flush=True)

  # holdout: learned c0 and default c0, both with default d_off
  res = {}
  for name, c in (('learned', c0), ('default', C0_DEFAULT)):
    if ho.sum() < MIN_HOLDOUT:
      probe.reason('holdout_sample_starved')
      break
    bias, med = holdout_eval(base[ho], code[ho], y_v[ho], c, D_OFF_DEFAULT, probe)
    ok = abs(bias) < BIAS_GATE and med < NOISE_GATE
    res[name] = (bias, med, ok)
    print(f'# holdout [{name:7s}] c0={c:7.3f}: bias={bias:+.4f} med|dy|={med:.4f} '
          f'-> {"PASS" if ok else "FAIL"} (gates |bias|<{BIAS_GATE}, med<{NOISE_GATE})',
          flush=True)

  # two-window stability by segment-order halves
  seg_ids = np.sort(np.unique(seg))
  mid = seg_ids[len(seg_ids) // 2]
  w1, w2 = ps & (seg < mid), ps & (seg >= mid)
  e1 = estimate_c0(base[w1], code[w1], probe, label='_w1')
  e2 = estimate_c0(base[w2], code[w2], probe, label='_w2')
  dc0 = None
  if e1 and e2:
    dc0 = abs(e1['c0'] - e2['c0'])
    print(f'# two-window: w1 c0={e1["c0"]:.3f} (n={int(w1.sum())})  '
          f'w2 c0={e2["c0"]:.3f} (n={int(w2.sum())})  |dc0|={dc0:.3f} '
          f'(gate <{STABILITY_GATE})', flush=True)
  else:
    print('# two-window: a window estimate was rejected', flush=True)

  # sensitivity sweep: gate margins vs c0
  lo, hi, step = (float(x) for x in a.sweep.split(':'))
  sweep = []
  c = lo
  while c <= hi + 1e-9:
    bias, med = holdout_eval(base[ho], code[ho], y_v[ho], c, D_OFF_DEFAULT, probe)
    sweep.append((c, bias, med))
    c += step
  ok_cs = [c for c, b, m in sweep if abs(b) < BIAS_GATE and m < NOISE_GATE]
  if ok_cs:
    margin_lo, margin_hi = min(ok_cs), max(ok_cs)
    print(f'# sensitivity: gates PASS for c0 in [{margin_lo:.2f}, {margin_hi:.2f}] '
          f'-> margin around default {C0_DEFAULT}: '
          f'[{margin_lo - C0_DEFAULT:+.2f}, {margin_hi - C0_DEFAULT:+.2f}]', flush=True)
  else:
    margin_lo = margin_hi = None
    print('# sensitivity: gates FAIL across the whole sweep', flush=True)

  # verdict tree
  q1 = (c0 is not None and abs(c0 - C0_DEFAULT) < STABILITY_GATE)
  q2 = all(v[2] for v in res.values()) if res else False
  q3_ok = (cross_delta is not None and cross_delta < CROSS_GATE
           and dc0 is not None and dc0 < STABILITY_GATE)
  print('# verdict tree:', flush=True)
  print(f'#   Q1 c0_learn vs default(-15, +-2): {"CONVERGED" if q1 else "DEVIATED"}', flush=True)
  print(f'#   Q2 vendor acceptance gates:       {"PASS" if q2 else "FAIL"}', flush=True)
  print(f'#   Q3 cross-check + two-window:      {"PASS" if q3_ok else "FAIL"}', flush=True)
  if q1 and q2 and q3_ok:
    print('#   -> branch: 互证收口 (defaults valid for this unit; '
          'per-unit learning layer adds nothing for display)', flush=True)
  elif q2 and not q1:
    print('#   -> branch: 安装公差或估计偏差 (holdout passes but c0 moved) '
          '-> C-full with context needed', flush=True)
  else:
    print('#   -> branch: FAIL -> pairing-caliber echo -> C-full with '
          'association hygiene stats', flush=True)

  print(f'# reasons: {dict(probe.reasons)}', flush=True)
  print(f'# done in {time.time() - t0:.0f}s', flush=True)

  if a.json_out:
    import json
    json.dump(dict(pairs=n, sanity_rate=sanity_rate, c0_learn=c0,
                   c0_ungated=est_all['c0'] if est_all else None,
                   cross_median=xmed, cross_delta=cross_delta, n_cross=n_cross,
                   holdout={k: dict(bias=v[0], med=v[1], ok=v[2]) for k, v in res.items()},
                   two_window=dict(dc0=dc0,
                                   w1=e1['c0'] if e1 else None,
                                   w2=e2['c0'] if e2 else None),
                   sweep=dict(lo=lo, hi=hi, step=step,
                              pass_range=[margin_lo, margin_hi]),
                   q1=q1, q2=q2, q3=q3_ok, reasons=dict(probe.reasons)),
              open(a.json_out, 'w'), indent=1)
    print(f'# json -> {a.json_out}', flush=True)
  return 0


if __name__ == '__main__':
  raise SystemExit(main())
