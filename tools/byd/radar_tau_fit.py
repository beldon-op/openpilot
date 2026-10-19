"""Per-gear headway (tau) fit: the last log-side discriminator for u (task #2).

HUD bar classes are TIME-GAP bands (established: corr(b7,vEgo)=0.70-0.95 within
class), so within a steady hold of class c:   b7*u ~= tau_c * vEgo + d0_c.
Fit per segment per class (frames inside constant-class holds >= 4 s, trimmed
2 s each side, b7 in [9,246], presence==7, vEgo > 3 m/s, >=100 frames, spread
of vEgo >= 1 m/s), then report the pooled ladder tau_c(u) for the surviving
unit candidates u in {0.3048 feet, 0.25}. ACC headway settings are published
values (industry ladder e.g. 1.0/1.4/1.8 s style steps, d0 ~ 3-6 m standstill
gap); whichever u yields a ladder closest to a plausible published step set
gets support. This is EVIDENCE ONLY - dbc factor stays 0.3048+HYPOTHESIS and
the road-test gate is unchanged (docs/byd-radar-010x.md risk #1).

Usage: .venv/bin/python tools/byd/radar_tau_fit.py --out /tmp/radar_tau.txt
"""
import sys, argparse
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import radar_seglib as L

MIN_V = 3.0        # m/s, exclude near-standstill regime for tau regression
MAX_V = 25.0       # m/s, c4 @30m/s rides the range cap (b7~246 pins, 247 census)
MAX_B7 = 200       # counts, stay clear of saturation ceiling for slope fits
MAX_ACC = 0.4      # |m/s^2|, steady cruise only (lead accelerating contaminates)
MIN_FR = 100
MIN_VSPREAD = 1.0  # m/s
MIN_HOLD_S = 4.0
TRIM_S = 2.0
CANDS = [(0.3048, "feet"), (0.25, "0.25"), (0.42, "u=0.42(bound)")]


def theil_sen(x, y):
  """Robust slope/intercept via pairwise medians (subsampled for cost)."""
  n = len(x)
  if n > 250:  # deterministic subsample (no RNG): stride to ~250
    stride = n // 250 + 1
    x, y = x[::stride], y[::stride]
    n = len(x)
  slopes = []
  for i in range(n):
    dx = x[i + 1:] - x[i]
    dy = y[i + 1:] - y[i]
    ok = np.abs(dx) > 0.15
    slopes.append(dy[ok] / dx[ok])
  slopes = np.concatenate(slopes)
  if len(slopes) == 0:
    return None, None
  m = float(np.median(slopes))
  b = float(np.median(y - m * x))
  return m, b


def fits_for_seg(s):
  """Return {cls: list of (slope, intercept, n, r2)} across that seg's holds."""
  t = s["109_t"]
  if len(t) < 500:
    return {}
  b7 = s["109_d"][:, 7].astype(np.float64)
  ve = L.vego_at(t, s)
  pr = L.pres_at(t, s)
  ht, hc = s["hud_t"], s["hud_cls"]
  # own-vehicle acceleration on the carState grid, interpolated onto 0x109 t
  ct, cv = s["cs_t"], s["cs_v"]
  if len(ct) >= 20:
    acc = np.gradient(cv, ct / L.SEC)
    ae = np.interp(t, ct, acc, left=0.0, right=0.0)
  else:
    ae = np.zeros_like(b7)
  out = {}
  # constant-class HUD runs
  i = 0
  while i < len(hc):
    j = i
    while j < len(hc) and hc[j] == hc[i]:
      j += 1
    c = int(hc[i])
    a_t, z_t = ht[i], ht[max(j - 1, i)]
    if c in (2, 3, 4) and (z_t - a_t) >= MIN_HOLD_S * L.SEC:
      lo, hi = a_t + TRIM_S * L.SEC, z_t - TRIM_S * L.SEC
      k = np.searchsorted(t, [lo, hi])
      sel = slice(k[0], k[1])
      m = ((b7[sel] >= 9) & (b7[sel] <= MAX_B7) & (pr[sel] == 7) &
           (ve[sel] > MIN_V) & (ve[sel] <= MAX_V) & (np.abs(ae[sel]) < MAX_ACC) &
           np.isfinite(ve[sel]))
      x, y = ve[sel][m], b7[sel][m]
      if len(x) >= MIN_FR and (x.max() - x.min()) >= MIN_VSPREAD:
        slope, itc = theil_sen(x, y)
        if slope is None or slope <= 0:
          i = j; continue
        ym = y.mean()
        yhat = slope * x + itc
        ss_res = float(np.dot(y - yhat, y - yhat))
        ss_tot = float(np.dot(y - ym, y - ym))
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0
        if r2 >= 0.3:
          out.setdefault(c, []).append((slope, itc, len(x), r2))
    i = j
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--pkl", default="/tmp/seg1005.pkl")
  ap.add_argument("--out", default="/tmp/radar_tau.txt")
  a = ap.parse_args()
  cache = L.load(a.pkl)
  lines = []
  def say(*x):
    s = " ".join(str(v) for v in x)
    print(s, flush=True)
    lines.append(s)

  per_cls = {c: [] for c in (2, 3, 4)}   # (slope, itc, r2, n) weighted pool
  seg_ok = {c: 0 for c in (2, 3, 4)}
  for name, s in cache.items():
    res = fits_for_seg(s)
    for c, lst in res.items():
      for slope, itc, n, r2 in lst:
        if r2 >= 0.4:   # TS line r2 is stricter than OLS by construction
          per_cls[c].append((slope, itc, n, r2))
          seg_ok[c] += 1

  say(f"# segments with usable hold-fits per class: "
      f"{ {c: sum(1 for x in per_cls[c]) for c in per_cls} }")
  say("# (slope = counts per m/s = tau/u ; itc = counts)")
  ladder = {}
  for c in (2, 3, 4):
    if not per_cls[c]:
      say(f"class {c}: NO FITS")
      continue
    arr = np.array([[sl, it, n] for sl, it, n, r2 in per_cls[c]])
    w = arr[:, 2]
    # pooled through weighted median of per-hold slopes (robust)
    order = np.argsort(arr[:, 0])
    cw = np.cumsum(w[order]) / w.sum()
    med_slope = float(np.interp(0.5, cw, arr[order, 0]))
    med_itc = float(np.median(arr[:, 1]))
    p25 = float(np.percentile(arr[:, 0], 25)); p75 = float(np.percentile(arr[:, 0], 75))
    say(f"class {c}: holds={len(arr)} slope med={med_slope:.3f} c/(m/s) "
        f"[p25={p25:.2f},p75={p75:.2f}]  itc med={med_itc:.1f} c")
    ladder[c] = (med_slope, med_itc)

  say("\n== implied tau_c and standstill offset d0_c under each candidate u ==")
  for u, nm in CANDS:
    row = []
    for c in (2, 3, 4):
      if c in ladder:
        sl, itc = ladder[c]
        row.append(f"c{c}: tau={u*sl:.2f}s d0={u*itc:.1f}m")
    say(f"  u={u:<8} ({nm:<12}) " + "   ".join(row))

  say("\n== ratios (u-invariant shape of the ladder) ==")
  if 2 in ladder and 3 in ladder:
    say(f"  slope3/slope2 = {ladder[3][0]/ladder[2][0]:.3f}")
  if 3 in ladder and 4 in ladder:
    say(f"  slope4/slope3 = {ladder[4][0]/ladder[3][0]:.3f}")
  if 2 in ladder and 4 in ladder:
    say(f"  slope4/slope2 = {ladder[4][0]/ladder[2][0]:.3f}")

  with open(a.out, "w") as fp:
    fp.write("\n".join(lines) + "\n")


if __name__ == "__main__":
  main()
