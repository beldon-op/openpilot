"""Quantified feasibility of DERIVED vRel: vRel = d(0x109 b7 * u)/dt (risk #4 route 3).

Answers "can't we just compute relative speed?" with numbers, on the 415-seg corpus:
  A. noise floor on steady-follow windows (true vRel ~= 0): raw vs median-filtered
  B. tracking quality on stop-approach windows (true vRel ~= -vEgo): r, slope, lag
  C. same for the 15Hz 12-bit slot-C x (8x finer) where windows are available

Usage:
  .venv/bin/python tools/byd/radar_dvrel_quality.py            # A+B from /tmp/seg1005.pkl
  .venv/bin/python tools/byd/radar_dvrel_quality.py --slotc    # +C from /tmp/slot67.pkl
Caches: /tmp/seg1005.pkl (radar_seg_extract.py), /tmp/slot67.pkl (radar_slot67_extract.py)
"""
import sys, pickle, argparse
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

U = 0.3048  # feet candidate; all m/s numbers scale by 0.82 if u turns out 0.25
FILTERS = ((0.4, "0.4s med"), (1.0, "1.0s med"), (2.0, "2.0s med"))


def resample(ts, vs, td, maxdt=60e6):
  idx = np.clip(np.searchsorted(ts, td), 1, len(ts) - 1)
  pick = np.where(td - ts[idx - 1] <= ts[idx] - td, idx - 1, idx)
  ok = np.abs(ts[pick] - td) <= maxdt
  v = np.full(len(td), np.nan)
  v[ok] = vs[pick[ok]]
  return v


def medfilt(dv, nw):
  sm = np.nanmedian(sliding_window_view(dv, nw), axis=1)
  smf = np.full(len(dv), np.nan)
  h = nw // 2
  smf[h:-h] = sm
  return smf


def rate(t, x, dt_floor=1e-3):
  dt = np.diff(t) / 1e9
  ok = dt > dt_floor
  return np.where(ok, np.diff(x) / np.where(ok, dt, np.nan), np.nan), ok


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--slotc", action="store_true")
  a = ap.parse_args()
  S10 = pickle.load(open("/tmp/seg1005.pkl", "rb"))

  noise_raw, noise_sm = [], {lab: [] for _, lab in FILTERS}
  tr_r, tr_sl, tr_lag = [], [], []
  nseg = nstill = nclosing = 0
  for name, k in S10.items():
    t = k["109_t"].astype(np.float64)
    if len(t) < 500 or not len(k["10a_t"]):
      continue
    b7 = k["109_d"][:, 7].astype(np.float64)
    pres = resample(k["10a_t"].astype(np.float64), ((k["10a_d"][:, 3] >> 5) & 7).astype(float), t)
    v = resample(k["cs_t"].astype(np.float64), k["cs_v"].astype(np.float64), t)
    lead = (pres == 7) & (b7 > 0) & (b7 <= 245) & ~np.isnan(v)
    if lead.sum() < 300:
      continue
    nseg += 1
    dv, ok = rate(t, b7 * U)
    dv *= ok  # keep NaN outside ok
    dv = np.where(ok, dv, np.nan)
    flat = np.convolve(b7, np.ones(25) / 25, mode="same")
    still = lead[:-1] & ok & (np.abs(b7[:-1] - flat[:-1]) <= 0.5) & (v[:-1] > 0.5) & ~np.isnan(dv)
    if still.sum() > 100:
      nstill += 1
      noise_raw.append(float(np.nanstd(dv[still])))
      for w_s, lab in FILTERS:
        smf = medfilt(dv, int(w_s * 25) | 1)
        pts = smf[still]
        if np.sum(~np.isnan(pts)) >= 50:
          noise_sm[lab].append(float(np.nanstd(pts)))
    # B: stop-approach: vRel ~= -vEgo while closing onto a (near-)stopped lead
    closing = lead[:-1] & ok & (dv < -0.5) & (np.abs(v[:-1]) > 1.0)
    smf = medfilt(dv, 25 | 1)
    cl = closing & ~np.isnan(smf) & ~np.isnan(v[:-1])
    if cl.sum() > 150 and np.std(v[:-1][cl]) > 0.3:
      nclosing += 1
      A, B = smf[cl], -v[:-1][cl]
      tr_r.append(np.corrcoef(A, B)[0, 1])
      tr_sl.append(np.polyfit(B, A, 1)[0])
      # lag by peak cross-correlation (in 0.04s frames)
      n = len(A)
      cc = np.correlate(A - A.mean(), B - B.mean(), "full")
      tr_lag.append((np.argmax(cc) - (n - 1)) * 0.04)

  print(f"# corpus: {nseg} segs, {nstill} steady windows")
  print(f"\nA. vRel=d(b7*u)/dt noise on steady following (true vRel~0), u={U}m:")
  print(f"   raw 40ms diff : std med = {np.median(noise_raw):5.2f} m/s (quant floor u/0.04s = {U/0.04:.1f})")
  for _, lab in FILTERS:
    arr = noise_sm[lab]
    print(f"   {lab:11s}: std med = {np.median(arr):5.3f} m/s  (n={len(arr)})")

  # B: stop-to-lead events (radar_stop2lead.py /tmp/radar_stop2lead.json, series=[t_rel, vEgo, b7]);
  #    lead near-stopped -> true vRel(t) = -vEgo(t). Tracking quality of the DERIVED vRel.
  import json, os
  print(f"\nB. stop-to-lead events (true vRel ~ -vEgo), derived vRel = med-diff of b7:")
  print(f"   {'seg':28s} {'win_s':>5} {'raw_r':>6} {'raw_slp':>7} | {'r@0.4s':>6} {'slp@0.4s':>8} {'lag_s':>6} | {'r@1.0s':>6} {'slp@1.0s':>8} {'lag_s':>6}")
  for ev in json.load(open("/tmp/radar_stop2lead.json")):
    S = np.array(ev["series"])
    t, v, b7 = S[:, 0], S[:, 1], S[:, 2]  # t_rel negative up to 0 (stop)
    dt = np.diff(t)
    if (dt <= 0).any() or len(t) < 30:
      continue
    true_vrel = -v
    out = []
    for w_s in (0.0, 0.4, 1.0):
      if w_s == 0:
        dv = np.diff(b7) * U / dt
      else:
        dv0 = np.diff(b7) * U / dt
        nw = max(3, int(w_s / np.median(dt)) | 1)
        sm = np.nanmedian(sliding_window_view(dv0, nw), axis=1)
        f = np.full(len(dv0), np.nan)
        h = nw // 2
        f[h:-h] = sm
        dv = f
      tc = (t[:-1] + t[1:]) / 2
      ok = ~np.isnan(dv)
      # event-window regression vs true
      r = np.corrcoef(dv[ok], true_vrel[:-1][ok])[0, 1]
      sl = np.polyfit(true_vrel[:-1][ok], dv[ok], 1)[0]
      # lag: first crossing of -0.5 m/s, derived vs truth
      def cross(arr, tt, thr=-0.5):
        idx = np.where(arr < thr)[0]
        return tt[idx[0]] if len(idx) else np.nan
      lag = cross(dv, tc) - cross(true_vrel, t) if w_s else np.nan
      out.append((r, sl, lag))
    print(f"   {ev['seg']:28s} {t[0]:5.1f} {out[0][0]:+6.2f} {out[0][1]:7.2f} | "
          f"{out[1][0]:+6.2f} {out[1][1]:8.2f} {out[1][2]:6.2f} | {out[2][0]:+6.2f} {out[2][1]:8.2f} {out[2][2]:6.2f}")

  if a.slotc:
    S67 = pickle.load(open("/tmp/slot67.pkl", "rb"))
    n_sc = n_sc_win = 0
    noise_sc = {lab: [] for _, lab in FILTERS}
    tr_sc_r, tr_sc_sl, tr_sc_lag = [], [], []
    for name in sorted(set(S67) & set(S10)):
      s, k = S67[name], S10[name]
      t380 = s.get("380_t")
      if t380 is None or len(t380) < 500 or not len(k["10a_t"]):
        continue
      t = t380.astype(np.float64)           # native 15Hz 12-bit timeline
      x = (s["380_d"][:, 3].astype(np.float64) % 16) * 256 + s["380_d"][:, 2]
      x = np.where(x > 50, x, np.nan)        # 12-bit counts, 1/8-ft scale each
      b7_380 = resample(k["109_t"].astype(np.float64), k["109_d"][:, 7].astype(np.float64), t)
      pres380 = resample(k["10a_t"].astype(np.float64), ((k["10a_d"][:, 3] >> 5) & 7).astype(float), t)
      v = resample(k["cs_t"].astype(np.float64), k["cs_v"].astype(np.float64), t)
      lock = (pres380 == 7) & (b7_380 > 0) & (b7_380 <= 245) \
             & ~np.isnan(x) & (np.abs(x / 8.0 - b7_380) <= 3.0) & ~np.isnan(v)
      if lock.sum() < 300:
        continue
      n_sc += 1
      # derived vRel from the 12-bit x itself: 1 LSB = U/8 m, 67ms sampling
      dvx, okd = rate(t, x * (U / 8.0))
      dvx = np.where(okd, dvx, np.nan)
      flat = np.convolve(x / 8.0, np.ones(15) / 15, mode="same")  # b7-equiv, +-1s
      still = lock[:-1] & okd & (np.abs(x[:-1] / 8.0 - flat[:-1]) <= 0.5) & (v[:-1] > 0.5) & ~np.isnan(dvx)
      if still.sum() > 100:
        n_sc_win += 1
        for w_s, lab in FILTERS:
          nw = max(3, int(w_s * 15) | 1)
          smf = medfilt(dvx, nw)
          pts = smf[still]
          if np.sum(~np.isnan(pts)) >= 50:
            noise_sc[lab].append(float(np.nanstd(pts)))
      closing = lock[:-1] & okd & (dvx < -0.5) & (np.abs(v[:-1]) > 1.0)
      nw = 7  # ~0.47s median at 15Hz
      smf = medfilt(dvx, nw)
      cl = closing & ~np.isnan(smf) & ~np.isnan(v[:-1])
      if cl.sum() > 150 and np.std(v[:-1][cl]) > 0.3:
        tr_sc_r.append(np.corrcoef(smf[cl], -v[:-1][cl])[0, 1])
        tr_sc_sl.append(np.polyfit(-v[:-1][cl], smf[cl], 1)[0])
        A, B = smf[cl], -v[:-1][cl]
        n = len(A)
        cc = np.correlate(A - A.mean(), B - B.mean(), "full")
        tr_sc_lag.append((np.argmax(cc) - (n - 1)) * 0.067)
    print(f"\nC. slot-C 12-bit x derived vRel (1 LSB = u/8 m, 67ms; slot-1-locked): segs={n_sc} still={n_sc_win}")
    print(f"   raw 67ms diff quant floor = (u/8)/0.067s = {U/8/0.067:.2f} m/s (vs b7's {U/0.04:.1f})")
    for _, lab in FILTERS:
      arr = noise_sc[lab]
      if arr:
        print(f"   {lab:11s}: std med = {np.median(arr):5.3f} m/s  (n={len(arr)})")
    if tr_sc_r:
      print(f"   stop-approach ~0.5s med: r med = {np.median(tr_sc_r):+.3f}  slope med = {np.median(tr_sc_sl):.2f}  lag med = {np.median(np.abs(tr_sc_lag)):.2f}s (n={len(tr_sc_r)})")


if __name__ == "__main__":
  main()
