"""b3 phase-2: the decisive lane-change regression (yRel or not).

Phase 1 (radar_b3_mine) showed b3 leaves its 0x7F idle center for ~75-90% of
lead frames, moves slower than everything, and re-centers after target
appearances - all yRel-consistent but nothing disambiguates "slow lateral
position" from "slow bias".

Phase 2 asks the geometry question: when the EGO moves laterally (a lane
change), a target it keeps tracking (same-lane lead left behind, or an
adjacent-lane car that becomes the new lead) MUST show a lateral offset sweep
in the ego frame of ~ +-lane width if b3 is yRel, and must stay flat if b3 is
a constant bias / something else.

Lateral displacement is double-integrated from steeringAngleDeg with the
vendor-recorded car specs (steerRatio 19.5, wheelbase 2.765, values.py line
0037) - yawRate is 0 in these logs, angle is valid.

Event extraction per segment:
  - y_lat(t) from the steer model, smoothed; lane-change candidate = a run
    where |y_lat| reaches >= LAT_MIN m within <= LAT_WINDOW s with a plateau
    after the sweep (crossing + settling).
  - require a target CONTINUOUSLY visible (presence 7 & 1<=b7<=245) for the
    whole window +- 1 s and vEgo > 3.
  - regress b3 (signed off 127) on y_lat over the sweep: slope LSB/m, R^2,
    endpoint delta. A real yRel gives one clean slope whose SIGN flips with
    the direction of the lateral move (left vs right).

Usage:
  .venv/bin/python tools/byd/radar_b3_lat.py extract  # -> /tmp/seg1005_steer.pkl (background)
  .venv/bin/python tools/byd/radar_b3_lat.py analyze  # reports the regressions
"""
import sys, os, glob, pickle, argparse, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
import radar_seglib as L

SEC = 1_000_000_000
STEER_RATIO = 19.5     # vendor live carParams (values.py comment)
WHEELBASE = 2.765      # BYDCarSpecs
LAT_MIN = 2.0          # m, a real lane crossing
LAT_WINDOW = 10.0      # s, max sweep duration
V_MIN = 3.0
PKL = "/tmp/seg1005_steer.pkl"


def _one(f):
  from openpilot.tools.lib.logreader import LogReader
  t, v, a, bl = [], [], [], []
  try:
    for m in LogReader(f):
      if m.which() == "carState":
        c = m.carState
        t.append(m.logMonoTime / SEC)
        v.append(c.vEgo)
        a.append(c.steeringAngleDeg)
        bl.append(1 if (c.leftBlinker or c.rightBlinker) else 0)
  except Exception:
    pass
  return os.path.basename(os.path.dirname(f)), (np.asarray(t, np.float64),
    np.asarray(v, np.float32), np.asarray(a, np.float32), np.asarray(bl, np.int8))


def extract_main():
  from multiprocessing import Pool
  segs = sorted(glob.glob("docs_site/byd_logs_2026-10-05_07/realdata/*/rlog.zst"))
  print(f"# {len(segs)} segments", flush=True)
  with Pool(10) as p:
    res = p.map(_one, segs, chunksize=2)
  pickle.dump(dict(res), open(PKL, "wb"), protocol=4)
  print(f"# cached -> {PKL}", flush=True)


def lat_displacement(t, v, ang):
  beta = np.deg2rad(ang / STEER_RATIO)
  yaw = v * np.tan(beta) / WHEELBASE
  heading = np.concatenate([[0.0], np.cumsum(np.diff(t) * yaw[1:])])
  y = np.concatenate([[0.0], np.cumsum(np.diff(t) * v[1:] * np.sin(heading[1:]))])
  # 1 s moving-average to kill sensor noise
  k = max(1, int(1.0 / np.median(np.diff(t))))
  kern = np.ones(k) / k
  return np.convolve(y, kern, mode="same")


def analyze_main(a):
  cache = L.load(a.pkl)
  steer = pickle.load(open(PKL, "rb"))
  events = []
  for name, s in cache.items():
    if name not in steer:
      continue
    st_t, sv, sang, sbl = steer[name]
    if len(st_t) < 300:
      continue
    ylat = lat_displacement(st_t, sv.astype(np.float64), sang.astype(np.float64))
    t9, d9 = s["109_t"] / SEC, s["109_d"]
    b7 = d9[:, 7].astype(np.int32); b3 = d9[:, 3].astype(np.int32) - 127
    pr = L.pres_at(s["109_t"], s)
    vis = (pr == 7) & (b7 >= 1) & (b7 <= 245)
    v9 = np.interp(t9, st_t, sv.astype(np.float64))
    # sweep detection on the 0x109 time base
    y9 = np.interp(t9, st_t, ylat)
    dt_med = float(np.median(np.diff(t9))) if len(t9) > 2 else 0.04
    step_fwd = int(LAT_WINDOW / dt_med)
    i = 0
    n = len(t9)
    while i < n:
      # rising edge: |y| passes 0.6 m and reaches LAT_MIN within the window
      if v9[i] > V_MIN and 0.6 <= abs(y9[i]) < LAT_MIN:
        j = i
        while j < n and j - i < step_fwd and abs(y9[j]) < LAT_MIN:
          j += 1
        if j >= n or j - i >= step_fwd or abs(y9[i]) >= LAT_MIN or np.sign(y9[j]) != np.sign(y9[i]):
          i = max(i + 1, j); continue
        # backtrack sweep start to last |y| <= 0.6 (cap 12 s)
        k = i
        while k > 0 and abs(y9[k]) > 0.6 and (t9[i] - t9[k]) < 12.0:
          k -= 1
        s0 = k
        # forward end: motion settles (<=0.15 m/s for 1 s) or +LAT_WINDOW
        e = j
        while e < n - 25 and (t9[e] - t9[j]) < LAT_WINDOW:
          if np.abs(np.interp(t9[e] + 1.0, t9, y9) - y9[e]) < 0.15:
            break
          e += 1
        e0 = min(n - 1, e + 25)
        win = slice(s0, e0)
        x = y9[win]; yy = b3[win].astype(np.float64)
        msk = vis[win] & (v9[win] > V_MIN) & np.isfinite(x)
        if (msk.sum() > 60 and vis[win].mean() > 0.85 and np.nanstd(v9[win]) < 4.0
            and abs(float(x[msk][-1] - x[msk][0])) >= LAT_MIN * 0.9):
          xd = float(x[msk][-1] - x[msk][0])
          yd = float(yy[msk][-1] - yy[msk][0])
          xm, ym = float(x[msk].mean()), float(yy[msk].mean())
          sxy = float(np.dot(x[msk] - xm, yy[msk] - ym))
          sxx = float(np.dot(x[msk] - xm, x[msk] - xm))
          slope = sxy / sxx if sxx > 1e-6 else float("nan")
          yhat = slope * (x[msk] - xm) + ym
          resid = float(np.dot(yy[msk] - yhat, yy[msk] - yhat))
          ss = float(np.dot(yy[msk] - ym, yy[msk] - ym))
          r2 = 1 - resid / ss if ss > 1.0 else 0.0
          events.append({"seg": name, "t": round(float(t9[s0]), 1), "dy": round(xd, 2),
                         "db3": round(yd, 1), "slope": round(float(slope), 1),
                         "r2": round(float(r2), 3),
                         "blink": int(sbl[int(np.searchsorted(st_t, t9[s0]))]),
                         "n": int(msk.sum())})
        i = e0
      else:
        i += 5
  json.dump(events, open(a.json, "w"))
  ev = [e for e in events if e["n"] > 100 and abs(e["dy"]) >= LAT_MIN]
  print(f"# events: {len(events)} kept: {len(ev)}", flush=True)
  def rep(sub, label):
    if not sub:
      print(f"  {label}: none"); return
    sl = sorted(e["slope"] for e in sub)
    r2 = sorted(e["r2"] for e in sub)
    n = len(sub)
    print(f"  {label}: n={n} slope med={sl[n//2]:+.1f} LSB/m "
          f"[p25={sl[n//4]:+.1f},p75={sl[3*n//4]:+.1f}] "
          f"r2 med={r2[n//2]:.2f} r2>0.5: {sum(1 for e in sub if e['r2'] > 0.5)}/{n}")
  rep(ev, "all")
  rep([e for e in ev if e["dy"] > 0], "dy>0 (right)")
  rep([e for e in ev if e["dy"] < 0], "dy<0 (left)")
  good = [e for e in ev if e["r2"] > 0.5]
  rep(good, "good-fit subset")
  for e in sorted(good, key=lambda z: -abs(z["slope"]))[:15]:
    print("   ", e)


if __name__ == "__main__":
  ap = argparse.ArgumentParser()
  ap.add_argument("mode", choices=["extract", "analyze"])
  ap.add_argument("--pkl", default="/tmp/seg1005.pkl")
  ap.add_argument("--json", default="/tmp/radar_b3_lat.json")
  a = ap.parse_args()
  if a.mode == "extract":
    extract_main()
  else:
    analyze_main(a)
