"""Unit/physical-consistency audit for the claim '0x109 b7 = forward distance in meters'.

For each rlog segment: ego speed stats (carState.vEgo), per-HUD-bar-class b7
distribution (quantiles), 0x109 frame rate, frame-to-frame |db7| -> implied
closing speed if 1 unit = 1 m, deceleration-window b7 trends, implied headway
(b7 / vEgo) per class, standstill b7, and signed readings of 0x109 b2/b3/b6.

Usage: .venv/bin/python tools/byd/radar_unit_check.py [seg ...]
"""
import sys, os, json, bisect, statistics as st
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANParser
from opendbc.car import Bus, CanData
from opendbc.car.byd.values import DBC, CAR
from opendbc.car.byd.values import RADAR_HUD_BARS_TO_M

SEC = 1_000_000_000
HUD_MSG = 0x32D
BASE = "/Users/wujiafu/Documents/work/op/sunnypilot/docs_site/hour_logs/radar_rlogs"


def q(a):
  a = np.asarray(a, dtype=float)
  if a.size == 0:
    return None
  return {p: float(np.percentile(a, p)) for p in (5, 25, 50, 75, 90, 95, 99)}


def load(seg):
  f = os.path.join(BASE, seg, "rlog.zst")
  hp = CANParser(DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar], [(HUD_MSG, 50)], 2)
  cs_t, cs_v, cs_a = [], [], []
  hud_t, hud_cls = [], []
  rows109 = []   # (t_ns, class, b0..b7 as tuple)
  rows101 = {}   # t -> (b7, bytes)
  t10a = []
  for m in LogReader(f):
    w = m.which()
    if w == "can":
      hf = []
      for c in m.can:
        d = bytes(c.dat)
        if c.src == 1 and 0x101 <= c.address <= 0x10A:
          if c.address == 0x109:
            rows109.append((m.logMonoTime, d))
          elif c.address == 0x101:
            rows101[m.logMonoTime] = d
          elif c.address == 0x10A:
            t10a.append(m.logMonoTime)
        elif c.src == 2 and c.address == HUD_MSG:
          hf.append(CanData(c.address, d, c.src))
      if hf:
        hp.update([(m.logMonoTime, hf)])
        v = hp.vl[HUD_MSG]
        cls = int(v["HasLead"]) * int(v["LeadingDistance"]) if int(v["HasLead"]) else 0
        hud_t.append(m.logMonoTime)
        hud_cls.append(cls)
    elif w == "carState":
      cs_t.append(m.logMonoTime)
      cs_v.append(m.carState.vEgo)
      try:
        cs_a.append(m.carState.aAccel)
      except AttributeError:
        cs_a.append(0.0)
  # assign HUD class to each 0x109 frame (same +/-0.5 s rule as radar_0109_validate)
  out = []
  for t, d in rows109:
    i = bisect.bisect_left(hud_t, t)
    cls = None
    for j in (i - 1, i):
      if 0 <= j < len(hud_t) and abs(hud_t[j] - t) <= SEC // 2:
        cls = hud_cls[j]
        break
    if cls is None:
      continue
    out.append((t, cls, d))
  return out, (cs_t, cs_v, cs_a), t10a, hud_cls, rows101


def interp(t_ns, ts, vs):
  # nearest-window linear interp of a carState series (monotonic ts) onto t_ns
  ts = np.asarray(ts)
  vs = np.asarray(vs)
  idx = np.searchsorted(ts, np.asarray(t_ns))
  idx = np.clip(idx, 1, len(ts) - 1)
  t0 = ts[idx - 1] / SEC; t1 = ts[idx] / SEC; tn = np.asarray(t_ns) / SEC
  v0 = np.asarray(vs)[idx - 1]; v1 = np.asarray(vs)[idx]
  span = np.maximum(t1 - t0, 1e-6)
  x = np.clip((tn - t0) / span, 0, 1)
  return v0 + (v1 - v0) * x


def analyze(seg):
  rows, (cs_t, cs_v, cs_a), t10a, hud_cls, rows101 = load(seg)
  R = {"seg": seg}
  v = np.asarray(cs_v); a = np.asarray(cs_a)
  R["dur_s"] = float((cs_t[-1] - cs_t[0]) / SEC)
  R["vEgo_kph"] = {"mean": float(v.mean() * 3.6), "median": float(np.median(v) * 3.6),
                   "p05": float(np.percentile(v, 5) * 3.6), "p95": float(np.percentile(v, 95) * 3.6),
                   "max": float(v.max() * 3.6)}
  R["cls_dist"] = {int(c): hud_cls.count(c) for c in set(hud_cls)}

  t = np.asarray([r[0] for r in rows])
  cls = np.asarray([r[1] for r in rows])
  b = np.asarray([list(r[2]) for r in rows])  # n x 8
  R["n109"] = int(len(t))
  if len(t) > 1:
    dt = np.diff(np.sort(t)) / SEC
    dt = dt[dt > 0]
    R["rate_hz_median"] = float(1 / np.median(dt))
  # 0x10A cadence & phase vs 0x109
  if len(t10a) > 1 and len(t) > 1:
    dta = np.diff(np.sort(np.asarray(t10a))) / SEC; dta = dta[dta > 0]
    R["10a_rate_hz_median"] = float(1 / np.median(dta))

  lead = np.isin(cls, [1, 2, 3, 4, 5, 6, 7])
  ve = interp(t, cs_t, cs_v) if len(t) else np.array([])

  # per-class b7 distribution
  R["b7"] = {}
  for c in (0, 2, 3, 4):
    v7 = b[cls == c, 7]
    if v7.size < 30:
      continue
    R["b7"][c] = {"n": int(v7.size), "mean": float(v7.mean()),
                  "max": int(v7.max()), "min": int(v7.min()),
                  "q": q(v7), "pct255": float((v7 == 255).mean() * 100),
                  "pct247": float((v7 == 247).mean() * 100)}

  # band-overlap test: threshold partition of distance would give non-overlapping bands
  if all(c in R["b7"] for c in (2, 3, 4)):
    R["bands"] = {"c2_p95": R["b7"][2]["q"][95], "c2_max": R["b7"][2]["max"],
                  "c3_p95": R["b7"][3]["q"][95], "c3_max": R["b7"][3]["max"],
                  "c3_p05": R["b7"][3]["q"][5], "c4_p05": R["b7"][4]["q"][5],
                  "c4_p25": R["b7"][4]["q"][25], "c3_p25": R["b7"][3]["q"][25]}

  # frame-to-frame |db7| -> implied m/s if 1u = 1m (consecutive same-class frames)
  o = np.argsort(t); ts, cs_, b7s = t[o], cls[o], b[o, 7]
  dts = np.diff(ts) / SEC
  adj = (cs_[1:] == cs_[:-1]) & (cs_[1:] != 0) & (dts > 0) & (dts < 0.1)
  db = np.abs(np.diff(b7s))[adj]
  if db.size:
    dtv = dts[adj]
    R["db7"] = {"n": int(db.size), "mean_abs_du": float(db.mean()),
                "p95_abs_du": float(np.percentile(db, 95)),
                "implied_mps_p95": float(np.percentile(db / dtv, 95)),
                "implied_mps_p99": float(np.percentile(db / dtv, 99)),
                "vEgo_mps_mean": float(ve[lead].mean()) if lead.any() else None}

  # implied headway if meters: b7 / vEgo, per class (lead, moving)
  R["headway_s"] = {}
  for c in (2, 3, 4):
    msk = (cls == c) & (ve > 3.0)
    if msk.sum() > 30:
      hw = b[msk, 7] / ve[msk]
      R["headway_s"][c] = {"mean": float(hw.mean()), "median": float(np.median(hw)),
                           "in_1_2p5_pct": float(((hw >= 1.0) & (hw <= 2.5)).mean() * 100)}

  # standstill behind lead: distance should be small (5-15 m typical)
  msk = lead & (ve < 0.3)
  if msk.sum() > 20:
    R["standstill_b7"] = {"n": int(msk.sum()), "mean": float(b[msk, 7].mean()),
                          "median": float(np.median(b[msk, 7])), "p90": float(np.percentile(b[msk, 7], 90))}

  # unit-fit vs project's own bar->meter midpoints, under scale u->m
  fit = {}
  for c in (2, 3, 4):
    if c in R["b7"]:
      fit[c] = R["b7"][c]["mean"] / RADAR_HUD_BARS_TO_M[c]
  R["mean_over_bar_midpoint"] = fit

  # (b) deceleration windows: 2 s sliding with dv <= -3 kph, lead present
  ve_kph = ve * 3.6
  step = max(5, int(round(2.0 / np.median(dts)))) if len(t) > 1 and np.median(dts) > 0 else 100  # 2 s window in frames
  slopes = []
  n100 = len(ts)
  if n100 > 2 * step:
    dec_rate = ve_kph[:-step] - ve_kph[step:]   # positive when slowing over 2 s
    for i in range(0, n100 - step, step // 2):
      if dec_rate[i] >= 3.0 and cs_[i] != 0:  # braking window, lead present
        sl_b = np.polyfit(ts[i:i + step] / SEC, b7s[i:i + step], 1)[0]
        slopes.append((float(dec_rate[i]), float(sl_b), float(ve_kph[i])))
  if slopes:
    dr = np.asarray([s[0] for s in slopes]); sl = np.asarray([s[1] for s in slopes])
    R["decel_windows"] = {"n": len(slopes),
                          "frac_b7_down": float((sl < 0).mean() * 100),
                          "frac_b7_down_gt5u_s": float((sl < -5).mean() * 100),
                          "frac_b7_up_gt5u_s": float((sl > 5).mean() * 100),
                          "median_slope_du_s": float(np.median(sl)),
                          "corr_dbraking_slope": float(np.corrcoef(dr, sl)[0, 1]) if dr.std() > 0 and sl.std() > 0 else None}

  # level correlation b7 vs vEgo across lead frames
  if lead.sum() > 100 and ve[lead].std() > 0.5:
    R["corr_b7_vEgo_leadframes"] = float(np.corrcoef(b[lead, 7], ve[lead])[0, 1])

  # (d) signed-byte readings for 0x109 b2/b3/b6
  R["signed"] = {}
  for bi in (0, 2, 3, 6):
    col = b[:, bi].astype(int)
    s128 = ((col + 128) % 256) - 128  # two's complement
    med = {c: {"med_raw": int(np.median(col[cls == c])), "med_signed": int(np.median(s128[cls == c])),
               "mode_raw": int(np.bincount(col[cls == c], minlength=256).argmax())}
           for c in (0, 2, 3, 4) if (cls == c).sum() > 30}
    # smoothness: mean |d/dt of this byte| (du/s) on lead frames, consecutive
    col_s = col[o]
    adj_b = (cs_[1:] != 0) & (dts > 0) & (dts < 0.1)
    rate = np.abs(np.diff(col_s))[adj_b] / dts[adj_b] if adj_b.any() else np.array([0.0])
    R["signed"][bi] = {"per_class": med, "mean_abs_du_s": float(np.mean(rate)),
                       "frac_consecutive_equal": float((np.diff(col_s) == 0).mean())}

  # 0x101 vs 0x109 b7 agreement in this seg + 0x101 b7 class0 behavior
  eq = tot = 0
  sorted101 = sorted(rows101)
  for tt, clsx, d in rows:
    # nearest 0x101 within 20 ms
    i = bisect.bisect_left(sorted101, tt)
    for j in (i - 1, i):
      if 0 <= j < len(sorted101) and abs(sorted101[j] - tt) <= 0.02 * SEC:
        tot += 1
        eq += int(rows101[sorted101[j]][7] == d[7])
        break
  R["101_vs_109_b7_equal_pct"] = float(eq / tot * 100) if tot else None
  b7_101 = np.asarray([d[7] for d in rows101.values()])
  R["101_b7_global"] = {"n": int(b7_101.size), "mean": float(b7_101.mean())}
  return R


if __name__ == "__main__":
  segs = sys.argv[1:] or ["f4_16", "fa_2", "f8_8", "f9_12"]
  allR = []
  for s in segs:
    try:
      allR.append(analyze(s))
    except Exception as e:
      import traceback; traceback.print_exc()
  with open("/tmp/radar_unit_check.json", "w") as fh:
    json.dump(allR, fh, indent=1)
  for R in allR:
    print("=" * 70)
    print(json.dumps(R, indent=1, default=str))
