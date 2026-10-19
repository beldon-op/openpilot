"""Pass 2: hard physics bounds on the '0x109 b7 = meters' hypothesis.

1) decel-window slope vs ego speed: gap-close rate cannot exceed vEgo (lead car
   can't close on us faster than we drive when it is stopped) -> |slope_du/s| > vEgo_m/s
   fraction bounds the unit from above (1 u < ... m).
2) class-transition thresholds: b7 value at the instant HUD bar flips 2<->3, 3<->4,
   per segment - distance thresholds should be speed-invariant.
3) standstill-behind-lead b7 (physical prior: stopped gap ~3-12 m).
4) launch (0->2 m/s with lead): db7 over 6 s; true gap gain ~<=15 m.
5) b6 signed vs db7/dt regression (if b6 = vRel in m/s, slope gives u-per-m).
6) b2 low nibble==0xF share (nibble-invalid structure vs linear byte).
7) cruiseState.active share (is this actually ACC-following footage?).
"""
import sys, os, json
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANParser
from opendbc.car import Bus, CanData
from opendbc.car.byd.values import DBC, CAR

SEC = 1_000_000_000
HUD_MSG = 0x32D
BASE = "/Users/wujiafu/Documents/work/op/sunnypilot/docs_site/hour_logs/radar_rlogs"


def load(seg):
  f = os.path.join(BASE, seg, "rlog.zst")
  hp = CANParser(DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar], [(HUD_MSG, 50)], 2)
  cs_t, cs_v = [], []
  cr_active = []
  hud_t, hud_cls = [], []
  rows = []
  for m in LogReader(f):
    w = m.which()
    if w == "can":
      hf = []
      for c in m.can:
        d = bytes(c.dat)
        if c.src == 1 and c.address == 0x109:
          rows.append((m.logMonoTime, d))
        elif c.src == 2 and c.address == HUD_MSG:
          hf.append(CanData(c.address, d, c.src))
      if hf:
        hp.update([(m.logMonoTime, hf)])
        v = hp.vl[HUD_MSG]
        cls = int(v["HasLead"]) * int(v["LeadingDistance"]) if int(v["HasLead"]) else 0
        hud_t.append(m.logMonoTime); hud_cls.append(cls)
    elif w == "carState":
      cs_t.append(m.logMonoTime); cs_v.append(m.carState.vEgo)
      cr_active.append(bool(m.carState.cruiseState.enabled) and not bool(m.carState.cruiseState.nonAdaptive))
  import bisect
  out = []
  for t, d in rows:
    i = bisect.bisect_left(hud_t, t)
    cls = None
    for j in (i - 1, i):
      if 0 <= j < len(hud_t) and abs(hud_t[j] - t) <= SEC // 2:
        cls = hud_cls[j]; break
    if cls is None:
      continue
    out.append((t, cls, d))
  return out, (np.asarray(cs_t), np.asarray(cs_v)), np.asarray(cr_active)


def interp1(tq, ts, vs):
  idx = np.clip(np.searchsorted(ts, tq), 1, len(ts) - 1)
  t0, t1 = ts[idx - 1], ts[idx]
  x = np.clip((tq - t0) / np.maximum(t1 - t0, 1), 0, 1)
  return vs[idx - 1] + (vs[idx] - vs[idx - 1]) * x


def analyze(seg):
  rows, (cs_t, cs_v), cr_active = load(seg)
  t = np.asarray([r[0] for r in rows]); cls = np.asarray([r[1] for r in rows])
  b = np.asarray([list(r[2]) for r in rows])
  b7 = b[:, 7].astype(float); t_s = t / SEC
  ve = interp1(t, cs_t, cs_v)  # m/s
  R = {"seg": seg, "cruise_active_pct": float(cr_active.mean() * 100)}
  lead = np.isin(cls, [1, 2, 3, 4, 5, 6, 7])
  dt = np.diff(t_s)
  ok = (dt > 0) & (dt < 0.1) & (cls[1:] == cls[:-1]) & (cls[1:] != 0)
  db7 = np.diff(b7)
  slope_all = db7[ok] / dt[ok]

  # (1) decel windows: 2 s windows with ego slowing >=3 kph; |slope| vs ego m/s
  step = max(5, int(round(2.0 / np.median(dt)))) if len(dt) else 100
  ve_k = ve * 3.6
  bounds = []
  for i in range(0, len(t_s) - step, max(1, step // 2)):
    if ve_k[i] - ve_k[i + step] >= 3.0 and cls[i] != 0:
      sl = np.polyfit(t_s[i:i + step], b7[i:i + step], 1)[0]
      bounds.append((sl, float(np.mean(ve[i:i + step]))))
  if bounds:
    sl = np.asarray([x[0] for x in bounds]); vm = np.asarray([x[1] for x in bounds])
    close = -sl  # u/s closing
    R["decel_unit_bound"] = {
      "n": len(bounds),
      "frac_close_gt_vEgo": float((close > vm).mean() * 100),
      "median_close_u_s": float(np.median(close)),
      "median_vEgo_mps": float(np.median(vm)),
      "implied_unit_max_m": float(np.median(vm) / np.median(close)) if np.median(close) > 0 else None,
      # if closing_u_s must be <= vEgo_mps, unit u_m satisfies close*u_m<=vEgo
    }

  # (2) transition thresholds 2<->3, 3<->4
  thr = {}
  for a, c in ((2, 3), (3, 4)):
    for up, dn in ((a, c), (c, a)):
      idx = np.where((cls[:-1] == up) & (cls[1:] == dn))[0]
      vals = []
      for i in idx[:40]:
        # first frame of new class
        vals.append(float(b7[i + 1]))
      if vals:
        thr[f"{up}->{dn}"] = {"n": len(vals), "median": float(np.median(vals)),
                              "p25": float(np.percentile(vals, 25)), "p75": float(np.percentile(vals, 75))}
  R["transitions"] = thr

  # (3) standstill behind lead
  msk = lead & (ve < 0.5)
  if msk.sum():
    R["standstill_lead"] = {"n": int(msk.sum()), "b7_mean": float(b7[msk].mean()),
                            "b7_median": float(np.median(b7[msk])), "b7_max": float(b7[msk].max())}

  # (4) launch events: vEgo crosses 0.3->2.0 while lead present, b7 gain over 6 s
  launches = []
  in_lead = lead
  i = 0
  while i < len(ve):
    if in_lead[i] and ve[i] < 0.3:
      j = i + 1
      while j < len(ve) and in_lead[j] and ve[j] < 1.5 and t_s[j] - t_s[i] < 6.0:
        j += 1
      if j < len(ve) and in_lead[j] and ve[j] >= 1.5:
        k = np.searchsorted(t_s, t_s[i] + 6.0)
        if k < len(b7) and in_lead[k]:
          launches.append((float(b7[i]), float(b7[k] - b7[i])))
        i = j + 1
        continue
    i += 1
  if launches:
    st = np.asarray(launches)
    R["launch"] = {"n": len(launches), "b7_at_start_median": float(np.median(st[:, 0])),
                   "gain_6s_median_u": float(np.median(st[:, 1])), "gain_6s_p90": float(np.percentile(st[:, 1], 90))}

  # (5) b6 signed vs db7/dt (regression) over consecutive lead frames
  b6 = b[:, 6].astype(int); s6 = ((b6 + 128) % 256) - 128
  if ok.sum() > 50:
    y = slope_all
    x = s6[1:][ok].astype(float)
    if x.std() > 0.5 and y.std() > 0:
      a1, b0v = np.polyfit(x, y, 1)
      r = np.corrcoef(x, y)[0, 1]
      R["b6_vs_db7dt"] = {"slope_u_s_per_ms": float(a1), "r": float(r),
                          "note": "expect slope approx -1 (u/s per m/s closing) if b7 meters & b6 signed m/s vRel"}
    else:
      R["b6_vs_db7dt"] = {"degenerate": True, "b6_std": float(x.std()), "slope_std": float(y.std())}

  # (6) b2 nibble structure
  b2 = b[:, 2].astype(int)
  R["b2_low_nibF_lead"] = float(((b2[lead] & 0xF) == 0xF).mean() * 100)
  R["b2_low_nibF_nolead"] = float(((b2[~lead] & 0xF) == 0xF).mean() * 100) if (~lead).any() else None

  # (7) b7 value inventory on lead frames: small-gap support
  bl = b7[lead]
  R["b7_lead_frac_lt13"] = float((bl < 13).mean() * 100)
  R["b7_lead_frac_247"] = float((bl == 247).mean() * 100)
  R["b7_all247_frac_nolead"] = float((b7[~lead] == 247).mean() * 100) if (~lead).any() else None
  return R


if __name__ == "__main__":
  res = {}
  for s in sys.argv[1:] or ["f4_16", "fa_2"]:
    try:
      res[s] = analyze(s)
    except Exception:
      import traceback; traceback.print_exc()
  print(json.dumps(res, indent=1))
