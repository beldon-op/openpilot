"""Pin the unit u of 0x109 b7 (meters per count) via stationary-approach ratio windows.

Method A (primary): find windows where b7 decreases monotonically (>2 s, holds<=5%,
per-frame step<=3) while vEgo mean>2 m/s, |dv/dt|<1.5 m/s2, 0x10A presence bit=1.
Per window slope c=|db7/dt| (counts/s) and u_hat = median(vEgo)/c: closing a
stationary lead converges the gap at vEgo. Note v_lead>=0 on any decreasing window
=> u <= vEgo_i/c_i holds per frame (hard upper bound), so the lower tail of u_hat
bounds u from above regardless of lead motion.
Method B: at each HUD bar flip take (b7, vEgo) -> regression slope = tau_band/u.
Validation (independent set hour_logs_2): per-segment cross-class monotonicity of
b7, presence-vs-lead rates, class-0 zero rate.

Usage:
  .venv/bin/python tools/byd/radar_unit_pin.py <dir-glob> [--pattern '*/rlog.zst']
      [--workers 10] --json /tmp/out.json [--quiet]
"""
import sys, os, glob, json, argparse, bisect
from multiprocessing import Pool
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANParser
from opendbc.car import Bus, CanData
from opendbc.car.byd.values import DBC, CAR

SEC = 1_000_000_000
HUD_MSG = 0x32D
MAX_DT = 0.075          # 0x109 cadence 25 Hz; gap-drop => break the run
MAX_STEP = 3            # counts per frame
HOLD_FRAC = 0.05
MIN_DUR = 2.0           # s
VE_MEAN = 2.0           # m/s
ACC_MAX = 1.5           # m/s^2
PRES_FRAC = 0.95        # 0x10A b3[7:5]==0b111 share of window frames


def seg_work(args):
  f, collect_crc = args
  hp = CANParser(DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar], [(HUD_MSG, 50)], 2)
  t9, b9 = [], []          # 0x109 frames
  tp, bp3 = [], []         # 0x10A b3
  ht, hc = [], []          # HUD class series
  cst, csv_ = [], []       # carState
  crc_rows = []
  for m in LogReader(f):
    w = m.which()
    if w == "can":
      hf = []
      for c in m.can:
        if c.src != 1:
          if c.src == 2 and c.address == HUD_MSG:
            hf.append(CanData(c.address, bytes(c.dat), c.src))
          continue
        d = bytes(c.dat)
        if c.address == 0x109:
          t9.append(m.logMonoTime); b9.append(d[7])
          if collect_crc and len(crc_rows) < 20000:
            crc_rows.append((m.logMonoTime, d.hex()))
        elif c.address == 0x10A:
          tp.append(m.logMonoTime); bp3.append(d[3])
      if hf:
        hp.update([(m.logMonoTime, hf)])
        v = hp.vl[HUD_MSG]
        ht.append(m.logMonoTime)
        hc.append(int(v["HasLead"]) * int(v["LeadingDistance"]) if int(v["HasLead"]) else 0)
    elif w == "carState":
      cst.append(m.logMonoTime); csv_.append(m.carState.vEgo)
  seg = os.path.basename(os.path.dirname(f)) if os.path.basename(f) == "rlog.zst" else os.path.basename(f)
  if len(t9) < 60 or len(cst) < 100:
    return {"seg": seg, "windows": [], "flips": [], "cls_stats": {}, "pres": [0, 0, 0, 0], "n109": len(t9)}

  t = np.asarray(t9, float) / SEC
  b7 = np.asarray(b9, float)
  tp = np.asarray(tp, float) / SEC
  pres = ((np.asarray(bp3) >> 5) & 7) == 7
  cs_t = np.asarray(cst, float) / SEC
  cs_v = np.asarray(csv_, float)
  ve = np.interp(t, cs_t, cs_v)
  pr = np.interp(t, tp, pres.astype(float)) if len(tp) else np.zeros_like(t)
  # HUD class per 0x109 frame (nearest within 0.5 s) for validation stats
  hi = np.searchsorted(np.asarray(ht, float) / SEC, t)
  cls = np.zeros(len(t), int)
  hcv = np.asarray(hc, int)
  hts = np.asarray(ht, float) / SEC
  for i in range(len(t)):
    j0, j1 = hi[i] - 1, hi[i]
    best = None
    for j in (j0, j1):
      if 0 <= j < len(hts):
        d = abs(hts[j] - t[i])
        if d <= 0.5 and (best is None or d < best[0]):
          best = (d, hcv[j])
    cls[i] = best[1] if best else -1

  # ---- validation aggregates (0x109 b7 by HUD class) + presence rates
  cs_stats = {}
  for c in (0, 2, 3, 4):
    msk = cls == c
    n = int(msk.sum())
    if n:
      v = b7[msk]
      cs_stats[c] = [n, float(v.sum()), int((v == 0).sum()), int(v.max())]
  lead_m = np.isin(cls, [1, 2, 3, 4, 5, 6, 7])
  nolead_m = cls == 0
  pres = [float(pr[lead_m].sum()) if lead_m.any() else 0.0, int(lead_m.sum()),
          float(pr[nolead_m].sum()) if nolead_m.any() else 0.0, int(nolead_m.sum())]

  # ---- run detection (maximal non-increasing runs, step in [-3, 0])
  dt = np.diff(t)
  db = np.diff(b7)
  ok_step = (dt > 0) & (dt <= MAX_DT) & (db >= -MAX_STEP) & (db <= 0) & (b7[1:] >= 1) & (b7[:-1] <= 246) & (b7[1:] <= 246)
  breaks = np.where(~ok_step)[0]
  starts = np.concatenate(([0], breaks + 1))
  ends = np.concatenate((breaks, [len(dt)]))  # run = frames [start .. end] inclusive
  windows = []
  for s, e in zip(starts, ends):
    rs, re_ = s, e + 1
    n = re_ - rs
    if n < 20:
      continue
    # trim trailing stop-plateau (gap frozen while ego came to a stop): q walks back
    q = re_ - 1
    while q > rs + 10 and abs(float(b7[q]) - float(b7[re_ - 1])) <= 1:
      q -= 1
    plat = re_ - 1 - q
    stop_hold = int(plat >= 25 and float(ve[re_ - 1]) < 1.0 and pr[q + 1:re_].mean() >= 0.9)
    we = q + 1 if stop_hold else re_
    dur = t[we - 1] - t[rs]
    if dur < MIN_DUR:
      continue
    holdfrac = float((db[rs:we - 1] == 0).mean()) if we - 1 > rs else 1.0
    tier = "A" if holdfrac <= HOLD_FRAC else "B"
    vm = ve[rs:we]
    if vm.mean() <= VE_MEAN:
      continue
    if pr[rs:we].mean() < PRES_FRAC:
      continue
    if b7[rs:we].min() < 1 or b7[rs:we].max() > 246:
      continue
    drop = b7[rs] - b7[we - 1]
    if tier == "A":
      # spec gate: |dv/dt| smooth < 1.5 m/s2 (0.5 s moving average on carState grid)
      i0, i1 = np.searchsorted(cs_t, [t[rs], t[we - 1]])
      seg_v, seg_t = cs_v[max(i0 - 1, 0):i1 + 1], cs_t[max(i0 - 1, 0):i1 + 1]
      if len(seg_v) > 10:
        acc = np.gradient(seg_v, seg_t)
        k = max(3, int(0.5 / max(np.median(np.diff(seg_t)), 1e-6)))
        accs = np.convolve(acc, np.ones(k) / k, "valid") if len(acc) >= k else acc
        if len(accs) and np.abs(accs).max() >= ACC_MAX:
          tier = "B"  # fails the strict-accel gate, demote
    if tier == "B" and (drop < 25 or holdfrac > 0.85):
      continue
    if drop <= 0:
      continue
    w_t = t[rs:we] - t[rs]
    slope = float(np.polyfit(w_t, b7[rs:we], 1)[0])
    cbar = -slope
    if cbar <= 0:
      continue
    # smooth closing rate c_i (1 s centered) vs vEgo_i: stationary lead => c = ve/u
    kk = 12
    cs_, vs_ = [], []
    for i in range(max(rs, kk), min(we, len(t) - kk)):
      c = (b7[i - kk] - b7[i + kk]) / (t[i + kk] - t[i - kk])
      if c > 0:
        cs_.append(c); vs_.append(ve[i])
    cs_ = np.asarray(cs_); vs_ = np.asarray(vs_)
    r_cve = float(np.corrcoef(cs_, vs_)[0, 1]) if len(cs_) > 20 and cs_.std() > 0 and vs_.std() > 0 else float("nan")
    # hard per-frame upper bound u <= vEgo/c  (centered diff, k=5 -> 0.4 s)
    k = 5
    u_max = float("inf")
    ratios = []
    for i in range(rs + k, min(we - k, len(t) - k)):
      c = (b7[i - k] - b7[i + k]) / (t[i + k] - t[i - k])
      if c > 0.2:
        u_max = min(u_max, ve[i] / c)
        if c > 0.5 * cbar:
          ratios.append(ve[i] / c)
    u_ratio_med = float(np.median(ratios)) if len(ratios) > 10 else float("nan")
    # integral estimator: u = dist_traveled / gap_closed  (>= u, equality if lead stationary)
    i0, i1 = np.searchsorted(cs_t, [t[rs], t[we - 1]])
    dist = float(np.trapezoid(cs_v[max(i0 - 1, 0):i1 + 1], cs_t[max(i0 - 1, 0):i1 + 1]))
    u_int = dist / drop
    # within-window regression c = a*v + b (slope = 1/u if lead stationary)
    a_v, b_v = (float("nan"), float("nan"))
    if len(cs_) > 20 and vs_.std() > 0.25:
      a_v, b_v = np.polyfit(vs_, cs_, 1)
    windows.append({"seg": seg, "t0": float(t[rs]), "dur": float(dur), "n": int(n), "tier": tier,
                    "b0": float(b7[rs]), "b1": float(b7[we - 1]), "drop": float(drop), "hold": round(holdfrac, 3),
                    "slope_ct_s": cbar, "ve_mean": float(vm.mean()), "ve_med": float(np.median(vm)),
                    "ve_p95": float(np.percentile(vm, 95)), "ve_p05": float(np.percentile(vm, 5)),
                    "u_hat": float(np.median(vm) / cbar), "u_max": float(u_max), "u_int": u_int,
                    "u_rmed": u_ratio_med,
                    "r_cve": r_cve, "a_v": float(a_v), "b_v": float(b_v), "stop_hold": stop_hold,
                    "b_gap": float(b7[re_ - 1]) if stop_hold else float("nan"),
                    "ve_end": float(vm[-1]), "pres": float(pr[rs:we].mean()), "cls": int(np.median(cls[rs:we]))})
  # keep the most significant windows per segment (largest drop first)
  if len(windows) > 200:
    windows = sorted(windows, key=lambda w: -w["drop"])[:200]

  # ---- Method B: HUD bar flip pairs (need HUD t series + b7/ve series)
  flips = []
  if len(hts) > 30:
    hcls = hcv
    ch = np.where(np.diff(hcls) != 0)[0]
    used = set()
    for i in ch:
      c0, c1 = int(hcls[i]), int(hcls[i + 1])
      if c0 not in (1, 2, 3, 4) or c1 not in (1, 2, 3, 4):
        continue
      pair = (min(c0, c1), max(c0, c1))
      tf = hts[i + 1]
      j = int(np.argmin(np.abs(t - tf))) if len(t) else -1
      if j < 0 or abs(t[j] - tf) > 0.3:
        continue
      key = (pair, int(tf * 10))
      if key in used:
        continue
      used.add(key)
      flips.append({"pair": pair, "up": int(c1 > c0), "b7": float(b7[j]), "ve": float(ve[j]),
                    "pres": float(pr[j])})
      if len(flips) >= 400:
        break
  return {"seg": seg, "windows": windows, "flips": flips, "cls_stats": cs_stats,
          "pres": pres, "n109": len(t9), "crc": crc_rows if collect_crc else []}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("dir")
  ap.add_argument("--pattern", default="*/rlog.zst")
  ap.add_argument("--workers", type=int, default=10)
  ap.add_argument("--crc", action="store_true")
  ap.add_argument("--json", default="/tmp/radar_unit_pin_seg.json")
  ap.add_argument("--quiet", action="store_true")
  a = ap.parse_args()
  segs = sorted(glob.glob(f"{a.dir}/{a.pattern}" if a.pattern else a.dir))
  if not a.quiet:
    print(f"segments: {len(segs)}", file=sys.stderr)
  with Pool(a.workers) as p:
    res = p.map(seg_work, [(s, a.crc) for s in segs], chunksize=2)
  with open(a.json, "w") as fp:
    json.dump(res, fp)
  if not a.quiet:
    nw = sum(len(r["windows"]) for r in res)
    print(f"segments={len(res)} windows={nw} flips={sum(len(r['flips']) for r in res)}", file=sys.stderr)


if __name__ == "__main__":
  main()
