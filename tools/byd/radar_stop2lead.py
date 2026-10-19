"""Stop-to-lead equality calibration for 0x109 b7 unit u (m/count).

The user asserts byd_logs_2026-10-05_07 contains ACC stop-to-lead events.
During an approach that ends in a standstill behind the lead, the lead is
(stationary for at least the final phase; for red-light queues it was stopped
throughout). Then the closed gap equals distance traveled:

    u = D / delta_b7      (equality if lead stationary during approach;
                           a late-stopping lead inflates u -> so the LOW tail
                           of the event distribution converges to u from above)

Event definition per segment:
  - standstill: carState.vEgo < 0.2 m/s sustained >= 2 s, reached from vEgo > 2
  - at standstill: 0x109 b7 in [5,80] counts, 0x10A presence(>>5)==7,
    HUD HasLead==1 (ACC still holding the lead)
  - approach phase: from the last frame with vEgo > 5 before the standstill,
    to standstill start; require delta_b7 = b7(approach_start) - b7(plateau) >= 10
  - D = integral vEgo over approach (trapezoid), u_event = D/delta_b7
Report: n, u_event min/p05/p10/p25/med, final-gap counts, and for candidate
u in {0.3048 feet, 0.25, 0.5}: implied stopped gap in meters = gap_counts*u
(physical sanity 2-8 m) and fraction of events within [0.9,1.1]*candidate.

Usage:
  .venv/bin/python tools/byd/radar_stop2lead.py docs_site/byd_logs_2026-10-05_07/realdata [--workers 10] [--json out.json]
"""
import sys, os, glob, json, argparse
from multiprocessing import Pool
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANParser
from opendbc.car import Bus, CanData
from opendbc.car.byd.values import DBC, CAR

HUD_MSG = 0x32D
SEC = 1_000_000_000


def seg_events(f):
  hp = CANParser(DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar], [(HUD_MSG, 50)], 2)
  ht, hlead = [], []
  t9, b7s, pres = [], [], []
  vt, vv = [], []
  for m in LogReader(f):
    w = m.which()
    if w == "carState":
      vt.append(m.logMonoTime / SEC); vv.append(m.carState.vEgo)
    if w != "can":
      continue
    hf = []
    for c in m.can:
      if c.src == 1 and c.address == 0x109:
        t9.append(m.logMonoTime / SEC); b7s.append(c.dat[7])
      elif c.src == 1 and c.address == 0x10A:
        pres.append((m.logMonoTime / SEC, c.dat[3] >> 5))
      elif c.src == 2 and c.address == HUD_MSG:
        hf.append(CanData(c.address, bytes(c.dat), c.src))
    if hf:
      hp.update([(int(m.logMonoTime), hf)])
      v = hp.vl[HUD_MSG]
      ht.append(m.logMonoTime / SEC); hlead.append(int(v["HasLead"]))
  if len(t9) < 400 or len(vt) < 200:
    return []
  t9 = np.asarray(t9); b7s = np.asarray(b7s, float)
  vt = np.asarray(vt); vv = np.asarray(vv)
  pres_t = np.asarray([p[0] for p in pres]); pres_v = np.asarray([p[1] for p in pres])
  ht = np.asarray(ht); hl = np.asarray(hlead)
  events = []
  # find standstill runs
  below = vv < 0.2
  i = 0
  while i < len(vv):
    if not below[i]:
      i += 1; continue
    j = i
    while j < len(vv) and below[j]:
      j += 1
    dur = vt[j - 1] - vt[i]
    if dur >= 2.0 and i > 0:
      # reached from motion within previous 40s
      k0 = np.searchsorted(vt, vt[i] - 40)
      moved_before = vv[k0:i].max() > 2.0 if i > k0 else False
      if moved_before:
        t_stop = vt[i]
        # b7 plateau + presence + HasLead at standstill
        a = np.searchsorted(t9, t_stop)
        if a >= len(t9) or abs(t9[a] - t_stop) > 2.0:
          i = j; continue
        w_end = np.searchsorted(t9, min(t_stop + 2.0, t9[-1]))
        gap_seg = b7s[a:w_end]
        gap = float(np.median(gap_seg))
        pa = np.searchsorted(pres_t, t_stop)
        pr = float(np.median(pres_v[pa:pa + 50])) if pa < len(pres_v) else 0
        la = np.searchsorted(ht, t_stop)
        lead = float(hl[max(0, la - 2):la + 50].mean()) if la < len(ht) else 0
        if 5 <= gap <= 80 and pr == 7 and lead >= 0.9:
          # approach: walk back from t_stop until vEgo > 5
          s = np.searchsorted(vt, t_stop)
          back = s
          while back > 0 and vt[back - 1] > t_stop - 45 and vv[back - 1] <= 5.0:
            back -= 1
          if back < s and vv[back] > 3.0:
            t_a0 = vt[back]
            aa = np.searchsorted(t9, t_a0)
            b7_0 = float(np.median(b7s[aa:aa + 5]))
            dlt = b7_0 - gap
            if dlt >= 10:
              seg = slice(back, s)
              D = float(np.trapezoid(vv[seg], vt[seg])) if hasattr(np, "trapezoid") else float(np.trapz(vv[seg], vt[seg]))
              if D > 3.0:
                # tail series (last 20 s rel. to stop): (t_rel, vEgo, b7) @4Hz
                t_lo = max(t_a0, t_stop - 20)
                sel_v = (vt >= t_lo) & (vt <= t_stop)
                sel_b = (t9 >= t_lo) & (t9 <= t_stop)
                step = 4
                series = [[round(float(t - t_stop), 2), round(float(v), 2), float(b)]
                          for t, v, b in zip(t9[sel_b][::step],
                                             np.interp(t9[sel_b][::step], vt[sel_v], vv[sel_v]),
                                             b7s[sel_b][::step])]
                events.append({"seg": os.path.basename(os.path.dirname(f)) if os.path.basename(f) == "rlog.zst" else os.path.basename(f),
                               "gap": gap, "b7_0": b7_0, "dlt": dlt, "D": round(D, 2),
                               "u": round(D / dlt, 4), "v_app_max": round(float(vv[seg].max()), 1),
                               "dur": round(float(vt[s - 1] - t_a0), 1), "series": series})
    i = j
  return events


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("dir")
  ap.add_argument("--workers", type=int, default=10)
  ap.add_argument("--json", default="/tmp/radar_stop2lead.json")
  ap.add_argument("--pattern", default="*/rlog.zst")
  a = ap.parse_args()
  segs = sorted(glob.glob(f"{a.dir}/{a.pattern}"))
  print(f"# {len(segs)} segments")
  with Pool(a.workers) as p:
    res = p.map(seg_events, segs, chunksize=4)
  events = [e for r in res for e in r]
  json.dump(events, open(a.json, "w"))
  print(f"# events n={len(events)}")
  if not events:
    print("NO STOP-TO-LEAD EVENTS FOUND - user assertion check: maybe ACC never fully stopped, loosen gates")
    return
  u = np.array([e["u"] for e in events])
  gap = np.array([e["gap"] for e in events])
  dlt = np.array([e["dlt"] for e in events])
  q = np.percentile(u, [0, 5, 10, 25, 50, 75])
  print(f"u_event: min={q[0]:.4f} p05={q[1]:.4f} p10={q[2]:.4f} p25={q[3]:.4f} med={q[4]:.4f} p75={q[5]:.4f}")
  print("\ncandidate u -> implied stopped gap (m) = gap_counts*u:")
  for cand, name in ((0.25, "0.25"), (0.3048, "feet"), (0.5, "0.5")):
    gm = gap * cand
    ok = ((gm >= 2) & (gm <= 8)).mean()
    inb = ((u >= cand * 0.95) & (u <= cand * 1.5)).mean()
    print(f"  {name}: gap_m med={np.median(gm):.1f} p10-p90=[{np.percentile(gm,10):.1f},{np.percentile(gm,90):.1f}] "
          f"physical[2,8]m {ok:.0%}  | u_event within [0.95c,1.5c]: {inb:.0%}")
  print("\nlowest-u events (closest to lead-stationary equality):")
  for e in sorted(events, key=lambda x: x["u"])[:12]:
    print(f"  u={e['u']:.4f} b7 {e['b7_0']:.0f}->{e['gap']:.0f} (dlt {e['dlt']:.0f}) D={e['D']}m v_app_max={e['v_app_max']}m/s dur={e['dur']}s seg={e['seg']}")
  print("\nlargest-drop events:")
  for e in sorted(events, key=lambda x: -x["dlt"])[:8]:
    print(f"  u={e['u']:.4f} b7 {e['b7_0']:.0f}->{e['gap']:.0f} D={e['D']}m dur={e['dur']}s")

  print("\n## tail convergence: u_L = D_L/(b7(-L)-gap), lead stationary in tail => u_L -> u")
  for L in (3, 4, 6, 8, 10, 14, 20):
    vals = []
    for e in events:
      s = e.get("series") or []
      if not s:
        continue
      tail = [(t, v, b) for t, v, b in s if t >= -L]
      if len(tail) < 8:
        continue
      b_head = np.median([b for t, v, b in tail if t <= tail[0][0] + 0.6])
      dlt_l = b_head - e["gap"]
      if dlt_l < 4:
        continue
      D_l = float(np.trapezoid([v for t, v, b in tail], [t for t, v, b in tail])) if hasattr(np, "trapezoid") else float(np.trapz([v for t, v, b in tail], [t for t, v, b in tail]))
      vals.append(D_l / dlt_l)
    if len(vals) >= 4:
      print(f"  L={L:>2}s: n={len(vals):>2}  min={min(vals):.4f} p25={np.percentile(vals,25):.4f} med={np.median(vals):.4f}")
  # per-event tail table for the biggest events
  print("\n  per-event u_L (L=4/6/8/12):")
  for e in sorted(events, key=lambda x: -x["dlt"])[:10]:
    s = e.get("series") or []
    if not s:
      continue
    row = []
    for L in (4, 6, 8, 12):
      tail = [(t, v, b) for t, v, b in s if t >= -L]
      if len(tail) < 6:
        row.append("-"); continue
      b_head = np.median([b for t, v, b in tail if t <= tail[0][0] + 0.6])
      dlt_l = b_head - e["gap"]
      if dlt_l < 3:
        row.append("-"); continue
      D_l = float(np.trapezoid([v for t, v, b in tail], [t for t, v, b in tail])) if hasattr(np, "trapezoid") else float(np.trapz([v for t, v, b in tail], [t for t, v, b in tail]))
      row.append(f"{D_l/dlt_l:.2f}")
    print(f"    seg={e['seg']} b7 {e['b7_0']:.0f}->{e['gap']:.0f}: " + "/".join(row))


if __name__ == "__main__":
  main()
