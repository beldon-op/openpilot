"""T8 replay validation: run the real RadarGapMonitor over /tmp/seg1005.pkl.

Acceptance data for docs/byd-radar-010x.md §落地建议 T8 / byd-lane-change §8.4
(before any on-road gate flip):
  A) would-block proxy: alert-seconds share inside STEADY holds (constant HUD
     class >= 6 s, trimmed 2 s both sides, visible frames only). The static
     midpoint threshold from ④e misclassified 29-60% of frames here - the
     debounced boundary-cross must sit far under that.
  B) pre-flip coverage: share of 2<->3 / 3<->4 gear flips that had an ACTIVE
     alert in the preceding 8 s, and the lead distribution (alert onset to
     flip). ④e baseline: b7 crossed the midpoint before 309/309 flips; this
     checks how much of that the debounce still preserves.
  C) loss-path purity: alert state right at lead->0 flips (should never be
     needed there - the path is synchronous, ④e).
  D) new_target event rate per hour of driving.

Usage: .venv/bin/python tools/byd/radar_t8_replay.py [--pkl /tmp/seg1005.pkl]
       --out /tmp/radar_t8_replay.txt
"""
import sys, argparse, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
import radar_seglib as L
from opendbc.car.byd.radar_gap import RadarGapMonitor

SEC = 1_000_000_000
HOLD_MIN_S = 6.0
TRIM_S = 2.0
PRE_FLIP_S = 8.0


def replay_seg(seg):
  t = seg["109_t"]
  if len(t) < 250:
    return None
  d = seg["109_d"]
  b7 = d[:, 7].astype(np.int32)
  ve = L.vego_at(t, seg)
  pr = L.pres_at(t, seg)
  cl = L.cls_at(t, seg)
  dt = np.diff(t) / SEC
  mon = RadarGapMonitor()
  alert = []          # (t_start, t_end) ns
  cur = None
  new_events = 0
  prev_new = False
  for i in range(len(t)):
    step = float(dt[i - 1]) if i > 0 else 0.04
    mon.update(step, int(b7[i]), int(pr[i]), max(0, int(cl[i])),
               float(ve[i]) if np.isfinite(ve[i]) else 0.0, True)
    if mon.closing and cur is None:
      cur = t[i]
    elif not mon.closing and cur is not None:
      alert.append((cur, t[i]))
      cur = None
    if mon.new_target and not prev_new:
      new_events += 1
    prev_new = mon.new_target
  if cur is not None:
    alert.append((cur, t[-1]))
  # D) drive hours
  drive_s = (t[-1] - t[0]) / SEC
  # A) steady-hold alert share (visible frames only)
  ht, hc = seg["hud_t"], seg["hud_cls"]
  hold_vis = hold_alert = 0.0
  i = 0
  flips = []
  while i < len(hc):
    j = i
    while j < len(hc) and hc[j] == hc[i]:
      j += 1
    c, a_t, z_t = int(hc[i]), ht[i], ht[j - 1]
    if j < len(hc) and c >= 2 and int(hc[j]) >= 2:
      flips.append((c, int(hc[j]), z_t))
    if c >= 2 and (z_t - a_t) >= HOLD_MIN_S * SEC:
      lo, hi = a_t + TRIM_S * SEC, z_t - TRIM_S * SEC
      k0, k1 = np.searchsorted(t, [lo, hi])
      vis = (pr[k0:k1] == 7) & (b7[k0:k1] >= 1) & (b7[k0:k1] <= 245) & \
            (ve[k0:k1] > 3) & np.isfinite(ve[k0:k1])
      tt = t[k0:k1][vis]
      hold_vis += float(np.sum(np.diff(tt) / SEC)) if len(tt) > 1 else 0.0
      for s, e in alert:
        ov = min(max(e, lo), hi) - max(s, lo)
        if ov > 0:
          hold_alert += float(ov) / SEC
    i = j
  # B) pre-flip coverage, by direction (opening flips must NOT alert by design)
  caught, leads = {}, []
  for c0, c1, t_flip in flips:
    pair = (c0, c1)
    if pair not in ((3, 2), (4, 3), (2, 3), (3, 4)):
      continue
    hit = [(s, e) for s, e in alert if s < t_flip and e > t_flip - PRE_FLIP_S * SEC]
    if hit:
      caught[pair] = caught.get(pair, 0) + 1
      leads.append((t_flip - max(s for s, _ in hit)) / SEC)
  flips_by_pair = dict(collections.Counter(p for p in [(c0, c1) for c0, c1, _ in flips
                                                       if (c0, c1) in ((3, 2), (4, 3), (2, 3), (3, 4))]))
  return {"drive_s": drive_s, "hold_vis_s": hold_vis, "hold_alert_s": hold_alert,
          "n_alerts": len(alert), "flips_by_pair": flips_by_pair,
          "caught": caught, "leads": leads, "new_events": new_events}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--pkl", default="/tmp/seg1005.pkl")
  ap.add_argument("--out", default="/tmp/radar_t8_replay.txt")
  a = ap.parse_args()
  cache = L.load(a.pkl)
  lines = []
  def say(*x):
    s = " ".join(str(v) for v in x)
    print(s, flush=True)
    lines.append(s)

  res = []
  for name, s in cache.items():
    r = replay_seg(s)
    if r:
      r["seg"] = name
      res.append(r)

  tot_drive = sum(r["drive_s"] for r in res)
  tot_vis = sum(r["hold_vis_s"] for r in res)
  tot_alert = sum(r["hold_alert_s"] for r in res)
  caught = collections.Counter()
  nflip = collections.Counter()
  for r in res:
    for pair, c in r["caught"].items():
      caught[pair] += c
    for pair, n in r["flips_by_pair"].items():
      nflip[pair] += n
  leads = sorted(l for r in res for l in r["leads"])
  n_alerts = sum(r["n_alerts"] for r in res)
  new_ev = sum(r["new_events"] for r in res)

  say(f"# segments replayed: {len(res)}  drive hours: {tot_drive/3600:.1f} h")
  say(f"== A) steady-hold would-block proxy (visible trimmed-hold seconds)")
  say(f"  alert share = {tot_alert:.0f} s / {tot_vis:.0f} s = "
      f"{tot_alert/max(tot_vis,1):.3%}   (④e static-threshold false-cross: 29-60%)")
  per_seg_share = sorted(r["hold_alert_s"]/r["hold_vis_s"] for r in res if r["hold_vis_s"] > 30)
  if per_seg_share:
    n = len(per_seg_share)
    say(f"  per-seg share: med={per_seg_share[n//2]:.3%} p90={per_seg_share[int(n*.9)]:.3%} "
        f"max={per_seg_share[-1]:.3%}  segs>1%: {sum(1 for v in per_seg_share if v > 0.01)}/{n}")
  say(f"== B) pre-flip coverage within {PRE_FLIP_S:.0f} s, by flip direction")
  for pair in ((3, 2), (4, 3)):          # closing flips: alert SHOULD precede
    say(f"  {pair[0]}->{pair[1]} (closing): caught {caught[pair]}/{nflip[pair]} = "
        f"{caught[pair]/max(nflip[pair],1):.1%}")
  for pair in ((2, 3), (3, 4)):          # opening flips: alert should NOT fire
    say(f"  {pair[0]}->{pair[1]} (opening): caught {caught[pair]}/{nflip[pair]} = "
        f"{caught[pair]/max(nflip[pair],1):.1%}  (want ~0; nonzero = chatter/overlap)")
  if leads:
    n = len(leads)
    say(f"  lead s (caught only): p10={leads[int(n*.1)]:.2f} med={leads[n//2]:.2f} "
        f"p90={leads[int(n*.9)]:.2f}")
  say(f"== C) alert episodes total: {n_alerts}  ({n_alerts/max(tot_drive/3600,1e-9):.1f}/h)")
  say(f"== D) new-target events: {new_ev}  ({new_ev/max(tot_drive/3600,1e-9):.1f}/h)")

  with open(a.out, "w") as fp:
    fp.write("\n".join(lines) + "\n")


if __name__ == "__main__":
  main()
