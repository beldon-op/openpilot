"""0x109 b3 yRel re-probe (risk #4 close-out attempt; field-table row "UNKNOWN").

Idle pattern is [b2,b3,b4,b5,b6]=[FF,7F,FF,7F,7F]: b3's 0x7F center looks like
the Continental signed-byte invalid marker, and ARS410-family frames pair one
LongDist byte with a LatDist byte per object. If b3 = yRel (signed, ~center
127), it is exactly the missing lane-disambiguation input for T8 level-2
(abort only for TARGET-LANE approaches, docs/byd-radar-010x.md §⑤).

First-order characterizations on /tmp/seg1005.pkl:
  A) state-conditional distribution: b3 by lead class (0/2/3/4), presence,
     b7 range - does it actually LEAVE 0x7F when a target is tracked?
  B) slow-variness: lag-1 |db3| vs |db7| in lead windows (geometry says yRel
     moves much slower than range for same-lane traffic).
  C) correlation re-check: r(b3, b7), r(b3, vEgo), r(b3-127, |b7-v*tau|)...
     keep only the cheap u-free ones: r(b3, b7), r(b3, vEgo), and the |r|<0.11
     2026-10-18 claim reproduced archive-wide.
  D) context probes: b3 at the 13 stop-to-lead plateaus (same-lane stopped
     lead -> if yRel, b3 ~ centered) and inside the 247 loss-edge episodes.
  E) appearance trajectories: visible rising edge (presence==7 & 1<=b7<=245
     after >=1 s invisible, vEgo>3, stays visible >=8 s) -> b3 at k seconds
     after the edge; aggregate median/p10/p90 per k. A same-lane pickup
     starts centered; a lane-adjacent pickup should be offset.
  F) 0x101 b7 table bytes (216 / 0 / 29-34 / 146-152) vs b3_109 joint:
     is the 0x101 slot carrying the paired lateral of the same object?

Usage: .venv/bin/python tools/byd/radar_b3_mine.py --out /tmp/radar_b3.txt
"""
import sys, argparse, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import radar_seglib as L

SEC = 1_000_000_000


def pct(v, q):
  v = sorted(v)
  if not v:
    return float("nan")
  return v[min(int(len(v) * q), len(v) - 1)]


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--pkl", default="/tmp/seg1005.pkl")
  ap.add_argument("--out", default="/tmp/radar_b3.txt")
  a = ap.parse_args()
  cache = L.load(a.pkl)
  lines = []
  def say(*x):
    s = " ".join(str(v) for v in x)
    print(s, flush=True)
    lines.append(s)

  dist = {c: collections.Counter() for c in (0, 2, 3, 4)}
  by_pres = {p: collections.Counter() for p in (0, 1, 7, "other")}
  d3, d7 = [], []
  corr_b7, corr_v = [], []
  stop_b3, epi_b3 = [], []
  traj = collections.defaultdict(list)     # k -> list of b3 signed values
  joint = {}                               # (b101 bucket) -> Counter of b3
  app_n = 0

  for name, s in cache.items():
    t = s["109_t"]
    if len(t) < 300:
      continue
    d9 = s["109_d"]
    b7 = d9[:, 7].astype(np.int32)
    b3 = d9[:, 3].astype(np.int32)
    ve = L.vego_at(t, s)
    pr = L.pres_at(t, s)
    cl = L.cls_at(t, s)
    lead = (cl >= 2) & (pr == 7) & (b7 >= 1) & (b7 <= 245) & np.isfinite(ve)
    # class-conditional (vectorized via masks)
    for c in (0, 2, 3, 4):
      m = (cl == c) & (pr == 7) if c else (cl == 0)
      for v in b3[m]:
        dist[c][int(v)] += 1
    for p, v in zip(pr, b3):
      key = int(p) if p in (0, 1, 7) else "other"
      by_pres[key][int(v)] += 1
    # B) lag-1
    m = (lead[:-1] & lead[1:]).astype(bool)
    if m.sum() > 100:
      d3.extend(np.abs(b3[1:][m] - b3[:-1][m]).tolist())
      d7.extend(np.abs(b7[1:][m] - b7[:-1][m]).tolist())
    # C) correlations
    if lead.sum() > 800 and b3[lead].std() > 0.5 and b7[lead].std() > 3:
      corr_b7.append(float(np.corrcoef(b3[lead], b7[lead])[0, 1]))
      if ve[lead].std() > 0.5:
        corr_v.append(float(np.corrcoef(b3[lead], ve[lead])[0, 1]))
    # D) stop plateaus: reuse stop2lead criterion light: standstill>=2s, b7 in [5,80]
    below = np.isfinite(ve) & (ve < 0.2)
    i = 0
    while i < len(ve):
      if not below[i]:
        i += 1; continue
      j = i
      while j < len(ve) and below[j]:
        j += 1
      if (t[j - 1] - t[i]) / SEC >= 2.0:
        k0, k1 = np.searchsorted(t, [t[i], t[j - 1]])
        sel = lead[k0:k1]
        bb7, bb3 = b7[k0:k1][sel], b3[k0:k1][sel]
        if len(bb7):
          m2 = (bb7 >= 5) & (bb7 <= 80)
          if m2.sum() > 10:
            stop_b3.append(int(np.median(bb3[m2])))
      i = j
    for idx in np.where((b7 == 247) | (b7 >= 246))[0]:
      epi_b3.append(int(b3[idx]))
    # E) appearance trajectories
    vis = (pr == 7) & (b7 >= 1) & (b7 <= 245)
    idx = np.where(vis)[0]
    if len(idx) == 0:
      continue
    runs = np.split(idx, np.where(np.diff(idx) > 3)[0] + 1)
    for g in runs:
      if len(g) < 200:                      # >=8 s visible
        continue
      i0 = g[0]
      if ve[i0] <= 3:
        continue
      # require >=1 s invisible before
      pre = vis[max(0, i0 - 25):i0 - 5]
      if pre.any():
        continue
      app_n += 1
      for k in range(0, 11):
        ii = i0 + k * 25
        if ii < len(t) and vis[ii]:
          traj[k].append(int(b3[ii]) - 127)
    # F) 0x101 joint
    t1, d1 = s["101_t"], s["101_d"]
    if len(t1) > 300:
      b101 = np.interp(t, t1, d1[:, 7].astype(np.float64), left=-1, right=-1).round()
      ok = b101 >= 0
      def bucket(x):
        if x == 0: return "0"
        if 25 <= x <= 40: return "29-34"
        if 140 <= x <= 160: return "146-152"
        if x >= 200: return "216"
        return "other"
      for v9, x1 in zip(b3[ok], b101[ok].astype(int)):
        joint.setdefault(bucket(x1), collections.Counter())[int(v9)] += 1

  say(f"== A) b3 distribution by HUD class (signed offset from 0x7F=127)")
  for c in (0, 2, 3, 4):
    cc = dist[c]
    n = sum(cc.values())
    if not n:
      continue
    off = np.array([v - 127 for v in cc.elements()])
    say(f"  class {c}: n={n:8d} b3==127 {cc.get(127, 0)/n:6.2%}  |off| med={np.median(np.abs(off)):.0f} "
        f"p90={pct(np.abs(off), .9):.0f} max={np.abs(off).max()}  distinct={len(cc)}")
  say(f"  by presence: " + "; ".join(f"p{k}: n={sum(v.values())} ==127 {v.get(127,0)/max(sum(v.values()),1):6.2%}"
                                     for k, v in by_pres.items() if sum(v.values())))

  say(f"\n== B) lead-window lag-1 moves")
  if d3:
    say(f"  |db3| med={pct(d3,.5):.2f} p90={pct(d3,.9):.2f} max={max(d3)}"
        f"   |db7| med={pct(d7,.5):.2f} (b7 is the range byte)")

  say(f"\n== C) correlations per lead segment (reproduce 2026-10-18 |r|<0.11)")
  if corr_b7:
    say(f"  r(b3,b7): n={len(corr_b7)} med={np.median(np.abs(corr_b7)):.3f} "
        f"p90={np.percentile(np.abs(corr_b7),90):.3f} max={max(np.abs(corr_b7)):.3f}")
  if corr_v:
    say(f"  r(b3,vEgo): med={np.median(np.abs(corr_v)):.3f} p90={np.percentile(np.abs(corr_v),90):.3f} "
        f"max={max(np.abs(corr_v)):.3f}")

  say(f"\n== D) context probes")
  if stop_b3:
    off = np.array(stop_b3) - 127
    say(f"  13-ish stop plateaus b3 med={np.median(stop_b3):.0f} "
        f"spread {sorted(set(stop_b3))[:8]} (centered 127 -> yRel-consistent)")
  if epi_b3:
    say(f"  b3 at b7>=246 loss-edge frames: distinct {sorted(set(epi_b3))[:6]} n={len(epi_b3)}")

  say(f"\n== E) appearance events (>=8 s visible, v>3): b3-127 trajectory, n={app_n}")
  for k in range(0, 11):
    v = traj.get(k)
    if v:
      vv = sorted(v)
      say(f"  t+{k:2d}s n={len(vv):5d} med={np.median(vv):6.1f} p10={pct(vv,.1):6.1f} p90={pct(vv,.9):6.1f}")

  say(f"\n== F) b3_109 by 0x101 b7 bucket")
  for b, cc in sorted(joint.items(), key=lambda kv: -sum(kv[1].values())):
    n = sum(cc.values())
    if n < 2000:
      continue
    elems = np.repeat(list(cc.keys()), list(cc.values())) - 127
    say(f"  b101={b:7s} n={n:8d} b3==127 {cc.get(127,0)/n:6.2%} |off|med={np.median(np.abs(elems)):.0f} "
        f"top3={cc.most_common(3)}")

  with open(a.out, "w") as fp:
    fp.write("\n".join(lines) + "\n")


if __name__ == "__main__":
  main()
