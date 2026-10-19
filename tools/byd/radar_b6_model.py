"""Model 0x109 b6<->b7 and the 0x101/0x105 second b7 path (report 7.4, risks #6/#8).

Questions:
 A) b6 vs b7 shape: pooled archive-wide b6 as a function of b7 (binned means),
    the b6==0x7F(127) "idle/invalid" regime (247 census: b6 pinned 127 while
    b7=246), and per-segment fit b6 = alpha*b7 + beta on non-pegged frames
    (b7<=130). alpha~1 & beta~-6/-12 => offset/delayed copy; alpha~0.5 =>
    coarse/fine split; carries? -> test b6 step when b7 crosses boundaries.
 B) pipeline lag: normalized cross-correlation of b7(t) vs b6(t+L),
    L in [-30,30] frames (25 Hz => +-1.2 s), per qualifying segment.
 C) 0x101 b7 (and b4-b6 too, it is a raw slot): candidates
    (i) time-headway code   b7_101 ~ b7_109 / vEgo
    (ii) lagged copy         b7_101(t) ~ b7_109(t-L)
    (iii) second target      b7_101 orthogonal to b7_109
    class-0 census: distinct values, and correlation with vEgo / HUD bytes.

Usage: .venv/bin/python tools/byd/radar_b6_model.py --out /tmp/radar_b6.txt
"""
import sys, argparse, collections, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import radar_seglib as L


def corr_series(a, b, lags):
  """corr(a(t), b(t+L)) for integer lags (b shifted forward)."""
  out = []
  for L_ in lags:
    if L_ >= 0:
      x, y = a[:len(a) - L_], b[L_:]
    else:
      x, y = a[-L_:], b[:len(b) + L_]
    if len(x) < 500:
      out.append((L_, None))
      continue
    sx, sy = x.std(), y.std()
    out.append((L_, None if sx == 0 or sy == 0 else float(((x - x.mean()) * (y - y.mean())).mean() / (sx * sy))))
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--pkl", default="/tmp/seg1005.pkl")
  ap.add_argument("--out", default="/tmp/radar_b6.txt")
  ap.add_argument("--json", default="/tmp/radar_b6.json")
  a = ap.parse_args()
  cache = L.load(a.pkl)
  lines = []
  def say(*x):
    s = " ".join(str(v) for v in x)
    print(s, flush=True)
    lines.append(s)

  bins = list(range(0, 250, 20))
  b6sum = collections.defaultdict(lambda: [0, 0, 0])   # bin -> [n, sum_b6, n_peg127]
  b6vals = {"lead": collections.Counter(), "nolead": collections.Counter()}
  cond101 = {k: collections.Counter() for k in
             ("0", "1-30", "31-60", "61-120", "121-200", "201-247")}
  seg_fits = []
  lag_results = []
  hd_stats = {"lead": collections.Counter(), "nolead": collections.Counter()}
  c0_corrs = []
  c1_list = []          # guarded corr values (no nans)
  dlt_all = collections.Counter()

  for name, s in cache.items():
    t = s["109_t"]
    if len(t) < 300:
      continue
    d = s["109_d"]
    b7 = d[:, 7].astype(np.int32); b6 = d[:, 6].astype(np.int32)
    ve = L.vego_at(t, s); pr = L.pres_at(t, s); cl = L.cls_at(t, s)
    lead = (cl >= 2) & (pr == 7) & (b7 >= 9) & np.isfinite(ve)
    nol = (cl == 0)
    for v in b6[lead]:
      b6vals["lead"][int(v)] += 1
    for v in b6[nol]:
      b6vals["nolead"][int(v)] += 1
    # (A) pooled b6-vs-b7 shape over all lead frames
    for v7, v6 in zip(b7[lead], b6[lead]):
      bi = min(int(v7) // 20 * 20, 240)
      st = b6sum[bi]
      st[0] += 1; st[1] += int(v6); st[2] += int(v6 == 127)
    dl = b7[lead] - b6[lead]
    for v, n in zip(*np.unique(dl, return_counts=True)):
      dlt_all[int(v)] += int(n)
    # per-seg linear fit on b7<=130 & b6!=127
    m = lead & (b7 <= 130) & (b6 != 127)
    if m.sum() >= 300:
      x = b7[m].astype(np.float64); y = b6[m].astype(np.float64)
      if x.std() > 3:
        al = float(np.cov(x, y)[0, 1] / np.var(x))
        be = float(y.mean() - al * x.mean())
        seg_fits.append((al, be, int(m.sum()), float(np.corrcoef(x, y)[0, 1])))
    # (B) lag on one continuous lead run (hold class, no loss)
    runs = np.split(np.where(lead)[0], np.where(np.diff(np.where(lead)[0]) > 5)[0] + 1)
    for g in runs:
      if len(g) < 4000:
        continue
      x = b7[g].astype(np.float64); y = b6[g].astype(np.float64)
      if x.std() < 5 or (y == 127).mean() > 0.5:
        continue
      cc = corr_series(x, y, range(-30, 31))
      ccv = [(l_, r_) for l_, r_ in cc if r_ is not None]
      best = max(ccv, key=lambda z: z[1])
      lag_results.append((name, best[0], best[1], len(g)))
      break
    # (C) 0x101 stream aligned onto 109 time base
    t101, d101 = s["101_t"], s["101_d"]
    if len(t101) >= 300:
      b101 = np.interp(t, t101, d101[:, 7].astype(np.float64), left=-1, right=-1).round()
      b101m = (b101 >= 0)
      for v, n in zip(*np.unique(b101[lead & b101m], return_counts=True)):
        hd_stats["lead"][(int(v))] += int(n)
      nol = (cl == 0) & b101m
      for v, n in zip(*np.unique(b101[nol], return_counts=True)):
        hd_stats["nolead"][(int(v))] += int(n)
      # conditional map: 0x101 b7 value per 0x109 b7 magnitude bin
      def binkey(v):
        if v == 0: return "0"
        if v <= 30: return "1-30"
        if v <= 60: return "31-60"
        if v <= 120: return "61-120"
        if v <= 200: return "121-200"
        return "201-247"
      selm = b101m & np.isfinite(ve)
      for v9, v1 in zip(b7[selm], b101[selm].astype(int)):
        cond101[binkey(int(v9))][v1] += 1
      # hypotheses on lead frames, unpinned values only (0 and 216 look like codes)
      lm = lead & b101m & (ve > 2) & (b101 > 1) & (b101 < 215)
      if lm.sum() >= 800:
        yv = b101[lm].astype(np.float64)
        x1 = b7[lm].astype(np.float64)
        x2 = x1 / ve[lm]
        if x1.std() > 3 and yv.std() > 3:
          hd_stats.setdefault("c101", []).append(
              {"seg": name, "corr_raw": float(np.corrcoef(x1, yv)[0, 1]),
               "corr_over_v": float(np.corrcoef(x2, yv)[0, 1]) if x2.std() > 3 else None,
               "n": int(lm.sum()),
               "mean_ratio": float(np.mean(yv / x1))})
      # class-0: does b101 track vEgo or HUD bytes?
      nm = nol & (ve > 2)
      if nm.sum() >= 800:
        c_ve = (np.corrcoef(ve[nm], b101[nm])[0, 1]
                if ve[nm].std() > 0.5 and b101[nm].std() > 0.5 else None)
        c101 = collections.Counter(b101[nm].astype(int))
        c0_corrs.append((name, float(c_ve) if c_ve is not None else None,
                         len(c101), c101.most_common(6)))

  say("== (A0) 0x109 b6 full-value histogram (top 8) by window")
  for w in ("lead", "nolead"):
    tot = sum(b6vals[w].values())
    say(f"  {w} n={tot}: ", [(v, round(n / tot, 4)) for v, n in b6vals[w].most_common(8)])
  say("\n== (A) pooled b6 mean vs b7 bin, lead frames (peg=b6==127 share)")
  for bi in bins:
    st = b6sum.get(bi)
    if st and st[0] >= 500:
      say(f"  b7[{bi:3d}-{bi+19:3d}] n={st[0]:7d} b6_mean={st[1]/st[0]:6.1f} "
          f"peg127={st[2]/st[0]:6.2%}")
  top = sorted(dlt_all.items(), key=lambda kv: -kv[1])[:14]
  say("  top (b7-b6) deltas:", top)
  al = np.array([f[0] for f in seg_fits]); be = np.array([f[1] for f in seg_fits])
  say(f"  per-seg b6=alpha*b7+beta (b7<=130,b6!=127): n={len(al)} "
      f"alpha med={np.median(al):.3f} p25={np.percentile(al,25):.2f} p75={np.percentile(al,75):.2f} "
      f" beta med={np.median(be):.2f}")

  say("\n== (B) cross-correlation lag of b6 vs b7 (long lead runs)")
  if lag_results:
    lags = [lr[1] for lr in lag_results]
    say(f"  n={len(lags)} segs; peak-lag distribution:",
        dict(collections.Counter(lags)))
    say("  sample:", lag_results[:8])

  say("\n== (C) 0x101 b7 hypotheses")
  ln = sum(hd_stats["lead"].values()); nn = sum(hd_stats["nolead"].values())
  say(f"  lead frames: distinct={len(hd_stats['lead'])} n={ln} top:",
      hd_stats["lead"].most_common(8))
  say(f"  class-0 frames: distinct={len(hd_stats['nolead'])} n={nn} top:",
      hd_stats["nolead"].most_common(12))
  say("\n  conditional: 0x101 b7 (top3) per 0x109 b7 bin")
  for k in ("0", "1-30", "31-60", "61-120", "121-200", "201-247"):
    tot = sum(cond101[k].values())
    if tot:
      say(f"    b7_109={k:8s} n={tot:7d} b7_101 top:",
          [(v, round(n / tot, 3)) for v, n in cond101[k].most_common(3)])
  c101 = hd_stats.get("c101", [])
  if c101:
    cr = [r["corr_raw"] for r in c101 if r["corr_raw"] == r["corr_raw"]]
    cv = [r["corr_over_v"] for r in c101
          if isinstance(r.get("corr_over_v"), float) and r["corr_over_v"] == r["corr_over_v"]]
    if cv:
      say(f"  lead unpinned corr(b7_101 vs b7_109): med={np.median(cr):.3f} n={len(cr)}; "
          f"vs headway b7/vEgo med={np.median(cv):.3f}")
    else:
      say(f"  lead unpinned corr(b7_101 vs b7_109): med={np.median(cr):.3f} n={len(cr)}")
    say("  mean ratio b7_101/b7_109 med:",
        f"{np.median([r['mean_ratio'] for r in c101]):.3f}")
  if c0_corrs:
    cvs = [r[1] for r in c0_corrs if r[1] is not None and r[1] == r[1]]
    if cvs:
      say(f"  class-0 corr(b7_101, vEgo): n={len(cvs)} med={np.median(cvs):.3f} "
          f"p25={np.percentile(cvs,25):.3f} p75={np.percentile(cvs,75):.3f}")
    say("  examples(seg, corr, distinct, top6):", c0_corrs[:4])

  json.dump([[float(r[0]), float(r[1]), int(r[2]), float(r[3])] for r in seg_fits],
            open(a.json, "w"))
  with open(a.out, "w") as fp:
    fp.write("\n".join(lines) + "\n")


if __name__ == "__main__":
  main()
