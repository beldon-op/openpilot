"""415-segment full re-run of the HUD-echo / diversity checks (审2 a+c) on
docs_site/byd_logs_2026-10-05_07/realdata — the evidence that previously came
from only the two hour_logs follow segments (f4_16/fa_2, H(b7|bar)=2.244 bits,
hold change rate 8.7-14.6% vs class-0 0.02%) is thin; this scans every rlog.

Per segment (bus1 0x109/0x10A frames vs can2 0x32D HUD bars, HUD class =
HasLead*LeadingDistance, groups {0,2,3,4} like radar_0109_validate.py):
  1) follow-frame count: 0x109 b7 > 0 frames
  2) distinct b7 values (overall / nonzero-only)
  3) conditional entropy H(b7|bar) over classified 0x109 frames
  4) constant-bar holds >= 2.0s (lead classes 2/3/4): mean frame-to-frame
     change rate of b7 inside holds (echo-from-bar predicts ~0); class-0 holds
     tracked separately for the contrast
  5) 0x10A b3[7:5] presence nibble (111=target) agreement with HUD lead
Aggregate: global distinct b7 count, distributions (min/med/max) of per-seg
H(b7|bar) / hold change rate / presence agreement, and the direct answer to
"is b7 nonzero/diverse only in follow segments, zero in no-follow segments".

Usage (background, poll):
  .venv/bin/python tools/byd/radar_415_diversity.py \
      docs_site/byd_logs_2026-10-05_07/realdata --workers 10 \
      --out-txt /tmp/radar_415_diversity.txt --json /tmp/radar_415_diversity.json
"""
import sys, os, glob, json, argparse, bisect, math, collections
from multiprocessing import Pool

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANParser
from opendbc.car import Bus, CanData
from opendbc.car.byd.values import DBC, CAR

HUD_MSG = 0x32D
SEC = 1_000_000_000          # logMonoTime is ns in this env
CLASSES = (0, 2, 3, 4)       # bar classes used for conditioning (cls==1 skipped)
MIN_HOLD_S = 2.0
MIN_HOLD_FRAMES = 25         # ~1s of 0x109 at 25Hz; keep hold rates meaningful
MIN_ENTROPY_FRAMES = 300     # classified 0x109 frames needed to report seg H
MIN_10A_FRAMES = 50


def nearest_cls(t, hs_t, hs_c):
  """HUD class at the nearest 0x32D frame within +-0.5s (radar_0109_validate rule)."""
  i = bisect.bisect_left(hs_t, t)
  best = None
  for j in (i - 1, i):
    if 0 <= j < len(hs_t) and abs(hs_t[j] - t) <= SEC // 2:
      if best is None or abs(hs_t[j] - t) < abs(hs_t[best] - t):
        best = j
  return hs_c[best] if best is not None else None


def entropy(counter, total):
  return -sum((v / total) * math.log2(v / total) for v in counter.values() if v)


def seg_analyze(f):
  hp = CANParser(DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar], [(HUD_MSG, 50)], 2)
  hud_t, hud_cls = [], []
  hud_lead_frames = 0
  s109, s10a = [], []   # (t, b7), (t, presence3)
  err = None
  try:
    for m in LogReader(f):
      if m.which() != "can":
        continue
      hf = []
      for c in m.can:
        if c.src == 1 and c.address == 0x109:
          s109.append((m.logMonoTime, bytes(c.dat)[7]))
        elif c.src == 1 and c.address == 0x10A:
          s10a.append((m.logMonoTime, (bytes(c.dat)[3] >> 5) & 0x07))
        elif c.src == 2 and c.address == HUD_MSG:
          hf.append(CanData(c.address, bytes(c.dat), c.src))
      if hf:
        hp.update([(m.logMonoTime, hf)])
        v = hp.vl[HUD_MSG]
        hl = int(v["HasLead"])
        cls = int(v["LeadingDistance"]) * hl if hl else 0
        hud_t.append(m.logMonoTime)
        hud_cls.append(cls)
        hud_lead_frames += int(hl)
  except Exception as e:  # corrupted tail: keep the partial stream
    err = f"{type(e).__name__}: {str(e)[:100]}"

  # ---- 0x109 b7: diversity + conditional entropy given HUD bar ----
  b7_all = collections.Counter(v for _, v in s109)
  per_cls = {c: collections.Counter() for c in CLASSES}
  n_classified = 0
  if hud_t:
    for t, v in s109:
      c = nearest_cls(t, hud_t, hud_cls)
      if c in per_cls:
        per_cls[c][v] += 1
        n_classified += 1
  N = n_classified
  H_given_bar = None
  if N >= MIN_ENTROPY_FRAMES:
    H_given_bar = sum((sum(pc.values()) / N) * entropy(pc, sum(pc.values()))
                      for pc in per_cls.values() if pc)
  # ---- constant-bar holds: b7 change rate inside ----
  runs, start = [], 0
  for i in range(1, len(hud_cls) + 1):
    if i == len(hud_cls) or hud_cls[i] != hud_cls[start]:
      runs.append((hud_cls[start], hud_t[start], hud_t[min(i, len(hud_t) - 1)]))
      start = i
  t109 = [t for t, _ in s109]
  b7v = [v for _, v in s109]

  def hold_rates(want_cls):
    out, agg_n, agg_chg = [], 0, 0
    for cls, a, z in runs:
      if cls != want_cls or z - a < MIN_HOLD_S * SEC:
        continue
      i0, i1 = bisect.bisect_left(t109, a), bisect.bisect_left(t109, z)
      seq = b7v[i0:i1]
      if len(seq) < MIN_HOLD_FRAMES:
        continue
      chg = sum(1 for p, q in zip(seq, seq[1:]) if p != q)
      out.append(chg / (len(seq) - 1))
      agg_n += len(seq) - 1
      agg_chg += chg
    mean_rate = sum(out) / len(out) if out else None
    pooled = agg_chg / agg_n if agg_n else None
    return out, mean_rate, pooled

  lead_hold_rates, lead_mean, lead_pooled = None, None, None
  c0_hold_rates, c0_mean, c0_pooled = None, None, None
  # lead holds: pool per class-2/3/4 separately, then average the class numbers
  lr_all, lp_all = [], []
  lpair, lchg = 0, 0
  for want in (2, 3, 4):
    rates, mean, pooled = hold_rates(want)
    if mean is not None:
      lr_all.append(mean)
      lp_all.append(pooled)
  lead_hold_rates = lr_all
  lead_mean = sum(lr_all) / len(lr_all) if lr_all else None
  lead_pooled = sum(lp_all) / len(lp_all) if lp_all else None
  _, c0_mean, c0_pooled = hold_rates(0)
  # class-0 pooled needs the (n, chg) counters too -> recompute lightweight
  c0_n = c0_chg = 0
  for cls, a, z in runs:
    if cls != 0 or z - a < MIN_HOLD_S * SEC:
      continue
    i0, i1 = bisect.bisect_left(t109, a), bisect.bisect_left(t109, z)
    seq = b7v[i0:i1]
    if len(seq) < MIN_HOLD_FRAMES:
      continue
    c0_n += len(seq) - 1
    c0_chg += sum(1 for p, q in zip(seq, seq[1:]) if p != q)
  c0_pooled = c0_chg / c0_n if c0_n else None

  # ---- 0x10A presence vs HUD lead ----
  pres_n = pres_agree = lead_n = lead_hit = nol_n = nol_hit = 0
  if hud_t:
    for t, p in s10a:
      c = nearest_cls(t, hud_t, hud_cls)
      if c is None:
        continue
      seen = p == 7
      lead = c >= 2 or c == 1
      pres_n += 1
      pres_agree += int(seen == lead)
      if lead:
        lead_n += 1
        lead_hit += int(seen)
      else:
        nol_n += 1
        nol_hit += int(seen)

  nz0 = sum(per_cls[0].values()) - per_cls[0][0]  # class-0 frames with b7>0
  return {
    "seg": os.path.basename(os.path.dirname(f)),
    "err": err,
    "hud_n": len(hud_t), "hud_lead_frames": hud_lead_frames,
    "n109": len(s109), "n109_classified": N,
    "b7_nonzero": sum(c for v, c in b7_all.items() if v),
    "distinct_b7": len(b7_all),
    "distinct_b7_nonzero": len([v for v in b7_all if v]),
    "H_b7_given_bar": H_given_bar,
    "c0_nonzero_frames": nz0, "c0_frames": sum(per_cls[0].values()),
    "lead_hold_mean_rate": lead_mean, "lead_hold_pooled_rate": lead_pooled,
    "lead_hold_classes": len(lr_all),
    "c0_hold_mean_rate": c0_mean, "c0_hold_pooled_rate": c0_pooled,
    "n10a_classified": pres_n, "pres_agree": pres_agree,
    "pres_lead_n": lead_n, "pres_lead_hit": lead_hit,
    "pres_nolead_n": nol_n, "pres_nolead_hit": nol_hit,
    "b7_hist": {str(c): dict(per_cls[c]) for c in CLASSES if per_cls[c]},
  }


def qstat(say, vals, label, fmt="{:.3f}"):
  vals = sorted(v for v in vals if v is not None)
  if not vals:
    say(f"  {label}: (no qualifying segments)")
    return None
  n = len(vals)
  med = vals[n // 2] if n % 2 else (vals[n // 2 - 1] + vals[n // 2]) / 2
  out = {"n": n, "min": vals[0], "p25": vals[n // 4], "median": med,
         "p75": vals[3 * n // 4], "max": vals[-1],
         "mean": sum(vals) / n}
  say(f"  {label}: n={n} min={fmt.format(vals[0])} p25={fmt.format(vals[n//4])} "
      f"med={fmt.format(med)} p75={fmt.format(vals[3*n//4])} max={fmt.format(vals[-1])} "
      f"mean={fmt.format(out['mean'])}")
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("dir")
  ap.add_argument("--workers", type=int, default=10)
  ap.add_argument("--out-txt", default="/tmp/radar_415_diversity.txt")
  ap.add_argument("--json", default="/tmp/radar_415_diversity.json")
  a = ap.parse_args()

  class T:
    def __init__(self):
      self.f = open(a.out_txt, "w")
    def __call__(self, *args):
      s = " ".join(str(x) for x in args)
      print(s)
      self.f.write(s + "\n")
      self.f.flush()
  say = T()

  segs = sorted(glob.glob(f"{a.dir}/*/rlog.zst"))
  say(f"# radar_415_diversity: segments found = {len(segs)}, workers = {a.workers}")
  with Pool(a.workers) as p:
    results = p.map(seg_analyze, segs, chunksize=2)
  say(f"# segments analyzed = {len(results)}  with parse error (partial data kept) = "
      f"{sum(1 for r in results if r['err'])}")

  # ---------------- global b7 diversity ----------------
  g_all = collections.Counter()
  g_cls = {c: collections.Counter() for c in CLASSES}
  for r in results:
    for c, hist in r["b7_hist"].items():
      for v, n in hist.items():
        g_cls[int(c)][int(v)] += n
        g_all[int(v)] += n
  tot_f = sum(g_all.values())
  say("\n## (1) 0x109 b7 global diversity (echo-from-bars predicts ~3-6 levels)")
  say(f"  classified frames total = {tot_f}")
  say(f"  distinct b7 GLOBAL = {len(g_all)} (nonzero distinct = {len([v for v in g_all if v])})")
  for c in CLASSES:
    cc = g_cls[c]
    n = sum(cc.values())
    if not n:
      continue
    say(f"  class {c}: n={n} distinct={len(cc)} nonzero_share="
        f"{sum(v2 for v2, n2 in ((v, n2) for v, n2 in cc.items() if v))/n:6.1%} "
        f"top5_share={sum(n2 for _, n2 in cc.most_common(5))/n:6.1%}")

  # ---------------- per-segment distributions ----------------
  say("\n## (2) per-segment distributions")
  agg = {}
  agg["H_b7_given_bar"] = qstat(say, [r["H_b7_given_bar"] for r in results],
      f"H(b7|bar) bits over segments with >= {MIN_ENTROPY_FRAMES} classified 0x109 frames")
  agg["lead_hold_rate"] = qstat(say, [r["lead_hold_mean_rate"] for r in results],
      "lead-class hold(>=2s) b7 frame-to-frame change rate (mean per seg)", fmt="{:.4%}")
  agg["c0_hold_rate"] = qstat(say, [r["c0_hold_mean_rate"] for r in results],
      "class-0 hold(>=2s) b7 change rate (mean per seg)", fmt="{:.4%}")
  pres_rate = [r["pres_agree"] / r["n10a_classified"] for r in results
               if r["n10a_classified"] >= MIN_10A_FRAMES]
  agg["pres_agree"] = qstat(say, pres_rate,
      f"0x10A b3[7:5]==111 presence agreement with lead (>= {MIN_10A_FRAMES} cls'd frames)",
      fmt="{:.2%}")
  sens = [r["pres_lead_hit"] / r["pres_lead_n"] for r in results if r["pres_lead_n"] >= 30]
  fps = [r["pres_nolead_hit"] / r["pres_nolead_n"] for r in results if r["pres_nolead_n"] >= 30]
  agg["pres_sens"] = qstat(say, sens, "presence==111 rate during lead frames", fmt="{:.2%}")
  agg["pres_fp"] = qstat(say, fps, "presence==111 rate during no-lead frames", fmt="{:.2%}")

  # pooled rates across every qualifying hold in the whole archive
  lp = [r["lead_hold_pooled_rate"] for r in results if r["lead_hold_pooled_rate"] is not None]
  cp = [r["c0_hold_pooled_rate"] for r in results if r["c0_hold_pooled_rate"] is not None]
  say(f"  segments with >=1 lead hold(>=2s): {len(lp)} / {len(results)}; "
      f"with >=1 class-0 hold: {len(cp)}")

  # ---------------- the direct question: nonzero/diverse only in follow segs? ----------------
  say("\n## (3) b7 nonzero/diversity vs follow presence, per segment")
  lead_segs = [r for r in results if r["hud_lead_frames"] >= 10]
  nol_segs = [r for r in results if r["hud_lead_frames"] == 0]
  amb_segs = [r for r in results if 0 < r["hud_lead_frames"] < 10]
  say(f"  follow segments (HUD HasLead frames >= 10): {len(lead_segs)}")
  nz_in_lead = [r for r in lead_segs if r["b7_nonzero"] > 0]
  say(f"    of which b7 nonzero > 0: {len(nz_in_lead)} "
      f"({len(nz_in_lead)/max(len(lead_segs),1):.1%})")
  d_lead = sorted(r["distinct_b7"] for r in lead_segs)
  if d_lead:
    say(f"    distinct b7 per follow seg: med={d_lead[len(d_lead)//2]} "
        f"p25={d_lead[len(d_lead)//4]} p75={d_lead[3*len(d_lead)//4]} max={d_lead[-1]}")
  nz_in_nol = [r for r in nol_segs if r["b7_nonzero"] > 0]
  tot_nol_nz = sum(r["b7_nonzero"] for r in nol_segs)
  tot_nol_f = sum(r["n109"] for r in nol_segs)
  say(f"  NO-follow segments (HUD HasLead frames == 0): {len(nol_segs)}")
  say(f"    of which b7 nonzero > 0: {len(nz_in_nol)} "
      f"({len(nz_in_nol)/max(len(nol_segs),1):.1%})  "
      f"nonzero b7 frames = {tot_nol_nz}/{tot_nol_f} = {tot_nol_nz/max(tot_nol_f,1):.3%}")
  for r in nz_in_nol[:20]:
    say(f"      !! {r['seg']} b7_nonzero={r['b7_nonzero']}/{r['n109']} "
        f"distinct={r['distinct_b7']} hud_n={r['hud_n']}")
  tot_lead_nz = sum(r["b7_nonzero"] for r in lead_segs)
  tot_lead_f = sum(r["n109"] for r in lead_segs)
  say(f"  follow segments: nonzero b7 frames = {tot_lead_nz}/{tot_lead_f} = "
      f"{tot_lead_nz/max(tot_lead_f,1):.1%}")
  say(f"  ambiguous segments (0 < HasLead frames < 10): {len(amb_segs)}, "
      f"nonzero b7 frames = {sum(r['b7_nonzero'] for r in amb_segs)}/"
      f"{sum(r['n109'] for r in amb_segs)}")

  # class-0 residual within follow segments (capture/lose transients)
  c0f = sum(r["c0_frames"] for r in results)
  c0nz = sum(r["c0_nonzero_frames"] for r in results)
  say(f"  class-0 (no-lead window) frames archive-wide: {c0f}, "
      f"b7 nonzero residual = {c0nz} = {c0nz/max(c0f,1):.2%} "
      f"(echo model predicts ~0%; prior hour_logs number 3.7% transient)")

  # ---------------- verdict ----------------
  say("\n## (4) verdict")
  hq = agg.get("H_b7_given_bar")
  lq = agg.get("lead_hold_rate")
  cq = agg.get("c0_hold_rate")
  pq = agg.get("pres_agree")
  say(f"  H(b7|bar) archive median {hq['median']:.3f} bits (n={hq['n']} segs); "
      f"quantised echo-from-3-4-bars predicts ~0 bits -> "
      f"{'REFUTED (b7 carries info beyond the bar)' if hq['median'] > 0.5 else 'consistent with echo'}")
  if lq and cq:
    say(f"  hold change rate: lead med {lq['median']:.2%} vs class-0 med {cq['median']:.4%} "
        f"-> b7 moves inside constant-bar holds: "
        f"{'YES' if lq['median'] > 20 * max(cq['median'], 1e-9) else 'NO'}")
  if pq:
    say(f"  0x10A presence agreement med {pq['median']:.1%}; "
        f"sens(P(111|lead)) med {agg['pres_sens']['median']:.1%}, "
        f"fp(P(111|no-lead)) med {agg['pres_fp']['median']:.1%}")
  nlead_bad = len(nz_in_nol)
  say(f"  b7 nonzero only in follow segments: {len(nol_segs) - nlead_bad}/{len(nol_segs)} "
      f"no-follow segments fully zero ({(len(nol_segs)-nlead_bad)/max(len(nol_segs),1):.1%}); "
      f"archive-wide nonzero share outside follow segs = "
      f"{tot_nol_nz/max(tot_lead_nz + tot_nol_nz + sum(r['b7_nonzero'] for r in amb_segs),1):.2%}")

  # ---------------- dump json ----------------
  per_seg = [{k: v for k, v in r.items() if k != "b7_hist"} for r in results]
  out = {
    "segments": len(results),
    "segments_with_parse_error": sum(1 for r in results if r["err"]),
    "global": {"classified_109_frames": tot_f, "distinct_b7": len(g_all),
               "per_class": {str(c): {"n": sum(g_cls[c].values()),
                                      "distinct": len(g_cls[c]),
                                      "hist": {str(v): n for v, n in g_cls[c].items()}}
                             for c in CLASSES if g_cls[c]}},
    "aggregate": agg,
    "follow_cross": {
      "follow_segs": len(lead_segs), "follow_segs_b7_nonzero": len(nz_in_lead),
      "nofollow_segs": len(nol_segs), "nofollow_segs_b7_nonzero": nlead_bad,
      "nofollow_nonzero_frames": tot_nol_nz, "nofollow_frames": tot_nol_f,
      "follow_nonzero_frames": tot_lead_nz, "follow_frames": tot_lead_f,
      "ambiguous_segs": len(amb_segs),
      "class0_residual_frames": c0nz, "class0_frames": c0f},
    "per_segment": per_seg,
    "errs": {r["seg"]: r["err"] for r in results if r["err"]},
  }
  with open(a.json, "w") as fp:
    json.dump(out, fp)
  say(f"\n# json -> {a.json}   txt -> {a.out_txt}")
  say(f"# distinct b7 values archive-wide: {sorted(g_all)}")
  say("# DONE")


if __name__ == "__main__":
  main()
