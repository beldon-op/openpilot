"""HUD-echo adversarial checks on the cached 0x109/0x32D streams (stage 2).

Attack: "0x109 b7 is forward distance" vs "b7 is just the 0x32D HUD bar
re-broadcast onto bus1". If echo, b7 = quantise(bar): few distinct levels,
piecewise-constant while bar held, and cannot change before the bar.

a) diversity   - distinct b7 overall / per bar-class / within class-0,
                 top-value concentration, H(b7|class).
b) timing      - detailed +-1.5s 40ms-aligned timeline at the two named
                 transitions (f4_16@1305.18s, fa_2@266.69s, both 3->2) plus
                 lead/lag statistics (t_b7_reaches_new_regime - t_hud) over
                 every lead->lead transition in all 4 segments, and whether
                 b7 passes through intermediate values between plateaus.
c) hold-variance - for every HUD class hold >= 2.0s, how many distinct b7
                 values and what frame-to-frame change rate does b7 show.

Usage: .venv/bin/python tools/byd/radar_echo_checks.py /tmp/echo_cache.pkl
"""
import pickle, sys, math, collections, bisect

SEC = 1_000_000_000
CACHE = sys.argv[1] if len(sys.argv) > 1 else "/tmp/echo_cache.pkl"
data = pickle.load(open(CACHE, "rb"))

HUD_NAMED = {"f4_16": (1305.18, 3, 2), "fa_2": (266.69, 3, 2)}


def hud_cls_series(hud):
  """[(t, cls)] with cls = bar if HasLead else 0, one entry per 0x32D frame."""
  return [(t, (b if h else 0)) for t, h, b in hud]


def classify(t, hs_t, hs_c):
  i = bisect.bisect_right(hs_t, t) - 1
  return hs_c[i] if i >= 0 else None


def entropy(counter, total):
  return -sum((v / total) * math.log2(v / total) for v in counter.values() if v)


def main():
  # ---------------- (a) global diversity of 0x109 b7 ----------------
  print("## (a) 0x109 b7 value diversity (echo-from-bars predicts ~3-6 levels)")
  b7_all, per_cls_vals = collections.Counter(), collections.defaultdict(collections.Counter)
  for seg, d in sorted(data.items()):
    f109 = d["f109"]
    hs_t, hs_c = zip(*hud_cls_series(d["hud"])) if d["hud"] else ((), ())
    dv = collections.Counter(v for _, v in ((t, b[7]) for t, b in f109))
    n109 = len(f109)
    distinct = len(dv)
    top5 = sum(c for _, c in dv.most_common(5))
    nz = sum(c for v, c in dv.items() if v != 0)
    print(f"  {seg}: frames={n109} distinct_b7={distinct} "
          f"top5_share={top5/max(n109,1):6.1%} nonzero_share={nz/max(n109,1):6.1%} "
          f"values(<=25 shown)={sorted(dv)[:25] if distinct<=25 else '...'}")
    for t, b in f109:
      v = b[7]
      b7_all[v] += 1
      c = classify(t, hs_t, hs_c)
      if c is not None:
        per_cls_vals[c][v] += 1
    # 0x101 b7 same shape
    d101 = collections.Counter(b[7] for _, b in d["f101"])
    print(f"      0x101: frames={len(d['f101'])} distinct_b7={len(d101)} top5_share="
          f"{sum(c for _,c in d101.most_common(5))/max(len(d['f101']),1):6.1%}")
  tot = sum(b7_all.values())
  print(f"  GLOBAL: frames={tot} distinct_b7={len(b7_all)} "
        f"top5_share={sum(c for _, c in b7_all.most_common(5))/tot:6.1%}")
  joint = collections.Counter()
  cnt_cls = collections.Counter()
  for c, cv in per_cls_vals.items():
    for v, cnt_v in cv.items():
      joint[(c, v)] += cnt_v
      cnt_cls[c] += cnt_v
  hb_given_cls = sum((cnt_cls[c] / tot) * entropy(collections.Counter(
      {v: joint[(c, v)] for v in per_cls_vals[c]}), cnt_cls[c]) for c in per_cls_vals)
  h_cls = entropy(cnt_cls, tot)
  print(f"  H(b7|class)={hb_given_cls:.3f} bits, H(class)={h_cls:.3f} bits, "
        f"H(b7)={entropy(b7_all, tot):.3f} bits   [quantised echo => H(b7|class) ~ 0]")
  nz = sum(c for v, c in b7_all.items() if v != 0)
  print(f"  nonzero-only frames={nz} distinct={len([v for v in b7_all if v])} "
        f"top5_share={sum(c for v, c in b7_all.most_common(6) if v)/max(nz,1):6.1%} "
        f"median nonzero b7={sorted(v for v, c in b7_all.items() for _ in range(c))[nz//2]}")
  for c in sorted(per_cls_vals):
    cv = per_cls_vals[c]
    if not cv:
      continue
    print(f"    class {c}: n={sum(cv.values())} distinct={len(cv)} top10={cv.most_common(10)}")

  # ---------------- (c) variance of b7 while HUD class is HELD ----------------
  print("\n## (c) b7 activity within constant-bar holds >= 2.0s (echo => ~0 change)")
  agg = collections.defaultdict(lambda: [0, 0, [], []])  # cls -> [frames, changes, distinct list, hold secs]
  for seg, d in sorted(data.items()):
    hs = hud_cls_series(d["hud"])
    hs_t = [t for t, _ in hs]
    # maximal constant runs
    runs, start = [], 0
    for i in range(1, len(hs) + 1):
      if i == len(hs) or hs[i][1] != hs[start][1]:
        runs.append((hs[start][1], hs[start][0], hs[min(i, len(hs) - 1)][0]))
        start = i
    t109 = [t for t, _ in d["f109"]]
    b7 = [b[7] for _, b in d["f109"]]
    for cls, a, z in runs:
      if cls == 0 or z - a < 2.0 * SEC:
        continue
      i0, i1 = bisect.bisect_left(t109, a), bisect.bisect_left(t109, z)
      seq = b7[i0:i1]
      if len(seq) < 30:
        continue
      chg = sum(1 for p, q in zip(seq, seq[1:]) if p != q)
      agg[cls][0] += len(seq)
      agg[cls][1] += chg
      agg[cls][2].append(len(set(seq)))
      agg[cls][3].append((z - a) / SEC)
    # class-0 holds too (should be dead-flat if echo-of-bar with 0=off)
    for cls, a, z in runs:
      if cls != 0 or z - a < 2.0 * SEC:
        continue
      i0, i1 = bisect.bisect_left(t109, a), bisect.bisect_left(t109, z)
      seq = b7[i0:i1]
      if len(seq) < 30:
        continue
      agg[0][0] += len(seq)
      agg[0][1] += sum(1 for p, q in zip(seq, seq[1:]) if p != q)
      agg[0][2].append(len(set(seq)))
      agg[0][3].append((z - a) / SEC)
  for cls in sorted(agg):
    n, chg, dlist, dur = agg[cls]
    print(f"  class {cls}: holds={len(dlist)} frames={n} frame-to-frame change rate={chg/max(n-len(dlist),1):6.2%} "
          f"median distinct b7 per hold={sorted(dlist)[len(dlist)//2] if dlist else '-'} "
          f"max distinct={max(dlist) if dlist else '-'} median hold={sorted(dur)[len(dur)//2]:.1f}s")

  # ---------------- (b) transition lead/lag ----------------
  print("\n## (b) timing of b7 vs 0x32D at bar transitions (lead->lead, all 4 segs)")

  def med(x):
    return sorted(x)[len(x) // 2] if x else None

  lag_all = []
  inter_count = 0
  trans_total = 0
  for seg, d in sorted(data.items()):
    hs = hud_cls_series(d["hud"])
    f109 = d["f109"]
    transitions = []
    for i in range(1, len(hs)):
      pc, cc = hs[i - 1][1], hs[i][1]
      if pc != cc and pc != 0 and cc != 0:
        transitions.append((hs[i][0], pc, cc))
    for t_hud, pc, cc in transitions:
      trans_total += 1
      pre = [b[7] for t, b in f109 if t_hud - 2.0 * SEC <= t <= t_hud - 0.5 * SEC]
      post = [b[7] for t, b in f109 if t_hud + 0.5 * SEC <= t <= t_hud + 2.0 * SEC]
      v_old, v_new = med(pre), med(post)
      if v_old is None or v_new is None or v_old == v_new:
        continue
      window = [(t, b[7]) for t, b in f109 if t_hud - 2.5 * SEC <= t <= t_hud + 2.5 * SEC]
      # exit: first frame leaving a +-2 band around the old plateau (sustained 2 frames)
      t_exit = None
      for i in range(len(window) - 1):
        if all(abs(window[i + k][1] - v_old) > 2 for k in (0, 1)):
          t_exit = window[i][0]
          break
      if t_exit is None:
        continue
      lag = (t_exit - t_hud) / SEC
      lag_all.append((seg, t_hud, pc, cc, v_old, v_new, lag))
      band = max(2, abs(v_new - v_old) // 5)
      inter = [v for _, v in window if abs(v - v_old) > band and abs(v - v_new) > band]
      if inter:
        inter_count += 1
      print(f"    {seg} {pc}->{cc} @abs {t_hud/SEC:9.3f}s  plateaus b7 {v_old}->{v_new}  "
            f"b7-exit-HUD = {lag:+.3f}s  intermed={sorted(set(inter))[:8]}")
  neg = sum(1 for *_, l in lag_all if l < -0.06)
  zero = sum(1 for *_, l in lag_all if abs(l) <= 0.06)
  pos = sum(1 for *_, l in lag_all if l > 0.06)
  print(f"  transitions total={trans_total} measurable={len(lag_all)} "
        f"b7 LEADS(>60ms): {neg}  coincident(<=60ms,=1.5 tick): {zero}  b7 LAGS(>60ms): {pos}  "
        f"passed-through intermediates: {inter_count}/{len(lag_all)}")
  if lag_all:
    lags = sorted(l for *_, l in lag_all)
    print(f"  lag (t_b7_exit - t_hud) seconds: median={lags[len(lags)//2]:+.3f} "
          f"p25={lags[len(lags)//4]:+.3f} p75={lags[3*len(lags)//4]:+.3f} "
          f"min={lags[0]:+.3f} max={lags[-1]:+.3f}")

  # detailed timeline for the named transitions
  for seg, (target, p, c) in HUD_NAMED.items():
    d = data[seg]
    t0 = d["t0"]
    hs = hud_cls_series(d["hud"])
    hs_t = [x[0] for x in hs]
    hs_c = [x[1] for x in hs]
    # report timestamps are ABSOLUTE logMonoTime seconds (segments are ~72s long)
    cand = [(hs[i + 1][0], p, c) for i in range(len(hs) - 1)
            if hs[i][1] == p and hs[i + 1][1] == c and abs(hs[i + 1][0] / SEC - target) < 2.0]
    if not cand:
      print(f"\n  !! no {p}->{c} transition near abs {target}s in {seg}; closest class changes:")
      allp = [(hs[i][0] / SEC, hs[i - 1][1], hs[i][1]) for i in range(1, len(hs))
              if hs[i][1] != hs[i - 1][1]]
      print("     ", sorted(allp, key=lambda x: abs(x[0] - target))[:4])
      continue
    t_hud = min(cand, key=lambda x: abs(x[0] / SEC - target))[0]
    print(f"\n  ---- {seg}: {p}->{c} at abs {t_hud/SEC:.3f}s / rel {(t_hud-t0)/SEC:.3f}s (named {target}s) ----")
    print(f"     t-HUD   | 0x109 payload (b0..b7)   | b7  | 0x32D")
    t109 = [t for t, _ in d["f109"]]
    i0, i1 = bisect.bisect_left(t109, t_hud - 1.5 * SEC), bisect.bisect_left(t109, t_hud + 1.5 * SEC)
    prev_cls = None
    for k in range(i0, i1):
      t, b = d["f109"][k]
      cls_at = classify(t, hs_t, hs_c)
      j = bisect.bisect_right(hs_t, t) - 1
      hz = d["hud"][j] if j >= 0 else (0, 0, 0)
      tag = f"  <== HUD {prev_cls}->{cls_at}" if cls_at != prev_cls else ""
      print(f"     {(t - t_hud)/SEC:+7.3f} | {b.hex(' ')}  | {b[7]:3d} | {hz[1]},{hz[2]}{tag}")
      prev_cls = cls_at

  # period check: is 0x109 ticked with HUD or independently
  print("\n## side: frame periods (s)")
  for seg, d in sorted(data.items()):
    for name, seq in (("0x109", [t for t, _ in d["f109"]]), ("0x32D", [t for t, _, _ in d["hud"]])):
      dt = sorted((b - a) / SEC for a, b in zip(seq, seq[1:]))
      if dt:
        print(f"  {seg} {name}: median={dt[len(dt)//2]*1000:.1f}ms p90={dt[int(len(dt)*.9)]*1000:.1f}ms")


if __name__ == "__main__":
  main()
