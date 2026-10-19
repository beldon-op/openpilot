"""T8 design stats: how far does 0x109 b7 LEAD the HUD 0x32D gear flip (审2->全量).

审2 measured 10 flips by eye (8/10 b7 leaves the old plateau early). T8 will
use b7 (or presence) as an upstream continuous signal for BSD clear-window /
starting-abort - the design needs archive-wide numbers:
  A) lead-lead flips (c2/c3/c4 <->): plateau medians old/new, time at which
     b7 crosses the midpoint toward new vs the flip instant (lead>0 = b7
     crossed BEFORE the gear flipped), and fraction-of-journey-done at flip.
     u-free companion: same in normalized r=b7/vEgo (gear is a TIME-GAP band).
  B) acquisition (class 0 -> lead): first b7>=9 / first presence==7 before
     HasLead rises => radar-visible-before-HUD lead (the --142 mechanism,
     now quantized archive-wide).
  C) loss (lead -> 0): lag of b7==0 behind HasLead drop (downstream tail).
  D) false-cross rate inside constant holds: with a single midpoint threshold
     in r=b7/vEgo, share of frames in class c's hold that would sit on the
     neighbor's side -> hysteresis sizing for T8's window filter.

Usage: .venv/bin/python tools/byd/radar_t8_lead.py --out /tmp/radar_t8.txt
"""
import sys, argparse, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import radar_seglib as L

MIN_PLATEAU_FRAMES = 25   # ~1s of 0x109 inside the old/new plateau window
MIN_DLT = 4               # counts, ignore plateau pairs too close to be informative


def med_in(t, arr, a_ns, b_ns, mask_valid):
  k0, k1 = np.searchsorted(t, [a_ns, b_ns])
  seg = arr[k0:k1]
  ok = mask_valid[k0:k1]
  seg = seg[ok]
  return (float(np.median(seg)), len(seg)) if len(seg) else (None, 0)


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--pkl", default="/tmp/seg1005.pkl")
  ap.add_argument("--out", default="/tmp/radar_t8.txt")
  a = ap.parse_args()
  cache = L.load(a.pkl)
  lines = []
  def say(*x):
    s = " ".join(str(v) for v in x)
    print(s, flush=True)
    lines.append(s)

  leads = collections.defaultdict(list)      # (old,new) -> lead seconds
  fracs = collections.defaultdict(list)      # (old,new) -> journey fraction at flip
  acq_leads, acq_pres_leads, loss_lags = [], [], []
  rmed = {c: collections.defaultdict(list) for c in (2, 3, 4)}  # per-seg median r
  falsec = {c: [0, 0] for c in (2, 3, 4)}    # c -> [cross_frames, hold_frames]

  for name, s in cache.items():
    t = s["109_t"]; d = s["109_d"]
    if len(t) < 300 or len(s["hud_t"]) < 50:
      continue
    b7 = d[:, 7].astype(np.float64)
    ve = L.vego_at(t, s)
    pr = L.pres_at(t, s)
    valid = np.isfinite(ve)
    rv = np.where(valid & (ve > 2), b7 / np.where(ve > 0.1, ve, np.nan), np.nan)
    ht, hc = s["hud_t"], s["hud_cls"]
    runs = []
    i = 0
    while i < len(hc):
      j = i
      while j < len(hc) and hc[j] == hc[i]:
        j += 1
      runs.append((int(hc[i]), ht[i], ht[j - 1], i, j - 1))
      i = j
    for ri, (c, a_t, z_t, _, _) in enumerate(runs[:-1]):
      c2, a2_t, z2_t = runs[ri + 1][0], runs[ri + 1][1], runs[ri + 1][2]
      t_flip = a2_t
      if c in (2, 3, 4) and c2 in (2, 3, 4) and (z_t - a_t) >= 3e9 and (z2_t - a2_t) >= 3e9:
        p_old, n_old = med_in(t, b7, a_t, z_t - 0.5e9, valid)
        p_new, n_new = med_in(t, b7, t_flip + 0.5e9, z2_t, valid)
        if p_old and p_new and n_old >= MIN_PLATEAU_FRAMES and n_new >= MIN_PLATEAU_FRAMES \
           and abs(p_new - p_old) >= MIN_DLT:
          mid = (p_old + p_new) / 2
          k0, k1 = np.searchsorted(t, [t_flip - 8e9, t_flip])
          w = b7[k0:k1]; wt = t[k0:k1]; wv = valid[k0:k1]
          w = w[wv]; wt = wt[wv]
          if len(w):
            if p_new > p_old:
              crossed = np.where(w >= mid)[0]
            else:
              crossed = np.where(w <= mid)[0]
            if len(crossed):
              leads[(c, c2)].append((t_flip - wt[crossed[0]]) / L.SEC)
            # journey fraction done at flip time
            kf = np.searchsorted(t, t_flip)
            b_flip = float(np.median(b7[max(0, kf - 3):kf + 3]))
            fracs[(c, c2)].append((b_flip - p_old) / (p_new - p_old))
            # r-space companion
            r_old = med_in(t, rv, a_t, z_t - 0.5e9, np.isfinite(rv))[0]
            r_new = med_in(t, rv, t_flip + 0.5e9, z2_t, np.isfinite(rv))[0]
            if r_old and r_new and abs(r_new - r_old) > 0.2:
              rmed[c][">"].append(r_old)   # plateau r of class c (old side)
              rmed[c2]["<"].append(r_new)
      # acquisition / loss vs presence & b7
      if c == 0 and c2 in (2, 3, 4):
        k0, k1 = np.searchsorted(t, [t_flip - 10e9, t_flip])
        w = b7[k0:k1]; wp = pr[k0:k1]; wt = t[k0:k1]
        hit = np.where(w >= 9)[0]
        if len(hit):
          acq_leads.append((t_flip - wt[hit[0]]) / L.SEC)
        ph = np.where(wp == 7)[0]
        if len(ph):
          acq_pres_leads.append((t_flip - wt[ph[0]]) / L.SEC)
      if c in (2, 3, 4) and c2 == 0:
        k0, k1 = np.searchsorted(t, [t_flip, min(t_flip + 10e9, t[-1])])
        w = b7[k0:k1]
        hit = np.where(w < 1)[0]
        if len(hit):
          loss_lags.append((t[k0 + hit[0]] - t_flip) / L.SEC)
      # (D) false-cross inside holds >= 6s
      if c in (2, 3, 4) and (z_t - a_t) >= 6e9:
        r_other = None
        # neighbors' boundary in r-space from global medians (computed after pass 1
        # would be cleaner, but per-run compare against pooled medians filled so far
        # -> two-pass: stash frames counts only in pass 1) -- here pass 2 below.
    # pass 2 for (D) done in aggregate stage; store hold r sequences
    s["_holds"] = [(c, a_t, z_t) for c, a_t, z_t, _, _ in runs if c in (2, 3, 4)
                   and z_t - a_t >= 6e9]
    s["_rv"] = rv

  # (D) second pass with pooled r boundary table
  gb = {}
  for c in (2, 3, 4):
    vals = rmed[c][">"] + rmed[c]["<"]
    gb[c] = float(np.median(vals)) if vals else None
  say("== (pre) pooled r=b7/vEgo plateau medians per class:", gb)
  for name, s in cache.items():
    rv = s.pop("_rv", None)
    for c, a_t, z_t in s.pop("_holds", []):
      if rv is None or gb.get(c) is None:
        continue
      st = s["109_t"]
      k0, k1 = np.searchsorted(st, a_t), np.searchsorted(st, z_t)
      w = rv[k0:k1]
      w = w[np.isfinite(w)]
      if len(w) < 50:
        continue
      # boundary toward higher class = midpoint between class medians
      up = (gb[c] + gb[c + 1]) / 2 if c < 4 and gb.get(c + 1) else None
      dn = (gb[c] + gb[c - 1]) / 2 if c > 2 and gb.get(c - 1) else None
      falsec[c][1] += len(w)
      if up:
        falsec[c][0] += int((w > up).sum())
      if dn:
        falsec[c][0] += int((w < dn).sum())

  def pctl(v):
    v = sorted(v)
    if not v:
      return "(none)"
    n = len(v)
    return (f"n={n} p10={v[int(n*.1)]:.2f} med={v[n//2]:.2f} "
            f"p90={v[int(n*.9)]:.2f} share_neg(lag)={sum(1 for x in v if x < 0)/n:.0%}")

  say("\n== (A) lead-lead flips: b7 midpoint crossing LEAD time vs gear flip (s, + = b7 first)")
  for pair in sorted(leads):
    say(f"  {pair[0]}->{pair[1]}: {pctl(leads[pair])}  | frac-done-at-flip med="
        f"{np.median(fracs[pair]):.2f}")
  say("\n== (B) acquisition: first b7>=9 before HasLead (s)")
  say(f"  b7 lead:  {pctl(acq_leads)}")
  say(f"  pres lead: {pctl(acq_pres_leads)}")
  say("\n== (C) loss: b7==0 lag after HasLead drop (s)")
  say(f"  {pctl(loss_lags)}")
  say("\n== (D) false-cross rate inside holds w/ single midpoint threshold")
  for c in (2, 3, 4):
    n_f, n_t = falsec[c]
    say(f"  class {c}: {n_f}/{n_t} = {n_f/max(n_t,1):.2%} frames outside band")

  with open(a.out, "w") as fp:
    fp.write("\n".join(lines) + "\n")


if __name__ == "__main__":
  main()
