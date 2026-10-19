"""Event-locked decode of the 67ms slot table: per approach-window regressions.

Window definition (physics, like radar_stop2lead.py): on the 0x109 40ms grid, a run
where b7 descends >= 6 counts over 1.5-40 s with |vEgo| > 0.3 - the car is closing a
lead (moving or stopped). For every candidate slot field (pair lo/hi frame x bytes
{b2b3,b4b5,b6b7} x {LE,BE} u16), resampled onto the window:
  r_win  = corr(field, b7)                       # tracks the lead?
  rate   = mean slope(field) in counts/s
  unit   = rate / (-mean vEgo) in counts/(m/s)   # scale if it IS lead distance
  rv     = corr(rate_series, -vEgo)              # rate consistency (distance-like)
Aggregate over events: how often does each candidate behave like a distance field,
and with what unit. A true per-object X-distance must show unit ~10 counts/(m/s)
(dm) or ~3.3 (feet*3) etc CONSISTENTLY across segments - session confounds cannot.

Usage:
  .venv/bin/python tools/byd/radar_slot67_events.py [--min-events 30] [--top 40]
"""
import sys, pickle, argparse, collections
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np

BLOCK_B = [a for a in range(0x340, 0x34B) if a not in (0x342, 0x345, 0x348)]
PAIRS = [(BLOCK_B[2*i], BLOCK_B[2*i+1]) for i in range(4)]
for base, n in ((0x380, 24), (0x3C8, 28), (0x3E4, 28)):
  addrs = list(range(base, base + n))
  PAIRS += [(addrs[2*i], addrs[2*i+1]) for i in range(n // 2)]
ALL_ADDRS = sorted({a for p in PAIRS for a in p if a is not None} | {0x301})
BYTES = [(2, 3), (4, 5), (6, 7)]


def cands(d):
  out = {}
  for j, jj in BYTES:
    lo = d[:, j].astype(np.float64); hi = d[:, jj].astype(np.float64)
    le = lo + hi * 256.0
    be = hi + lo * 256.0
    out[f"b{j}LE"] = le
    out[f"b{j}BE"] = be
    out[f"b{j}BE_s"] = np.where(be > 32768, be - 65536, be)
    out[f"b{j}LE_s"] = np.where(le > 32768, le - 65536, le)
  return out


def resample(t_src, v_src, t_dst):
  idx = np.clip(np.searchsorted(t_src, t_dst), 1, len(t_src) - 1)
  pick = np.where(t_dst - t_src[idx-1] <= t_src[idx] - t_dst, idx-1, idx)
  return v_src[pick]


def find_events(t109, b7, v, pres):
  """descending-b7 runs; return list of (i0, i1) inclusive."""
  ev = []
  lead = (pres == 7) & (b7 > 0) & (b7 <= 245) & ~np.isnan(v)
  i = 0
  n = len(t109)
  while i < n:
    if lead[i]:
      j = i
      t0 = t109[i]
      while j + 1 < n and lead[j+1] and t109[j+1] - t0 < 40e9:
        j += 1
      # within [i,j] find monotone-ish descending subwindows
      k = i
      while k < j:
        m = min(j, k + 750)
        seg = b7[k:m+1]
        drop = seg[0] - seg[-1]
        dt = (t109[m] - t109[k]) / 1e9
        if m - k >= 40 and drop >= 6 and dt >= 1.5 and np.nanmean(np.abs(v[k:m+1])) > 0.3:
          ev.append((k, m))
          k = m + 20
        else:
          k += 40
      i = j + 1
      t0 = t109[min(j+1, n-1)]
    else:
      i += 1
  return ev


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--min-events", type=int, default=30)
  ap.add_argument("--top", type=int, default=40)
  ap.add_argument("--max-segs", type=int, default=0)
  a = ap.parse_args()
  S67 = pickle.load(open("/tmp/slot67.pkl", "rb"))
  S10 = pickle.load(open("/tmp/seg1005.pkl", "rb"))
  agg = collections.defaultdict(lambda: {"r": [], "unit": [], "rv": []})
  n_ev_total = 0
  segs = sorted(set(S67) & set(S10))
  if a.max_segs:
    segs = segs[:a.max_segs]
  for name in segs:
    s, k = S67[name], S10[name]
    t109 = k["109_t"].astype(np.float64)
    if len(t109) < 200:
      continue
    b7 = k["109_d"][:, 7].astype(np.float64)
    vp = resample(k["10a_t"].astype(np.float64), ((k["10a_d"][:, 3] >> 5) & 7).astype(np.float64), t109) \
        if len(k["10a_t"]) > 1 else np.zeros(len(t109))
    v = np.interp(t109, k["cs_t"].astype(np.float64), k["cs_v"].astype(np.float64))
    t0g = t109[0]
    for (i0, i1) in find_events(t109 - t0g, b7, v, vp):
      tw = t109[i0:i1+1]
      bw = b7[i0:i1+1]
      vw = v[i0:i1+1]
      if bw.std() < 1:
        continue
      n_ev_total += 1
      dur = (tw[-1] - tw[0]) / 1e9
      vb = -float(np.nanmean(vw))
      for lo, hi in PAIRS:
        for frame, addr in (("lo", lo), ("hi", hi)):
          if addr is None:
            continue
          ta = s[f"{addr:03x}_t"].astype(np.float64)
          da = s[f"{addr:03x}_d"]
          if len(ta) < 30:
            continue
          for cn, cv in cands(da).items():
            fs = resample(ta, cv, tw)
            if np.std(fs) < 1:
              continue
            r = np.corrcoef(fs, bw)[0, 1]
            if abs(r) < 0.5:
              continue
            rate = np.polyfit((tw - tw[0]) / 1e9, fs, 1)[0]        # counts/s
            # rate consistency: per-frame diff vs -vEgo
            dfs = np.diff(fs) / (np.diff(tw) / 1e9)
            mask = ~np.isnan(dfs) & ~np.isnan(vw[1:])
            if mask.sum() < 20 or np.std(dfs[mask]) < 1e-6:
              continue
            rv = np.corrcoef(dfs[mask], vw[1:][mask])[0, 1]
            g = agg[f"{addr:03x}/{frame}/{cn}"]
            g["r"].append(r)
            g["unit"].append(rate / vb if vb != 0 else np.nan)      # counts per (m/s), positive when field shrinks as we close
            g["rv"].append(rv)

  print(f"# events total: {n_ev_total}")
  rows = []
  for kk, g in agg.items():
    r = np.array(g["r"]); un = np.array(g["unit"]); rv = np.array(g["rv"])
    rows.append((kk, len(r), np.median(np.abs(r)), (r > 0.8).mean(),
                 np.median(np.abs(un)) if len(un) else np.nan,
                 (np.abs(un) > 0.3).mean() if len(un) else 0,
                 np.median(rv)))
  rows.sort(key=lambda x: -x[3])
  print(f" {'slot/frame/field':22s} {'n_ev':>5} {'r_med':>6} {'f_r80':>6} {'unit_med':>9} {'f|u|>.3':>8} {'rv_med':>7}")
  for kk, n, rm, fr, um, fu, rv in rows[:a.top]:
    if n < a.min_events:
      continue
    print(f" {kk:22s} {n:5d} {rm:6.3f} {fr:6.2f} {um:9.1f} {fu:8.2f} {rv:+7.3f}")


if __name__ == "__main__":
  main()
