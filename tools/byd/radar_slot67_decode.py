"""Supervised decode of the bus1 67ms slot table -> which (addr, bytes) are object X/Y/V.

Caches: /tmp/slot67.pkl (family frames, radar_slot67_extract.py)
        /tmp/seg1005.pkl (0x109 b7 / 0x10A presence / 0x32D class / vEgo, radar_seg_extract.py)

Per segment, for every pair-frame field candidate (LE/BE u16 over b2b3/b4b5/b6b7):
  r_lead   = corr(field, 0x109 b7) on lead windows (pres==7 & 0<b7<=245)
             -> distance-like if high (slot tracks the fused lead)
  r_v      = corr(field, vEgo) on stationary-lead subwindows (b7 flat, vEgo>0.5)
             -> RELV-like if high  (relV ~ +/-vEgo against a stopped object)
  r_dot_v  = corr(d field/dt, -vEgo) on lead windows
             -> the same distance field's rate consistency (physical anchor)
Aggregates over segments: #segs with |r|>0.6, median r, median scale.

Usage:
  .venv/bin/python tools/byd/radar_slot67_decode.py [--segs N] [--top 30]
"""
import sys, os, pickle, argparse, collections
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np

BLOCK_B = [a for a in range(0x340, 0x34B) if a not in (0x342, 0x345, 0x348)]
PAIRS = [(BLOCK_B[2*i], BLOCK_B[2*i+1]) for i in range(4)]
for base, n in ((0x380, 12), (0x3C8, 14), (0x3E4, 14)):
  addrs = list(range(base, base + 2 * n))
  PAIRS += [(addrs[2*i], addrs[2*i+1]) for i in range(n)]
PAIRS.append((0x301, None))  # header, treated as single frame
FIELDFRAMES = {0: ("b2b3", 2), 1: ("b4b5", 4), 2: ("b6b7", 6)}   # (name, start byte)


def candidates(d):
  """d[N,8] -> dict cand_name -> float array"""
  out = {}
  for j0 in (2, 4, 6):
    nm = {2: "23", 4: "45", 6: "67"}[j0]
    lo = d[:, j0].astype(np.float64)
    hi = d[:, j0 + 1].astype(np.float64)
    out[f"{nm}LE"] = lo + hi * 256.0
    out[f"{nm}BE"] = hi + lo * 256.0
    # i16 variants: reuse unsigned for corr (monotone equiv mostly), signed shift:
    out[f"{nm}LE_s"] = np.where(out[f"{nm}LE"] > 32768, out[f"{nm}LE"] - 65536, out[f"{nm}LE"])
    out[f"{nm}BE_s"] = np.where(out[f"{nm}BE"] > 32768, out[f"{nm}BE"] - 65536, out[f"{nm}BE"])
  return out


def resample(t_src, v_src, t_dst, maxdt=40e6):
  idx = np.clip(np.searchsorted(t_src, t_dst), 1, len(t_src) - 1)
  pick = np.where(t_dst - t_src[idx - 1] <= t_src[idx] - t_dst, idx - 1, idx)
  ok = np.abs(t_src[pick] - t_dst) <= maxdt
  v = np.full(len(t_dst), np.nan)
  v[ok] = v_src[pick[ok]]
  return v


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--segs", type=int, default=0)
  ap.add_argument("--top", type=int, default=30)
  a = ap.parse_args()
  S67 = pickle.load(open("/tmp/slot67.pkl", "rb"))
  S10 = pickle.load(open("/tmp/seg1005.pkl", "rb"))
  # rank by lead-frame count so --segs picks content-rich segments
  def lead_count(name):
    k = S10[name]
    if not len(k["109_t"]):
      return 0
    b7 = k["109_d"][:, 7]
    return int(((b7 > 0) & (b7 <= 245)).sum())
  seg_names = sorted(set(S67) & set(S10), key=lambda n: -lead_count(n))
  if a.segs:
    seg_names = seg_names[:a.segs]
  print(f"# {len(seg_names)} segments joined (top-lead), lead_frames={sum(lead_count(n) for n in seg_names)}")

  agg = collections.defaultdict(lambda: {"r_lead": [], "r_still": [], "r_dot": [], "scale": []})
  f_pass = f_lead = f_flat = 0
  for name in seg_names:
    s, k = S67[name], S10[name]
    t109 = k["109_t"].astype(np.float64)
    b7 = k["109_d"][:, 7].astype(np.float64)
    pres = (k["10a_d"][:, 3] >> 5) & 7 if len(k["10a_t"]) else np.zeros(len(t109))
    vp = resample(k["10a_t"].astype(np.float64), ((k["10a_d"][:, 3] >> 5) & 7).astype(float), t109) \
        if len(k["10a_t"]) else np.zeros(len(t109))
    v = resample(k["cs_t"].astype(np.float64), k["cs_v"].astype(np.float64), t109)
    lead = (vp == 7) & (b7 > 0) & (b7 <= 245)
    if lead.sum() < 100:
      continue
    # stationary-lead subwindow: b7 nearly constant over +-2s while moving
    flat = np.zeros(len(t109), bool)
    for i in range(10, len(t109) - 10):
      if lead[i] and np.ptp(b7[i-10:i+10]) <= 1 and np.nanmean(v[i-10:i+10]) > 0.8:
        flat[i] = True
    if flat.sum() < 50:
      continue
    for lo, hi in PAIRS:
      tl = s[f"{lo:03x}_t"].astype(np.float64)
      dl = s[f"{lo:03x}_d"]
      if len(tl) < 100:
        continue
      for frame, tf, df in (("lo", tl, dl), ("hi", s[f"{hi:03x}_t"].astype(np.float64) if hi else tl,
                            s[f"{hi:03x}_d"] if hi else dl)):
        if len(tf) < 100:
          continue
        cs = candidates(df)
        for cn, cv in cs.items():
          vs = resample(tf, cv, t109)
          okl = lead & ~np.isnan(vs)
          if okl.sum() < 100 or np.std(vs[okl]) < 1:
            continue
          r1 = np.corrcoef(vs[okl], b7[okl])[0, 1]
          # scale: least squares field = a*b7 + c on lead window
          if np.std(b7[okl]) > 1 and np.std(vs[okl]) > 1:
            A = np.polyfit(b7[okl], vs[okl], 1)
            agg[f"{lo:03x}/{frame}/{cn}"]["scale"].append((float(A[0]), float(A[1])))
          oks = flat & ~np.isnan(vs)
          r2 = np.corrcoef(vs[oks], v[oks])[0, 1] if oks.sum() >= 50 and np.std(vs[oks]) > 1 else np.nan
          # rate consistency: d field / dt vs -vEgo (stationary lead physics)
          dt = np.diff(tf) / 1e9
          dv = np.diff(cv) / dt
          td = tf[:-1]
          vd = resample(td, dv, t109)
          okr = lead & ~np.isnan(vd) & (np.abs(v) > 1.0)
          r3 = np.corrcoef(vd[okr], -v[okr])[0, 1] if okr.sum() >= 100 and np.std(vd[okr]) > 1e-6 else np.nan
          g = agg[f"{lo:03x}/{frame}/{cn}"]
          g["r_lead"].append(r1)
          if not np.isnan(r2):
            g["r_still"].append(r2)
          if not np.isnan(r3):
            g["r_dot"].append(r3)

  def summ(k):
    g = agg[k]
    r1 = np.array(g["r_lead"])
    r2 = np.array(g["r_still"])
    r3 = np.array(g["r_dot"])
    sc = np.array([x for x, _ in g["scale"]]) if g["scale"] else np.array([np.nan])
    ic = np.array([c for _, c in g["scale"]]) if g["scale"] else np.array([np.nan])
    return (len(r1), np.median(np.abs(r1)), (np.abs(r1) > 0.7).mean(),
            np.median(r2) if len(r2) else 0,
            np.median(r3) if len(r3) else 0,
            np.median(sc) if len(sc) else np.nan, np.median(ic) if len(ic) else np.nan)

  rows = sorted(agg, key=lambda k: -summ(k)[2])
  print("\n== ranked by frac(|r_lead|>0.7): which slot bytes track 0x109 b7 ==")
  print(f" {'slot/frame/fld':22s} {'nseg':>4} {'r_med':>6} {'f70':>4} {'r_still':>7} {'r_dotv':>7} {'slope':>8} {'inter':>7}")
  for k in rows[:a.top]:
    n, m, f70, m2, m3, sc, ic = summ(k)
    if n < 15:
      continue
    print(f" {k:22s} {n:4d} {m:6.3f} {f70:4.2f} {m2:+7.3f} {m3:+7.3f} {sc:8.2f} {ic:7.1f}")


if __name__ == "__main__":
  main()
