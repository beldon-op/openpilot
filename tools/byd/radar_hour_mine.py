"""Supervised re-mine of hour_logs (4 engaged rlogs, 2026-09-28) for relV/vLead/distance.

Why this corpus + supervisor set (docs/byd-radar-slot67-2026-10-18.md §3 verdict re-test):
  - The relV death verdicts (0x10x 486-seg, slot67 325-window) supervised ONLY the
    closing speed -d(b7*u)/dt. In *steady following* that signal has ~zero variance,
    so an ABSOLUTE vLead field (ARS410-class radars send both vRel and vLead) is
    systematically invisible to it. hour_logs' 4 segments are exactly that regime
    (engaged following 10.6/10.5/14/2.3 min, 41-72 km/h).
  - New supervisors here, per active bus1 address x byte-field variant:
      r_dist   corr(f, b7)          on lead windows     -> distance family (re-verify)
      r_vlead  corr(f, vEgo)         on lead windows     -> vLead  (steady: vLead~=vEgo)
      r_hf     corr(f, hf_vEgo)      on lead windows     -> survives detrending, kills
                                                            counters/slow integrators
      r_still  corr(f, vEgo)         on STILL-lead wins  -> vRel: expect r ~= -1 there
      r_dot    corr(df/dt, -vEgo)    on still-lead wins  -> distance-rate consistency
  - lead gate = presence==7 AND 0<b7<=245 (docs/byd-radar-010x.md filter).

Usage:
  .venv/bin/python tools/byd/radar_hour_mine.py [--top 25]
Cache: /tmp/hour_bus1.pkl from radar_hour_census.py extract.
"""
import sys, pickle, argparse, collections

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np

SEC = 1_000_000_000


def variants(d):
  """d[N,8] -> {name: float array}: single bytes, LE/BE u16 pairs, i16s, 12-bit slots."""
  out = {}
  for j in range(8):
    out[f"b{j}"] = d[:, j].astype(np.float64)
  for j in range(7):
    lo = d[:, j].astype(np.float64)
    hi = d[:, j + 1].astype(np.float64)
    le = lo + hi * 256.0
    be = hi + lo * 256.0
    out[f"b{j}{j+1}LE"] = le
    out[f"b{j}{j+1}BE"] = be
    out[f"b{j}{j+1}LEs"] = np.where(le > 32768, le - 65536, le)
    out[f"b{j}{j+1}BEs"] = np.where(be > 32768, be - 65536, be)
  # 12-bit slot装配（块C F0 已证 x=(b3&0xF)<<8|b2; 高低位两种序都试）
  b2 = d[:, 2].astype(np.float64)
  b3 = d[:, 3].astype(np.float64)
  b4 = d[:, 4].astype(np.float64)
  b5 = d[:, 5].astype(np.float64)
  out["x12lo"] = ((b3 % 16) * 256 + b2) if d.shape[1] > 3 else None
  out["x12hi"] = (b2 * 16 + (b3 % 16))
  out["x12lo45"] = ((b5 % 16) * 256 + b4)
  return {k: v for k, v in out.items() if v is not None}


def resample(t_src, v_src, t_dst, maxdt=60e6):
  idx = np.clip(np.searchsorted(t_src, t_dst), 1, len(t_src) - 1)
  pick = np.where(t_dst - t_src[idx - 1] <= t_src[idx] - t_dst, idx - 1, idx)
  ok = np.abs(t_src[pick] - t_dst) <= maxdt
  v = np.full(len(t_dst), np.nan)
  v[ok] = v_src[pick[ok]]
  return v


def hf_vego(t, v, win_s=5.0):
  """Detrended vEgo: v minus ~win_s moving average (kills counter/integrator artifacts)."""
  t0 = t - t[0]
  dt = np.diff(t) / SEC
  half = max(1, int(win_s / 2 / max(np.median(dt), 1e-3)))
  k = np.ones(2 * half + 1)
  sm = np.convolve(v, k / k.sum(), mode="same")
  return v - sm


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--top", type=int, default=25)
  ap.add_argument("--cache", default="/tmp/hour_bus1.pkl")
  a = ap.parse_args()
  C = pickle.load(open(a.cache, "rb"))

  agg = collections.defaultdict(lambda: collections.defaultdict(list))
  for seg, s in sorted(C.items()):
    fr = s["frames"]
    if 0x109 not in fr or 0x10A not in fr:
      continue
    t109, d109 = fr[0x109]
    ta, da = fr[0x10A]
    b7 = d109[:, 7].astype(np.float64)
    t = t109.astype(np.float64)
    pres = resample(ta.astype(np.float64), ((da[:, 3] >> 5) & 7).astype(float), t)
    vt = np.array([x[0] for x in s["vego"]], np.float64)
    vv = np.array([x[1] for x in s["vego"]], np.float64)
    v = resample(vt, vv, t)
    lead = (pres == 7) & (b7 > 0) & (b7 <= 245) & ~np.isnan(v)
    # still-lead: b7 flat +-2s while we move (relV = -vEgo physics anchor)
    still = np.zeros(len(t), bool)
    w = 50  # 25Hz*2s
    flat = np.convolve(b7, np.ones(w) / w, mode="same")
    dev = np.abs(b7 - flat)
    for i in range(w, len(t) - w):
      if lead[i] and dev[i - w:i + w].max() <= 1.0 and np.nanmean(v[i - w:i + w]) > 0.8:
        still[i] = True
    hf = resample(vt, hf_vego(vt, vv), t)
    print(f"{seg}: lead={lead.sum()} still={still.sum()} vEgo[{np.nanmin(vv):.1f},{np.nanmax(vv):.1f}]")
    if lead.sum() < 300:
      continue

    for addr, (ft, fd) in sorted(fr.items()):
      if len(ft) < 200:
        continue
      ft = ft.astype(np.float64)
      try:
        vs = variants(fd)
      except Exception:
        continue
      dtf = np.diff(ft) / 1e9
      good = dtf > 1e-4
      for cn, cv0 in vs.items():
        if np.std(cv0) < 0.5:
          continue
        cv = resample(ft, cv0, t)  # onto the 0x109 timeline where lead/still/vEgo live
        g = agg[f"0x{addr:03x}/{cn}"]
        # distance family re-verify
        okl = lead & ~np.isnan(cv)
        if okl.sum() >= 300 and np.std(cv[okl]) > 0.5:
          g["r_dist"].append(np.corrcoef(cv[okl], b7[okl])[0, 1])
        # vLead: steady following vLead ~= vEgo
        if okl.sum() >= 300:
          g["r_vlead"].append(np.corrcoef(cv[okl], v[okl])[0, 1])
          g["r_hf"].append(np.corrcoef(cv[okl], hf[okl])[0, 1])
        # vRel on still-lead windows: expect r ~= -1 if field is vRel
        oks = still & ~np.isnan(cv)
        if oks.sum() >= 150 and np.std(cv[oks]) > 0.5:
          g["r_still"].append(np.corrcoef(cv[oks], v[oks])[0, 1])
          g["n_still"].append(int(oks.sum()))
        # rate consistency: df/dt vs -vEgo on still windows (distance field check)
        dv0 = np.diff(cv0) / np.where(good, dtf, np.nan)
        vdr = resample(ft[:-1][good], dv0[good], t)
        okr = still & ~np.isnan(vdr) & (np.abs(v) > 1.0)
        if okr.sum() >= 150 and np.std(vdr[okr]) > 1e-6:
          g["r_dot"].append(np.corrcoef(vdr[okr], -v[okr])[0, 1])

  def summ(k):
    g = agg[k]
    out = {}
    for ch in ("r_dist", "r_vlead", "r_hf", "r_still", "r_dot"):
      arr = np.array(g[ch]) if g[ch] else np.array([np.nan])
      out[ch] = (len(arr), float(np.nanmedian(arr)), float((np.abs(arr) > 0.7).mean()) if len(arr) else 0.0)
    return out

  print(f"\n# variants={len(agg)}")
  for ch, desc in (("r_dist", "DISTANCE family: |r(f,b7)| on lead"),
                   ("r_vlead", "VLEAD  : |r(f,vEgo)| on lead (steady following)"),
                   ("r_hf", "VLEAD-hf: |r(f,detrended vEgo)| (counter-proof)"),
                   ("r_still", "VREL   : r(f,vEgo) on STILL lead (expect ~ -1)"),
                   ("r_dot", "DIST-rate: r(df/dt,-vEgo) on still (distance check)")):
    rows = sorted((k for k in agg if agg[k][ch]), key=lambda k: -abs(summ(k)[ch][1]))
    print(f"\n== {desc}")
    print(f" {'field':22s} {'nseg':>4} {'med':>7} {'f>0.7':>5}   [2nd..4th]")
    for k in rows[:a.top]:
      n, med, f70 = summ(k)[ch]
      print(f" {k:22s} {n:4d} {med:+7.3f} {f70:5.2f}")


if __name__ == "__main__":
  main()
