"""Supervised reverse of bus1 0x101-0x10A against can2 supervision signals.

Method (user-specified): bus1 is treated as the raw radar stream and every
candidate byte / byte-pair is fitted against EXTERNAL supervisors taken from
can2 + carState, instead of bus1 self-correlation:
  - 0x32D ACC_HUD_ADAS: HasLead, LeadingDistance (bar), AccState  [bus 2, 50Hz]
  - carState.vEgo (m/s)                                            [log, ~100Hz]
  - bootstrapped distance target: RADAR_HUD_BARS_TO_M[bar]
  - bootstrapped closing-speed target: -d(u*b7_109)/dt after first-pass unit fit

Stage 1 "extract": cache per-segment raw frames (t_ns, 8 bytes) for the six
bus1 addrs + HUD/vEgo series -> pickle.
Stage 2 "fit": supervised regression (single bytes + adjacent pairs LE/BE x
signed/unsigned), bit separation search, counter/CRC signature, redundancy
matrix, unit calibration (headway band 1.0-2.5 s), lead-window sample dumps.

Usage:
  .venv/bin/python tools/byd/radar_sup_mine.py extract docs_site/hour_logs/radar_rlogs /tmp/sup_cache.pkl
  .venv/bin/python tools/byd/radar_sup_mine.py fit /tmp/sup_cache.pkl /tmp/radar_sup.json
"""
import sys, os, glob, pickle, json
import numpy as np
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

BUS1_ADDRS = [0x101, 0x102, 0x105, 0x106, 0x109, 0x10A]
HUD_MSG = 0x32D

BARS_TO_M = {1: 6.0, 2: 13.0, 3: 22.0, 4: 35.0}  # opendbc/car/byd/values.py RADAR_HUD_BARS_TO_M (1-4 observed)


def extract_main():
  from openpilot.tools.lib.logreader import LogReader
  from opendbc.can import CANParser
  from opendbc.car import Bus, CanData
  from opendbc.car.byd.values import DBC, CAR

  def one(f):
    dbc = DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar]
    hp = CANParser(dbc, [(HUD_MSG, 50)], 2)
    frames = {a: [] for a in BUS1_ADDRS}
    hud, vego = [], []
    for m in LogReader(f):
      if m.which() == "can":
        hf = []
        for c in m.can:
          if c.src == 1 and c.address in frames:
            frames[c.address].append((m.logMonoTime, bytes(c.dat)))
          elif c.src == 2 and c.address == HUD_MSG:
            hf.append(CanData(c.address, bytes(c.dat), c.src))
        if hf:
          hp.update([(m.logMonoTime, hf)])
          v = hp.vl[HUD_MSG]
          hud.append((m.logMonoTime, int(v["HasLead"]), int(v["LeadingDistance"]), int(v["AccState"])))
      elif m.which() == "carState":
        vego.append((m.logMonoTime, float(m.carState.vEgo)))
    seg = os.path.basename(os.path.dirname(f))
    return seg, frames, hud, vego

  segs = sorted(glob.glob(sys.argv[2] + "/*/rlog.zst"))
  out = {}
  for f in segs:
    seg, frames, hud, vego = one(f)
    out[seg] = {"frames": frames, "hud": hud, "vego": vego}
    print(f"{seg}: " + " ".join(f"0x{a:x}={len(frames[a])}" for a in BUS1_ADDRS) + f" hud={len(hud)} vego={len(vego)}", flush=True)
  with open(sys.argv[3], "wb") as fp:
    pickle.dump(out, fp)
  print(f"cached -> {sys.argv[3]}")


# ---------------------------------------------------------------- fit stage

def zoh(times, vals, t):
  """zero-order hold: vals aligned to times (sorted), query array t."""
  idx = np.searchsorted(times, t, side="right") - 1
  idx = np.clip(idx, 0, len(times) - 1)
  return vals[idx]


def linfit(x, y):
  if len(x) < 20 or np.std(x) < 1e-9:
    return None
  A = np.vstack([x, np.ones_like(x)]).T
  slope, inter = np.linalg.lstsq(A, y, rcond=None)[0]
  pred = slope * x + inter
  ss_res = np.sum((y - pred) ** 2)
  ss_tot = np.sum((y - np.mean(y)) ** 2)
  r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else float("nan")
  rmse = float(np.sqrt(np.mean((y - pred) ** 2)))
  pear = float(np.corrcoef(x, y)[0, 1])
  return {"slope": float(slope), "inter": float(inter), "r2": float(r2), "rmse": rmse, "pearson": pear}


def pair_variants(d, i):
  lo, hi = d[:, i].astype(np.int64), d[:, i + 1].astype(np.int64)
  leu = lo + (hi << 8)
  beu = (lo << 8) + hi
  lus = np.where(leu >= (1 << 15), leu - (1 << 16), leu)
  bus_ = np.where(beu >= (1 << 15), beu - (1 << 16), beu)
  return [("LEU", leu), ("LUS", lus), ("BEU", beu), ("BES", bus_)]


def byte_series(addr_frames, i):
  t = np.array([x[0] for x in addr_frames], dtype=np.int64)
  d = np.array([list(x[1]) for x in addr_frames], dtype=np.uint8)
  return t, d


def classify_dynamics(v):
  """lag1 autocorr, change-rate, plateau fraction of a byte sequence."""
  dv = np.diff(v.astype(np.float64))
  chg = float(np.mean(np.abs(dv) > 1e-9))
  ac = float(np.corrcoef(v[:-1], v[1:])[0, 1]) if np.std(v) > 1e-9 else float("nan")
  plateau = float(np.mean(np.abs(dv) <= 1)) if len(dv) else float("nan")
  return chg, ac, plateau


def fit_main():
  cache = pickle.load(open(sys.argv[2], "rb"))
  report = {"segments": {}, "aggregate": {}, "samples": {}}
  per_seg = []

  for seg, s in sorted(cache.items()):
    hud_t = np.array([x[0] for x in s["hud"]], dtype=np.int64)
    haslead = np.array([x[1] for x in s["hud"]], dtype=np.int64)
    bar = np.array([x[2] for x in s["hud"]], dtype=np.int64)
    accst = np.array([x[3] for x in s["hud"]], dtype=np.int64)
    vg_t = np.array([x[0] for x in s["vego"]], dtype=np.int64)
    vg_v = np.array([x[1] for x in s["vego"]], dtype=np.float64)

    seg_r = {}
    arr = {}
    for a in BUS1_ADDRS:
      if len(s["frames"][a]) < 50:
        continue
      t, d = byte_series(s["frames"][a], 0)
      hl = zoh(hud_t, haslead, t)
      bd = zoh(hud_t, bar, t)
      ac = zoh(hud_t, accst, t)
      ve = zoh(vg_t, vg_v, t)
      lead = (hl == 1) & np.isin(bd, list(BARS_TO_M.keys()))
      arr[a] = {"t": t, "d": d, "hl": hl, "bd": bd, "ac": ac, "ve": ve, "lead": lead}

    # ---- per-byte profile
    for a, A in arr.items():
      prof = {}
      d, lead = A["d"], A["lead"]
      nlead, nnolead = int(lead.sum()), int((~lead).sum())
      for i in range(8):
        col = d[:, i].astype(np.float64)
        chg, ac1, plat = classify_dynamics(col)
        p = {
          "lead_mean": float(col[lead].mean()) if nlead else None,
          "nolead_mean": float(col[~lead].mean()) if nnolead else None,
          "lead_uniq": int(len(np.unique(d[lead, i]))) if nlead else 0,
          "nolead_top": np.bincount(d[~lead, i]).argmax() if nnolead else None,
          "chg_rate": chg, "lag1_autocorr": ac1, "plateau_frac": plat,
          "lead_lo_nib_F": float(np.mean((d[lead, i] & 0xF) == 0xF)) if nlead else None,
          "lead_hi_nib_7or8": float(np.mean(np.isin(d[lead, i] >> 4, (7, 8)))) if nlead else None,
          "nolead_lo_nib_F": float(np.mean((d[~lead, i] & 0xF) == 0xF)) if nnolead else None,
        }
        prof[f"b{i}"] = p
      seg_r[f"0x{a:x}_bytes"] = prof

    # ---- supervised fits: target = bar->m distance (lead frames only)
    fits = {}
    for a, A in arr.items():
      d, lead, ve = A["d"], A["lead"], A["ve"]
      dsup = np.array([BARS_TO_M.get(b, np.nan) for b in A["bd"]])[lead]
      xlead = {}
      for i in range(8):
        xlead[f"b{i}"] = d[lead, i].astype(np.float64)
      for i in range(7):
        for nm, v in pair_variants(d, i):
          xlead[f"b{i+1}b{i}:pair_{nm}"] = v[lead].astype(np.float64)
      for name, x in xlead.items():
        f = linfit(x, dsup)
        if f and f["r2"] > 0.15:
          f["n"] = int(lead.sum())
          fits.setdefault(f"0x{a:x}", {})[name] = {k: round(v, 4) for k, v in f.items()}
      # vEgo corr for every byte (lead + nolead)
      for i in range(8):
        f = linfit(d[:, i].astype(np.float64), ve)
        if f and abs(f["pearson"]) > 0.45:
          fits.setdefault(f"0x{a:x}_vego", {})[f"b{i}"] = {"pearson": round(f["pearson"], 3), "slope": round(f["slope"], 4), "r2": round(f["r2"], 3)}
    seg_r["fits_dist_bar"] = fits

    # ---- bit separation: which single bits mark lead vs no-lead
    seps = []
    for a, A in arr.items():
      d, lead = A["d"], A["lead"]
      if int(lead.sum()) < 200 or int((~lead).sum()) < 200:
        continue
      bits = np.unpackbits(d, axis=1, bitorder="little")
      on_lead = bits[lead].mean(0)
      on_no = bits[~lead].mean(0)
      for bi in range(64):
        if abs(on_lead[bi] - on_no[bi]) > 0.85:
          seps.append({"msg": f"0x{a:x}", "bit": bi, "byte": bi // 8, "on_lead": round(float(on_lead[bi]), 3), "on_nolead": round(float(on_no[bi]), 3)})
    seg_r["bit_separation"] = seps

    # ---- redundancy matrix (same-length frames by timestamp proximity)
    red = {}
    for a1 in arr:
      for a2 in arr:
        if a1 >= a2:
          continue
        A1, A2 = arr[a1], arr[a2]
        # align A2 onto A1 timestamps
        d2 = A2["d"][zoh(np.arange(len(A2["t"])), np.arange(len(A2["t"])), np.searchsorted(A2["t"], A1["t"], "left").clip(0, len(A2["t"]) - 1))] if False else None
        idx = np.searchsorted(A2["t"], A1["t"])
        idx = np.clip(idx, 0, len(A2["d"]) - 1)
        d2 = A2["d"][idx]
        eq = np.mean(d2 == A1["d"], axis=0)
        red[f"0x{a1:x}~0x{a2:x}"] = {"eq_frac": round(float(np.mean(eq)), 3), "per_byte": [round(float(e), 3) for e in eq]}
    seg_r["redundancy"] = red

    # ---- first-pass unit: 0x109 b7 slope from 3-level bar fit, then headway band
    if 0x109 in arr:
      A = arr[0x109]
      d, lead, ve = A["d"], A["lead"], A["ve"]
      b7 = d[lead, 7].astype(np.float64)
      dsup = np.array([BARS_TO_M.get(b, np.nan) for b in A["bd"]])[lead]
      f = linfit(b7, dsup)
      seg_r["u109_b7_fit"] = f
      if f and f["slope"] > 0:
        u = f["slope"]
        hw = u * np.vstack([b7, ve[lead]]).T
        seg_r["headway_fit_u"] = {"u_m_per_count": round(u, 4),
                                  "headway_s_median": round(float(np.median((u * b7) / np.maximum(ve[lead], 1.0))), 2)}
        cand = {"1.0m": 1.0, "0.5m": 0.5, "feet0.3048": 0.3048, "0.25m": 0.25, "fitted": u}
        seg_r["headway_band"] = {k: round(float(np.mean(((u0 * b7 / np.maximum(ve[lead], 1.0)) >= 1.0) &
                                                        ((u0 * b7 / np.maximum(ve[lead], 1.0)) <= 2.5))), 3)
                                 for k, u0 in cand.items()}
        # closing speed target from 0x109 b7 series (full lead runs)
        b7all = A["d"][:, 7].astype(np.float64)
        t_s = A["t"] / 1e9
        vclose = -np.gradient(u * b7all, t_s)
        seg_r["vclose_stats"] = {"median": round(float(np.median(vclose[lead])), 2),
                                 "p10": round(float(np.percentile(vclose[lead], 10)), 2),
                                 "p90": round(float(np.percentile(vclose[lead], 90)), 2),
                                 "frac_gt_vego": round(float(np.mean(np.abs(vclose[lead]) > ve[lead])), 3)}
        # second supervised pass: every byte/pair vs closing-speed estimate (lead)
        vc = {}
        for a2, B in arr.items():
          vcs = zoh(A["t"], vclose, B["t"])
          l2 = B["lead"]
          x2 = {}
          for i in range(8):
            x2[f"b{i}"] = B["d"][l2, i].astype(np.float64)
          for i in range(7):
            for nm, v in pair_variants(B["d"], i):
              x2[f"b{i+1}b{i}:pair_{nm}"] = v[l2].astype(np.float64)
          best = []
          for name, x in x2.items():
            f2 = linfit(x, vcs[l2])
            if f2 and abs(f2["pearson"]) > 0.4:
              best.append((name, round(f2["pearson"], 3), round(f2["slope"], 4), round(f2["r2"], 3)))
          if best:
            vc[f"0x{a2:x}"] = sorted(best, key=lambda z: -abs(z[1]))[:5]
        seg_r["fits_vclose"] = vc

        # target-absolute-speed supervision: vT = vEgo - vclose
        vT = ve - vclose
        vt_fit = {}
        for a2, B in arr.items():
          vTs = zoh(A["t"], vT, B["t"])
          l2 = B["lead"]
          x2 = {}
          for i in range(8):
            x2[f"b{i}"] = B["d"][l2, i].astype(np.float64)
          for i in range(7):
            for nm, v in pair_variants(B["d"], i):
              x2[f"b{i+1}b{i}:pair_{nm}"] = v[l2].astype(np.float64)
          best = []
          for name, x in x2.items():
            f2 = linfit(x, vTs[l2])
            if f2 and abs(f2["pearson"]) > 0.4:
              best.append((name, round(f2["pearson"], 3), round(f2["slope"], 4), round(f2["r2"], 3)))
          if best:
            vt_fit[f"0x{a2:x}"] = sorted(best, key=lambda z: -abs(z[1]))[:5]
        seg_r["fits_vtarget"] = vt_fit
    per_seg.append(seg_r)

  # ---- sample dumps: around HUD bar flips, print all bytes of 101/109/10A
  for seg, s in sorted(cache.items()):
    hud_t = np.array([x[0] for x in s["hud"]], dtype=np.int64)
    haslead = np.array([x[1] for x in s["hud"]], dtype=np.int64)
    bar = np.array([x[2] for x in s["hud"]], dtype=np.int64)
    vg_t = np.array([x[0] for x in s["vego"]], dtype=np.int64)
    vg_v = np.array([x[1] for x in s["vego"]], dtype=np.float64)
    if 0x109 not in ({a for a in BUS1_ADDRS if len(s["frames"][a]) >= 50}):
      continue
    t9, d9 = byte_series(s["frames"][0x109], 0)
    flips = np.where(np.diff(zoh(hud_t, bar, t9)) != 0)[0]
    rows = []
    for fi in flips:
      lo, hi = max(0, fi - 3), min(len(t9), fi + 6)
      for k in range(lo, hi):
        row = {"seg": seg, "t_s": round(float(t9[k] / 1e9 - t9[0] / 1e9), 3)}
        for a in (0x101, 0x109, 0x10A):
          fr = s["frames"][a]
          if not fr:
            continue
          j = int(np.clip(np.searchsorted([x[0] for x in fr], t9[k]), 0, len(fr) - 1))
          row[f"0x{a:x}"] = fr[j][1].hex()
        row["hl"] = int(zoh(hud_t, haslead, np.array([t9[k]]))[0])
        row["bar"] = int(zoh(hud_t, bar, np.array([t9[k]]))[0])
        row["vego"] = round(float(zoh(vg_t, vg_v, np.array([t9[k]]))[0]) * 3.6, 1)
        rows.append(row)
    report["samples"][seg] = rows[:80]

  report["segments"] = per_seg
  with open(sys.argv[3], "w") as fp:
    json.dump(report, fp, indent=1, default=lambda o: float(o) if isinstance(o, (np.integer, np.floating)) else str(o))
  # console digest
  print(f"wrote {sys.argv[3]}: {os.path.getsize(sys.argv[3])} bytes, {len(per_seg)} segments")


if __name__ == "__main__":
  if sys.argv[1] == "extract":
    extract_main()
  else:
    fit_main()
