"""Mine 0x418 BSD_RADAR for more than the two live bits - the Song's only rear-side frame.

Context (2026-10-18 bus3 exploration, docs/byd-radar-reverse-report §7.3): the full
415-seg census shows NO src==3 traffic at all, the repair manual puts the forward MRR
behind the MPC's *private* CAN, and the 2021 Song's side/rear "approach" detection is
documented as MPC-video based. So the rear story lives or dies on 0x418 itself.
Lane-change gating used only LEFT/RIGHT_APPROACH (dbc bits b1[1:0]/b1[3:2]); the
census saw byte_distinct b5=16, b7=81 over 594k frames - unexplained richness that
could be approach distance / target count / time-to-collision levels.

Subcommands:
  extract  --out /tmp/bsd418.pkl   # (t, d[N,8]) bus0 0x418 + vEgo + HUD class per seg
  analyze                          # structure + supervision against the two live bits

Usage:
  .venv/bin/python tools/byd/radar_bsd418_mine.py extract docs_site/byd_logs_2026-10-05_07/realdata --workers 10
  .venv/bin/python tools/byd/radar_bsd418_mine.py analyze
"""
import sys, os, glob, pickle, argparse
from multiprocessing import Pool

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np

SEC = 1_000_000_000
BSD = 0x418
HUD = 0x32D
CACHE = "/tmp/bsd418.pkl"


def crc_j1850(data, init=0xFF, xorout=0xFF, lo=1):
  crc = init
  for b in data[lo:8]:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ xorout


def seg_extract(f):
  from openpilot.tools.lib.logreader import LogReader
  from opendbc.can import CANParser
  from opendbc.car import Bus, CanData
  from opendbc.car.byd.values import DBC, CAR
  hp = CANParser(DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar], [(HUD, 50)], 2)
  bt, bd = [], []
  ht, hc, ct, cv = [], [], [], []
  err = None
  try:
    for m in LogReader(f):
      w = m.which()
      if w == "carState":
        ct.append(m.logMonoTime); cv.append(m.carState.vEgo); continue
      if w != "can":
        continue
      hf = []
      for c in m.can:
        if c.src == 0 and c.address == BSD:
          bt.append(m.logMonoTime); bd.append(bytes(c.dat))
        elif c.src == 2 and c.address == HUD:
          hf.append(CanData(c.address, bytes(c.dat), c.src))
      if hf:
        hp.update([(int(m.logMonoTime), hf)])
        v = hp.vl[HUD]
        hl = int(v["HasLead"])
        ht.append(m.logMonoTime); hc.append(int(v["LeadingDistance"]) * hl if hl else 0)
  except Exception as e:
    err = f"{type(e).__name__}: {str(e)[:80]}"
  return {"seg": os.path.basename(os.path.dirname(f)), "err": err,
          "t": np.asarray(bt, np.int64),
          "d": np.frombuffer(b"".join(bd), np.uint8).reshape(-1, 8) if bd else np.zeros((0, 8), np.uint8),
          "ht": np.asarray(ht, np.int64), "hc": np.asarray(hc, np.int8),
          "ct": np.asarray(ct, np.int64), "cv": np.asarray(cv, np.float32)}


def load_segs(pattern, workers):
  segs = sorted(glob.glob(f"{pattern}/*/rlog.zst"))
  print(f"# {len(segs)} segments", flush=True)
  with Pool(workers) as p:
    res = p.map(seg_extract, segs, chunksize=2)
  with open(CACHE, "wb") as fp:
    pickle.dump({r["seg"]: r for r in res}, fp, protocol=4)
  print(f"# cached {sum(len(r['t']) for r in res)} 0x418 frames, "
        f"{sum(1 for r in res if r['err'])} errs -> {CACHE}")


def interp(t, xt, xv, fill=np.nan):
  if len(xt) < 2:
    return np.full(len(t), fill)
  return np.interp(t, xt, xv, left=fill, right=fill)


def analyze():
  with open(CACHE, "rb") as fp:
    segs = pickle.load(fp)
  D = np.concatenate([s["d"] for s in segs.values() if len(s["d"])])
  print(f"# {len(segs)} segs, {len(D)} frames")

  # -- structure census
  print("\n== per-byte distinct / value spread ==")
  for j in range(8):
    col = D[:, j]
    u, cnts = np.unique(col, return_counts=True)
    top = sorted(zip(cnts, u))[-4:][::-1]
    print(f" b{j}: uniq={len(u):3d}  top: " + " ".join(f"{v:#04x}:{c/len(col)*100:.1f}%" for c, v in top))

  # -- b0 CRC check (is 0x418 in the same ARS410 family?)
  full = np.array([crc_j1850(D[i]) for i in range(len(D))])
  hit = (full == D[:, 0]).mean()
  print(f"\n== b0 vs SAE J1850 CRC(b1..b7): {hit*100:.2f}% "
        f"({'SAME FAMILY' if hit > 0.95 else 'not this CRC (different sender/algorithm)'})")

  # -- b1 bits: LEFT=b1[1:0], RIGHT=b1[3:2] per dbc 9|2@0+/11|2@0+
  L, R = D[:, 1] & 0x03, (D[:, 1] >> 2) & 0x03
  print(f"\n== b1 bits (dbc LEFT_APPROACH b1[1:0], RIGHT_APPROACH b1[3:2]) ==")
  for nm, x in (("LEFT", L), ("RIGHT", R)):
    u, c = np.unique(x, return_counts=True)
    print(f" {nm}: {dict(zip(u.tolist(), (c/len(x)*100).round(2).tolist()))}")
  anyB = ((L != 0) | (R != 0))
  print(f" any-approach frame share: {anyB.mean()*100:.2f}%")

  # -- what varies WITH the bits: per-byte value distribution split by bit state
  print("\n== byte vs approach-bit state (frames with bit set vs clear) ==")
  for j in range(8):
    a, b = D[anyB, j], D[~anyB, j]
    if len(a) == 0:
      continue
    ua, ub = len(np.unique(a)), len(np.unique(b))
    print(f" b{j}: set uniq={ua:3d} med={int(np.median(a)):3d} | clear uniq={ub:3d} med={int(np.median(b)):3d}"
          f" |diff-med|={abs(int(np.median(a))-int(np.median(b)))}")

  # -- episode dynamics: approach-bit onsets, track b7/b5 before/during/after
  print("\n== onset episodes: b5/b7 around LEFT/RIGHT bit-set starts (pre 2.5s / post 7.5s @20Hz) ==")
  ep = 0
  stats = []
  for name, s in sorted(segs.items()):
    d, t = s["d"], s["t"]
    if len(d) < 60:
      continue
    anyB = (((d[:, 1] & 0x03) != 0) | (((d[:, 1] >> 2) & 0x03) != 0)).astype(np.int8)
    on = np.where(anyB[1:] > anyB[:-1])[0] + 1
    for i in on:
      if i < 10 or i + 30 >= len(d):
        continue
      ep += 1
      pre, dur, post = d[i-10:i], d[i:i+30], d[i+30:i+60] if i+60 <= len(d) else d[-30:]
      v = float(np.nanmean(interp(t[i], s["ct"], s["cv"])))
      stats.append((int(np.median(dur[:, 7])), int(post[:, 7].max()) if len(post) else -1,
                    int(pre[:, 7].max()), v, int(np.median(dur[:, 5]))))
      if ep <= 15:
        side = "L" if (d[i, 1] & 0x03) else "R"
        print(f" {name[-26:]} @{(t[i]-t[0])/SEC:7.1f}s {side} vEgo={v:5.1f} "
              f"b7 pre_max={pre[:, 7].max():3d} dur_med={np.median(dur[:, 7]):5.1f} "
              f"post_max={post[:, 7].max() if len(post) else -1:3d} b5_med={np.median(dur[:, 5]):5.1f}")
  print(f" total onsets: {ep}")
  if stats:
    arr = np.array(stats)
    print(f" b7 across episodes: dur_med p50={np.median(arr[:,0])} (max {arr[:,0].max()}), "
          f"pre_max p50={np.median(arr[:,2])}, post_max p50={np.median(arr[:,1])}, "
          f"b5_dur_med p50={np.median(arr[:,4])}")
    print(f" corr(vEgo, b7_dur_med)={np.corrcoef(arr[:,3], arr[:,0])[0,1]:.3f}")


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("cmd", choices=["extract", "analyze"])
  ap.add_argument("dir", nargs="?", default="docs_site/byd_logs_2026-10-05_07/realdata")
  ap.add_argument("--workers", type=int, default=10)
  a = ap.parse_args()
  if a.cmd == "extract":
    load_segs(a.dir, a.workers)
  else:
    analyze()


if __name__ == "__main__":
  main()
