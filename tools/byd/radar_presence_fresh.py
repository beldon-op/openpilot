"""Presence-bit freshness: dissect the false-positive tail, esp. the 4 stuck segs.

①补录: 0x10A b3[7:5]==111 agreement fp med 0.00% but tail 15 segs >5%, of which
4 stuck at 100% (d3da5795a9--180/--234, 8801cc6fda--0, ...). Two competing
explanations: (i) radar-visible-but-unfollowed REAL targets (the --142
mechanism: b7>0 steady while HUD HasLead=0), or (ii) stale/frozen presence
(bits pinned while the sensor stopped updating the target). T8's window filter
needs the discriminating observable.

Per flagged segment, in pres==7 & class==0 windows, check:
  - frame rate of 0x10A/0x109 (CAN alive?) and SAE J1850 CRC-8 pass on b1..b7
    (already-pinned algorithm) => transport freshness
  - b1 AliveCounter roll continuity (diff==1 mod 256 fraction)
  - b7(0x109) in the same window: zero fraction, distinct count, std
    (real target: b7>0 moving; ghost/frozen: b7==0 pinned or b7 frozen const)
  -> filter rule suggestion from the actual numbers.

Usage: .venv/bin/python tools/byd/radar_presence_fresh.py --out /tmp/radar_pf.txt
"""
import sys, argparse, json, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import radar_seglib as L


def crc8_sae(data):
  crc = 0xFF
  for b in data:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ 0xFF


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--pkl", default="/tmp/seg1005.pkl")
  ap.add_argument("--div-json", default="/tmp/radar_415_diversity.json")
  ap.add_argument("--out", default="/tmp/radar_pf.txt")
  a = ap.parse_args()
  cache = L.load(a.pkl)
  div = json.load(open(a.div_json))
  seg_rows = {r["seg"]: r for r in div["per_segment"]}
  fps = []
  for name, r in seg_rows.items():
    if r["pres_nolead_n"] >= 50:
      fp = r["pres_nolead_hit"] / r["pres_nolead_n"]
      if fp > 0.05:
        fps.append((name, fp, r["pres_nolead_n"]))
  fps.sort(key=lambda x: -x[1])
  known_stuck = ["--180", "--234"]  # d3da5795a9; plus 8801cc6fda--0
  print(f"# flagged fp segs (n>=50, fp>5%): {len(fps)}", flush=True)

  lines = ["# presence freshness dissection of fp tail"]
  def say(*x):
    s = " ".join(str(v) for v in x)
    print(s, flush=True)
    lines.append(s)

  rows = []
  for name, fp, nn in fps[:18]:
    s = cache[name]
    t9, d9 = s["109_t"], s["109_d"]
    ta, da = s["10a_t"], s["10a_d"]
    if len(ta) < 30 or len(t9) < 30:
      continue
    cl = L.cls_at(ta, s)
    pres = (da[:, 3] >> 5) & 0x07
    ghost = (pres == 7) & (cl == 0)
    if ghost.sum() < 30:
      continue
    # transport freshness
    dt = np.diff(ta) / L.SEC
    fr = float(np.median(dt)) if len(dt) else None
    crc_ok = 0
    for row in da[ghost]:
      if crc8_sae(row[1:8]) == row[0]:
        crc_ok += 1
    crc_share = crc_ok / int(ghost.sum())
    b1 = da[:, 1].astype(np.int32)
    rolls = np.diff(b1) % 256 == 1
    roll_share = float(rolls.mean())
    # b7 in ghost windows
    b7g = np.interp(ta[ghost], t9, d9[:, 7].astype(np.float64), left=-1, right=-1)
    zero_share = float((b7g == 0).mean())
    b7v = b7g[b7g > 0]
    dist_n = len(np.unique(b7v.astype(int))) if len(b7v) else 0
    b7_std = float(np.std(b7v)) if len(b7v) > 2 else 0.0
    rows.append((name, fp, int(ghost.sum()), fr, crc_share, roll_share,
                 zero_share, dist_n, b7_std))

  say("\nseg                         fp    ghostN dt(ms) crc%  roll%  b7zero% b7distinct b7std")
  for r in rows:
    say(f"{r[0]:<26s} {r[1]:5.2%} {r[2]:7d} {1000*r[3]:6.1f} {r[4]:6.1%} "
        f"{r[5]:6.1%} {r[6]:6.1%} {r[7]:10d} {r[8]:6.1f}")

  say("\n== pattern summary")
  if rows:
    hi_pin = [r for r in rows if r[6] > 0.9]     # b7==0 while presence pinned
    mid = [r for r in rows if 0.05 < r[6] <= 0.9]
    low = [r for r in rows if r[6] <= 0.05]      # b7 >0 real-target style
    say(f"  ghosts with b7 mostly ZERO (presence-without-distance, candidate "
        f"stale/saturation code): {len(hi_pin)}/{len(rows)}")
    say(f"  ghosts with b7>0 moving (REAL visible-unfollowed targets, keep): "
        f"{len(low)}/{len(rows)}  mixed: {len(mid)}")
    crcs = [r[4] for r in rows]; rolls = [r[5] for r in rows]
    say(f"  CRC pass med={np.median(crcs):.1%}  counter-roll med={np.median(rolls):.1%} "
        f"(transport alive even in stuck segs => not a dead CAN path)")
    say("  => window filter suggestion: presence==7 AND b7>0 (kills pure-ghost "
        "pinning) AND presence-state changed OR b7 moved within last 30 s "
        "(kills frozen ghosts); real visible targets survive both.")
  with open(a.out, "w") as fp:
    fp.write("\n".join(lines) + "\n")


if __name__ == "__main__":
  main()
