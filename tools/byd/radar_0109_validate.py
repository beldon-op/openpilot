"""Validate the mined bus1 radar signal on the 2026-10-05_07 full archive (416 rlogs).

Candidate (from docs_site/hour_logs MI mining, tools/byd/radar_re_mine.py):
  0x109 b7 = forward target distance, ~meter scale. MI(bar-class)=1.28 bits,
  monotonic across two independent segments; 0x101/0x105 carry the same field.

Test (cross-segment, freshness-artifact-proof like the 0x38x refutation):
  per segment, require >=MIN frames of each HUD bar class {2,3,4}, then check
  mean(b7|c4) > mean(b7|c3) > mean(b7|c2) strictly. Report the pass rate.
  Also: b7==0 rate in class-0 (no-lead) windows, per-byte class means for the
  whole 0x101-0x10A block for follow-up field modeling.

Usage:
  .venv/bin/python tools/byd/radar_0109_validate.py docs_site/byd_logs_2026-10-05_07/realdata [--workers 8] [--json out.json]
"""
import sys, os, glob, json, argparse, bisect, collections
from multiprocessing import Pool
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANParser
from opendbc.car import Bus, CanData
from opendbc.car.byd.values import DBC, CAR

HUD_MSG = 0x32D
BLK = list(range(0x101, 0x10B))
CLASSES = (0, 2, 3, 4)
SEC = 1_000_000_000  # logMonoTime is ns
MIN_PER_CLASS = 50


def seg_analyze(f):
  hp = CANParser(DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar], [(HUD_MSG, 50)], 2)
  hud_t, hud_cls = [], []
  samples = []  # (t, addr, bytes)
  eq101_105 = (0, 0)
  for m in LogReader(f):
    if m.which() != "can":
      continue
    hf = []
    for c in m.can:
      if c.src == 1 and 0x101 <= c.address <= 0x10A:
        samples.append((m.logMonoTime, c.address, bytes(c.dat)))
      elif c.src == 2 and c.address == HUD_MSG:
        hf.append(CanData(c.address, bytes(c.dat), c.src))
    if hf:
      hp.update([(m.logMonoTime, hf)])
      v = hp.vl[HUD_MSG]
      hud_t.append(m.logMonoTime)
      hud_cls.append(int(v["HasLead"]) * int(v["LeadingDistance"]) if int(v["HasLead"]) else 0)

  agg = {a: {b: {c: [0, 0, 0, 0] for c in CLASSES}  # n, sum, zeros, max
             for b in range(8)} for a in BLK}
  n101 = eq105 = 0
  prev101 = prev105 = None
  for t, a, d in samples:
    i = bisect.bisect_left(hud_t, t)
    cls = None
    for j in (i - 1, i):
      if 0 <= j < len(hud_t) and abs(hud_t[j] - t) <= SEC // 2:
        cls = hud_cls[j]
        break
    if cls is None or cls not in CLASSES:
      continue
    for b, val in enumerate(d):
      row = agg[a][b][cls]
      row[0] += 1
      row[1] += val
      row[2] += int(val == 0)
      if val > row[3]:
        row[3] = val
    if a == 0x101:
      prev101 = d
    elif a == 0x105:
      eq105 += int(prev101 is not None and prev101[2:] == d[2:])
      n101 += 1
  return {"seg": os.path.basename(os.path.dirname(f)), "agg": agg,
          "hud_n": len(hud_t), "dup101_105": (eq105, n101)}


def merge(results):
  G = {a: {b: {c: [0, 0, 0, 0] for c in CLASSES} for b in range(8)} for a in BLK}
  mono_pass = {0x109: [0, 0], 0x101: [0, 0]}  # addr -> [pass, eligible]
  cls0 = {0x109: [0, 0], 0x101: [0, 0]}      # addr -> [nonzero_n, n]
  dup = [0, 0]
  segs_with_all_bars = 0
  for r in results:
    dup[0] += r["dup101_105"][0]
    dup[1] += r["dup101_105"][1]
    for a in BLK:
      for b in range(8):
        for c in CLASSES:
          row = G[a][b][c]
          s = r["agg"][a][b][c]
          row[0] += s[0]
          row[1] += s[1]
          row[2] += s[2]
          row[3] = max(row[3], s[3])
    for a in mono_pass:
      m = {c: (r["agg"][a][7][c][1] / r["agg"][a][7][c][0]) if r["agg"][a][7][c][0] >= MIN_PER_CLASS else None
           for c in CLASSES}
      cls0[a][0] += r["agg"][a][7][0][0] - r["agg"][a][7][0][2]
      cls0[a][1] += r["agg"][a][7][0][0]
      if None in (m[2], m[3], m[4]):
        continue
      mono_pass[a][1] += 1
      if m[4] > m[3] > m[2]:
        mono_pass[a][0] += 1
    if all(r["agg"][0x109][7][c][0] >= MIN_PER_CLASS for c in (2, 3, 4)):
      segs_with_all_bars += 1

  print(f"# segments: {len(results)}  with all 3 bar classes (>= {MIN_PER_CLASS}f): {segs_with_all_bars}")
  print(f"# 0x101==0x105 (b2-b7 equal): {dup[0]}/{dup[1]} = {dup[0]/max(dup[1],1):.1%}")
  print("\n## cross-segment monotonicity mean(b7|c4) > mean(b7|c3) > mean(b7|c2)")
  for a in (0x109, 0x101):
    p, e = mono_pass[a]
    print(f"  0x{a:03x} b7: {p}/{e} = {p/max(e,1):.1%} strict (segs where all 3 classes have >= {MIN_PER_CLASS} frames)")
  print("\n## class-0 (no lead) b7==0 rate")
  for a in (0x109, 0x101):
    nz, n = cls0[a]
    print(f"  0x{a:03x} b7: {(n-nz)/max(n,1):.1%} zero of {n} class-0 frames")
  print("\n## per-bar global mean/max of b7 (0x109 / 0x101)")
  for a in (0x109, 0x101):
    line = " ".join(f"c{c}: {G[a][7][c][1]/G[a][7][c][0]:6.2f} (n={G[a][7][c][0]:>8}, max={G[a][7][c][3]:3d})"
                    for c in CLASSES if G[a][7][c][0])
    print(f"  0x{a:03x}: {line}")
  print("\n## 0x101-0x10A full per-byte class means (field-modeling surface)")
  for a in BLK:
    rows = []
    for b in range(8):
      means = {c: (G[a][b][c][1] / G[a][b][c][0]) for c in CLASSES if G[a][b][c][0] >= 20}
      spread = (max(means.values()) - min(means.values())) if len(means) > 1 else 0
      if spread >= 1.0:
        rows.append(f"b{b}[{'/'.join(f'c{c}:{v:6.2f}' for c, v in sorted(means.items()))} s={spread:5.1f}]")
    if rows:
      print(f"  0x{a:03x}: " + " ".join(rows))
  return G, mono_pass, dup


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("dir")
  ap.add_argument("--workers", type=int, default=8)
  ap.add_argument("--json", default=None)
  a = ap.parse_args()
  segs = sorted(glob.glob(f"{a.dir}/*/rlog.zst"))
  with Pool(a.workers) as p:
    results = p.map(seg_analyze, segs, chunksize=4)
  G, mono, dup = merge(results)
  if a.json:
    with open(a.json, "w") as fp:
      json.dump({hex(x): G[x] for x in G}, fp)


if __name__ == "__main__":
  main()
