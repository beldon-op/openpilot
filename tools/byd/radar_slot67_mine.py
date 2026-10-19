"""Mine the bus1 67.16ms frame family (0x301, 0x340-0x34A, 0x380-0x397, 0x3C8-0x3E3).

Found by radar_bus3_census.py: 89 addresses, 53 with payload-rich bytes, b0 = SAE
J1850 CRC-8 (same algorithm as the 0x10x ARS410-family frames, 100% over 39.3M
frames), fixed 67.16ms period = 14.9 Hz = continental MRR scan rate. The 10-11
verdict "0x38x = state table (cross-gear monotonicity failed)" used a test built
for a single-lead field - a MULTI-TARGET slot table need not be gear-monotonic.
So: is this the Song's real multi-object track output (per-object dist / relV /
angle), or something else? This tool answers quantitatively.

Method per segment:
  - align every live slot byte to 0x109 b7 (fused lead dist) and vEgo on the 67ms grid;
  - corr(b_slot, b7_109), corr(b_slot, -db7/dt) [relV proxy], corr(b_slot, vEgo);
  - report the byte-level maxima: a slot exposing an object distance should track
    b7_109 on frames where that object IS the lead.

Usage:
  .venv/bin/python tools/byd/radar_slot67_mine.py segA segB ...     # segment dir names under realdata
  .venv/bin/python tools/byd/radar_slot67_mine.py --all N           # first N segs of the archive
"""
import sys, os, glob, argparse
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
from openpilot.tools.lib.logreader import LogReader

REALDATA = "docs_site/byd_logs_2026-10-05_07/realdata"
FAM = ([0x301, 0x302] + list(range(0x340, 0x34C)) + list(range(0x380, 0x398))
       + list(range(0x3C8, 0x3E4)))
GAP = 67_160_000   # ns nominal


def load(f):
  rows = {}
  fam = {a: [] for a in FAM}
  for m in LogReader(f):
    w = m.which()
    if w == "carState":
      rows.setdefault("cs", []).append((m.logMonoTime, m.carState.vEgo))
      continue
    if w != "can":
      continue
    for c in m.can:
        if c.src == 1 and c.address in fam:
          fam[c.address].append((m.logMonoTime, bytes(c.dat)))
        elif c.src == 1 and c.address in (0x109, 0x10A):
          fam.setdefault(c.address, []).append((m.logMonoTime, bytes(c.dat)))
        elif c.src == 2 and c.address == 0x32D:
          fam.setdefault(0x32D, []).append((m.logMonoTime, bytes(c.dat)))
  return fam, rows


def col(fam_or_rows, addr, j):
  s = fam_or_rows.get(addr) or []
  if not s:
    return None
  t = np.array([x[0] for x in s], float)
  v = np.array([x[1][j] for x in s], float)
  return t, v


def corr_on_grid(t1, v1, t2, v2):
  """nearest-t interpolation of series2 onto series1 grid, pearson r, |common| frames."""
  if t1 is None or t2 is None or len(t1) < 30 or len(t2) < 30:
    return None, 0
  lo, hi = max(t1[0], t2[0]), min(t1[-1], t2[-1])
  m = (t1 >= lo) & (t1 <= hi)
  if m.sum() < 30:
    return None, 0
  v2i = np.interp(t1[m], t2, v2)
  a, b = v1[m], v2i
  if a.std() < 1e-6 or b.std() < 1e-6:
    return 0.0, int(m.sum())
  return float(np.corrcoef(a, b)[0, 1]), int(m.sum())


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("segs", nargs="*")
  ap.add_argument("--all", type=int, default=0)
  a = ap.parse_args()
  if a.all:
    segs = sorted(glob.glob(f"{REALDATA}/*/rlog.zst"))[:a.all]
  else:
    segs = [f"{REALDATA}/{s}/rlog.zst" for s in a.segs]
  for f in segs:
    fam, rows = load(f)
    name = os.path.basename(os.path.dirname(f))
    d109 = col(fam, 0x109, 7)
    t109 = col(fam, 0x109, 0)[0]
    cs = rows.get("cs", [])
    tc = np.array([x[0] for x in cs], float); vc = np.array([x[1] for x in cs], float)
    # relV proxy: -db7/dt on the 40ms 0x109 grid, coarse (differenced, then sign by ego motion)
    if d109 is not None and len(d109[0]) > 60:
      dt = np.diff(d109[0]) / 1e9
      db7 = -np.diff(d109[1]) / dt
      tb = d109[0][:-1]
    else:
      tb = db7 = None
    lead_on = d109 is not None and (d109[1] > 0).mean() > 0.3
    print(f"\n=== {name}  0x109 frames={0 if d109 is None else len(d109[0])} "
          f"lead-share={0 if d109 is None else (d109[1]>0).mean():.2f} ===")
    hdr = " addr   byte  r(b7_109)   r(-db7/dt)  r(vEgo)"
    print(hdr)
    best = []
    for addr in FAM:
      s = fam.get(addr) or []
      if len(s) < 100:
        continue
      A = np.frombuffer(b"".join(d for _, d in s), np.uint8).reshape(-1, 8)
      t = np.array([tt for tt, _ in s], float)
      # skip placeholder frames (payload constant beyond b1)
      if np.all([A[:, j].std() < 1e-9 for j in range(2, 8)]):
        continue
      for j in range(2, 8):
        v = A[:, j].astype(float)
        if v.std() < 1:
          continue
        r1, n1 = corr_on_grid(t, v, d109[0], d109[1]) if d109 else (None, 0)
        r2, n2 = corr_on_grid(t, v, tb, db7) if tb is not None else (None, 0)
        r3, n3 = corr_on_grid(t, v, tc, vc) if len(tc) > 60 else (None, 0)
        rr = [x if x is not None else 0 for x in (r1, r2, r3)]
        best.append((max(abs(x) for x in rr), addr, j, rr, n1))
        print(f" {addr:#05x}  b{j}   {r1 if r1 is not None else float('nan'):7.3f}  "
              f"{r2 if r2 is not None else float('nan'):9.3f}  "
              f"{r3 if r3 is not None else float('nan'):7.3f}  n={n1}")
    best.sort(reverse=True)
    if best:
      print(" TOP correlated bytes:")
      for mx, addr, j, rr, n in best[:8]:
        print(f"   {addr:#05x} b{j}: r109={rr[0]:.3f} rrelV={rr[1]:.3f} rv={rr[2]:.3f} (n={n})")


if __name__ == "__main__":
  main()
