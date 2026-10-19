"""Full-frame cache for the bus1 67.16ms J1850-CRC family (89 addrs, 415 rlogs).

Family address set comes from radar_bus3_census.py (med_dt 67.16ms +-1):
  0x301 | 0x340-0x34A (no 342/345/348) | 0x380-0x397 | 0x3C8-0x3FF.
This is the multi-target slot-table candidate behind docs/byd-radar-rear-bus3-2026-10-18.md
sec 5. Keep all 8 bytes + timestamps per segment; supervision (0x109 b7 / 0x10A
presence / 0x32D HUD class / carState vEgo) is joined from /tmp/seg1005.pkl by
segment name, so this cache stores ONLY the family frames.

Usage (background recommended):
  .venv/bin/python tools/byd/radar_slot67_extract.py \
      docs_site/byd_logs_2026-10-05_07/realdata --workers 10 --out /tmp/slot67.pkl
"""
import sys, os, glob, pickle, argparse
from multiprocessing import Pool

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
from openpilot.tools.lib.logreader import LogReader

FAM = [0x301] + [a for a in range(0x340, 0x34B) if a not in (0x342, 0x345, 0x348)] \
      + list(range(0x380, 0x398)) + list(range(0x3C8, 0x400))
FAMSET = set(FAM)


def seg_extract(f):
  buf = {a: [[], []] for a in FAM}
  err = None
  try:
    for m in LogReader(f):
      if m.which() != "can":
        continue
      for c in m.can:
        if c.src == 1 and c.address in buf:
          b = buf[c.address]
          b[0].append(m.logMonoTime)
          b[1].append(bytes(c.dat))
  except Exception as e:  # corrupted tail: keep partial stream (precedent)
    err = f"{type(e).__name__}: {str(e)[:80]}"
  out = {"err": err}
  for a in FAM:
    t, ds = buf[a]
    out[f"{a:03x}_t"] = np.asarray(t, np.int64)
    out[f"{a:03x}_d"] = (np.frombuffer(b"".join(ds), np.uint8).reshape(-1, 8)
                         if ds else np.zeros((0, 8), np.uint8))
  out["seg"] = os.path.basename(os.path.dirname(f))
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("dir")
  ap.add_argument("--workers", type=int, default=10)
  ap.add_argument("--out", default="/tmp/slot67.pkl")
  a = ap.parse_args()
  segs = sorted(glob.glob(f"{a.dir}/*/rlog.zst"))
  print(f"# {len(segs)} segments, {len(FAM)} addrs, workers={a.workers}", flush=True)
  with Pool(a.workers) as p:
    res = p.map(seg_extract, segs, chunksize=2)
  cache = {r["seg"]: {k: v for k, v in r.items() if k != "seg"} for r in res}
  with open(a.out, "wb") as fp:
    pickle.dump(cache, fp, protocol=4)
  tot = sum(len(r[f"{a:03x}_t"]) for r in res for a in FAM)
  print(f"# done: {len(cache)} segs, {sum(1 for r in res if r['err'])} errs, "
        f"frames={tot}, {os.path.getsize(a.out)/1e6:.1f} MB -> {a.out}", flush=True)


if __name__ == "__main__":
  main()
