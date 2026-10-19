"""Does the bus1 67ms slot table respond to REAR/SIDE approach events (0x418 L/R bits)?

Direct rear-relevance test for the family found by radar_bus3_census.py. If the
67.16ms table carries rear/corner targets, slot payloads should activate/change
around 0x418 LEFT/RIGHT_APPROACH onsets beyond the baseline rate. Uses onset
times already in /tmp/bsd418.pkl (radar_bsd418_mine.py), then re-parses only the
segments containing the sampled onsets.

Metric per window (onset +-5 s vs same-seg control at random offset):
  n_live   = # frames whose b2..b7 spread > 0 in window (frames that "moved")
  n_new    = # addresses that were constant before onset and change during window

Usage:
  .venv/bin/python tools/byd/radar_slot67_rear.py [--max-segs 24] [--sample 40]
"""
import sys, os, glob, pickle, argparse, collections
from multiprocessing import Pool

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
from openpilot.tools.lib.logreader import LogReader

REALDATA = "docs_site/byd_logs_2026-10-05_07/realdata"
FAM = [0x301] + list(range(0x340, 0x34C)) + list(range(0x380, 0x398)) + list(range(0x3C8, 0x3E4))
SEC = 1_000_000_000
W = 5 * SEC


def window_stats(f, onsets, ctrl):
  """onsets/ctrl: lists of absolute logMonoTime anchors -> (n_live, n_new) each."""
  fam = collections.defaultdict(list)
  for m in LogReader(f):
    if m.which() != "can":
      continue
    for c in m.can:
      if c.src == 1 and c.address in FAM:
        fam[c.address].append((m.logMonoTime, bytes(c.dat)))
  if not fam:
    return None
  res = []
  for anchor in list(onsets) + list(ctrl):
    live, addr_new = set(), set()
    base_pre = {}
    for a, s in fam.items():
      arr = [(t, d) for t, d in s if anchor - 2 * W <= t < anchor - W]   # pre-baseline 5s
      cur = [(t, d) for t, d in s if anchor - W // 2 <= t <= anchor + W] # window
      if len(cur) < 8:
        continue
      A = np.frombuffer(b"".join(d for _, d in cur), np.uint8).reshape(-1, 8)
      moved = np.ptp(A[:, 2:], axis=0)
      if moved.max() > 0:
        live.add(a)
        if len(arr) >= 8:
          B = np.frombuffer(b"".join(d for _, d in arr), np.uint8).reshape(-1, 8)
          if np.ptp(B[:, 2:], axis=0).max() == 0:
            addr_new.add(a)   # was frozen pre-event, active in window
    res.append((len(live), len(addr_new)))
  return res


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--max-segs", type=int, default=24)
  ap.add_argument("--sample", type=int, default=40)
  a = ap.parse_args()
  segs = pickle.load(open("/tmp/bsd418.pkl", "rb"))
  rng = np.random.default_rng(7)

  events = []
  for name, s in sorted(segs.items()):
    d, t = s["d"], s["t"]
    if len(d) < 60 or not os.path.exists(f"{REALDATA}/{name}/rlog.zst"):
      continue
    anyB = (((d[:, 1] & 0x03) != 0) | (((d[:, 1] >> 2) & 0x03) != 0)).astype(np.int8)
    on = np.where(anyB[1:] > anyB[:-1])[0] + 1
    for i in on:
      if 10 < i < len(d) - 10:
        events.append((name, int(t[i]), int(t[0]), int(t[-1])))
  print(f"# {len(events)} onsets in archive; sampling {min(a.sample, len(events))}")
  picks = rng.choice(len(events), size=min(a.sample, len(events)), replace=False)
  by_seg = collections.defaultdict(list)
  for p in picks:
    by_seg[events[p][0]].append(events[p][1])
  by_seg = dict(list(by_seg.items())[:a.max_segs])

  jobs = []
  for name, ons in by_seg.items():
    ev = next(e for e in events if e[0] == name)
    span = ev[3] - ev[2]
    ctrls = [int(ev[2] + rng.uniform(W, max(W + 1, span - 2 * W))) for _ in ons]
    jobs.append((f"{REALDATA}/{name}/rlog.zst", ons, ctrls))

  with Pool(10) as p:
    out = p.starmap(window_stats, jobs)
  on_rows, ct_rows = [], []
  for r in out:
    if not r:
      continue
    h = len(r) // 2
    on_rows += r[:h]
    ct_rows += r[h:2 * h]
  on, ct = np.array(on_rows), np.array(ct_rows)
  print(f"# paired windows: events={len(on)} controls={len(ct)}")
  for j, nm in ((0, "live addrs (b2..b7 moved)"), (1, "FROZEN->active new addrs")):
    mo, mc = on[:, j].mean(), ct[:, j].mean()
    sd = np.sqrt(on[:, j].var() / len(on) + ct[:, j].var() / len(ct))
    print(f" {nm}: onset={mo:.2f} control={mc:.2f}  diff={mo-mc:+.2f} (se={sd:.2f}, z={(mo-mc)/sd if sd else 0:.1f})")


if __name__ == "__main__":
  main()
