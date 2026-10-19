"""Structure analysis of the bus1 67.16ms slot table (phase order, retrans pairs, raw series).

Run on single segments first (panda rlog parse ~60-90s each). Later rounds move to
the /tmp/slot67.pkl full cache.

Usage:
  .venv/bin/python tools/byd/radar_slot67_struct.py phase   <seg> [window_s]
  .venv/bin/python tools/byd/radar_slot67_struct.py pairs    <seg>          # retrans/eq test
  .venv/bin/python tools/byd/radar_slot67_struct.py dump     <seg> <addr> [n]
"""
import sys, os, collections
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
from openpilot.tools.lib.logreader import LogReader

REALDATA = "docs_site/byd_logs_2026-10-05_07/realdata"
FAM = set([0x301] + [a for a in range(0x340, 0x34B) if a not in (0x342, 0x345, 0x348)]
          + list(range(0x380, 0x398)) + list(range(0x3C8, 0x400)))


def load(seg):
  rows = collections.defaultdict(list)
  for m in LogReader(f"{REALDATA}/{seg}/rlog.zst"):
    if m.which() != "can":
      continue
    for c in m.can:
      if c.src == 1 and c.address in FAM:
        rows[c.address].append((m.logMonoTime, bytes(c.dat)))
  return rows


def phase(seg, win_s=0.4):
  rows = load(seg)
  allt = sorted([(t, a) for a, s in rows.items() for t, _ in s])
  if not allt:
    print("no frames")
    return
  t0, _ = allt[0]
  # one full cycle worth: frames within [first, first+win]
  cyc = [a for t, a in allt if t - t0 <= win_s * 1e9]
  print(f"== {seg}: emission order within first {win_s}s ({len(cyc)} frames, live addrs={len({a for a in cyc if (a - 0x301) >= 0})}) ==")
  # print as hex sequence
  print(" ".join(f"{a:03x}" for a in cyc[:80]))
  # per-addr phase histogram vs anchor cycle (median phase)
  anchor = min(t for t, _ in rows[0x301][:1])
  PER = 67_160_000
  ph = collections.defaultdict(list)
  for a, s in rows.items():
    for t, _ in s[:400]:
      ph[a].append(((t - anchor) % PER) / 1e6)
  med = sorted(((a, float(np.median(v))) for a, v in ph.items()), key=lambda x: x[1])
  print("median phase (ms into cycle), sorted:")
  print(" ".join(f"{a:03x}@{p:.1f}" for a, p in med[:35]))
  print(" ...")
  print(" ".join(f"{a:03x}@{p:.1f}" for a, p in med[-25:]))


def pairs(seg):
  rows = load(seg)
  # For every live address pair (a, a'), fraction of frames where payloads b2..b7 match,
  # aligning by nearest timestamp within +-33ms.
  live = {a for a, s in rows.items() if len(s) > 50 and
          np.ptp(np.frombuffer(b"".join(d for _, d in s), np.uint8).reshape(-1, 8)[:, 2:], axis=0).max() > 0}
  print(f"== {seg}: live addrs = {sorted(hex(a) for a in live)} ==")
  la = sorted(live)
  out = []
  for i, a in enumerate(la):
    ta = np.array([t for t, _ in rows[a]])
    da = np.frombuffer(b"".join(d for _, d in rows[a]), np.uint8).reshape(-1, 8)
    for b in la[i+1:]:
      tb = np.array([t for t, _ in rows[b]])
      db = np.frombuffer(b"".join(d for _, d in rows[b]), np.uint8).reshape(-1, 8)
      j = np.clip(np.searchsorted(tb, ta), 1, len(tb) - 1)
      j = np.where(ta - tb[j-1] <= tb[j] - ta, j-1, j)
      ok = np.abs(tb[j] - ta) <= 33_000_000
      if ok.sum() < 50:
        continue
      eq = (da[ok][:, 2:] == db[j[ok]][:, 2:]).all(axis=1).mean()
      if eq > 0.9:
        out.append((eq, hex(a), hex(b)))
  for eq, a, b in sorted(out, reverse=True)[:15]:
    print(f"  {a} ~ {b}: payload eq {eq*100:.1f}%  (possible retrans/echo pair)")


def dump(seg, addr_hex, n=24):
  rows = load(seg)
  a = int(addr_hex, 16)
  s = rows[a]
  print(f"== {seg} {a:#05x}: {len(s)} frames, first {n} payloads ==")
  for t, d in s[:n]:
    print(f"  {(t - s[0][0])/1e9:7.3f}s  {d.hex()}")


if __name__ == "__main__":
  cmd = sys.argv[1]
  seg = sys.argv[2]
  if cmd == "phase":
    phase(seg, float(sys.argv[3]) if len(sys.argv) > 3 else 0.4)
  elif cmd == "pairs":
    pairs(seg)
  elif cmd == "dump":
    dump(seg, sys.argv[3], int(sys.argv[4]) if len(sys.argv) > 4 else 24)
