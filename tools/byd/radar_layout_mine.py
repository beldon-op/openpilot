"""Bit-level layout mining of the BYD Song bus1 radar candidate blocks.

VERDICT (2026-10-11, final): 0x380-0x38f is NOT a target stream. The regime
"AUC 0.70-0.86 separation" seen on first pass is a freshness artifact: b1 is
a strict +17/frame mod-256 counter shared same-tick across slots (single
sending ECU), and the payload fields are per-slot constants plus checksum
noise - no byte/word is rank-consistent against the 0x32D distance bar across
the 92 multi-bar segments this tool screens. With 0x374 absent and 0x351/0x354
idle heartbeats (AUC 0.5 while following), NO target-level radar stream
(front or rear) exists on any recorded bus. The shipped radar path is the
0x32D HUD-bar synthesis in opendbc/car/byd/radar_interface.py instead.

2026-10-11 full-archive rescan (radar_1005_scan.py, 415 segs, docs §8.5.7)
swept EVERY high-rate bus1 block through the same screen - 0x101-0x10A @25Hz
(same tick as the track slots), 0x340-0x34A @14Hz, 0x301, 0x350, 0x396/0x397,
0x3F8-0x3FF - payload diversity said "RICH" but rank-consistency failed on
every u8/u16 field (all data bytes quasi-constant; variation is the shared
b1 counter, +17/frame mod 256, identical signature to 0x38x). One ECU status
broadcast family, not target streams. 0x374 also re-confirmed 0 frames on all
buses. The no-target-stream verdict now covers the entire bus1 address space.

Keep this tool for reuse if a new bus ever gets tapped (docs §8.5.3): ground
truth is 0x32d ACC_HUD_ADAS HasLead(bit9)/LeadingDistance(bit13-15), and a
real target field must be rank-consistent per bar ACROSS segments, not just
correlated within one - the within-segment trap is exactly what fooled the
first pass (and the naive payload-diversity screen in round two).

Usage (host):
  .venv/bin/python tools/byd/radar_layout_mine.py docs_site/byd_logs_2026-10-05_07/realdata
  .venv/bin/python tools/byd/radar_layout_mine.py <dir> --top 8 --slot 0x382
  ... --targets 0x101,0x102,... --segs segA,segB   (mine other blocks on chosen following segments)
"""
import sys, os, glob, argparse, bisect, statistics as st
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from openpilot.tools.lib.logreader import LogReader

TARGETS = list(range(0x380, 0x390)) + [0x3c8, 0x3c9, 0x3ca]
IDLE = {0x350: bytes.fromhex("00f0ffffff1f"), 0x351: bytes.fromhex("00002000e000"),
        0x354: bytes.fromhex("00002000e000")}

FIELDS = [
  ("u8@2",  lambda d: d[2]),   ("u8@3",  lambda d: d[3]),
  ("u8@4",  lambda d: d[4]),   ("u8@5",  lambda d: d[5]),
  ("u8@6",  lambda d: d[6]),   ("u8@7",  lambda d: d[7]),
  ("u16LE@2", lambda d: d[2] | d[3] << 8),  ("u16BE@2", lambda d: d[2] << 8 | d[3]),
  ("u16LE@4", lambda d: d[4] | d[5] << 8),  ("u16BE@4", lambda d: d[4] << 8 | d[5]),
  ("u16LE@6", lambda d: d[6] | d[7] << 8),  ("u16BE@6", lambda d: d[6] << 8 | d[7]),
]

def bar_of(d):  # 0x32d ACC_HUD_ADAS raw bytes -> (has_lead, dist 0-7)
  return (d[1] >> 1) & 1, (d[1] >> 5) & 7

def pearson(x, y):
  n = len(x)
  mx, my = sum(x)/n, sum(y)/n
  sx = sum((a-mx)**2 for a in x) ** .5
  sy = sum((b-my)**2 for b in y) ** .5
  return sum((a-mx)*(b-my) for a, b in zip(x, y)) / (sx*sy + 1e-9)

def rank(v):  # average-rank tie handling (Spearman)
  idx = sorted(range(len(v)), key=lambda i: v[i])
  r = [0.0]*len(v); i = 0
  while i < len(idx):
    j = i
    while j+1 < len(idx) and v[idx[j+1]] == v[idx[i]]: j += 1
    avg = (i+j)/2 + 1
    for k in range(i, j+1): r[idx[k]] = avg
    i = j+1
  return r

def spearman(x, y):
  return pearson(rank(list(map(float, x))), rank(list(map(float, y))))

def collect(f):
  frames = {a: [] for a in set(TARGETS) | set(IDLE)}
  hud_t, hud = [], []
  for m in LogReader(f):
    if m.which() != "can": continue
    t = m.logMonoTime/1e9
    for c in m.can:
      if c.src == 2 and c.address == 0x32d:
        h, b = bar_of(bytes(c.dat)); hud_t.append(t); hud.append((h, b))
      elif c.src == 1 and c.address in frames:
        frames[c.address].append((t, bytes(c.dat)))
  return frames, hud_t, hud

def at(hud_t, hud, t):
  i = bisect.bisect_right(hud_t, t) - 1
  return hud[i] if i >= 0 else (0, 0)

def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("dir")
  ap.add_argument("--top", type=int, default=6, help="use N strongest-following segments")
  ap.add_argument("--slot", default=None, help="dump raw byte timeline for one addr, e.g. 0x382")
  ap.add_argument("--targets", default=None,
                  help="comma list of bus1 addrs to mine instead of 0x38x, e.g. 0x101,0x102,...")
  ap.add_argument("--segs", default=None, help="comma list of segment dir names; skips selection pass")
  a = ap.parse_args()
  global TARGETS
  if a.targets:
    TARGETS = [int(x, 16) for x in a.targets.split(",")]

  segs = sorted(glob.glob(f"{a.dir}/*/rlog.zst"))
  if a.segs:
    names = set(a.segs.split(","))
    segs = [f for f in segs if f.split("/")[-2] in names]
    print(f"# using {len(segs)} explicit segments")
  if a.slot:
    addr = int(a.slot, 16)
    f = segs[0]
    frames, hud_t, hud = collect(f)
    rs = frames[addr]
    print(f"# {f.split('/')[-2]} slot 0x{addr:03x}: t  bar  b2..b7")
    for t, d in rs[::8][:120]:
      h, b = at(hud_t, hud, t)
      print(f"{t:9.2f} {'*' if h else ' '}{b}  " + " ".join(f"{x:3d}" for x in d[2:8]))
    return

  # selection pass: strongest following segments (lead frames with bar<=3, bar variety)
  if a.segs:
    use = segs
    print(f"# selection skipped, using {len(use)} explicit segments")
  else:
    cand = []
    for f in segs:
      _, hud_t, hud = collect(f)
      lead = [b for h, b in hud if h]
      low = sum(1 for b in lead if b <= 3)
      if len(lead) > 2000 and low > 200 and len(set(lead)) >= 3:
        cand.append((low, f))
    cand.sort(reverse=True)
    use = [f for _, f in cand[:a.top]]
    print(f"# following segments: {len(cand)} candidates, using {len(use)}")
    for f in use: print("#   ", f.split("/")[-2])

  # per-slot analysis (decode each used segment once)
  cache = {}
  for f in use:
    cache[f] = collect(f)
  for addr in TARGETS:
    xs, ys, contig = {}, {}, {}
    idle_frames = lead_frames = 0
    hz = 0
    for f in use:
      frames, hud_t, hud = cache[f]
      rs = frames[addr]
      if not rs: continue
      hz = len(rs) / max(rs[-1][0]-rs[0][0], 1)
      prev_bar = None
      for t, d in rs:
        h, b = at(hud_t, hud, t)
        if h and b <= 6:
          lead_frames += 1
          for name, dec in FIELDS:
            xs.setdefault(name, []).append(float(dec(d))); ys.setdefault(name, []).append(b)
          if prev_bar is not None and b == prev_bar:
            for name, dec in FIELDS:
              contig.setdefault(name, []).append(abs(dec(d) - dec(prev_d)))
          prev_bar, prev_d = b, d
        elif not h:
          idle_frames += 1
    if lead_frames < 500: continue
    res = []
    for name in FIELDS:
      x, y = xs.get(name, []), ys.get(name, [])
      if len(x) < 500 or len(set(x)) < 8: continue
      rho = spearman(x, y)
      jitter = st.median(contig.get(name, [0]))
      rng = max(x) - min(x)
      res.append((abs(rho), name, rho, jitter, rng, max(x)))
    res.sort(reverse=True)
    print(f"\n0x{addr:03x} (~{hz if use else 0:.0f}Hz) lead={lead_frames} idle={idle_frames}")
    for a_abs, name, rho, jitter, rng, mx in res[:4]:
      print(f"   {name:7} rho(bar)={rho:+.3f} median|Δ|={jitter:6.1f} range={rng:.0f} max={mx:.0f}")
  # counter check on first slot
  print("\n# byte0/1 sequence sample (0x380):")
  frames, _, _ = collect(use[0])
  print("  b0:", [d[0] for _, d in frames[TARGETS[0]][:30]])
  print("  b1:", [d[1] for _, d in frames[TARGETS[0]][:30]])

if __name__ == "__main__":
  main()
