"""Full-bus1 census + velocity-supervision cache for hour_logs (2026-09-28, 4 rlogs).

Why this exists (gaps in prior mining):
  - radar_re_mine.py MI scan supervised bus1 bytes against the discrete HUD bar
    CLASS only. A lead SPEED field (roughly constant during steady following,
    which is exactly what these 4 engaged segments are) has near-zero MI against
    a distance class and would be systematically missed.
  - radar_sup_mine.py vEgo regression covered ONLY the 0x10x six addrs.
  - The relV verdicts (0x10x 486-seg, slot67 325-window) all used the closing
    speed supervisor -d(b7*u)/dt, whose variance degenerates to ~0 in steady
    following; an ABSOLUTE lead-speed field (ARS410 sends vLead too) is invisible
    to that supervisor but not to vEgo.

Stages:
  census  <logdir>                 print per-address rate/period/entropy vs known families
  extract <logdir> <out.pkl>       cache ALL src==1 frames + vEgo + aEgo + HUD

Usage:
  .venv/bin/python tools/byd/radar_hour_census.py census docs_site/hour_logs/radar_rlogs
  .venv/bin/python tools/byd/radar_hour_census.py extract docs_site/hour_logs/radar_rlogs /tmp/hour_bus1.pkl
"""
import sys, os, glob, math, pickle, collections

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np

HUD_MSG = 0x32D
SEC = 1_000_000_000

# families already registered in docs/byd-radar-* (10-05_07 corpus)
KNOWN = {0x101, 0x102, 0x105, 0x106, 0x109, 0x10A, 0x301, 0x418}
KNOWN |= set(range(0x340, 0x34B))
KNOWN |= set(range(0x380, 0x398))
KNOWN |= set(range(0x3C8, 0x3E4))


def byte_entropy(col):
  n = len(col)
  if n == 0:
    return 0.0
  h = 0.0
  for v in collections.Counter(col).values():
    p = v / n
    h -= p * math.log2(p)
  return h


def parse_seg(f):
  from openpilot.tools.lib.logreader import LogReader
  from opendbc.can import CANParser
  from opendbc.car import Bus, CanData
  from opendbc.car.byd.values import DBC, CAR

  dbc = DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar]
  hp = CANParser(dbc, [(HUD_MSG, 50)], 2)
  frames = collections.defaultdict(list)
  hud, vego, accel = [], [], []
  for m in LogReader(f):
    if m.which() == "can":
      hf = []
      for c in m.can:
        if c.src == 1:
          frames[c.address].append((m.logMonoTime, bytes(c.dat)) if len(c.dat) == 8 else (m.logMonoTime, bytes(c.dat)[:8].ljust(8, b"\x00")))
        elif c.src == 2 and c.address == HUD_MSG:
          hf.append(CanData(c.address, bytes(c.dat), c.src))
      if hf:
        hp.update([(m.logMonoTime, hf)])
        v = hp.vl[HUD_MSG]
        hud.append((m.logMonoTime, int(v["HasLead"]), int(v["LeadingDistance"]), int(v["AccState"])))
    elif m.which() == "carState":
      vego.append((m.logMonoTime, float(m.carState.vEgo)))
      if m.carState.aEgo is not None:
        accel.append((m.logMonoTime, float(m.carState.aEgo)))
  seg = os.path.basename(os.path.dirname(f))
  out = {}
  for a, lst in frames.items():
    t = np.array([x[0] for x in lst], dtype="int64")
    d = np.frombuffer(b"".join(x[1] for x in lst), dtype="uint8").reshape(-1, 8)
    out[int(a)] = (t, d)
  return seg, out, hud, vego, accel


def load_all(segs_paths, workers=1):
  if workers > 1:
    from multiprocessing import Pool
    with Pool(workers) as p:
      results = p.map(parse_seg, segs_paths)
  else:
    results = [parse_seg(f) for f in segs_paths]
  cache = {}
  for seg, frames, hud, vego, accel in results:
    cache[seg] = {"frames": frames, "hud": hud, "vego": vego, "accel": accel}
    print(f"{seg}: bus1 addrs={len(frames)} frames={sum(len(t) for t,_ in frames.values())} "
          f"hud={len(hud)} vego={len(vego)}", flush=True)
  return cache


def census(cache):
  agg = {}
  for seg, s in sorted(cache.items()):
    for a, (t, d) in s["frames"].items():
      if len(t) < 20:
        continue
      dur = max((t[-1] - t[0]) / SEC, 1e-9)
      info = agg.setdefault(a, {"n": 0, "ft": 0.0, "ent": collections.defaultdict(list), "segs": 0})
      info["n"] += len(t)
      info["ft"] += dur
      info["segs"] += 1
      for i in range(8):
        info["ent"][i].append(byte_entropy(d[:, i]))
  print(f"\n{'addr':>6} {'n':>9} {'Hz':>6} {'ent/b8avg':>9} {'minByteEnt':>10}  status")
  new = []
  for a in sorted(agg):
    info = agg[a]
    ents = [float(np.mean(info["ent"][i])) for i in range(8)]
    mark = ""
    if a not in KNOWN:
      mark = "  <== NOT IN KNOWN FAMILIES"
      new.append(a)
    print(f"0x{a:03x} {info['n']:>9} {info['n']/info['ft']:>6.1f} {np.mean(ents):>9.2f} {min(ents):>10.2f}{mark}")
  print(f"\nnew/unregistered: {[hex(x) for x in new] or 'NONE'}")
  known_missing = sorted(KNOWN - set(agg))
  print(f"known families absent in hour_logs: {[hex(x) for x in known_missing] or 'NONE'}")


def main():
  mode, logdir = sys.argv[1], sys.argv[2]
  # census <logdir> [workers] | extract <logdir> [workers] <out.pkl>
  rest = sys.argv[3:]
  workers = 1
  out = None
  if mode == "census" and rest:
    workers = int(rest[0])
  if mode == "extract":
    if len(rest) == 1:
      out = rest[0]
    elif len(rest) >= 2:
      workers, out = int(rest[0]), rest[1]
  segs = sorted(glob.glob(logdir + "/*/rlog.zst"))
  if not segs:
    sys.exit(f"no rlogs under {logdir}")
  if out is None and mode == "extract":
    sys.exit("extract needs an output path: extract <logdir> [workers] <out.pkl>")
  cache = load_all(segs, workers)
  if mode == "census":
    census(cache)
  else:
    with open(out, "wb") as fp:
      pickle.dump(cache, fp)
    print(f"cached -> {out}")


if __name__ == "__main__":
  main()
