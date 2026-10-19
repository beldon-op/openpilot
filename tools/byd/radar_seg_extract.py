"""Full-frame segment cache for byd_logs_2026-10-05_07 (415 rlogs).

Supersedes sup_cache.pkl (hour_logs 4 segs only) for the 10-05_07 archive and
keeps ALL 8 bytes of the six bus1 frames (earlier tools kept only b7/presence),
so follow-on analyses (b6<->b7 modeling, 247 sentinel census, 0x101 second
path, tau-per-gear fit, T8 lead-time stats) run off one parse of the 6.9 GB
archive instead of re-reading it per analysis.

Per segment -> dict of numpy arrays (t = logMonoTime ns, int64):
  0x101/102/105/106/109/10A : (t, d[N,8] uint8)   src==1
  hud  : (t, cls int8, d[N,8]) src==2 addr 0x32D; cls = HasLead*LeadingDistance
                                            (0 = no lead; classes {0,2,3,4})
  cs   : (t, vEgo float32) from carState

Usage (background recommended, ~415 zstd rlogs):
  .venv/bin/python tools/byd/radar_seg_extract.py \
      docs_site/byd_logs_2026-10-05_07/realdata --workers 10 --out /tmp/seg1005.pkl
"""
import sys, os, glob, pickle, argparse
from multiprocessing import Pool

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANParser
from opendbc.car import Bus, CanData
from opendbc.car.byd.values import DBC, CAR

HUD_MSG = 0x32D
ADDRS = (0x101, 0x102, 0x105, 0x106, 0x109, 0x10A)


def seg_extract(f):
  hp = CANParser(DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar], [(HUD_MSG, 50)], 2)
  buf = {a: [[], []] for a in ADDRS}          # addr -> [t list, bytes list]
  hud_t, hud_cls, hud_d = [], [], []
  cs_t, cs_v = [], []
  err = None
  try:
    for m in LogReader(f):
      w = m.which()
      if w == "carState":
        cs_t.append(m.logMonoTime)
        cs_v.append(m.carState.vEgo)
        continue
      if w != "can":
        continue
      hf = []
      for c in m.can:
        if c.src == 1 and c.address in buf:
          b = buf[c.address]
          b[0].append(m.logMonoTime)
          b[1].append(bytes(c.dat))
        elif c.src == 2 and c.address == HUD_MSG:
          hf.append(CanData(c.address, bytes(c.dat), c.src))
      if hf:
        hp.update([(int(m.logMonoTime), hf)])
        v = hp.vl[HUD_MSG]
        hl = int(v["HasLead"])
        hud_t.append(m.logMonoTime)
        hud_cls.append(int(v["LeadingDistance"]) * hl if hl else 0)
        hud_d.append(hf[0].dat if hasattr(hf[0], "dat") else bytes(hf[0][1]))
  except Exception as e:  # corrupted tail: keep partial stream (diversity precedent)
    err = f"{type(e).__name__}: {str(e)[:100]}"

  out = {"err": err}
  for a in ADDRS:
    t, ds = buf[a]
    out[f"{a:03x}_t"] = np.asarray(t, np.int64)
    out[f"{a:03x}_d"] = (np.frombuffer(b"".join(ds), np.uint8).reshape(-1, 8)
                         if ds else np.zeros((0, 8), np.uint8))
  out["hud_t"] = np.asarray(hud_t, np.int64)
  out["hud_cls"] = np.asarray(hud_cls, np.int8)
  out["hud_d"] = (np.frombuffer(b"".join(hud_d), np.uint8).reshape(-1, 8)
                  if hud_d else np.zeros((0, 8), np.uint8))
  out["cs_t"] = np.asarray(cs_t, np.int64)
  out["cs_v"] = np.asarray(cs_v, np.float32)
  out["seg"] = os.path.basename(os.path.dirname(f))
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("dir")
  ap.add_argument("--workers", type=int, default=10)
  ap.add_argument("--out", default="/tmp/seg1005.pkl")
  ap.add_argument("--pattern", default="*/rlog.zst")
  a = ap.parse_args()
  segs = sorted(glob.glob(f"{a.dir}/{a.pattern}"))
  print(f"# {len(segs)} segments, workers={a.workers}", flush=True)
  with Pool(a.workers) as p:
    res = p.map(seg_extract, segs, chunksize=2)
  cache = {r["seg"]: {k: v for k, v in r.items() if k != "seg"} for r in res}
  with open(a.out, "wb") as fp:
    pickle.dump(cache, fp, protocol=4)
  nerr = sum(1 for r in res if r["err"])
  sz = os.path.getsize(a.out) / 1e6
  n109 = sum(len(r["109_t"]) for r in res)
  print(f"# done: {len(cache)} segs, {nerr} with parse err, 0x109 frames={n109}, "
        f"{sz:.1f} MB -> {a.out}", flush=True)


if __name__ == "__main__":
  main()
