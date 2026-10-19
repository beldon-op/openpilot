"""Per-bus CAN census over the 10-05_07 rlog archive - where could a rear radar live?

Motivation: bus1 0x10x (front MRR, docs/byd-radar-010x.md) proved the archive
contains traffic beyond the known bus0/bus2 wiring narrative. BSD lane-change
work (docs/byd-lane-change.md sec 8) needs a *rear* target source (0x418 gives
only two blind-spot bits). This census answers, purely from logs:
  1. which src buses appear at all, and their address sets;
  2. per (src,addr): frame count, median period, distinct DLC;
  3. per (src,addr): byte-column distinct counts + SAE J1850 CRC-8 b0 hit-rate
     on a 512-frame reservoir sample (the 0x10x radar family signature:
     poly 0x1D / init 0xFF / no reflect / xorout 0xFF over b1..b7);
  4. also records where the known supervision frames live: 0x418 BSD bits,
     0x32D HUD lead gears, carState vEgo.

Usage:
  .venv/bin/python tools/byd/radar_bus3_census.py \
      docs_site/byd_logs_2026-10-05_07/realdata --workers 10 \
      --out /tmp/bus3_census.json [--max-segs N]
"""
import sys, os, glob, json, argparse, collections
from multiprocessing import Pool

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np
from openpilot.tools.lib.logreader import LogReader

RESERVOIR = 512
WATCH = {0x418: "BSD_RADAR", 0x32D: "ACC_HUD_ADAS", 0x374: "RADAR_MRR"}


def crc_j1850(data, init=0xFF, xorout=0xFF, lo=1, hi=8):
  crc = init
  for byte in data[lo:hi]:
    crc ^= byte
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ xorout


CRC_VARIANTS = [  # (name, init, xorout, lo, hi)
  ("J1850 b1..b7", 0xFF, 0xFF, 1, 8),
  ("J1850 b0..b6", 0xFF, 0xFF, 0, 7),
  ("J1850 i0 x0", 0x00, 0x00, 1, 8),
  ("J1850 iFF x00", 0xFF, 0x00, 1, 8),
]


def seg_census(f):
  cnt = collections.Counter()                       # (src,addr) -> frames
  last_t = {}                                       # (src,addr) -> last ts
  dts = collections.defaultdict(lambda: [0.0, 0.0, 0])  # sum, sumsq, n (ns)
  dlcs = collections.defaultdict(set)
  samples = collections.defaultdict(list)           # (src,addr) -> reservoir of bytes
  rng = np.random.default_rng(abs(hash(os.path.basename(os.path.dirname(f)))) % 2**32)
  err = None
  try:
    for m in LogReader(f):
      if m.which() != "can":
        continue
      for c in m.can:
        k = (c.src, c.address)
        n = cnt[k]
        cnt[k] = n + 1
        dlcs[k].add(len(c.dat))
        t = m.logMonoTime
        if k in last_t:
          d = t - last_t[k]
          if 0 < d < 5 * 1_000_000_000:             # ignore segment-join gaps
            s = dts[k]
            s[0] += d; s[1] += d * d; s[2] += 1
        last_t[k] = t
        dat = bytes(c.dat)
        if len(samples[k]) < RESERVOIR:
          samples[k].append(dat)
        else:                                       # reservoir swap keeps unbiased bytes
          j = rng.integers(n + 1)
          if j < RESERVOIR:
            samples[k][j] = dat
  except Exception as e:
    err = f"{type(e).__name__}: {str(e)[:100]}"

  out = {}
  for k, n in cnt.items():
    src, addr = k
    s = dts.get(k, [0.0, 0, 0])
    mean_ns = s[0] / s[2] if s[2] else None
    arr = np.frombuffer(b"".join(samples[k]), np.uint8).reshape(-1, len(samples[k][0])) \
        if len({len(x) for x in samples[k]}) == 1 else None
    ent = {"frames": int(n), "dlcs": sorted(dlcs[k]), "med_dt_ms": round(mean_ns / 1e6, 2) if mean_ns else None}
    if arr is not None and arr.shape[1] == 8:
      ent["byte_distinct"] = [int(len(np.unique(arr[:, j]))) for j in range(8)]
      ent["crc"] = {}
      for name, init, xorout, lo, hi in CRC_VARIANTS:
        ent["crc"][name] = sum(1 for row in samples[k] if crc_j1850(row, init, xorout, lo, hi) == row[0])
      ent["crc_n"] = len(samples[k])   # 8-byte sample count for cross-seg pooling
    out[f"{src}/{addr:03x}"] = ent
  return {"seg": os.path.basename(os.path.dirname(f)), "err": err, "addrs": out}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("dir")
  ap.add_argument("--workers", type=int, default=10)
  ap.add_argument("--out", default="/tmp/bus3_census.json")
  ap.add_argument("--pattern", default="*/rlog.zst")
  ap.add_argument("--max-segs", type=int, default=0)
  a = ap.parse_args()
  segs = sorted(glob.glob(f"{a.dir}/{a.pattern}"))
  if a.max_segs:
    segs = segs[:a.max_segs]
  print(f"# {len(segs)} segments, workers={a.workers}", flush=True)
  with Pool(a.workers) as p:
    res = p.map(seg_census, segs, chunksize=2)

  agg = {}
  for r in res:
    for k, ent in r["addrs"].items():
      g = agg.setdefault(k, {"frames": 0, "segs": 0, "dlcs": set(), "dt_wsum": 0.0, "dt_n": 0,
                             "crc_hits": collections.Counter(), "crc_n": 0,
                             "bd_max": [0]*8})
      g["frames"] += ent["frames"]; g["segs"] += 1
      g["dlcs"].update(ent["dlcs"])
      if ent["med_dt_ms"]:
        g["dt_wsum"] += ent["med_dt_ms"] * ent["frames"]; g["dt_n"] += ent["frames"]
      if ent.get("crc"):
        g["crc_n"] += ent.get("crc_n", 0)          # once per seg, not per variant
      for name, hits in ent.get("crc", {}).items():
        g["crc_hits"][name] += hits
      if "byte_distinct" in ent:
        g["bd_max"] = [max(x, y) for x, y in zip(g["bd_max"], ent["byte_distinct"])]

  rows = []
  for k, g in agg.items():
    src, addr = k.split("/")
    rows.append({"key": k, "src": int(src), "addr": int(addr, 16), "frames": g["frames"],
                 "segs": g["segs"], "dlcs": sorted(g["dlcs"]),
                 "med_dt_ms": round(g["dt_wsum"] / g["dt_n"], 2) if g["dt_n"] else None,
                 "crc": {n: round(h / g["crc_n"], 3) for n, h in g["crc_hits"].items()} if g["crc_n"] else {},
                 "byte_distinct_sample_max": g["bd_max"]})
  rows.sort(key=lambda r: (r["src"], -r["frames"]))

  by_bus = collections.Counter()
  for r in rows:
    by_bus[r["src"]] += r["frames"]
  print(f"# buses seen (frames): {dict(sorted(by_bus.items()))}")
  for src in sorted({r['src'] for r in rows}):
    rs = [r for r in rows if r["src"] == src]
    print(f"\n== bus {src}: {len(rs)} addresses ==")
    for r in rs:
      watch = WATCH.get(r["addr"], "")
      crc = max(r["crc"].items(), key=lambda kv: kv[1])[0] + f"={max(r['crc'].values()):.2f}" if r["crc"] else ""
      bd8 = r["byte_distinct_sample_max"] and max(r["byte_distinct_sample_max"])
      flag = "  <== CRC radar-family" if r["crc"] and max(r["crc"].values()) > 0.9 else ""
      print(f"  {r['addr']:#06x} {r['frames']:>9}f {r['segs']:>3}seg dt={r['med_dt_ms']}ms "
            f"dlcs={r['dlcs']} bd={r['byte_distinct_sample_max']} {crc}{flag} {watch}")
  with open(a.out, "w") as fp:
    json.dump({"segs": len(segs), "rows": rows, "bus_frames": dict(sorted(by_bus.items()))}, fp, indent=1)
  print(f"\n# -> {a.out}")


if __name__ == "__main__":
  main()
