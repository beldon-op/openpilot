"""Full-archive radar scan of docs_site/byd_logs_2026-10-05_07 (416 segments).

Question set (driven by the mypilot community dbcall repo + commits 5c1066e/9a5e488):
  1. 0x351-0x354 tangdm slots on bus1 - ANY frame with IsValid=1 or TrackID!=0
     across the whole archive? (first real frames would calibrate the TODOs)
  2. 0x374 RADAR_MRR - the community byd源码 radar_interface parses it
     (TargetID 1:left/2:front/3:right, 60Hz). Earlier audit said 0 frames;
     re-check every bus, every segment.
  3. Complete per-bus address histogram - surface radar-ish IDs we never mined.
  4. 0x380-0x38f - re-confirm the status-table verdict on this archive
     (b0 counter deltas + payload diversity).
  5. 0x32D HUD - HasLead/bar distribution for fallback-path sanity.

Track/HUD fields are decoded by the production CANParser using our own
byd_general_pt.dbc, so scan semantics == ship semantics.

Usage:
  .venv/bin/python tools/byd/radar_1005_scan.py docs_site/byd_logs_2026-10-05_07/realdata [--workers 8] [--json out.json]
"""
import sys, os, glob, json, argparse, collections
from multiprocessing import Pool
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANParser
from opendbc.car import Bus, CanData
from opendbc.car.byd.values import DBC, CAR

TRACK_MSGS = [0x351, 0x352, 0x353, 0x354]
HUD_MSG = 0x32D
MRR_MSG = 0x374
C38X = list(range(0x380, 0x390))
# smoke run surfaced two never-mined bus1 blocks: 0x101-0x10A @25Hz (same tick
# as the track slots) and 0x340-0x34A @~14Hz (Tang dbc names MSG_341/344/345
# with target-style 0.0625/-40 fields). Probe their payload diversity.
PROBE_MSGS = [0x101, 0x102, 0x103, 0x104, 0x105, 0x106, 0x107, 0x108, 0x109, 0x10A] + \
             [0x340, 0x341, 0x342, 0x343, 0x344, 0x345, 0x346, 0x347, 0x348, 0x349, 0x34A]


def _parsers():
  radar_dbc = DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar]
  tp = CANParser(radar_dbc, [(a, 25) for a in TRACK_MSGS], 1)
  hp = CANParser(radar_dbc, [(HUD_MSG, 50)], 2)
  return tp, hp


def _hexp(d):
  return bytes(d).hex()


def seg_analyze(f):
  tp, hp = _parsers()
  hist = collections.Counter()                       # (src, addr) -> n
  track = {a: collections.defaultdict(int) for a in TRACK_MSGS}
  for a in TRACK_MSGS:
    track[a]["payloads"] = collections.Counter()
    track[a]["tid"] = collections.Counter()
    track[a]["min"] = {k: None for k in ("d", "y", "v")}
    track[a]["max"] = {k: None for k in ("d", "y", "v")}
  mrr = collections.defaultdict(lambda: {"n": 0, "payloads": collections.Counter(), "buses": collections.Counter()})
  c38 = {a: {"n": 0, "b0d": collections.Counter(), "payloads": collections.Counter()} for a in C38X}
  probe = {a: collections.Counter() for a in PROBE_MSGS}
  hud = collections.defaultdict(int)
  hud["bars"] = collections.Counter()
  hud["transitions"] = 0
  prev = None

  for m in LogReader(f):
    if m.which() != "can":
      continue
    tf, hf = [], []
    for c in m.can:
      a, s, d = c.address, c.src, bytes(c.dat)
      hist[(s, a)] += 1
      if s == 1 and a in track:
        tf.append(CanData(a, d, s))
        track[a]["payloads"][_hexp(d)] += 1
      if s == 1 and a in probe and len(probe[a]) < 60:
        probe[a][_hexp(d)] += 1
      if a == MRR_MSG:
        mrr[a]["n"] += 1
        mrr[a]["buses"][s] += 1
        if len(mrr[a]["payloads"]) < 40:
          mrr[a]["payloads"][_hexp(d)] += 1
      if a in c38:
        row = c38[a]
        row["n"] += 1
        if "last_b0" in row:
          row["b0d"][(d[0] - row["last_b0"]) % 256] += 1
        row["last_b0"] = d[0]
        if len(row["payloads"]) < 40:
          row["payloads"][_hexp(d)] += 1
      if s == 2 and a == HUD_MSG:
        hf.append(CanData(a, d, s))
    if tf:
      tp.update([(m.logMonoTime, tf)])
      for a in TRACK_MSGS:
        if a not in tp.vl:
          continue
        v = tp.vl[a]
        row = track[a]
        row["n"] += 1
        row["valid"] += int(v["IsValid_tang"])
        tid = int(v["TrackID_tang"])
        if tid:
          row["tid_nonzero"] += 1
          row["tid"][tid] += 1
        for key, name in (("d", "LongDist_tang"), ("y", "LatDist_tang"), ("v", "RelSpeed_tang")):
          x = float(v[name])
          if row["min"][key] is None or x < row["min"][key]:
            row["min"][key] = x
          if row["max"][key] is None or x > row["max"][key]:
            row["max"][key] = x
    if hf:
      hp.update([(m.logMonoTime, hf)])
      v = hp.vl[HUD_MSG]
      h, b = int(v["HasLead"]), int(v["LeadingDistance"])
      hud["n"] += 1
      hud["lead"] += h
      hud["bars"][b] += h
      if prev is not None and (h, b) != prev:
        hud["transitions"] += 1
      prev = (h, b)

  return {
    "seg": os.path.basename(os.path.dirname(f)),
    "hist": {f"{s:03x}:{a:03x}": n for (s, a), n in hist.items()},
    "track": {hex(a): {k: (dict(v) if isinstance(v, collections.Counter) else v)
                       for k, v in row.items() if k != "last_b0"} for a, row in track.items()},
    "mrr": {hex(MRR_MSG): {"n": mrr[MRR_MSG]["n"], "buses": dict(mrr[MRR_MSG]["buses"]),
                           "payloads": mrr[MRR_MSG]["payloads"].most_common(5)}},
    "c38": {hex(a): {"n": row["n"], "b0d": dict(row["b0d"]), "payloads": row["payloads"].most_common(3)}
            for a, row in c38.items()},
    "probe": {hex(a): row.most_common(3) for a, row in probe.items()},
    "hud": {"n": hud["n"], "lead": hud["lead"], "bars": dict(hud["bars"]), "transitions": hud["transitions"]},
  }


def merge(results):
  out = {}
  hist = collections.Counter()
  track = {hex(a): collections.defaultdict(lambda: 0) for a in TRACK_MSGS}
  tpay = {hex(a): collections.Counter() for a in TRACK_MSGS}
  ttid = {hex(a): collections.Counter() for a in TRACK_MSGS}
  tmin = {hex(a): {k: None for k in "dyv"} for a in TRACK_MSGS}
  tmax = {hex(a): {k: None for k in "dyv"} for a in TRACK_MSGS}
  mrr_n, mrr_buses, mrr_pay = 0, collections.Counter(), collections.Counter()
  c38 = {hex(a): {"n": 0, "b0d": collections.Counter(), "payloads": collections.Counter()} for a in C38X}
  probe = {hex(a): collections.Counter() for a in PROBE_MSGS}
  hud = collections.Counter()
  hud["bars"] = collections.Counter()
  hud_segs_with_lead = 0
  for r in results:
    hist.update(r["hist"])
    for a, row in r["track"].items():
      for k in ("n", "valid", "tid_nonzero"):
        track[a][k] += row.get(k, 0)
      tpay[a].update(row["payloads"])
      ttid[a].update(row["tid"])
      for key in "dyv":
        if row.get("min") and row["min"][key] is not None:
          cur = tmin[a][key]
          tmin[a][key] = row["min"][key] if cur is None else min(cur, row["min"][key])
          cur = tmax[a][key]
          tmax[a][key] = row["max"][key] if cur is None else max(cur, row["max"][key])
    m = r["mrr"][hex(MRR_MSG)]
    mrr_n += m["n"]
    mrr_buses.update(m["buses"])
    mrr_pay.update(m["payloads"])
    for a, row in r["c38"].items():
      c38[a]["n"] += row["n"]
      c38[a]["b0d"].update(row["b0d"])
      c38[a]["payloads"].update(row["payloads"])
    for a, pay in r["probe"].items():
      for h, n in pay:
        probe[a][h] += n
    h = r["hud"]
    hud["n"] += h["n"]; hud["lead"] += h["lead"]; hud["transitions"] += h["transitions"]
    hud["bars"].update(h["bars"])
    if h["lead"] > 0:
      hud_segs_with_lead += 1

  print(f"# segments scanned: {len(results)}")
  print(f"\n## bus1 track slots (tangdm layout, production CANParser)")
  for a in [hex(x) for x in TRACK_MSGS]:
    row = track[a]
    print(f"{a}: n={row.get('n',0)} IsValid=1:{row.get('valid',0)} TrackID!=0:{row.get('tid_nonzero',0)} "
          f"distinct_payloads={len(tpay[a])}")
    if tpay[a]:
      print(f"   top payload: {tpay[a].most_common(1)[0]}")
      print(f"   tid hist: {ttid[a].most_common(4)}")
      print(f"   field ranges d[{tmin[a]['d']:.2f},{tmax[a]['d']:.2f}] y[{tmin[a]['y']:.2f},{tmax[a]['y']:.2f}] "
            f"v[{tmin[a]['v']:.0f},{tmax[a]['v']:.0f}]")
  print(f"\n## 0x374 RADAR_MRR: total frames {mrr_n}  per bus {dict(mrr_buses) or '{}'}")
  if mrr_pay:
    print(f"   payloads: {mrr_pay.most_common(5)}")

  print("\n## 0x38x counter/status re-check")
  for a in [hex(x) for x in C38X]:
    row = c38[a]
    if not row["n"]:
      continue
    bd = row["b0d"]
    tot = sum(bd.values())
    dominant = max(bd.items(), key=lambda kv: kv[1]) if tot else None
    print(f"{a}: n={row['n']} distinct<=40 payload_capped, b0 delta dominant={dominant[0] if dominant else '-'}"
          f"({(dominant[1]/tot if tot else 0):.0%} of deltas), top payload {row['payloads'].most_common(1)}")

  print("\n## bus1 probe blocks 0x101-0x10A / 0x340-0x34A (payload diversity, per-seg cap 60)")
  for a in [hex(x) for x in PROBE_MSGS]:
    pc = probe[a]
    tot = sum(pc.values())
    if not tot:
      print(f"{a}: no frames")
      continue
    dcnt = len(pc)
    verdict = "CONSTANT" if dcnt == 1 else "state-ish" if dcnt <= 12 else "RICH"
    print(f"{a}: sampled={tot} distinct={dcnt} [{verdict}] top={pc.most_common(2)}")

  print(f"\n## 0x32D HUD: frames={hud['n']} lead={hud['lead']} ({hud['lead']/max(hud['n'],1):.1%}) "
          f"bar-switches={hud['transitions']} segments_with_lead={hud_segs_with_lead}/{len(results)}")
  print(f"   bars: {sorted(hud['bars'].items())}")

  print("\n## per-bus address histogram (top 25 by count; * = not in prior radar scope)")
  scope = {f"{s:03x}:{a:03x}" for s in range(3) for a in TRACK_MSGS + [HUD_MSG, MRR_MSG, 0x350]}
  scope |= {f"{s:03x}:{x:03x}" for s in range(3) for x in C38X}
  for k, n in hist.most_common(25):
    print(f"   {k}  {n:>9}  {'*' if k not in scope else ''}")
  print("\n## bus1-only top 30 (new-ID surface)")
  b1 = {k: v for k, v in hist.items() if k.startswith("001:")}
  for k, n in sorted(b1.items(), key=lambda kv: -kv[1])[:30]:
    print(f"   {k}  {n:>9}  {'*' if k not in scope else ''}")
  lead_by_seg = sorted(((r["hud"]["lead"], r["seg"]) for r in results), reverse=True)
  print("# top segments by lead-frames (rank-consistency candidates):")
  for lead, s in lead_by_seg[:12]:
    print(f"   {s}  lead={lead}")
  out = {"track": {a: dict(track[a]) for a in track}, "mrr_frames": mrr_n,
         "hud": {"n": hud["n"], "lead": hud["lead"]}, "hist_bus1": dict(b1),
         "probe": {a: {"distinct": len(pc), "top": pc.most_common(3)} for a, pc in probe.items()},
         "lead_by_seg": lead_by_seg[:20]}
  return out


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("dir")
  ap.add_argument("--workers", type=int, default=8)
  ap.add_argument("--limit", type=int, default=0)
  ap.add_argument("--json", default=None)
  a = ap.parse_args()
  segs = sorted(glob.glob(f"{a.dir}/*/rlog.zst"))
  if a.limit:
    segs = segs[:a.limit]
  with Pool(a.workers) as p:
    results = p.map(seg_analyze, segs, chunksize=2)
  agg = merge(results)
  if a.json:
    with open(a.json, "w") as fp:
      json.dump(agg, fp, indent=1, default=str)


if __name__ == "__main__":
  main()
