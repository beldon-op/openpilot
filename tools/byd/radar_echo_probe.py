"""HUD-echo falsification probe for bus1 0x109 b7 (adversarial review, 2026-10-18).

Hypothesis under attack: "0x109 b7 is forward target distance (metres)" vs the
echo alternative: b7 is just the ACC re-broadcasting the 0x32D HUD bar (or the
E2E result derived from it) onto bus1, i.e. b7 = quantise(bar), no new info.

Stage 1 (this script): extract, per segment, only what's needed and cache it:
  - bus1 0x109 frames: (t_ns, 8 bytes)
  - bus1 0x101 frames: (t_ns, 8 bytes)
  - bus2 0x32D decoded: (t_ns, HasLead, LeadingDistance)  [production CANParser]
Stage 2 (radar_echo_checks.py): checks a/b/c against the cache.

Usage:
  .venv/bin/python tools/byd/radar_echo_probe.py docs_site/hour_logs/radar_rlogs /tmp/echo_cache.pkl
"""
import sys, os, glob, pickle
from multiprocessing import Pool
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANParser
from opendbc.car import Bus, CanData
from opendbc.car.byd.values import DBC, CAR

HUD_MSG, A109, A101 = 0x32D, 0x109, 0x101


def extract(f):
  dbc = DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar]
  hp = CANParser(dbc, [(HUD_MSG, 50)], 2)
  r109, r101, hud = [], [], []
  t0 = None
  for m in LogReader(f):
    if m.which() != "can":
      continue
    if t0 is None:
      t0 = m.logMonoTime
    hf = []
    for c in m.can:
      if c.src == 1 and c.address == A109:
        r109.append((m.logMonoTime, bytes(c.dat)))
      elif c.src == 1 and c.address == A101:
        r101.append((m.logMonoTime, bytes(c.dat)))
      elif c.src == 2 and c.address == HUD_MSG:
        hf.append(CanData(c.address, bytes(c.dat), c.src))
    if hf:
      hp.update([(m.logMonoTime, hf)])
      v = hp.vl[HUD_MSG]
      hud.append((m.logMonoTime, int(v["HasLead"]), int(v["LeadingDistance"])))
  seg = os.path.basename(os.path.dirname(f))
  return seg, t0, r109, r101, hud


def main():
  segs = sorted(glob.glob(sys.argv[1] + "/*/rlog.zst"))
  out = {}
  with Pool(min(4, len(segs))) as p:
    for seg, t0, r109, r101, hud in p.imap_unordered(extract, segs):
      out[seg] = {"t0": t0, "f109": r109, "f101": r101, "hud": hud}
      print(f"{seg}: 0x109={len(r109)} 0x101={len(r101)} hud={len(hud)} t0={t0}", flush=True)
  with open(sys.argv[2], "wb") as fp:
    pickle.dump(out, fp)
  print(f"cached -> {sys.argv[2]}")


if __name__ == "__main__":
  main()
