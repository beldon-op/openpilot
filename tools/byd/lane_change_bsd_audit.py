#!/usr/bin/env python3
"""Offline audit of blinker-lane-change vs rear-traffic signals in BYD rlogs.

Run on the host against archived route folders (docs_site/byd_*):
    .venv/bin/python tools/byd/lane_change_bsd_audit.py <dir with */rlog.zst> [max_segments]

Reports, per run:
  - every laneChangeStarting transition with the freshest carState at that
    instant (vEgo, same-side blindspot, steeringPressed/torque, blinker hold
    length) - the gate audit asked for in docs/byd-lane-change.md section 8;
  - laneChangeBlocked event count and preLaneChange frames;
  - BSD_RADAR (0x418) raw 2-bit value distribution, cross-checked against
    carState.left/rightBlindspot;
  - whether RADAR_MRR (0x374) appears on any logged bus at all.
"""
import sys
import os
import glob
import collections

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))

from openpilot.tools.lib.logreader import LogReader
from opendbc.can.parser import CANParser
from opendbc.car.byd.values import DBC, CAR
from opendbc.car import Bus
from cereal import log

LaneChangeState = log.LaneChangeState

# (addr, freq) on bus 0
BSD_KEY = [(0x418, 10), (0x374, 20)]


def scan(path: str, seg_limit: int | None) -> None:
  files = sorted(glob.glob(os.path.join(path, "**", "rlog.zst"), recursive=True))
  if seg_limit:
    files = files[:seg_limit]
  cp = CANParser(DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.pt], BSD_KEY, 0)

  starts = []
  blocked_events = 0
  pre_frames = 0
  bs_raw = collections.Counter()
  mrr_frames = 0
  mrr_valid = 0
  cs_frames = 0
  cs_bs_true = 0
  hold_start_mono = None
  hold_t = 0.0

  for f in files:
    seg = os.path.basename(os.path.dirname(f))
    prev_state = None
    last_cs = None
    for m in LogReader(f):
      w = m.which()
      if w == "can":
        pairs = [(c.address, c.dat, c.src) for c in m.can]
        if any(p[0] == 0x374 for p in pairs):
          mrr_frames += 1
        cp.update([(m.logMonoTime, pairs)])
        vl = cp.vl
        if any(p[0] == 0x418 for p in pairs):
          bs_raw[(int(vl["BSD_RADAR"]["LEFT_APPROACH"]), int(vl["BSD_RADAR"]["RIGHT_APPROACH"]))] += 1
        if any(p[0] == 0x374 for p in pairs) and int(vl["RADAR_MRR"]["IsValid"]):
          mrr_valid += 1
      elif w == "carState":
        cs = m.carState
        cs_frames += 1
        cs_bs_true += int(cs.leftBlindspot or cs.rightBlindspot)
        if cs.leftBlinker != cs.rightBlinker:
          if hold_start_mono is None:
            hold_start_mono = m.logMonoTime
          hold_t = m.logMonoTime
        else:
          hold_start_mono = None
        last_cs = (cs.vEgo * 3.6, bool(cs.leftBlindspot), bool(cs.rightBlindspot),
                   bool(cs.steeringPressed), int(cs.steeringTorque))
      elif w == "modelV2":
        st = m.modelV2.meta.laneChangeState
        if st == LaneChangeState.preLaneChange:
          pre_frames += 1
        if st == LaneChangeState.laneChangeStarting and prev_state not in (
            LaneChangeState.laneChangeStarting, LaneChangeState.laneChangeFinishing):
          hold_dur = (hold_t - hold_start_mono) / 1e9 if hold_start_mono else 0.0
          v, lbs, rbs, sp, tq = last_cs if last_cs else (0, 0, 0, 0, 0)
          starts.append((seg[-6:], str(m.modelV2.meta.laneChangeDirection), round(v, 1),
                         lbs, rbs, sp, tq, round(hold_dur, 2)))
        prev_state = st
      elif w == "onroadEvents":
        blocked_events += sum(1 for e in m.onroadEvents if str(e.name) == "laneChangeBlocked")

  same_side_violations = sum(1 for s in starts if (s[1] == "right" and s[4]) or (s[1] == "left" and s[3]))
  print(f"segments scanned: {len(files)}")
  print(f"carState frames: {cs_frames}, blindspot-true: {cs_bs_true} ({100*cs_bs_true/max(cs_frames,1):.2f}%)")
  print(f"BSD_RADAR 0x418 (LEFT_APPROACH, RIGHT_APPROACH) raw distribution: {dict(bs_raw)}")
  print(f"preLaneChange frames: {pre_frames}, laneChangeBlocked events: {blocked_events}")
  print(f"laneChangeStarting transitions: {len(starts)}, with same-side blindspot at start: {same_side_violations}")
  for s in starts[:80]:
    print("   (seg, dir, vEgo_kph, LBS, RBS, steerPressed, torque, blinker_hold_s):", s)
  print(f"RADAR_MRR 0x374 frames on logged buses: {mrr_frames} (IsValid: {mrr_valid})")


if __name__ == "__main__":
  scan(sys.argv[1], int(sys.argv[2]) if len(sys.argv) > 2 else None)
