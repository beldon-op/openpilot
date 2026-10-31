#!/usr/bin/env python3
"""Carrier-frame dump for the device-side .so probe harness (byd-radar-multitarget-port).

Extracts the full bus1 (src==1) frame stream of one rlog segment plus the
carState context the vendor stack's update_can() would see (adapter.py fills
v_ego/gear/belt/door/blinker/brake/gas per event), into a fixed-layout binary
for upload to the device. The device harness replays this stream through
libobject_stream.so with a real-frame carrier (design D1: continuous real
frames keep the engine's cross-frame slot identity; only the probed bit is
XOR-flipped in-band).

Record layout (little-endian, 36 B):
  i64 ts_ns | u16 addr | i8 gear | u8 misc | f32 steer | f32 v_ego | f32 a_ego | f32 yaw | 8B dat
misc bits: 0 belt(clear=unlatched? inverted: set=buckled), 1 door_open,
           2 left_blinker, 3 right_blinker, 4 brake_pressed, 5 gas_pressed.
gear uses the adapter mapping: 4 if gearShifter=='drive' else 0.

Usage:
  .venv/bin/python tools/byd/radar_probe_dump.py \
      ~/Documents/work/op/sunnypilot/docs_site/byd_logs_2026-10-05_07/realdata/00000001--ec0323b5d6--0 \
      --out /tmp/probe_carrier.bin
"""
import argparse
import os
import struct
import sys

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from openpilot.tools.lib.logreader import LogReader  # noqa: E402

REC = struct.Struct('<qHBbffff')  # ts, addr, gear, misc, steer, v_ego, a_ego, yaw = 28 B
HAS_YAW = None


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('segment', help='segment dir containing rlog.zst')
  ap.add_argument('--out', default='/tmp/probe_carrier.bin')
  ap.add_argument('--src', type=int, default=1, help='CAN source bus to keep')
  a = ap.parse_args()

  rlog = os.path.join(a.segment, 'rlog.zst')
  if not os.path.exists(rlog):
    sys.exit(f'no {rlog}')

  # carState carry values (adapter.py context mapping)
  cs = {'v_ego': 0.0, 'a_ego': 0.0, 'steer': 0.0, 'yaw': 0.0, 'gear': 0,
        'buckled': False, 'door': False, 'left': False, 'right': False,
        'brake': False, 'gas': False}
  global HAS_YAW

  n = 0
  t_first = None
  with open(a.out, 'wb') as fp:
    fp.write(b'BPRB' + struct.pack('<II', 1, 0))  # magic, version, count placeholder
    for m in LogReader(rlog):
      w = m.which()
      if w == 'carState':
        cs_raw = m.carState
        cs['v_ego'] = float(cs_raw.vEgo)
        cs['a_ego'] = float(cs_raw.aEgo)
        cs['steer'] = float(cs_raw.steeringAngleDeg)
        if HAS_YAW is None:
          HAS_YAW = 'yawRate' in cs_raw.to_dict()
        cs['yaw'] = float(cs_raw.yawRate) if HAS_YAW else 0.0
        cs['gear'] = 4 if str(cs_raw.gearShifter) == 'drive' else 0
        cs['buckled'] = not bool(cs_raw.seatbeltUnlatched)
        cs['door'] = bool(cs_raw.doorOpen)
        cs['left'] = bool(cs_raw.leftBlinker)
        cs['right'] = bool(cs_raw.rightBlinker)
        cs['brake'] = bool(cs_raw.brakePressed)
        cs['gas'] = bool(cs_raw.gasPressed)
      elif w == 'can':
        misc = (cs['buckled'] | cs['door'] << 1 | cs['left'] << 2 | cs['right'] << 3 |
                cs['brake'] << 4 | cs['gas'] << 5)
        for c in m.can:
          if c.src != a.src or c.address > 0x7FF:
            continue  # extended CAN (29-bit) frames: engine rejects them anyway
          ts = m.logMonoTime
          if t_first is None:
            t_first = ts
          dat = bytes(c.dat)[:8].ljust(8, b'\0')
          fp.write(REC.pack(ts, c.address, cs['gear'], misc,
                            cs['steer'], cs['v_ego'], cs['a_ego'], cs['yaw']) + dat)
          n += 1
  # patch count
  with open(a.out, 'r+b') as fp:
    fp.seek(8)
    fp.write(struct.pack('<I', n))
  print(f'# {n} frames, t0={t_first}, {os.path.getsize(a.out)/1e6:.1f} MB -> {a.out}')


if __name__ == '__main__':
  main()
