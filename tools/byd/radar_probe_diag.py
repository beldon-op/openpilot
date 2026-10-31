#!/usr/bin/env python3
"""Decisive A/B: drive the FULL production UnifiedShadowRadar (stream.py) with
the same carrier frames the raw harness feeds, per segment. If the class path
produces real tracks where the raw path doesn't, diff the two call sequences;
if neither does, the difference vs the 10-31 validation is environmental.

  python3 diag_class.py probe_carrier.bin [probe_carrier12.bin ...]
"""
import sys
import os
import time
import ctypes as ct
from types import SimpleNamespace

SP = '/data/openpilot/selfdrive/controls/lib/object_stream_sp'
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, SP)
from radar_probe_run import load_dump, Engine  # noqa
import stream  # the vendor's own module (stdlib-only)  # noqa

FP = 'BYD_SONG_PLUS_DMI_22'


def run_class(dump_path, remap=True, v_ego_mode='real'):
  frames = load_dump(dump_path)
  frames.sort(key=lambda r: r[0])
  t0 = frames[0][0]
  t1 = frames[-1][0] + 1
  shadow = stream.UnifiedShadowRadar(FP)
  anchor = time.monotonic_ns() - (t1 - t0) if remap else 0

  # group frames into can events (same log ts = one event, like production)
  events = {}
  for (ts, addr, gear, misc, steer, v_ego, a_ego, yaw, dat) in frames:
    if not (t0 <= ts < t1):
      continue
    events.setdefault(ts, []).append((addr, gear, misc, steer, v_ego, a_ego, yaw, dat))

  n_events = 0
  last_ctx_v = 0.0
  for ts in sorted(events):
    ts_f = anchor + (ts - t0) if remap else ts
    items = [SimpleNamespace(address=a, src=1, dat=d) for (a, g, mi, st, ve, ae, yw, d)
             in events[ts]]
    last_ctx_v = 0.0 if v_ego_mode == 'zero' else events[ts][0][4]
    first = events[ts][0]
    shadow.update_can(items, ts_f,
                      v_ego=last_ctx_v,
                      gear=int(first[1]), buckled=bool(first[2] & 1),
                      door_open=bool(first[2] & 2),
                      left_blinker=bool(first[2] & 4), right_blinker=bool(first[2] & 8),
                      brake_pressed=bool(first[2] & 16), gas_pressed=bool(first[2] & 32),
                      steer_angle=float(first[3]), steer_torque=0.0,
                      yaw_rate=float(first[6]), v_lead=0.0, vision_dist=0.0, v_rel=0.0,
                      a_ego=float(first[5]))
    n_events += 1

  st = dict(shadow._stats)
  dec = shadow._decoder
  tracks = {}
  for tid, tr in list(shadow._tracks.items())[:8]:
    try:
      tracks[str(tid)] = {k: (round(v, 2) if isinstance(v, float) else v)
                          for k, v in dict(tr).items()
                          if isinstance(v, (int, float))}
    except Exception:
      tracks[str(tid)] = repr(tr)[:120]
  try:
    leads = shadow.display_leads(last_ctx_v, now_ns=anchor + (t1 - t0))
    leads_s = [(round(l.get('dRel', float('nan')), 1), round(l.get('yRel', float('nan')), 2),
                round(l.get('vRel', float('nan')), 2)) for l in leads[:6]]
  except Exception as e:
    leads_s = f'ERR {e}'
  print(f'[class {os.path.basename(dump_path)} remap={remap} vego={v_ego_mode}] '
        f'events={n_events} stats={st} profile={dec.profile()}')
  print(f'   tracks({len(shadow._tracks)}):', tracks)
  print(f'   display_leads:', leads_s)
  return shadow


def main():
  for dump in sys.argv[1:]:
    run_class(dump, remap=True, v_ego_mode='real')
    run_class(dump, remap=True, v_ego_mode='zero')


if __name__ == '__main__':
  main()
