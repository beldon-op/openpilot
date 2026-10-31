#!/usr/bin/env python3
"""415-segment offline replay diff (byd-radar-multitarget-port task 2.7).

Feeds the four-layer radar chain (RadarInterface) over the archive cache
(radar_multitarget_extract.py) at slot-table level 0 and level 1 and reports:
  - CONTROL-VISIBILITY DIFF: the measured=True point sequence (what radard /
    radarState can consume) must be byte-identical between levels - the
    display-level guarantee, verified empirically over real segments;
  - slot-table availability stats (level 1): segments with live rows,
    concurrent-target histogram, trackId churn, dRel coverage - what the
    layer would offer at control level (2).

The full 415-seg RI run is ~2 h of CANParser time; default samples N
segments spread across routes for the diff, and the pure decoder (no
CANParser) sweeps the full archive for the availability stats.

  .venv/bin/python tools/byd/radar_multitarget_replay.py /tmp/mt_replay.pkl \
      [--sample 40] [--stats-sample 100]
"""
import argparse
import pickle
import sys
import time
from collections import Counter

sys.path.insert(0, __import__('os').path.abspath(__import__('os').path.join(__file__, '..', '..', '..')))
from opendbc.car import structs  # noqa: E402
from opendbc.car.byd.radar_interface import RadarInterface  # noqa: E402
from opendbc.car.byd.radar_multitarget import MultitargetDecoder, C_DIST_SCALE, C_DIST_OFFSET  # noqa: E402
from opendbc.car.byd.values import CAR  # noqa: E402

BUS2_ADDRS = {0x32D}


def make_ri(level: int) -> RadarInterface:
  CP = structs.CarParams.new_message()
  CP.carFingerprint = CAR.BYD_SONG_PLUS_DMI_22
  CP.radarUnavailable = False
  ri = RadarInterface(CP)
  ri.slot_table_level = level
  return ri


def seg_events(seg):
  """(t, [CanData...]) events in ts order from the cache layout."""
  from opendbc.car import CanData
  addrs = [a for a in range(0x101, 0x1000) if f'{a:03x}_t' in seg]
  streams = []
  for a in addrs:
    ts = seg[f'{a:03x}_t']
    dat = seg[f'{a:03x}_d']
    if not len(ts):
      continue
    src = 2 if a in BUS2_ADDRS else 1
    streams.append((a, src, ts, dat))
  events = {}
  for a, src, ts, dat in streams:
    for i, t in enumerate(ts):
      events.setdefault(t, []).append(CanData(a, dat[i * 8:(i + 1) * 8], src))
  return sorted(events.items())


def control_points(ri, events):
  """measured=True point sequence over a segment: list of (n, tuple of (id,d))"""
  seq = []
  for t, frames in events:
    ret = ri.update([(t, frames)])
    if ret is None:
      continue
    pts = tuple(sorted((p.trackId, round(p.dRel, 4))
                       for p in ret.points if p.measured))
    seq.append(pts)
  return seq


def stats_points(events, sample_every=1):
  """Pure-decoder multi-target stats for one segment (no CANParser)."""
  dec = MultitargetDecoder()
  obs = Counter()
  live_hist = Counter()
  ids = set()
  churn = 0
  n = 0
  ds = []
  for t, frames in events:
    n += 1
    for o in dec.update([(t, frames)]):
      if o.get('valid'):
        obs['points'] += 1
        ds.append(o['d'])
        if o['track_id'] not in ids:
          if o['track_id'] < 201 + 10000:
            ids.add(o['track_id'])
    if n % sample_every == 0:
      live = sum(1 for tr in dec.tracks if tr.misses == 0)
      live_hist[live] += 1
  churn = len(ids)
  return {'points': obs['points'], 'tracks': churn, 'live_hist': dict(live_hist),
          'd_min': min(ds) if ds else None, 'd_max': max(ds) if ds else None}


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('cache')
  ap.add_argument('--sample', type=int, default=40, help='segments for the RI diff')
  ap.add_argument('--stats-sample', type=int, default=100, help='segments for pure stats')
  ap.add_argument('--out', default='/tmp/mt_replay_diff.md')
  a = ap.parse_args()
  cache = pickle.load(open(a.cache, 'rb'))
  segs = sorted(cache.keys())
  step = max(1, len(segs) // a.sample)
  diff_segs = segs[::step][:a.sample]
  sstep = max(1, len(segs) // a.stats_sample)
  stats_segs = segs[::sstep][:a.stats_sample]

  lines = ['# multitarget replay diff (task 2.7)',
           f'# cache: {a.cache} ({len(segs)} segs); RI diff sample {len(diff_segs)}, '
           f'stats sample {len(stats_segs)}', '']

  # 1) control-visibility diff (level 0 vs 1)
  t0 = time.time()
  mismatches = 0
  checked = 0
  for si, sname in enumerate(diff_segs):
    events = seg_events(cache[sname])
    seq0 = control_points(make_ri(0), events)
    seq1 = control_points(make_ri(1), events)
    checked += len(seq0)
    for p0, p1 in zip(seq0, seq1):
      if p0 != p1:
        mismatches += 1
        if mismatches <= 3:
          lines.append(f'- MISMATCH {sname}: {p0} vs {p1}')
    if len(seq0) != len(seq1):
      mismatches += 1
    if (si + 1) % 10 == 0:
      print(f'# diff {si+1}/{len(diff_segs)} ({time.time()-t0:.0f}s)', flush=True)
  verdict = 'IDENTICAL' if mismatches == 0 else f'{mismatches} MISMATCHES'
  lines.append(f'## control-visible points (measured=True): {verdict} '
               f'over {checked} ticks x {len(diff_segs)} segs')
  lines.append('')

  # 2) availability stats
  t0 = time.time()
  agg = Counter()
  live_agg = Counter()
  dmin, dmax = 1e9, 0.0
  for si, sname in enumerate(stats_segs):
    st = stats_points(seg_events(cache[sname]))
    agg['segs_with_points'] += 1 if st['points'] else 0
    agg['points'] += st['points']
    agg['tracks'] += st['tracks']
    for k, v in st['live_hist'].items():
      live_agg[k] += v
    if st['d_min'] is not None:
      dmin = min(dmin, st['d_min'])
      dmax = max(dmax, st['d_max'])
    if (si + 1) % 10 == 0:
      print(f'# stats {si+1}/{len(stats_segs)} ({time.time()-t0:.0f}s)', flush=True)
  lines.append(f'## availability (level 1, {len(stats_segs)} segs)')
  lines.append(f'- segs with live rows: {agg["segs_with_points"]}/{len(stats_segs)}')
  lines.append(f'- decoded points total: {agg["points"]}, distinct trackIds: {agg["tracks"]}')
  lines.append(f'- concurrent-target histogram: {dict(sorted(live_agg.items()))}')
  lines.append(f'- dRel range: {dmin:.1f}..{dmax:.1f} m')

  report = '\n'.join(lines) + '\n'
  open(a.out, 'w').write(report)
  print(report)


if __name__ == '__main__':
  main()
