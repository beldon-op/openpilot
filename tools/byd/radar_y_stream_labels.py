#!/usr/bin/env python3
"""Verbatim vendor-stream label dump (byd-radar-y-wire-port 2.6, design D6).

Runs the vendor's own UnifiedShadowRadar (stream.py v72 + libobject_stream.so,
same md5 as /tmp/objsod) UNMODIFIED under the aarch64 container, driven exactly
like the production adapter: ONE decoder across the whole drive, per update_can
call -> _push_ctx (rd_set_state with random noise + rd_set_context) -> accepts
-> decode. We only OBSERVE by wrapping _SoDecoder.decode to capture (completing
wire frame <-> records) pairs; basis codes come from the stateless rd_auto_basis,
so the wire field identity can be re-established under engine-truth supervision
(2.7) instead of the failed vision-supervised scan (VERDICT 2026-11-01).

Why this can disagree with radar_y_engine_labels.py (the 2026-11-01 synthetic-
scene dump): that harness hand-rolled the ctypes binding and
  (a) never called rd_set_state (production calls it every frame),
  (b) re-anchored ts to FIXED_EPOCH=1e9 (~1 s "uptime" as seen by the engine),
  (c) fed a constant vEgo=10.
Ablation flags probe each hypothesis independently:
  --no-state      skip rd_set_state (reproduces the old harness behaviour)
  --uptime-days N monotonic base for the engine (default 2.6 d, device-like)
  --vego-floor F  feed max(vEgo, F): probes an ego-motion decode gate

Modes (design D6 ablation):
  mode wire         CAN only, no vision feeds
  mode wire+vision  CAN + modelV2 feeds via the vendor's own feed_* methods
                    (adapter.py transforms: feed_vision x-1.52/-y; geometry raw)

Sanity gates printed per segment: cross-segment duplicate (slot,d,y) triplets
(the synthetic signature), unique-value counts, y spread, and (wire+vision)
record-to-vision pairing rate.

  docker run --rm --platform linux/arm64 \
    -v /Users/wujiafu/Documents/work/op/carrotpilot/byd/byd/selfdrive/controls/lib:/vendor_lib:ro \
    -v /tmp:/cache:ro -v $PWD:/repo:ro -v /tmp/yout:/out \
    pyarmor-local python /repo/tools/byd/radar_y_stream_labels.py \
    --mode wire --limit 3 --out /out/y_stream_labels_a.pkl
"""
import argparse
import bisect
import importlib
import os
import pickle
import sys
import time
from types import SimpleNamespace

FP = 'BYD_SONG_PLUS_DMI_22'
VENDOR_LIB = '/vendor_lib'          # mount of vendor selfdrive/controls/lib/


def load_vendor(lib_dir):
  """Import the vendor package verbatim; SP semantics come from the dir name."""
  sys.path.insert(0, lib_dir)
  mod = importlib.import_module('object_stream_sp.stream')
  so = os.path.join(lib_dir, 'object_stream_sp', 'libobject_stream.so')
  print(f'[vendor] loaded {mod.__file__}', flush=True)
  print(f'[vendor] .so present: {os.path.exists(so)}', flush=True)
  return mod


def load_vego(vision_pkl):
  import numpy as np
  vis = pickle.load(open(vision_pkl, 'rb'))
  ts = []
  val = []
  rows_by_seg = {}
  for seg, blk in vis.items():
    rows = blk['rows']
    rows_by_seg[seg] = rows
    ts.extend(r[0] for r in rows)
    val.extend(r[8] for r in rows)
  order = np.argsort(np.asarray(ts, dtype=np.int64))
  print(f'[data] vEgo series: {len(ts)} samples', flush=True)
  return np.asarray(val, dtype=np.float64)[order], np.asarray(ts, dtype=np.int64)[order], rows_by_seg


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--mode', choices=['wire', 'wire+vision'], default='wire')
  ap.add_argument('--cache', default='/cache/mt_replay.pkl')
  ap.add_argument('--vision', default='/cache/y_vision.pkl')
  ap.add_argument('--out', default='/out/y_stream_labels.pkl')
  ap.add_argument('--limit', type=int, default=0, help='only first N segments (smoke)')
  ap.add_argument('--no-state', action='store_true', help='skip rd_set_state (old-harness behaviour)')
  ap.add_argument('--uptime-days', type=float, default=2.6,
                  help='monotonic base the engine sees (anti-extraction gate probe)')
  ap.add_argument('--vego-floor', type=float, default=0.0,
                  help='feed max(vEgo, floor): probes the ego-motion decode gate')
  ap.add_argument('--max-frames', type=int, default=0, help='cap frames per segment (0=all)')
  ap.add_argument('--per-seg', action='store_true',
                  help='fresh decoder + per-segment ts anchor (old harness shape)')
  a = ap.parse_args()

  vendor = load_vendor(VENDOR_LIB)
  UnifiedShadowRadar = vendor.UnifiedShadowRadar

  base_ns = int(a.uptime_days * 86_400_000_000_000)
  print(f'[cfg] mode={a.mode} uptime_base={a.uptime_days:.2f}d '
        f'state={"off" if a.no_state else "random(per-frame)"} '
        f'segments={"per-seg" if a.per_seg else "drive-grouped"}', flush=True)

  vego_v = vego_t = None
  rows_by_seg = {}
  if os.path.exists(a.vision):
    vego_v, vego_t, rows_by_seg = load_vego(a.vision)

    def vego_at(t):
      import numpy as np
      j = min(max(int(np.searchsorted(vego_t, t)), 0), len(vego_v) - 1)
      return float(vego_v[j])
  else:
    def vego_at(_t):
      return 10.0
    print('[data] no vision cache - vEgo pinned to 10.0', flush=True)

  cache = pickle.load(open(a.cache, 'rb'))
  segs = sorted(cache.keys())
  if a.limit:
    segs = segs[:a.limit]
  print(f'[data] {len(segs)} segments', flush=True)

  # production feeds ONE decoder across the whole drive. Group segments by
  # drive prefix (name = <drive>--<chunk>), keep chunks in numeric order, and
  # anchor ts once per drive (chunks are boot-monotonic, so continuity holds).
  def drive_of(seg):
    return seg.rsplit('--', 1)[0]

  def chunk_idx(seg):
    try:
      return int(seg.rsplit('--', 1)[1])
    except ValueError:
      return 0

  groups = {}
  for seg in segs:
    groups.setdefault(drive_of(seg), []).append(seg)
  drives = [(dv, sorted(ch, key=chunk_idx)) for dv, ch in sorted(groups.items())]

  result = {}
  t0wall = time.time()
  for si, (drive, seg_list) in enumerate(drives):
    shadow = UnifiedShadowRadar(FP)

    # observation wrapper: capture completing frame <-> records pairs
    cap = []
    orig_decode = shadow._decoder.decode

    def wrapped(addr, data, ts, _orig=orig_decode, _cap=cap):
      recs = _orig(addr, data, ts)
      if recs:
        _cap.append((ts, addr, bytes(data), recs))
      return recs

    shadow._decoder.decode = wrapped

    drive_t0 = None
    drive_n_rec = 0
    for seg in seg_list:
      d = cache[seg]
      frames = []
      for k_t in (k for k in d if k.endswith('_t')):
        k_d = k_t[:-2] + '_d'
        addr = int(k_t[:-2], 16)
        if addr == 0x32D:  # bus-2 HUD: engine accepts src==1 only
          continue
        ts_list = d[k_t]
        raw = d[k_d]
        if not ts_list or not raw:
          continue
        n = min(len(raw) // 8, len(ts_list))
        frames.extend((ts_list[i], addr, raw[i * 8:(i + 1) * 8]) for i in range(n))
      if not frames:
        continue
      frames.sort(key=lambda f: f[0])
      if a.max_frames:
        frames = frames[:a.max_frames]
      if a.per_seg or drive_t0 is None:
        drive_t0 = frames[0][0]
      seg_t0 = frames[0][0]

      # vision rows for this segment (mode B)
      vis_rows = rows_by_seg.get(seg, []) if a.mode == 'wire+vision' else []
      vis_ts = [r[0] for r in vis_rows]

      def vision_at(t, _rows=tuple(vis_rows), _ts=tuple(vis_ts)):
        j = bisect.bisect_left(_ts, t - 75_000_000)
        out = []
        while j < len(_rows) and _rows[j][0] <= t + 75_000_000:
          out.append(_rows[j])
          j += 1
        return out

      n_frames = n_rec = n_basis = 0
      t_start = time.time()
      for ts, addr, data in frames:
        feed_ts = ts - drive_t0 + base_ns
        ve = max(vego_at(ts), a.vego_floor)
        kw = dict(v_ego=ve, gear=4, buckled=True, door_open=False,
                  brake_pressed=False, gas_pressed=False,
                  left_blinker=False, right_blinker=False,
                  steer_angle=0.0, steer_torque=0.0, yaw_rate=0.0, a_ego=0.0)
        if a.mode == 'wire+vision':
          for row in vision_at(ts):
            _, x0, y0, p0, v0 = row[0], row[1], row[2], row[3], row[4]
            if x0 is None or y0 is None or p0 is None or p0 < 0.7:
              continue
            if not all(map(lambda z: z == z and abs(z) != float('inf'), (x0, y0))):
              continue
            model_ego = ve  # vision vRel uses model velocity; carState vEgo is the offline stand-in
            shadow.feed_vision(x0 - 1.52, -y0)
            shadow.feed_geometry_vision(x0, -y0, p0, feed_ts)
            if v0 is not None:
              shadow.feed_primary_vision(x0 - 1.52, -y0, v0 - model_ego, p0, feed_ts)
        shadow.update_can([SimpleNamespace(address=addr, src=1, dat=data)], feed_ts, **kw)
        n_frames += 1

      # flatten this segment's captures (feed-ts window = [seg_lo, seg_hi]) +
      # basis codes (stateless rd_auto_basis, engine-decoded)
      seg_lo = seg_t0 - drive_t0 + base_ns
      seg_hi = frames[-1][0] - drive_t0 + base_ns
      rows = []
      for ts, addr, data, recs in cap:
        if ts < seg_lo or ts > seg_hi:
          continue
        for rec in recs:
          if rec['flags'] & 32768:
            continue
          basis = shadow._decoder.position_basis(rec)
          rows.append((
            int(ts), int(addr), data,
            int(rec['slot']), int(rec['native']), int(rec['flags']), int(rec['profile']),
            int(rec['counter']),
            float(rec['d']), float(rec['y']), float(rec['vr']), float(rec['move']),
            tuple(float(x) for x in rec['details']) if rec['details'] else None,
            (float(basis[0]), float(basis[1])) if basis else None,
          ))
          if basis is not None:
            n_basis += 1
        n_rec += len(recs)

      # sanity: synthetic signature = few unique (slot,d,y) within the segment
      uniq = len({(r[3], round(r[8], 2), round(r[9], 2)) for r in rows if r[8] == r[8]})
      ys = [r[9] for r in rows if r[9] == r[9]]
      y_spread = (min(ys), max(ys)) if ys else (None, None)
      paired = 0
      if a.mode == 'wire+vision':
        for r in rows:
          orig_ts = r[0] - base_ns + drive_t0  # recover original logMonoTime
          for row in vision_at(orig_ts):
            if row[1] is not None and row[2] is not None and abs(r[8] - (row[1] - 1.52)) <= 2.5:
              paired += 1
              break
      rate = paired / n_rec if n_rec else 0.0
      drive_n_rec += n_rec
      result[seg] = dict(mode=a.mode, n_frames=n_frames, n_rec=n_rec, n_basis=n_basis,
                         uniq_triplets=uniq, y_range=y_spread, vision_pair_rate=rate, rows=rows)
      print(f'[{si + 1}/{len(drives)}] {seg[:24]} frames={n_frames} rec={n_rec} basis={n_basis} '
            f'uniq={uniq} y∈({y_spread[0]},{y_spread[1]}) pair={rate:.2f} '
            f'({time.time() - t_start:.1f}s)', flush=True)

  # cross-segment duplicate triplets: the 2026-11-01 synthetic signature
  seen = {}
  dup = 0
  for seg, blk in result.items():
    for r in blk['rows']:
      if r[8] != r[8]:
        continue
      key = (r[3], round(r[8], 2), round(r[9], 2))
      if key in seen and seen[key] != seg:
        dup += 1
      seen[key] = seg
  total = sum(b['n_rec'] for b in result.values())
  print(f'[sanity] records={total} cross-seg duplicate triplets={dup} '
        f'({time.time() - t0wall:.0f}s wall)', flush=True)
  print(f'[sanity] {"SYNTHETIC-suspect" if total and dup > 0.05 * total else "plausibly REAL"}', flush=True)

  pickle.dump(result, open(a.out, 'wb'))
  print(f'[out] {a.out}', flush=True)


if __name__ == '__main__':
  main()
