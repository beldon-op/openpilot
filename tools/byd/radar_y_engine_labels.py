#!/usr/bin/env python3
"""Engine label dump for the offline lateral-y calibration (byd-radar-y-wire-port 1.1).

Feeds the archived real bus-1 frames (radar_multitarget_extract.py cache) through
the vendor libobject_stream.so under an aarch64 container, and dumps per-record
engine outputs PLUS the inverse-projected (base, code) pair (rd_auto_basis) so the
wire field identity for the angle code can be established by regression (2.1)
instead of blind search.

Discipline carried over from the probe harness (docs/byd-radar-multitarget-re-
2026-10-31.md §0 四坑):
  - only feeds addresses the engine accepts (rd_auto_accepts, src==1)
  - per-segment begin_pass: choose_shared(0) + reset + choose_vehicle(fp)
  - logMonoTime re-anchored to a fresh monotonic base (stale-guard)
  - records keyed by the completing frame (addr, ts, first bytes) for wire alignment

  docker run --rm --platform linux/arm64 \
    -v /tmp/objsod:/so:ro -v /tmp:/cache:ro -v <repo>:/repo:ro -v /tmp/yout:/out \
    pyarmor-local python /repo/tools/byd/radar_y_engine_labels.py \
    --so /so/libobject_stream.so --cache /cache/mt_replay.pkl --out /out/labels.pkl
"""
import argparse
import ctypes as _ct
import hashlib as _hl
import os
import pickle
import re
import sys
import time

BASELINE_CORE_REVISION = 'dbc-core-20261030.1'
FP = 'BYD_SONG_PLUS_DMI_22'
FIXED_EPOCH = 1_000_000_000  # re-anchor base: segments start at t=+1s, monotonic

# Engine lateral model for profile 2, recovered stateless 2026-11-01 (oracle
# grid, worst identity error 1e-5, closed form <=1.1 mm):
#   y = (base + d_off) * K * (code - c0);  base = d + 1.671 (= 0.062319 * wire);
#   defaults c0 = -15 (code units), d_off = -1.671 m  =>  geometry(basis(d,y)) = (d,y).
# implied code from a record: code = y / (d * K) - 15
K = 0.00050091


def version_gate(so_path, stream_py):
  """CORE_REVISION/TELEMETRY gate (probe_run precedent): refuse on core drift."""
  src = open(stream_py, encoding='utf-8', errors='replace').read()
  core = re.search(r"CORE_REVISION\s*=\s*'([^']+)'", src)
  tel = re.search(r"TELEMETRY_VERSION\s*=\s*'([^']+)'", src)
  core = core.group(1) if core else '?'
  tel = tel.group(1) if tel else '?'
  print(f"[gate] TELEMETRY_VERSION={tel} CORE_REVISION={core}", flush=True)
  if core != BASELINE_CORE_REVISION:
    print(f"[gate] REFUSING: CORE_REVISION drifted from {BASELINE_CORE_REVISION} "
          f"- rerun the probe baseline first", flush=True)
    sys.exit(2)
  if tel not in ('v72', 'v73'):
    print(f"[gate] note: telemetry version {tel} unseen before (labels still valid: "
          f".so unchanged, python-only delta)", flush=True)


# ── minimal ctypes binding, adapted from vendor stream.py:355-514 ─────────────
class _VehicleContext(_ct.Structure):
  _fields_ = [('ts', _ct.c_uint64)] + [(n, _ct.c_float) for n in
             ('ego', 'accel', 'steer', 'torque', 'yaw', 'lead', 'vision', 'relative')] + \
             [(n, _ct.c_int32) for n in ('gear', 'belt', 'door', 'left', 'right', 'brake', 'gas')]


class _NativeTarget(_ct.Structure):
  _fields_ = [(name, _ct.c_float) for name in ('d', 'y', 'vr', 'va', 'a', 'vl')] + [
      (name, _ct.c_int32) for name in ('native', 'slot', 'flags', 'profile', 'age', 'state')] + [
      ('ts', _ct.c_uint64), ('move', _ct.c_float), ('counter', _ct.c_int32),
      ('closing', _ct.c_float), ('detail_mask', _ct.c_uint32), ('details', _ct.c_float * 32)]


DETAIL_NAMES = ('headingDeg', 'lateralAccelerationCandidate', 'confidenceRaw', 'uncertaintyRaw',
                'width', 'azimuthRad', 'rightEdgeRad', 'leftEdgeRad', 'normalizedRangeRate',
                'nativeRelativeVelocity', 'radarEgoSpeed', 'radarYawRateDeg', 'quality0',
                'quality1', 'quality2', 'quality3', 'quality4', 'quality5', 'positionMaturityX',
                'positionMaturityY', 'trackMaturity', 'motionState', 'qualityFlag', 'qualityStatus',
                'trackUnstable', 'velocitySaturated', 'measurementPoor', 'objectFlag', 'stateCode',
                'headerStatus', 'headerConstant', 'headerCounter')

FLOAT_FIELDS = [n for n, _ in _NativeTarget._fields_ if n not in ('details',)]
INT_FIELDS = ['native', 'slot', 'flags', 'profile', 'age', 'state', 'counter', 'detail_mask']


def load_lib(so_path):
  lib = _ct.CDLL(so_path)
  lib.rd_auto_abi.argtypes = []
  lib.rd_auto_abi.restype = _ct.c_int
  if lib.rd_auto_abi() != ((66 << 16) | _ct.sizeof(_NativeTarget)):
    raise OSError('Shared decoder ABI mismatch')
  lib.rd_init.argtypes = [_ct.POINTER(_ct.c_uint8), _ct.c_int]
  lib.rd_init.restype = None
  lib.rd_set_state.argtypes = [_ct.c_int] * 6
  lib.rd_set_state.restype = None
  lib.rd_set_context.argtypes = [_ct.POINTER(_VehicleContext), _ct.c_int]
  lib.rd_set_context.restype = _ct.c_int
  lib.rd_auto_reset.argtypes = []
  lib.rd_auto_reset.restype = None
  lib.rd_auto_profile.argtypes = []
  lib.rd_auto_profile.restype = _ct.c_int
  lib.rd_auto_accepts.argtypes = [_ct.c_int]
  lib.rd_auto_accepts.restype = _ct.c_int
  lib.rd_auto_feed.argtypes = [_ct.c_int, _ct.POINTER(_ct.c_uint8), _ct.c_int,
                               _ct.c_uint64, _ct.c_float, _ct.POINTER(_NativeTarget), _ct.c_int]
  lib.rd_auto_feed.restype = _ct.c_int
  lib.rd_auto_basis.argtypes = [_ct.c_int, _ct.c_float, _ct.c_float, _ct.POINTER(_ct.c_float)]
  lib.rd_auto_basis.restype = _ct.c_int
  lib.rd_auto_geometry.argtypes = [_ct.c_int] + [_ct.c_float] * 4 + [_ct.POINTER(_ct.c_float)]
  lib.rd_auto_geometry.restype = _ct.c_int
  lib.rd_auto_choose_shared.argtypes = [_ct.c_int]
  lib.rd_auto_choose_shared.restype = _ct.c_int
  lib.rd_auto_choose_vehicle.argtypes = [_ct.c_char_p]
  lib.rd_auto_choose_vehicle.restype = _ct.c_int
  lib.rd_auto_shared_seen.argtypes = []
  lib.rd_auto_shared_seen.restype = _ct.c_int
  key = _hl.sha256(b'').digest()[:2]  # universal package: empty dump id
  lib.rd_init((_ct.c_uint8 * len(key))(*key), len(key))
  return lib


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('--so', default='/so/libobject_stream.so')
  ap.add_argument('--cache', default='/cache/mt_replay.pkl')
  ap.add_argument('--out', default='/out/y_engine_labels.pkl')
  ap.add_argument('--stream-py', default=None, help='vendor stream.py for the version gate')
  ap.add_argument('--limit', type=int, default=0, help='only first N segments (smoke run)')
  ap.add_argument('--force-profile', type=int, default=0,
                  help='rd_auto_choose_shared(N) instead of vehicle-identity auto')
  ap.add_argument('--no-ctx', action='store_true', help='skip per-frame rd_set_context')
  ap.add_argument('--real-vego', default=None,
                  help='y_vision.pkl: feed the real per-frame vEgo (carState) instead of 10.0')
  a = ap.parse_args()

  if a.stream_py is None:
    cand = os.path.join(os.path.dirname(a.so), 'stream.py')
    a.stream_py = cand if os.path.exists(cand) else '/dev/null'
  if os.path.exists(a.stream_py):
    version_gate(a.so, a.stream_py)
  else:
    print('[gate] stream.py not provided - skipping version gate', flush=True)

  lib = load_lib(a.so)
  out_buf = (_NativeTarget * 64)()
  accepts_cache = {}

  vego_ts = vego_v = None
  if a.real_vego:
    import numpy as np
    vis = pickle.load(open(a.real_vego, 'rb'))
    vego_ts = np.asarray([r[0] for rows in vis.values() for r in rows['rows']], dtype=np.int64)
    vego_v = np.asarray([r[8] for rows in vis.values() for r in rows['rows']], dtype=np.float64)
    order = np.argsort(vego_ts)
    vego_ts, vego_v = vego_ts[order], vego_v[order]
    print(f'# real vEgo series: {len(vego_ts)} samples', flush=True)

  def vego_at(t):
    if vego_ts is None:
      return 10.0
    j = np.searchsorted(vego_ts, t)
    j = min(max(j, 0), len(vego_v) - 1)
    return float(vego_v[j])

  cache = pickle.load(open(a.cache, 'rb'))
  segs = sorted(cache.keys())
  if a.limit:
    segs = segs[:a.limit]
  print(f'# {len(segs)} segments', flush=True)

  result = {}
  t_start = time.time()
  n_rec_total = 0
  for si, seg in enumerate(segs):
    d = cache[seg]
    # merge all per-address streams into one time-ordered frame list
    frames = []
    for k_t in (k for k in d if k.endswith('_t')):
      k_d = k_t[:-2] + '_d'
      addr = int(k_t[:-2], 16)
      if addr == 0x32D:      # bus-2 HUD frame: engine only accepts src==1
        continue
      ts_list = d[k_t]
      raw = d[k_d]
      if not ts_list or not raw:
        continue
      n = len(raw) // 8
      for i in range(min(n, len(ts_list))):
        frames.append((ts_list[i], addr, raw[i * 8:(i + 1) * 8]))
    if not frames:
      continue
    frames.sort(key=lambda f: f[0])
    t0 = frames[0][0]

    # begin_pass (probe-harness discipline): shared+reset+vehicle identity
    if a.force_profile:
      hint = lib.rd_auto_choose_shared(a.force_profile)
      lib.rd_auto_reset()
    else:
      lib.rd_auto_choose_shared(0)
      lib.rd_auto_reset()
      hint = lib.rd_auto_choose_vehicle(FP.encode())
    profile = lib.rd_auto_profile()
    # per-frame context (stream.py:1277) WITHOUT rd_set_state: any rd_set_state
    # call suppresses all real records (everything becomes empty-slot flags,
    # bisected 2026-11-01: zero noise, random noise, per-frame all identical).
    # rd_auto_geometry/basis need no state either (stateless diag, same day).
    ctx = _VehicleContext(FIXED_EPOCH, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                          4, 1, 0, 0, 0, 0, 0)
    if not a.no_ctx:
      lib.rd_set_context(_ct.byref(ctx), _ct.sizeof(ctx))

    # column store (compact): labels + wire alignment keys
    cols = {k: [] for k in ('t', 'slot', 'native', 'counter', 'flags', 'd', 'y',
                            'code', 'addr', 'hdr', 'cnt')}
    prof_seen = {}
    frame_i = 0
    raw_out = flagged = n_kill = n_range = 0
    for ts, addr, data in frames:
      if addr not in accepts_cache:
        accepts_cache[addr] = bool(lib.rd_auto_accepts(addr))
      if not accepts_cache[addr]:
        continue
      frame_i += 1
      if frame_i % 5000 == 0:
        p_now = lib.rd_auto_profile()
        prof_seen[p_now] = prof_seen.get(p_now, 0) + 1
      feed_ts = _ct.c_uint64(ts - t0 + FIXED_EPOCH)
      ve = vego_at(ts)
      if not a.no_ctx:
        ctx.ts = int(feed_ts.value)
        ctx.ego = ve
        lib.rd_set_context(_ct.byref(ctx), _ct.sizeof(ctx))
      buf = (_ct.c_uint8 * len(data))(*data)
      n = lib.rd_auto_feed(addr, buf, len(data), feed_ts, ve, out_buf, 64)
      raw_out += n
      for i in range(n):
        item = out_buf[i]
        if item.flags & 32768:   # empty-slot marker
          flagged += 1
          continue
        # NOTE: flags&256 = "velocity is decoder-estimated" marker, set on
        # essentially ALL Song records (wire has no native doppler) - NOT a kill
        # bit. Collecting regardless; wire-side bit27 kill discipline stays in
        # our own decoder.
        # closed-form implied code (identity recovered 2026-11-01, worst err 1e-5):
        #   y = (base + d_off)*K*(code - c0), base = d + 1.671, defaults c0=-15, d_off=-1.671
        if not (2.0 <= item.d <= 80.0) or abs(item.y) > 3.5 or item.d <= 0:
          n_range += 1
          continue
        code = item.y / (item.d * K) - 15.0
        cols['t'].append(ts)
        cols['slot'].append(item.slot)
        cols['native'].append(item.native)
        cols['counter'].append(item.counter)
        cols['flags'].append(item.flags)
        cols['d'].append(item.d)
        cols['y'].append(item.y)
        cols['code'].append(code)
        cols['addr'].append(addr)
        cols['hdr'].append(data[0])
        cols['cnt'].append(data[1])
    # dedupe: engine re-emits the same object on every completing frame of a
    # cycle - keep the last emission per (cycle counter, slot). numpy conversion
    # per segment keeps RSS bounded (python lists OOM-killed the container).
    import numpy as np
    arr = {k: np.asarray(v) for k, v in cols.items()}
    if len(arr['d']):
      key = (arr['cnt'].astype(np.int64) << 8) | arr['slot'].astype(np.int64)
      _, keep_idx = np.unique(key, return_index=True)   # unique keeps LAST via flip
      keep_idx = np.unique(key[::-1], return_index=True)[1]
      keep_idx = len(key) - 1 - keep_idx                # map back to original order
      keep_idx.sort()
      arr = {k: v[keep_idx] for k, v in arr.items()}
    result[seg] = {'hint': hint, 'profile': profile, 'profile_seen': prof_seen,
                   'raw_out': raw_out, 'flagged': flagged, 'n_kill': n_kill,
                   'n_range': n_range, 'cols': arr}
    n_rec_total += len(arr['d'])
    if (si + 1) % 25 == 0 or si == len(segs) - 1:
      print(f'# {si+1}/{len(segs)} segs, {n_rec_total} recs ({time.time()-t_start:.0f}s)', flush=True)

  os.makedirs(os.path.dirname(a.out), exist_ok=True)
  with open(a.out, 'wb') as fp:
    pickle.dump(result, fp, protocol=4)

  # summary
  n_seg = len(result)
  profiles = {}
  prof_seen_all = {}
  raw_all = flag_all = kill_all = range_all = 0
  dmin, dmax, ymin, ymax = 1e9, -1e9, 1e9, -1e9
  for r in result.values():
    profiles[r['profile']] = profiles.get(r['profile'], 0) + 1
    for k, v in r['profile_seen'].items():
      prof_seen_all[k] = prof_seen_all.get(k, 0) + v
    raw_all += r['raw_out']
    flag_all += r['flagged']
    kill_all += r['n_kill']
    range_all += r['n_range']
    ds, ys = r['cols']['d'], r['cols']['y']
    if len(ds):
      dmin, dmax = min(dmin, float(ds.min())), max(dmax, float(ds.max()))
      ymin, ymax = min(ymin, float(ys.min())), max(ymax, float(ys.max()))
  print(f'# raw feed outputs={raw_all}, flagged(empty-slot)={flag_all}, '
        f'kill-bit={kill_all}, out-of-range={range_all}')
  print(f'# done: {n_seg} segs, {n_rec_total} recs, profiles_at_begin={profiles}, '
        f'profile_samples_during={prof_seen_all}, d=[{dmin:.1f},{dmax:.1f}], '
        f'y=[{ymin:.2f},{ymax:.2f}] -> {a.out}', flush=True)


if __name__ == '__main__':
  main()
