#!/usr/bin/env python3
"""Device-side probe harness: real-frame carrier through libobject_stream.so +
in-band single-bit probes (design D1, byd-radar-multitarget-port stage 1).

Runs on the device (aarch64, stdlib only) against the vendor build's
object_stream_sp/libobject_stream.so. Feeds a continuous REAL bus1 frame
stream (radar_probe_dump.py output) so the engine's cross-frame slot identity
stays alive, XOR-flips exactly one payload bit in K consecutive frames of one
address, and diffs the decoder outputs against an unflipped baseline pass.
delta = scale * 2^(bit-start) attributes the bit to a (field, layout) entry.

Version gate: prints CORE_REVISION/TELEMETRY_VERSION and refuses to probe when
they drift from the dbc-core-20261030.1 baseline (spec: re-run the baseline and
confirm the diff by hand first). Determinism self-check: the baseline pass is
run twice and diffed; nonzero drift means the engine is not state-deterministic
under this harness (then retry with --noise zero).

Usage (device):
  python3 radar_probe_run.py /data/tmp/probe_carrier.bin \
      --window-begin 0 --window-len 15 --out /data/tmp/probe_table.json \
      [--max-probes 4] [--bits 16-63] [--flip-k 3] [--flip-frac 0.6]
      [--addrs 380-397,3c8-3cb,3d0,301] [--noise random|zero]
"""
import argparse
import ctypes as ct
import hashlib
import json
import os
import re
import struct
import sys
import time

BASELINE_CORE_REVISION = 'dbc-core-20261030.1'
BASELINE_TELEMETRY_VERSION = 'v72'
FP_SONG = b'BYD_SONG_PLUS_DMI_22'
KEY = hashlib.sha256(b'').digest()[:2]  # universal package key (e3 b0)

FIELDS_F = ('d', 'y', 'vr', 'va', 'a', 'vl', 'move', 'closing')
FIELDS_I = ('slot', 'counter', 'flags', 'native', 'age', 'state')
EPS_F = 1e-4

from array import array


class _NativeTarget(ct.Structure):
  _fields_ = [(n, ct.c_float) for n in ('d', 'y', 'vr', 'va', 'a', 'vl')] + [
      (n, ct.c_int32) for n in ('native', 'slot', 'flags', 'profile', 'age', 'state')] + [
      ('ts', ct.c_uint64), ('move', ct.c_float), ('counter', ct.c_int32),
      ('closing', ct.c_float), ('detail_mask', ct.c_uint32), ('details', ct.c_float * 32)]


class _VehicleContext(ct.Structure):
  _fields_ = [('ts', ct.c_uint64)] + [(n, ct.c_float) for n in
      ('ego', 'accel', 'steer', 'torque', 'yaw', 'lead', 'vision', 'relative')] + [
      (n, ct.c_int32) for n in ('gear', 'belt', 'door', 'left', 'right', 'brake', 'gas')]


def j1850_crc8(payload: bytes) -> int:
  """SAE J1850 CRC-8: poly 0x1D, init 0xFF, no reflection, xorout 0xFF."""
  crc = 0xFF
  for b in payload:
    crc ^= b
    for _ in range(8):
      crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
  return crc ^ 0xFF


def version_gate(sp_dir: str) -> dict:
  so_path = os.path.join(sp_dir, 'libobject_stream.so')
  ver = open(os.path.join(sp_dir, 'VERSION')).read().strip()
  src = open(os.path.join(sp_dir, 'stream.py')).read()
  core = re.search(r"CORE_REVISION\s*=\s*'([^']+)'", src)
  tel = re.search(r"TELEMETRY_VERSION\s*=\s*'([^']+)'", src)
  core = core.group(1) if core else '?'
  tel = tel.group(1) if tel else '?'
  sha = hashlib.sha256(open(so_path, 'rb').read()).hexdigest()
  info = {'core_revision': core, 'telemetry_version': tel, 'version': ver, 'so_sha256': sha,
          'so_path': so_path}
  print(f"[gate] VERSION={ver} CORE_REVISION={core} TELEMETRY_VERSION={tel}")
  print(f"[gate] so sha256={sha}")
  if core != BASELINE_CORE_REVISION or tel != BASELINE_TELEMETRY_VERSION:
    print(f"[gate] DRIFT: baseline is CORE_REVISION={BASELINE_CORE_REVISION} "
          f"TELEMETRY_VERSION={BASELINE_TELEMETRY_VERSION} -> refusing to probe. "
          f"Re-run the baseline and confirm the diff by hand first.")
    sys.exit(2)
  print(f"[gate] baseline match, proceeding")
  return info


class Engine:
  """Minimal production-faithful wrapper of the vendor _SoDecoder path."""

  def __init__(self, sp_dir, noise='random'):
    lib = ct.CDLL(os.path.join(sp_dir, 'libobject_stream.so'))
    lib.rd_auto_abi.argtypes, lib.rd_auto_abi.restype = [], ct.c_int
    abi = lib.rd_auto_abi()
    if abi != ((66 << 16) | ct.sizeof(_NativeTarget)):
      sys.exit(f'[abi] mismatch: {abi} != {(66 << 16) | ct.sizeof(_NativeTarget)}')
    lib.rd_init.argtypes = [ct.POINTER(ct.c_uint8), ct.c_int]
    lib.rd_init.restype = None
    lib.rd_set_state.argtypes = [ct.c_int] * 6
    lib.rd_set_state.restype = None
    lib.rd_set_context.argtypes = [ct.POINTER(_VehicleContext), ct.c_int]
    lib.rd_set_context.restype = ct.c_int
    lib.rd_auto_reset.argtypes, lib.rd_auto_reset.restype = [], None
    lib.rd_auto_profile.argtypes, lib.rd_auto_profile.restype = [], ct.c_int
    lib.rd_auto_accepts.argtypes, lib.rd_auto_accepts.restype = [ct.c_int], ct.c_int
    lib.rd_auto_choose_shared.argtypes, lib.rd_auto_choose_shared.restype = [ct.c_int], ct.c_int
    lib.rd_auto_choose_vehicle.argtypes, lib.rd_auto_choose_vehicle.restype = [ct.c_char_p], ct.c_int
    lib.rd_auto_feed.argtypes = [ct.c_int, ct.POINTER(ct.c_uint8), ct.c_int,
                                 ct.c_uint64, ct.c_float, ct.POINTER(_NativeTarget), ct.c_int]
    lib.rd_auto_feed.restype = ct.c_int
    lib.rd_init((ct.c_uint8 * len(KEY))(*KEY), len(KEY))
    self.lib = lib
    self.out = (_NativeTarget * 64)()
    self.noise = noise

  def begin_pass(self):
    self.lib.rd_auto_choose_shared(0)
    self.lib.rd_auto_reset()
    hint = self.lib.rd_auto_choose_vehicle(FP_SONG)
    self.hint = hint
    return hint

  def push_ctx(self, ts, v_ego, a_ego, steer, yaw, gear, misc):
    if self.noise == 'random':
      noise = os.urandom(20)
      f = [int.from_bytes(noise[i:i + 4], 'little') & 0x7FFFFFFF for i in range(0, 20, 4)]
    else:
      f = [0, 0, 0, 0, 0]
    vf = 1 if misc & 1 else 0
    self.lib.rd_set_state(f[0], f[1], vf, f[2], f[3], f[4])
    c = _VehicleContext(int(ts), float(v_ego), float(a_ego), float(steer), 0.0, float(yaw),
                        0.0, 0.0, 0.0, int(gear), vf, 1 if misc & 2 else 0,
                        1 if misc & 4 else 0, 1 if misc & 8 else 0,
                        1 if misc & 16 else 0, 1 if misc & 32 else 0)
    return bool(self.lib.rd_set_context(ct.byref(c), ct.sizeof(c)))

  def feed(self, addr, buf, ts, v_ego):
    n = self.lib.rd_auto_feed(int(addr), buf, len(buf), int(ts), float(v_ego),
                              self.out, 64)
    recs = []
    for i in range(n):
      t = self.out[i]
      # first 8 = FIELDS_F, last 6 = FIELDS_I (Capture.add slices depend on this)
      recs.append((t.d, t.y, t.vr, t.va, t.a, t.vl, t.move, t.closing,
                   t.slot, t.counter, t.flags, t.native, t.age, t.state))
    return recs


def load_dump(path):
  raw = open(path, 'rb').read()
  if raw[:4] != b'BPRB':
    sys.exit('bad dump magic')
  count = struct.unpack_from('<I', raw, 8)[0]
  recs = []
  off = 12
  for _ in range(count):
    ts, addr, gear, misc, steer, v_ego, a_ego, yaw = struct.unpack_from('<qHBbffff', raw, off)
    dat = raw[off + 28: off + 36]
    recs.append((ts, addr, gear, misc, steer, v_ego, a_ego, yaw, dat))
    off += 36
  return recs


def parse_addrs(spec):
  out = []
  for part in spec.split(','):
    if '-' in part:
      lo, hi = part.split('-')
      out.extend(range(int(lo, 16), int(hi, 16) + 1))
    else:
      out.append(int(part, 16))
  return out


def parse_bits(spec):
  out = []
  for part in spec.split(','):
    if '-' in part:
      lo, hi = part.split('-')
      out.extend(range(int(lo), int(hi) + 1))
    else:
      out.append(int(part))
  return sorted(set(out))


class Capture:
  """Flat-column capture of pass outputs, diffable against another capture.
  ft: per-feed log time (seconds from window start) for region filtering -
  the parked warm-up phase emits nondeterministic decoys that must be
  excluded from pass diffs (see diag: deterministic from ~5 s onward)."""

  def __init__(self):
    self.n = array('B')
    self.ft = array('f')
    self.cols_f = {k: array('f') for k in FIELDS_F}
    self.cols_i = {k: array('i') for k in FIELDS_I}

  def add(self, recs, t_sec):
    self.n.append(len(recs))
    self.ft.append(t_sec)
    for r in recs:
      for k, v in zip(FIELDS_F, r[:8]):
        self.cols_f[k].append(v)
      for k, v in zip(FIELDS_I, r[8:14]):
        self.cols_i[k].append(v)
    if len(self.cols_i['state']) != len(self.cols_f['d']):
      raise AssertionError('capture column mismatch')  # layout guard

  def __len__(self):
    return len(self.n)

  def rec(self, i, j):
    off = sum(self.n[:i]) + j  # only used in tests; hot path uses direct scan
    return tuple(self.cols_f[k][off] for k in FIELDS_F) + \
        tuple(self.cols_i[k][off] for k in FIELDS_I)


def run_pass(engine, frames, t0, t1, flips, capture: Capture, ts_anchor=None):
  """flips: {(addr, frame_seq_of_addr): set(bit_positions)}; CRC recomputed.
  ts_anchor: if set, feed timestamps are remapped to (anchor + ts - t0) so the
  engine's freshness logic sees a CURRENT clock (the 10-31 feed scripts needed
  the same monotonic remap; raw logMonoTime from an old recording reads as
  permanently stale and the engine emits residual/decoy tracks)."""
  bufs = {}
  seq_of = {}
  idx = 0
  total = 0
  for (ts, addr, gear, misc, steer, v_ego, a_ego, yaw, dat) in frames:
    if ts < t0 or ts >= t1:
      continue
    if not engine.lib.rd_auto_accepts(int(addr)):
      continue  # production filters rejected addrs BEFORE decode (0x351-354
              # heartbeats fed raw would pollute the engine's slot state)
    t_sec = (ts - t0) / 1e9  # log-time axis, BEFORE any anchor remap
    if ts_anchor is not None:
      ts = ts_anchor + (ts - t0)
    s = seq_of.get(addr, 0)
    seq_of[addr] = s + 1
    d = dat
    fl = flips.get((addr, s))
    if fl:
      d = bytearray(d)
      for b in fl:
        d[b // 8] ^= 1 << (b % 8)
      d[0] = j1850_crc8(d[1:8])  # keep the frame CRC-valid after the payload flip
      d = bytes(d)
    buf = bufs.get(d)
    if buf is None:
      buf = (ct.c_uint8 * 8)(*d)
      bufs[d] = buf
    engine.push_ctx(ts, v_ego, a_ego, steer, yaw, gear, misc)
    capture.add(engine.feed(addr, buf, ts, v_ego), t_sec)
    total += 1
    idx += 1
  return total


def diff_pass(base: Capture, probe: Capture, changed_at, diff_after=-1.0):
  """Accumulate (field,slot)->summary for probe-vs-baseline differences.
  changed_at: dict to receive (field,slot)->first feed index.
  diff_after: only compare feeds with log-time >= this (skip the parked
  warm-up region, whose decoy outputs are nondeterministic by design)."""
  summary = {}
  if len(base.n) != len(probe.n):
    return {'FATAL': f'feed count differs {len(base.n)} vs {len(probe.n)}'}
  bi = pi = 0
  for i in range(len(base.n)):
    if base.ft[i] < diff_after:
      bi += base.n[i]
      pi += probe.n[i]
      continue
    nb, np_ = base.n[i], probe.n[i]
    if nb != np_:
      e = summary.setdefault('RECORD_COUNT', {'max_abs': 0, 'first_feed': i,
                                              'first_delta': float(np_ - nb), 'feeds': 0})
      e['max_abs'] = max(e['max_abs'], abs(np_ - nb))
      e['feeds'] += 1
    for j in range(min(nb, np_)):
      bo, po = bi + j, pi + j
      sb, sp = base.cols_i['slot'][bo], probe.cols_i['slot'][po]
      for k in FIELDS_F:
        dv = probe.cols_f[k][po] - base.cols_f[k][bo]
        if abs(dv) > EPS_F:
          key = (k, sp if sp == sb else f'{sb}->{sp}')
          e = summary.setdefault(key, {'max_abs': 0.0, 'first_delta': 0.0,
                                       'first_feed': i, 'feeds': 0})
          e['max_abs'] = max(e['max_abs'], abs(dv))
          if e['feeds'] == 0:
            e['first_delta'] = dv
            changed_at[key] = i
          e['feeds'] += 1
      for k in FIELDS_I:
        dv = probe.cols_i[k][po] - base.cols_i[k][bo]
        if dv != 0:
          key = (k, sp if sp == sb else f'{sb}->{sp}')
          e = summary.setdefault(key, {'max_abs': 0.0, 'first_delta': float(dv),
                                       'first_feed': i, 'feeds': 0})
          e['max_abs'] = max(e['max_abs'], abs(dv))
          if e['feeds'] == 0:
            e['first_feed'] = i
          e['feeds'] += 1
    bi += nb
    pi += np_
  return summary


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('dump')
  ap.add_argument('--sp-dir', default='/data/openpilot/selfdrive/controls/lib/object_stream_sp')
  ap.add_argument('--window-begin', type=float, default=0.0)
  ap.add_argument('--window-len', type=float, default=0.0,
                  help='seconds; 0 = to end of dump (full-segment carrier is '
                       'required for track establishment, diag-verified)')
  ap.add_argument('--diff-after', type=float, default=10.0,
                  help='skip feeds before this log-time in diffs/determinism '
                       '(parked warm-up decoys are nondeterministic)')
  ap.add_argument('--flip-at', type=float, default=30.0,
                  help='log-time (s) where probe flips begin (driving region)')
  ap.add_argument('--out', default='/data/media/0/probe/probe_table.json')
  ap.add_argument('--addrs', default='380-397,3c8-3cb,3d0,301')
  ap.add_argument('--bits', default='16-63')
  ap.add_argument('--flip-k', type=int, default=3)
  ap.add_argument('--noise', choices=['random', 'zero'], default='zero',
                  help='rd_set_state noise args: zero = deterministic passes '
                       '(default; production uses urandom which breaks pass '
                       'reproducibility), random = production-faithful')
  ap.add_argument('--max-probes', type=int, default=0, help='0 = all')
  ap.add_argument('--skip-determinism', action='store_true')
  ap.add_argument('--batch', action='store_true',
                  help='batch probes by frame-type group: one pass flips the '
                       'same bit on every address of a group (6x fewer passes; '
                       'deltas attribute per-slot, collisions re-run solo)')
  ap.add_argument('--dump-baseline', action='store_true',
                  help='serialize the baseline driving-region records for the '
                       'layout solver')
  ap.add_argument('--resume', action='store_true',
                  help='skip probes already recorded in <out>.jsonl (the bench '
                       'device reboots under load; per-probe checkpointing '
                       'makes the campaign re-entrant)')
  a = ap.parse_args()

  info = version_gate(a.sp_dir)
  frames = load_dump(a.dump)
  frames.sort(key=lambda r: r[0])
  t0 = frames[0][0] + int(a.window_begin * 1e9)
  t1 = t0 + int(a.window_len * 1e9) if a.window_len > 0 else frames[-1][0] + 1
  win = [f for f in frames if t0 <= f[0] < t1]
  print(f"[carrier] {len(frames)} frames total, window {a.window_begin}s+"
        f"{'end' if a.window_len == 0 else a.window_len}s = {len(win)} frames")
  addrs = parse_addrs(a.addrs)
  bits = parse_bits(a.bits)

  # CRC self-check on the whole window: the J1850 family must be 100% here too
  bad = sum(1 for f in win if f[8][0] != j1850_crc8(f[8][1:8]))
  print(f"[crc] window CRC self-check: {len(win) - bad}/{len(win)} pass ({bad} bad)")
  if bad:
    print("[crc] WARNING: unexpected bad-CRC frames in carrier - layout attribution "
          "may be polluted by engine-side rejection")

  engine = Engine(a.sp_dir, noise=a.noise)
  win_ns = int((t1 - t0))

  # each pass anchors its feed clock to "now" (freshness gates see current
  # monotonic time); determinism across passes relies on the engine being
  # ts-delta-driven, which the double-baseline check verifies (restricted to
  # the deterministic region: parked warm-up decoys are nondeterministic).
  # begin_pass() (choose_shared+reset+choose_vehicle) is REQUIRED per pass:
  # without the fingerprint the engine runs unidentified (hint 0) and never
  # decodes real targets - the diag scripts call it, the runner must too.
  base = Capture()
  engine.begin_pass()
  n = run_pass(engine, frames, t0, t1, {}, base, ts_anchor=time.monotonic_ns() - win_ns)
  base2 = Capture()
  engine.begin_pass()
  n2 = run_pass(engine, frames, t0, t1, {}, base2, ts_anchor=time.monotonic_ns() - win_ns)
  drift = 0
  if not a.skip_determinism:
    d = diff_pass(base, base2, {}, diff_after=a.diff_after)
    drift = sum(e['feeds'] for e in d.values() if isinstance(e, dict) and 'feeds' in e)
    det_detail = {str(k): v for k, v in d.items() if k != 'FATAL'}
    print(f"[determinism] double-baseline diff feeds (t>={a.diff_after}s): {drift}"
          + (f" detail={json.dumps(det_detail)[:300]}" if det_detail else '')
          + (f" FATAL={d.get('FATAL')}" if 'FATAL' in d else ''))
    if drift:
      print("[determinism] WARNING: engine is not state-deterministic under this "
            "harness; deltas below may be noise. Try --noise zero.")

  # baseline summary: live slots in the deterministic region only (empty-slot
  # markers excluded - their NaNs would otherwise mask the real targets)
  slots = {}
  off = 0
  for i in range(len(base.n)):
    ni = base.n[i]
    if base.ft[i] >= a.diff_after:
      for j in range(ni):
        idx = off + j
        if base.cols_i['flags'][idx] & 32768:
          continue
        s = base.cols_i['slot'][idx]
        e = slots.setdefault(s, {'n': 0, 'd': [], 'y': [], 'vr': [], 'flags': set()})
        e['n'] += 1
        e['flags'].add(base.cols_i['flags'][idx])
        if len(e['d']) < 3:
          e['d'].append(round(base.cols_f['d'][idx], 2))
          e['y'].append(round(base.cols_f['y'][idx], 2))
          e['vr'].append(round(base.cols_f['vr'][idx], 2))
    off += ni
  for e in slots.values():
    e['flags'] = sorted(e['flags'])
  print(f"[baseline] {n} feeds, {sum(base.n)} records, live slots (t>={a.diff_after}s): "
        f"{ {s: v['d'] for s, v in sorted(slots.items())} }")

  # probe plan: flip the K consecutive frames of addr whose log time starts at
  # flip_at (driving region; parked decoys would drown deltas). Flip keys are
  # PER-ADDR frame counters (run_pass counts each address's frames within the
  # window) - win indices would never match (the earlier zero-delta mystery).
  per_addr_ctr = {}
  flip_frames = {}   # addr -> [per-addr counter of the first flip_at frames]
  for f in win:
    ctr = per_addr_ctr.get(f[1], 0)
    per_addr_ctr[f[1]] = ctr + 1
    if (f[0] - t0) / 1e9 >= a.flip_at:
      flip_frames.setdefault(f[1], []).append(ctr)

  if a.batch:
    c_base = [0x380 + 4 * k for k in range(6)]
    groups = {'C-F0': c_base, 'C-F1': [a + 1 for a in c_base],
              'C-F2': [a + 2 for a in c_base], 'C-F3': [a + 3 for a in c_base],
              'D': [0x3C8, 0x3C9, 0x3CA, 0x3CB, 0x3D0], 'H': [0x301]}
    plan = []
    for gname, gaddrs in groups.items():
      flips_by_addr = {addr: flip_frames[addr][:a.flip_k] for addr in gaddrs
                       if flip_frames.get(addr)}
      if not flips_by_addr:
        plan.append((gname, None, None, None))
        continue
      for b in bits:
        flips = {(addr, s): {b} for addr, seq in flips_by_addr.items()
                 for s in seq}
        plan.append((gname, b, flips, {f'0x{ad:03x}': s
                                       for ad, s in flips_by_addr.items()}))
  else:
    plan = []
    for addr in addrs:
      seq = flip_frames.get(addr)
      if not seq:
        plan.append((f'0x{addr:03x}', None, None, None))
        continue
      fl_idx = seq[:a.flip_k]
      for b in bits:
        plan.append((f'0x{addr:03x}', b, {(addr, s): {b} for s in fl_idx},
                     [f'0x{addr:03x}']))

  probes = []
  t_start = time.time()
  jl_path = a.out + '.jsonl'
  done_keys = set()
  if a.resume and os.path.exists(jl_path):
    with open(jl_path) as fp:
      for line in fp:
        line = line.strip()
        if line:
          e = json.loads(line)
          probes.append(e)
          done_keys.add((e.get('group'), e.get('bit')))
    print(f"[resume] {len(probes)} probes already done in {jl_path}")

  base_dump = None
  if a.dump_baseline:
    # driving-region baseline records for the layout solver: per-feed lists so
    # deltas can be re-localized in time without re-running the engine
    feeds = []
    off = 0
    for i in range(len(base.n)):
      if base.ft[i] >= a.diff_after:
        feeds.append({'t': round(base.ft[i], 4),
                      'recs': [[round(base.cols_f[k][off + j], 6) for k in FIELDS_F] +
                               [base.cols_i[k][off + j] for k in FIELDS_I]
                               for j in range(base.n[i])]})
      off += base.n[i]
    base_dump = feeds

  jl_fp = open(jl_path, 'a') if a.resume else None
  for p in plan:
    if p[2] is None:
      if (p[0], None) not in done_keys:
        probes.append({'group': p[0], 'bit': None, 'changes': {},
                       'note': f'no frames after flip_at={a.flip_at}s'})
      continue
    gname, b, flips, fl_map = p
    if (gname, b) in done_keys:
      continue
    cap = Capture()
    engine.begin_pass()
    run_pass(engine, frames, t0, t1, flips, cap, ts_anchor=time.monotonic_ns() - win_ns)
    changed_at = {}
    summ = diff_pass(base, cap, changed_at, diff_after=a.diff_after)
    changes = {f'{k[0]}@slot{k[1]}': {'max_abs': round(e['max_abs'], 6),
                                      'first_delta': round(e['first_delta'], 6),
                                      'first_feed': e['first_feed'], 'feeds': e['feeds']}
               for k, e in sorted(summ.items(), key=lambda kv: -kv[1]['max_abs'])
               if k != 'FATAL'}
    if 'FATAL' in summ:
      changes['FATAL'] = summ['FATAL']
    entry = {'group': gname, 'bit': b, 'flip_addrs': fl_map, 'changes': changes}
    probes.append(entry)
    if jl_fp is not None:
      jl_fp.write(json.dumps(entry) + '\n')
      jl_fp.flush()
    if a.max_probes and len([x for x in probes if x['bit'] is not None]) >= a.max_probes:
      print(f"[probe] --max-probes {a.max_probes} reached, stopping early")
      break
  if jl_fp is not None:
    jl_fp.close()

  out = {'meta': {**info, 'dump': a.dump, 'window_begin': a.window_begin,
                  'window_len': a.window_len, 'diff_after': a.diff_after,
                  'flip_at': a.flip_at, 'noise': a.noise, 'flip_k': a.flip_k,
                  'batch': a.batch, 'n_window_frames': len(win),
                  'determinism_drift_feeds': drift,
                  'elapsed_s': round(time.time() - t_start, 1),
                  'slot_summary': {str(k): v for k, v in sorted(slots.items())}},
        'probes': probes}
  if base_dump is not None:
    out['baseline_records'] = base_dump
  with open(a.out, 'w') as fp:
    json.dump(out, fp, indent=1)
  print(f"[done] {len(probes)} probes -> {a.out} in {out['meta']['elapsed_s']}s")


if __name__ == '__main__':
  main()
