#!/usr/bin/env python3
# BYD 台架闭环回放：用 process_replay 在设备内重生成 controlsd/card 输出，并与 rlog 里实车记录的 sendcan 对比
import sys, collections

from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.test.process_replay.process_replay import replay_process_with_name, get_custom_params_from_lr

SEG = sys.argv[1] if len(sys.argv) > 1 else '/data/media/0/realdata/0000000000000002/00000002--d3da5795a9--3/rlog.zst'
PROCS = (sys.argv[2] if len(sys.argv) > 2 else 'card').split(',')
FP = 'BYD_SONG_PLUS_DMI_22'

def decode_addr(msg, addr_dec):
  # 返回 {信号名: 值} for a known BYD TX frame, else None
  for cd in msg.sendcan:
    if cd.address != addr_dec:
      continue
    d = bytes(cd.dat)
    if addr_dec == 814:  # ACC_CMD: AccelCmd 0|8@1+ (0.05,-5)
      return {'AccelCmd': round(d[0] * 0.05 - 5, 3)}
    if addr_dec == 482:  # STEERING_MODULE_ADAS: STEER_ANGLE 24|16@1- (0.1), STEER_REQ bit21
      angle = int.from_bytes(d[3:5], 'little', signed=True) * 0.1
      return {'STEER_ANGLE': round(angle, 2), 'STEER_REQ': int(d[2] >> 5) & 1}
    if addr_dec == 508:  # STEERING_TORQUE: MAIN_TORQUE 0|16@1- (0.1)
      return {'MAIN_TORQUE': round(int.from_bytes(d[0:2], 'little', signed=True) * 0.1, 2)}
  return None

def collect(msgs, name):
  rows = collections.defaultdict(list)
  addrs = collections.Counter()
  for m in msgs:
    if m.which() != 'sendcan':
      continue
    for cd in m.sendcan:
      addrs[cd.address] += 1
      sig = decode_addr(m, cd.address)
      if sig:
        for k, v in sig.items():
          rows[f'{cd.address:03X}.{k}'].append(v)
  print(f'--- {name}: sendcan 帧数 top ---')
  print({hex(a): c for a, c in addrs.most_common(10)})
  return rows

lr_all = list(LogReader(SEG))
logged = collect(lr_all, 'LOG(实车原值)')
print('engaged_s:', round(sum(1 for m in lr_all if m.which() == 'selfdriveState' and m.selfdriveState.enabled) / 20, 1))

custom = get_custom_params_from_lr(lr_all)
out = replay_process_with_name(PROCS, LogReader(SEG), fingerprint=FP, custom_params=custom, disable_progress=False)
gen = collect(out, 'REPLAY(重生成)')

print('--- 数值对比 ---')
for k in sorted(set(logged) & set(gen)):
  a, b = logged[k], gen[k]
  n = min(len(a), len(b))
  if n == 0:
    continue
  diff = [abs(x - y) for x, y in zip(a[:n], b[:n])]
  print(f'{k}: n={n} mean|Δ|={sum(diff)/n:.3f} max|Δ|={max(diff):.3f}')
missing = set(gen) - set(logged)
extra = set(logged) - set(gen)
if missing: print('replay 新增信号(旧固件没有):', missing)
if extra: print('replay 缺失(旧固件发了新的没发):', extra)
