#!/usr/bin/env python3
# BYD 台架闭环回放：用 process_replay 在设备内重生成 controlsd/card 输出，并与 rlog 里实车记录的 sendcan 对比
import os, sys, collections

# c3l 克隆板只有 4 核：config_realtime_process 请求 core 5 → sched_setaffinity EINVAL。
# 子进程不继承 monkeypatch（multiprocessing 非 fork 场景），因此写一个 sitecustomize
# 到 PYTHONPATH 目录，让所有子解释器启动时自动降级绑核。
if (os.cpu_count() or 8) < 6:
  _sp = os.environ.get("PYTHONPATH", "").split(":")
  for _d in _sp:
    if _d and os.access(_d, os.W_OK):
      with open(os.path.join(_d, "sitecustomize.py"), "w") as _f:
        _f.write(
          "import os\n"
          "_o = os.sched_setaffinity\n"
          "def _s(pid, mask):\n"
          "  cores = {mask} if isinstance(mask, int) else set(mask)\n"
          "  avail = set(range(os.cpu_count())) & cores\n"
          "  if not avail:\n"
          "    avail = set(os.sched_getaffinity(0))\n"
          "  return _o(pid, avail)\n"
          "os.sched_setaffinity = _s\n"
        )
      break

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
    if addr_dec == 790:  # 0x316: LKAS_Output 16|11@1- (1,0) —— 力矩指令主通道
      raw = d[2] + ((d[3] & 0x7) << 8)
      if raw >= 0x400:
        raw -= 0x800
      return {'LKAS_Output': raw}
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
