#!/usr/bin/env python3
# BYD 台架闭环回放：用 process_replay 在设备内重生成 controlsd/card 输出，并与 rlog 里实车记录的 sendcan 对比
import os, sys, collections

# macOS 兼容性（2026-10-09 实测）：
# - 不能用 fork：launcher 子进程一进来就 setproctitle → CoreFoundation
#   "multi-threaded process forked" SIGSEGV，macOS 保持默认 spawn。
# - spawn 的子解释器会重新 import __main__（本脚本），所以执行体必须包在
#   __main__ 守卫里，否则子进程递归重跑整个回放（第一次 Mac 实测即崩在这）。
# - realtime.py 的 sched_setaffinity/prctl 都有 sys.platform=='linux' 守卫，Mac 自动跳过。

# c3l 克隆板只有 4 核：config_realtime_process 请求 core 5 → sched_setaffinity EINVAL。
# spawn/子解释器场景不继承父进程 monkeypatch，靠 sitecustomize 兜底；fork 子进程再靠
# 进程内补丁双保险。设备 UI 更新会清掉 sitecustomize 文件（2026-10-09 实测 bench3 因此
# 崩），所以写入 PYTHONPATH 上所有可写目录、并验证子解释器确实打上补丁。
if (os.cpu_count() or 8) < 6:
  _sc_src = (
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
  for _d in os.environ.get("PYTHONPATH", "").split(":"):
    if _d and os.access(_d, os.W_OK):
      try:
        with open(os.path.join(_d, "sitecustomize.py"), "w") as _f:
          _f.write(_sc_src)
      except OSError:
        pass

  # 进程内降级（fork 出的子进程继承）
  _orig_setaffinity = os.sched_setaffinity
  def _safe_setaffinity(pid, mask):
    cores = {mask} if isinstance(mask, int) else set(mask)
    avail = set(range(os.cpu_count())) & cores
    if not avail:
      avail = set(os.sched_getaffinity(0))
    return _orig_setaffinity(pid, avail)
  os.sched_setaffinity = _safe_setaffinity

  # 验证：子解释器必须能扛住绑不存在的核，否则更新后环境不对，直接报错退出
  import subprocess as _sp
  _chk = _sp.run([sys.executable, "-c", "import os; os.sched_setaffinity(0, [os.cpu_count() + 99])"],
                 env=os.environ, capture_output=True, text=True)
  if _chk.returncode != 0:
    sys.exit("sitecustomize affinity patch not active in child interpreters:\n" + _chk.stderr)

from openpilot.tools.lib.logreader import LogReader
from openpilot.selfdrive.test.process_replay.process_replay import replay_process_with_name, get_custom_params_from_lr

# Mac 没有 /dev/shm，而车端代码（cruise/carrot_man 等）硬编码 Params("/dev/shm/params")。
# 不改标品：只在 bench 脚本内把 openpilot.common.params.Params 重定向（Mac shm 路径=/tmp）。
# spawn 子解释器会重新 import __mp_main__（本脚本），补丁在 onroad 模块执行
# `from openpilot.common.params import Params` 绑定之前生效；设备（fork、/dev/shm 存在）
# 永远不走这个分支，车端行为零变化。
if sys.platform == "darwin":
  from openpilot.common import params as _pm
  _orig_Params = _pm.Params
  def _shm_agnostic_Params(d=""):
    return _orig_Params("/tmp/params" if d == "/dev/shm/params" else d)
  _shm_agnostic_Params.__name__ = "Params"
  _pm.Params = _shm_agnostic_Params

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

def main():
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

if __name__ == "__main__":
  main()
