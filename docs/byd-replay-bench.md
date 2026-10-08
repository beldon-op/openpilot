# BYD 台架日志重放：不动车验证闭环

日期：2026-10-07。设备：comma@192.168.3.113（c3l 克隆板，carrot-byd @ d5fc348，DongleId=UnregisteredDevice）。

## 结论速览

1. **设备实时 replay 二进制在本 fork 上不可用（已证实，非推测）**：根 `SConstruct` 在 `arch != "larch64"` 条件里才纳入 `openpilot/tools/replay/SConscript`（同 cabana/jotpluggler）。本克隆板 uname 即 `larch64`，`scons -u openpilot/tools/replay` 报 "up to date"（实际无构建规则）。所以设备端不产出 `replay` 二进制。
2. **设备侧"模拟数据"的可行路线 = `process_replay` 离线闭环**（`openpilot/selfdrive/test/process_replay/`，纯 Python，用 rlog 喂真实 onroad 进程再 diff）。它自带 `REPLAY=1 / SKIP_FW_QUERY=1 / FINGERPRINT=<车型>` 环境注入（`process_replay.py:829 generate_environ_config`），card/selfdrived 认 `REPLAY` 跳过 pandaStates 检查，台架不需要点火、不需要 panda、不接车。
3. **数据源已就位**：2026-10-05 路测语料（sunnypilot 固件录的 BYD 真车日志）本地在
   `~/Documents/work/op/sunnypilot/docs_site/byd_logs_2026-10-05_07/realdata/`，
   其中 **26 段带 `fcamera.hevc` 视频**，多段全程 engaged；已 scp 到设备
   `/data/media/0/realdata/0000000000000002/00000002--d3da5795a9--{2,3}/`（rlog+qlog+fcamera，seg--3 为 360s 全程 engaged，含 `sendcan` 实测 TX，可做逐信号 diff）。

## 日志盘点（实测）

| 语料 | 性质 | camera 帧 | 备注 |
|---|---|---|---|
| `~/Documents/work/op/byd_logs/00000004--76681a4453--{19..23}` | 2026-09-26，rlog+qlog 仅日志 | **无** | carParams `fingerprint=MOCK`（未指纹）、0s engaged（vEgo≈0.1 泊车段）→ 只能验 CAN 解析层 |
| `docs_site/byd_logs_2026-10-05_07/realdata/00000002--d3da5795a9--{2..5}` | 2026-10-05，带视频 | 6/235 段 | fingerprint=`BYD_SONG_PLUS_DMI_22`，--3/--4/--5 全程 engaged；rlog 含 `sendcan`(7206条)/`can`/`modelV2` |
| 同上 `00000003--8801cc6fda--{0..5}`、`00000001--ec0323b5d6--{2..5}`、`00000005--7a78691537--5` | 同路测 | 各 6 段 | 多数全程 engaged；`--2` 为 engage 过渡段（42.8s/360s） |
| 同上 `00000004--7338dd4cc7--0` 等单段路线 | 短程 | 有 | 0s engaged，适合纯 CAN/carstate 验证 |

用本仓 schema 读 sunnypilot 日志可行：基础字段（fingerprint/mass/centerToFront/can/sendcan/selfdriveState）读取正确，仅伴随 `Corrupted events detected` 警告（sunny 扩展字段与 carrot 编号有差异的残留），不影响核心解析。carrot `CarParams` 无 `experimentalMode/carName` 字段——脚本里不要引用。

## 台架闭环步骤（process_replay）

脚本：`scripts/byd_replay_bench.py`（随本仓）。在设备上：

```bash
cd /data/openpilot
# 依赖（一次性，已完成）：libjpeg/catch2 是 SConstruct 顶层 import 所需，
# 装在 /data/local_pkgs 绕过只读 venv：
#   /usr/local/venv/bin/python3 -m pip install --target /data/local_pkgs \
#     --no-index --find-links /data/openpilot/third_party/wheels libjpeg
#   /usr/local/venv/bin/python3 -m pip install --target /data/local_pkgs \
#     /data/openpilot/third_party/wheels/comma_deps_catch2-2.13.10.post96-py3-none-any.whl
#   （tinygrad 在 /data/openpilot/tinygrad_repo，PYTHONPATH 带上即可）

PYTHONPATH=/data/local_pkgs timeout 600 /usr/local/venv/bin/python3 scripts/byd_replay_bench.py \
  /data/media/0/realdata/0000000000000002/00000002--d3da5795a9--3/rlog.zst card
# 第二个参数可给 'card,controlsd'（多进程 DAG 会重排 carControl 喂 card）
```

原理与输出：
- `get_custom_params_from_lr(lr)` 从 rlog 提取 CarParams/liveCalibration 等做进程参数播种，`fingerprint=BYD_SONG_PLUS_DMI_22` 显式指定车型 → card 不跑 fw query。
- 重生成消息 = `carControl`/`sendcan` 等进程输出；脚本按总线地址对齐原日志 `sendcan`，对 **0x32E `ACC_CMD.AccelCmd`、0x1E2 `STEERING_MODULE_ADAS.STEER_ANGLE/STEER_REQ`、0x1FC `STEERING_TORQUE.MAIN_TORQUE`** 输出 mean|Δ|/max|Δ|，并列出两代固件 TX 信号的有无差异。
- 横向 siglin 模型验证点：controlsd 重生成路径会调 `CI.torque_from_lateral_accel()`（新移植代码），输入是原日志 `modelV2`（真车 plan）→ **同一场景下新旧固件的力矩输出直接可比**。

**状态：⏳ 实测数值待补**——脚本已在 `scripts/` 就位、日志已传设备，但 23:49 设备断网（待上电/确认 IP），跑通后把 diff 表贴回本节。

## 实时 replay（PC 侧）与设备侧的分工

| 目的 | 方案 |
|---|---|
| 看画面/告警/整段轨迹（可视化、cabana、jotpluggler） | PC 编译 `tools/replay/replay`（Mac/Linux 均可，SConstruct 仅 larch64 排除），`replay <dongle>/<route> --data_dir <realdata>` |
| 验证本仓 BYD 代码（carstate/carcontroller/lat 模型/slew/迟滞） | 设备 `process_replay`（本节）或本机 pytest——同一份代码，设备侧多一道真 venv/依赖一致性 |
| 验证 panda 固件 safety TX 放行 | 台架接车端断电、上电跑 `process_replay`/replay 时看 STATUS_PACKET，或继续用 `scripts/byd_long_capture.py` |
| 模型选择器/boot_compile/Web :7000 | 无需日志，offroad 桌面验证即可 |

## 坑清单

- `LOG_ROOT` 设备默认 `/data/media/0/realdata/`（`openpilot/system/hardware/hw.py:Paths.log_root`），段目录必须 `<16位hex dongle>/<8位数字>--<10位hex>--<N>/`（`tools/lib/helpers.py:RE.SEGMENT_RANGE` 强校验，随意目录名会被 auto_source 拒掉）；`--N` 个位数段号在部分 LogReader 分支解析异常，用 `--10` 以上或显式 `--<start>:<end>` 切片。
- 无视频帧的段（只有 rlog/qlog）做不了 modeld 回放（`model_replay` 需 `fcamera.hevc`），但 process_replay 不重跑 modeld、直接用日志里的 `modelV2`，无帧也能闭环 controlsd/card。
- 台架不接车时 process_replay 完全不碰 panda/总线，无 EPS 锁风险；但任何让真实 card 在 onroad 断流的动作仍受 BYD「0x316 断流锁 EPS」红线约束（见 byd-lateral-vendor-model.md）。
- 设备 `/usr/local` 只读：pip 装包一律 `--target /data/local_pkgs` + PYTHONPATH。
