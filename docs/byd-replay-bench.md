# BYD 台架日志重放：不动车验证闭环

日期：2026-10-07（2026-10-08 实测回填）。设备：comma@192.168.3.124（c3l 克隆板，carrot-byd @ d5fc348，DongleId=UnregisteredDevice）。

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

# PYTHONPATH 必须对齐 manager 运行环境：pydeps 提供 pyserial 等，缺了会在
# system.hardware 导入处 ModuleNotFoundError；/data/openpilot + /data/pythonpath 提供包路径。
# （/tmp 副本=已含克隆板绑核补丁；本仓文件入库后两者等价）
PYTHONPATH=/data/local_pkgs:/data/openpilot/pydeps:/data/openpilot:/data/pythonpath \
  timeout 900 /usr/local/venv/bin/python3 /tmp/byd_replay_bench.py \
  /data/media/0/realdata/0000000000000002/00000002--d3da5795a9--3/rlog.zst card,controlsd
# 第二参数 'card'=CC 输入用日志原值（纯执行层 diff）；'card,controlsd'=全链 diff
```

原理与输出：
- `get_custom_params_from_lr(lr)` 从 rlog 提取 CarParams/liveCalibration 等做进程参数播种，`fingerprint=BYD_SONG_PLUS_DMI_22` 显式指定车型 → card 不跑 fw query。
- 重生成消息 = `carControl`/`sendcan` 等进程输出；脚本按总线地址对齐原日志 `sendcan`，对 **0x32E `ACC_CMD.AccelCmd` 与 0x316(`BO_790 ACC_MPC_STATE.LKAS_Output`，11bit 力矩主通道)** 输出 mean|Δ|/max|Δ|，并列出两代固件 TX 信号的有无差异。
- 横向 siglin 模型验证点：controlsd 重生成路径会调 `CI.torque_from_lateral_accel()`（新移植代码），输入是原日志 `modelV2`（真车 plan）→ **同一场景下新旧固件的力矩输出直接可比**。

### 实测结果（2026-10-08，seg=00000002--d3da5795a9--3，360s 全程 engaged）

| 链 | 重生成 0x316 | mean|Δ| | max|Δ| | 说明 |
|---|---|---|---|---|
| card only（CC 输入=日志原值） | 3561 帧 | 21.6 | 174 | 纯执行层映射差：carrot 回播/编码 vs sunny |
| card+controlsd（全链） | 3561 帧 | 43.9 | 200 | 叠加控制律差（siglin/ki=0/deadzone0.1 生效） |
| seg--2 过渡段 全链（engaged 仅 42.8s） | 3558 帧 | 7.9 | 210 | 未 engage 窗口以回播帧为主、高度一致；差异集中在 engage 弯道窗口（max 与 --3 同量级）——**新码偏差集中在真控制段，直线/停走 echo 无误** |

- 指纹：日志 CAN 在设备内被 fuzzy match 到 `BYD_SONG_PLUS_DMI_22`（source=2）✓。
- **0x32E 重生成缺失 = 设计行为**：`byd/interface.py:147` 只有 `alpha_long` 参数开启才置 `openpilotLongitudinalControl=True`，而日志里 sunny 的 CarParams oPIC=False → `carcontroller.update()` 按门跳过纵向。要 bench 纵向链：设备先 `Params().put_bool('AlphaLongitudinalEnabled', True)` 并保证 bench 重生成 CP（或改写 custom_params['CarParams'] 的 oPIC+safetyParam LONGITUDINAL 位），⏳ 该变体未实测。
- 注意：diff 数值是**新旧行为差异量**，不是对错——对错仍以本机单测（test_byd_lateral_model/test_byd_carcontroller 等）为准；bench 的价值在"同场景回归 + 差异幅度可见"。

### 克隆板适配（已内置在脚本）

c3l 克隆板只有 4 核，`config_realtime_process(5)` 绑核 5 会 EINVAL，且 multiprocessing 子进程不继承父 monkeypatch。`scripts/byd_replay_bench.py` 自动向 PYTHONPATH 第一个可写目录写 `sitecustomize.py`（绑核降级为可用核交集）——设备/真实 8 核 c3 都安全。

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
- **UI 更新会清掉 `/data/local_pkgs/sitecustomize.py`**（2026-10-09 实测：更新后 card 子进程绑核 EINVAL 复活）。脚本 bootstrap 已加固：写入 PYTHONPATH 全部可写目录 + fork 场景进程内补丁 + 子解释器验证（不通过直接报错退出）。
- 更新回归基线：d5fc348→d97a83a 无 BYD 控制码变更，重放结果逐位一致（`316.LKAS_Output n=3561 mean|Δ|=43.851 max|Δ|=200.000`，指纹/engaged 同）。以后若此数字变化，说明改动真的动了横向输出，可据此定位。
- d97a83a→bf3a799d 更新复验（2026-10-09）：更新+重启后直接重跑 bench 成功（bootstrap 每次运行都会把 sitecustomize 重写进全部可写 PYTHONPATH 目录，即使文件被更新清掉也能自愈），--3 基线数字逐位不变。
