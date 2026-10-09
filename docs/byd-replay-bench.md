# BYD 台架日志重放：不动车验证闭环

日期：2026-10-07（2026-10-08 实测回填，2026-10-09 增补「本机 Mac replay」一节）。设备：comma@192.168.3.124（c3l 克隆板，carrot-byd @ d5fc348，DongleId=UnregisteredDevice）。

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

## 本机 Mac 实机 replay（Apple Silicon，2026-10-09 实测）

根 `SConstruct` 的 arch 白名单含 `Darwin`（macOS arm64，x86 Mac 不支持），且 `tools/replay` 在 `arch != "larch64"` 时纳入——即 **Mac 可直接编 C++ replay，不需要 Docker**。M4 上已实测：sunny 10-05 日志能放，benchmark 模式 ~1100x 实时。

### 构建

```bash
cd <repo 根>
export PATH="$PWD/.venv/bin:$PATH"   # 必须：capnpc 不在 PATH 时 capnp 代码生成步骤报 command not found
scons -u openpilot/tools/replay      # 产物 = openpilot/tools/replay/replay
```

依赖 wheel（capnproto/ffmpeg/libyuv/zstd/catch2 等）`.venv` 已带齐（comma-deps 有 Darwin 轮子），无需 brew 装。

### 放本地日志

`--data_dir` **直接指 realdata 目录**（段目录的顶层父目录）：C++ `Route::loadFromLocal` 只扫 `data_dir` **顶层**名字含 `<8位>--<10hex>--` 的目录，不认 dongle 层级——与 Python auto_source 要求的 `<dongle>/<route>--<N>` 布局不同，别多包一层。route 串可省 dongle 前缀，用 `start:end` 切片：

```bash
# 自动化冒烟：跑完即退，exit 0（实测 00000003--8801cc6fda 段 10:11 通过）
openpilot/tools/replay/replay "00000003--8801cc6fda/10:11" \
  --data_dir ~/Documents/work/op/sunnypilot/docs_site/byd_logs_2026-10-05_07/realdata \
  --benchmark -x 3

# 交互播放：ConsoleUI 是 ncurses，必须在真实终端跑（重定向/无 TTY 报 "Error opening terminal: unknown"）
openpilot/tools/replay/replay "00000002--d3da5795a9/3:3" \
  --data_dir ~/Documents/work/op/sunnypilot/docs_site/byd_logs_2026-10-05_07/realdata -x 2
# 按键：s/shift+s ±10s，m/shift+m ±60s，space 暂停，e/d 跳 engage/disengage，+/- 变速，enter seek，q 退出
```

sunny 日志的 schema 差异在 C++ 读取路径无碍（capnp 跳过未知字段），连含 `fcamera.hevc` 的段也正常解码——Python 侧的 `Corrupted events` 警告在这里连出现都没有。

### 必打补丁：replay 启动即崩（本仓 `openpilot/tools/replay/replay.cc` 已修）

`Replay::setupServices` 用 `getFieldByName` 遍历 `services.h` 解析 Event union 判别值，但 fork 的 services 表里有几个 schema 中已不存在的残留名（`navModel`→已改 `navModelDEPRECATED`；`customReservedRawData1/2`→@124 已改名 customReservedRawData0、@125/126 被 navRouteNavd 等占用），查不到直接抛 kj 异常，二进制在读段之前就 abort。修复 = `openpilot/tools/replay/replay.cc` 改用 `findFieldByName`，查不到跳过。此 bug 影响**任何 PC 平台**（Linux x86_64/aarch64 同崩），只是设备 larch64 不编 replay 所以一直没暴露。

### 定位

Mac replay 只做**开环发布消息**（msgq pub），本机没有 onroad 进程消费——适合：cabana/jotpluggler 查看、自写 msgq/ZMQ(`ZMQ=1`) 订阅端对比、`--benchmark` 快速回归。BYD 控制链闭环 diff 仍走上文设备侧 `process_replay` 路线。

## 本机 Mac 跑 process_replay 闭环（2026-10-09 实测）

**可行，且与设备逐位一致**：`scripts/byd_replay_bench.py` 在 M4 本机跑出——card-only `316.LKAS_Output n=3561 mean|Δ|=21.586 max|Δ|=174.000`、全链 card,controlsd `mean|Δ|=43.851 max|Δ|=200.000`（=设备 10-08/10-09 基线原值）。指纹同样精确匹配 `BYD_SONG_PLUS_DMI_22`（fuzzy=False），0x32E 缺失同样为设计行为。

```bash
cd <repo 根>
.venv/bin/python scripts/byd_replay_bench.py \
  ~/Documents/work/op/sunnypilot/docs_site/byd_logs_2026-10-05_07/realdata/00000002--d3da5795a9--3/rlog.zst \
  card,controlsd
```

Mac 适配三处（已改，设备上全部为 no-op）：

1. **bench 脚本主流程包进 `if __name__ == "__main__":`**。macOS 的 multiprocessing 默认 start method 是 **spawn**，`PythonProcess` 的子解释器会重新 import 主脚本；脚本无守卫时整段回放被递归重跑（首测即崩）。设备/Linux 是 fork，所以从未暴露。
2. **不要试图改回 fork**。fork 出的 card 子进程一执行 `setproctitle` 就 SIGSEGV——CoreFoundation「multi-threaded process forked」检查判定 fork 后加载 CF 插件非法（崩溃报告见 `~/Library/Logs/DiagnosticReports/python3.12-*.ips`）。spawn + 守卫才是 Mac 正路。
3. **8 处 `Params("/dev/shm/params")` 硬编码 → `Params(Paths.shm_path() + "/params")`**（cruise / selfdrived / paramsd / ui_state / carrot_serv / carrot_man / cluster_live / cluster_system_monitor 共 8 个 .py）。macOS 无 /dev/shm；`Paths.shm_path()` 在设备/L Linux 仍返回 `/dev/shm`。首崩点即 `cruise.py` VCruiseCarrot 初始化。

已知边界：
- `realtime.py` 的 sched_setaffinity/prctl 都有 `sys.platform=='linux'` 守卫，Mac 自动跳过；克隆板绑核 sitecustomize 块（<6 核触发）在 Mac 不生效，无需处理。
- paramsd/torqued 等卡尔曼进程 import 依赖 rednose cython 扩展（`rednose.helpers.ekf_sym_pyx`），本机未编（card/controlsd 链不需要——liveParameters/livePose 直接从日志喂）。要重放这些进程需先构建 rednose cython 目标。
- modeld 链（NN 推理）未在 Mac 验证过，不要顺手加进 PROCS。
- Mac 上 `/tmp/params` 跨运行持久（shm 回退目录），要干净基线先 `rm -rf /tmp/params`。

定位升级：此前「闭环验证优先 Linux PC/设备」的结论可改为——**M4 本机就能跑 card/controlsd 闭环回归，数值与设备基线可互相校验**；设备侧的不可替代价值收敛到 panda 固件、真机 venv 依赖一致性与 onroad 编排。

## 实时 replay（PC 侧）与设备侧的分工

| 目的 | 方案 |
|---|---|
| 看画面/告警/整段轨迹（可视化、cabana、jotpluggler） | PC 编译 `tools/replay/replay`（Mac/Linux 均可，SConstruct 仅 larch64 排除；Mac 步骤见上节「本机 Mac 实机 replay」），`replay "<route>/start:end" --data_dir <realdata>` |
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
- PC 构建后 git status 会冒出 `openpilot/cereal/gen/`、`openpilot/cereal/services.h`：根 `.gitignore` 的老模式 `cereal/gen`/`cereal/services.h` 因 fork 把 cereal 挪到 `openpilot/` 下而失配，已补两条带前缀的忽略规则。
- 交互 replay 别把 stdout 重定向到文件：ncurses 初始化失败（`Error opening terminal: unknown`）；要脚本化验证就用 `--benchmark`。
