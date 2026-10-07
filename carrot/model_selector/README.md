# Carrot 模型选择器

Carrot 专属的驾驶模型下载/安装/应用模块。c3x·c4 硬件通用，
UI 位于 **carrot web（端口 7000）**。

## 设计原则

1. **引擎分离 + 新架构复用 upstream 引擎**
   - 默认模型（`openpilot/selfdrive/modeld/models/`）→ **upstream `openpilot/selfdrive/modeld/modeld.py` 原样负责**
   - 自定义**新模型（supercombo，lebowski 结构）**（`/data/models/driving_tinygrad.pkl`）
     → **原样复用 upstream `modeld.py`** — 仅设置 `MODELD_MODELS_DIR=/data/models` env
     （`openpilot/selfdrive/modeld/helpers.py::modeld_pkl_path()` 读取 env 切换路径，无 monkey-patch）
   - 自定义**旧模型（vision+policy 分离）**（`/data/models/`）→ **`carrot.model_selector.carrot_modeld` 负责**（自带 3 模型 on/off policy 支持）
   - `modeld_runner` 开机时按 `/data/models` 内容三选一执行
   - upstream 变更不影响 legacy 引擎的独立演进

2. **upstream 文件最小侵入**（各 1~3 行）
   - `openpilot/system/manager/process_config.py` — modeld 条目替换为 `openpilot.carrot.model_selector.modeld_runner`
   - `openpilot/system/manager/manager.py::main()` 开头 — 调用 `boot_compile.run()`
   - `openpilot/selfdrive/modeld/helpers.py::modeld_pkl_path()` — `MODELD_MODELS_DIR` env 覆写（2 行）
   - `openpilot/selfdrive/carrot/server/app.py` — 注册路由
   - `openpilot/selfdrive/carrot/web/index.html` — 导航按钮 + 加载 `page_models.js`（`?v=…` 缓存 busting）
   - `openpilot/selfdrive/ui/mici/layouts/home.py` — c4 主页提交哈希旁显示 `DrivingModelName`
   - `openpilot/selfdrive/ui/carrot_param_cache.py` — 边框底栏显示 `branch (模型名)`
   - `openpilot/common/params_keys.h` — 注册 `DrivingModelName`、`PendingModelName` 参数
   - `launch_chffrplus.sh` — 内置模型 build_stamp 改用 `config.compile_env_tag()` 指纹

3. **独立包**
   - 模型选择器连同自带引擎全部收在 `carrot/model_selector/` 内
   - `carrot_parse_model_outputs.py` 是支持 3 模型分区的 c3-ms parser 移植版
   - 原始文件（`openpilot/selfdrive/modeld/*`、`openpilot/common/file_chunker.py` 等）只读/只 import（helpers.py 的 env 钩子除外）

4. **文件组兼容**（新架构单 onnx 或旧架构 vision + policy）
   - **新架构（supercombo）**：`driving_supercombo.onnx` 单独 — 存在时优先应用
   - **旧架构（legacy）**：`driving_vision` 必需 +（`driving_on_policy` 或 `driving_policy`）二选一必需
   - `driving_off_policy` 可选，存在时启用 3 模型架构（仅旧架构）

## 模块结构

| 文件 | 职责 |
|------|------|
| `config.py` | 路径·文件名常量、允许的 onnx 列表（含 supercombo）、tinygrad 编译标志 |
| `keys.py` | Ed25519 公钥映射 + `MODEL_SELECTOR_VERSION`（v4：支持 supercombo） |
| `manifest.py` | 拉取 `models_v4.json`（完整目录）→ 失败时回退 `models.json`（旧架构，已冻结）。缓存 busting `?t=unixtime` + no-cache 头 + canonical JSON Ed25519 验签。版本门槛先于文件名检查执行，无法解析的条目跳过（前向兼容）。snake_case/camelCase 字段均接受 |
| `downloader.py` | ONNX 流式下载 + SHA256/大小校验。含空格文件名做 URL percent-encoding。允许 supercombo 单独或 legacy 文件组 |
| `validator.py` | `/data/models/` 文件组有效性校验（`has_supercombo` / legacy 组）+ `describe()` 标签 |
| `installer.py` | 新架构：用 `compile_modeld.py` 生成整合 pkl。旧架构：tinygrad 编译（compile3）+ warp pkl。共同：原子交换 + 失败时从 backup 还原 |
| `carrot_modeld.py` | **自带 modeld 引擎** — 旧架构 2/3 模型通用，专管 `/data/models` |
| `carrot_parse_model_outputs.py` | 自带 parser（c3-ms 移植，含 `parse_off_policy_outputs`） |
| `modeld_runner.py` | 开机三选一调度：supercombo（upstream+env）/ legacy（carrot_modeld）/ 默认（upstream）。引擎信息写入 `/data/model_selector_status` |
| `boot_compile.py` | `manager.main` 开头钩子，触发 pending 模型编译 |
| `jobs.py` | 简单内存 job runner（web 进度轮询） |
| `web/routes.py` | aiohttp 路由（list / status / install / job / apply / reset） |
| `web/frontend/page_models.html` | carrot web 页面模板（基于 Material 3 token） |
| `web/frontend/page_models.js` | 前端逻辑（fragment 自动注入 + Job 轮询 + 确认框 + 重启倒计时） |

## Params

| 名称 | 类型 | 含义 |
|------|------|------|
| `DrivingModelName` | STRING | 当前使用中模型的显示名（c4 主页提交哈希旁、UI 边框底栏也显示） |
| `PendingModelName` | STRING | 重启后等待编译的模型 ID |

## 数据流

### 安装（Install）
```
[Web UI: 点击「安装」]
  └─ appConfirm 对话框（"正在安装模型。下载完成后将自动重启。"）
       └─ POST /api/models/install {id}
            └─ manifest 重新校验 → jobs.start("install_model")
                 └─ downloader.download_model(entry, progress_cb)   # 按文件校验 size+sha256
                      → /data/models_tmp/
                 └─ Params.put(PendingModelName=entry.id)
                 └─ job.finish(ok=True, progress=100)

[客户端轮询] GET /api/models/job?id=<job_id>  （320ms）
  └─ snap.done && status=="done"
       └─ POST /api/models/apply  （服务器预约 5 秒后 sudo reboot）
            └─ rebootCountdown(5)  — 0→100 进度条，"下载完成 — N 秒后重启…"

[重启 → manager.main() 开头]
  └─ boot_compile.run() → installer.compile_pending()
       ├─ [新架构] 存在 driving_supercombo.onnx 时:
       │    └─ compile_modeld.py --model-size(metadata img 尺寸) --camera-resolutions 1928x1208 1344x760
       │       --frame-skip(MODEL_RUN_FREQ//MODEL_CONTEXT_FREQ)
       │       → 整合 driving_tinygrad.pkl（metadata + 模型 JIT + 各分辨率 warp JIT）
       ├─ [旧架构] ONNX → tinygrad.pkl + metadata.pkl（QCOM/FLOAT16/NOLOCALS/JIT_BATCH_SIZE=0/IMAGE=1/OPENPILOT_HACKS=1）
       │    └─ 按所选模型 metadata 的 img 输入尺寸重新生成独立 warp pkl（不复用内置 warp）
       ├─ 原子交换: /data/models_tmp → /data/models （失败时从 backup 还原）
       └─ Params: 记录 DrivingModelName，删除 PendingModelName

[process manager → `modeld` 进程（only_onroad）]
  └─ modeld_runner.main()
       ├─ validator.is_valid_supercombo_model_dir(/data/models) ?
       │    └─ 是 → 写 status(engine=upstream_modeld_custom)
       │             设置 MODELD_MODELS_DIR=/data/models 后运行 upstream modeld.main()  ← 复用新架构引擎
       ├─ validator.is_valid_legacy_model_dir(/data/models) ?
       │    └─ 是 → 写 status(engine=carrot_modeld)
       │             carrot_modeld.main()        ← 旧架构 3 模型引擎
       └─ 其余 → 写 status(engine=upstream_modeld)
                  upstream modeld.main()          ← 默认引擎
```

### 还原默认模型（Reset）
```
[Web UI: 点击「还原默认模型」]
  └─ appConfirm 对话框
       └─ POST /api/models/reset
            ├─ installer.reset_to_default()   # 删除 /data/models + /data/models_tmp，清除参数
            └─ 预约 5 秒后 sudo reboot
       └─ rebootCountdown(5) — "还原默认模型 — N 秒后重启…"
```

### 运行期引擎确认
```
$ cat /data/model_selector_status
engine=carrot_modeld
pid=53610
started=1776677826
describe=/data/models: vision+on_policy+off_policy
```
`engine` 取值：`upstream_modeld_custom`（新架构 supercombo 自定义）/ `carrot_modeld`（旧架构自定义）/ `upstream_modeld`（默认）。
`ps` 因 `setproctitle` 会隐去 Python 模块路径，此文件才是确定性依据。

## 安全机制

- **manifest 前向兼容 + 双轨制**（`manifest.py`）
  — 旧版（v3）parser 存在缺陷：manifest 条目里出现任何一个未知文件名就会让
  整个列表失败（文件名检查早于版本门槛），因此远端仓库分双文件
  `models.json`（仅旧文件名，冻结）+ `models_v4.json`（完整目录）。v4 parser
  先执行版本门槛再解析、跳过无法解析的条目，此后新增文件名不会重蹈覆辙。
  含新文件名的条目绝不能放进 `models.json`（openpilot-models 的
  `update_models.py` 自动分离）。
- **安装前 metadata 校验**（`installer._validate_supercombo_metadata`）
  — 检查必需输入（img/big_img/desire_pulse/traffic_convention/action_t/features_buffer）、
  `hidden_state` output slice、模型输入尺寸 512x256（MEDMODEL — 运行期 warp 矩阵按此尺寸固定），
  不满足则中止安装（还原备份）。提前阻断不兼容模型装机后陷入崩溃循环。
- **崩溃循环熔断器**（`modeld_runner._arm_crash_loop_breaker`）
  — 自定义引擎启动时递增 `/data/models/.load_attempts`，存活 90 秒自动清除。
  连续 3 次早期崩溃（戳是新的但 pkl 损坏、输出 slice 不符等）时把 `/data/models`
  隔离到 `/data/models_quarantined`，自动回退默认内置模型。无需手动 reset 即恢复可驾驶状态。
- **tinygrad 代际预检**（`modeld_runner._select_engine`）
  — `/data/models/.compile_env` 与当前编译器 tag 不一致时不反序列化旧 pkl，
  立即使用默认内置模型。保留的 onnx 自动重编译成功后连同新戳原子替换；
  重编译失败或没有 onnx 时也一样以默认模型开机，不会因旧 pkl 反复重启。
  现有自定义文件保留，供下一个编译器版本再试。
- **防止 stale PendingModelName**
  — 安装开始时（下载前）先清除旧的 pending（`routes.api_install`），
  开机时 tmp 不完整则连同 pending 一起清除（`installer.compile_pending`）。
  避免失败下载的残留文件按以前的模型名被编译、或 UI 永久显示「安装中」。
- **PC 编译后端探测**（`installer._probe_tg_devices`）
  — 非 TICI 设备装模型时探测 tinygrad 设备，按 CUDA > CPU 顺序选择
  （避免与运行期 tg_input_devices.json 的后端不一致）。SCons 构建按架构选择，实机固定 QCOM。
- **USBGPU 行为** — 安装自定义 supercombo 时，自定义模型目录里没有
  `big_driving_<sha16>_tinygrad.pkl`，upstream 的 USBGPU 探测自动失效（小模型跑在 QCOM 上）。
  不支持编译自定义 big 模型（属预期的安全行为）。

## Web UI 特性

- **Material 3 卡片布局** — `page_models.html` 内联 CSS，使用 carrot web 的 `--md-*` 设计 token
- **固定高度 + 内部滚动** — `body[data-page="models"] { overflow: hidden }` + `.ms-list { flex:1; overflow-y:auto }`，只滚动列表
- **排序** — `added_at` 降序 → `name` 自然排序降序（v11 > v10）
- **确认框** — `window.appConfirm(message, {title, confirmLabel})`（与工具页 reboot 弹窗一致）
- **倒计时统一** — 下载、重启都按 0→100 方向填充
- **导航 active 同步** — `carrot:pagechange` 事件监听切换 `btnModels.active`
- **中文 UI** — 「模型选择器」「当前模型」「安装中」「剩余空间」「可用模型」「还原默认模型」，导航按钮标签 "Models"
- **自动 bootstrap** — 没有 `#pageModels` 时把 `/models/page_models.html` fragment 注入 `#swipeContainer`（幂等）
- **PAGE_ELEMENTS 集成** — 通过 `showPage("models")` 与其他页签行为一致，离开页签时正常隐藏

## 维护 — 应对 upstream 自动 cherry-pick（Hermes）

upstream 提交大多经由自动 cherry-pick 进入，因此选择器的保护机制全部
**自行运转**，不依赖人的记忆（检查清单、手动更新版本号）：

- **编译环境戳自动派生 tag**（`config.compile_env_tag()`）
  — 从 `tinygrad_repo`、modeld 编译·warp·metadata 脚本、模型选择器 installer
  与编译标志的 git 哈希派生 tag。默认模型的 `.build_stamp` 也使用同一 tag。
  触及这些路径的提交被 cherry-pick 进来时 tag 自动变化 → 与装机戳不一致 →
  开机时用保留的 onnx 自动重编译。
  仅无法使用 git 的环境需手动维护 `_COMPILE_ENV_TAG_FALLBACK`。
- **helpers 钩子自检**（`modeld_runner._upstream_hook_alive`）
  — 启动自定义 supercombo 前确认 `modeld_pkl_path()` 确实响应 `MODELD_MODELS_DIR`。
  钩子被 squash/cherry-pick 弄丢时（表现为无崩溃静默加载内置，隔离回退抓不到），
  显式记日志 + 写状态文件后回退内置。
- **契约检查脚本**（`check_contracts.py`）
  — cherry-pick/merge 后运行 `python3 carrot/model_selector/check_contracts.py`。
  检查编译脚本存在性、compile_modeld CLI 参数、metadata 文件名约定、
  pkl 序列化配对（dump_oob/load_oob）、Buffer pickle 方案配对、SConscript↔config 标志、
  fill_model_msg 签名、ModelConstants、进程/参数接线。**FAIL 即表示需要让选择器
  代码随之适配。** 只用标准库，未构建的检出也能跑（helpers 钩子检查在老版本
  Python 上自动退回静态检查）。
- **镜像文件变更侦测**（`check_contracts.py` 的 `modeld-mirror` 检查）
  — `carrot_modeld.py`/`carrot_parse_model_outputs.py` 是对 upstream `modeld.py`/
  `parse_model_outputs.py` 的**镜像**独立引擎，原文件逻辑变化时
  （例如 has_wide_camera 分区）不会像 import 那样自动反映。
  把当前文件与 `upstream_baseline/` 快照比对，上次 review 之后的变更以 FAIL 提示。
  处理：与快照 diff → 把所需逻辑移植进镜像（无关则跳过）→
  `python3 carrot/model_selector/check_contracts.py --sync-baselines` 刷新快照 →
  重跑确认 PASS。
