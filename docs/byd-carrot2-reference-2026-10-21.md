# BYD 移植参考：carrot2（huheas/carrotpilot）调研档案（2026-10-21）

> 来源：`/opsx:explore` 探索模式调研，对照本工程 carrot-byd @ d5fc77c4。
> 调研对象：`/Users/wujiafu/Documents/work/op/carrotpilot/carrotpilot`（remote `huheas/carrotpilot`，分支 carrot2-v8，基线 upstream openpilot `ce355250` 2025-03-07 ≈0.10.1 代，末次提交 2025-12-28）。
> BYD 开发轨迹：d90932f（20250803）→ 59a5387（cp20250825）→ 32686ed / 013cda0（2025-09-01 按钮功能）→ 02775a0/de799cf（Han DBC 微调），此后沉寂。

## 0. 一句话定位

carrot2 的 BYD 适配是同一社区血缘（Atto3/bukapilot 逆向的 checksum、Han DBC 同源——`byd_checksum(0xAF,...)` 与本工程 `bydcan.py:12` 同算法）的**平行分叉**。它的架构与 sunnypilot 协作式相反，是**替换相机式**：

```
carrot2 架构（opendbc_repo/opendbc/car/byd/bydcan.py + safety_byd.h）：
  OP→ESC(bus0): 0x316 ACC_MPC_STATE(自设扭矩) + 0x32E ACC_CMD
  OP→MPC(bus2): 假 0x318 ACC_EPS_STATE（骗相机存活防 DTC，AEB 保留→但见 §5 警告）
  fwd 屏蔽: bus2→bus0 的 0x316/0x32E（厂商指令被掐断）；bus0→bus2 的 0x318（真 EPS 反馈不给相机）
  总线定义: ESC=0 / MRR=1 / MPC=2（与本工程 bus0底盘/bus1雷达/bus2 ADAS 同构）
```

**宋 PLUS 路径本身是半成品，不能当范本**（§5 列完成度实况）。价值集中在下面四件可直接服务本工程下一阶段的东西。

## 1. 价值①：宋 PLUS DMI 22 实车指纹（对表素材）

文件：`opendbc_repo/opendbc/car/byd/fingerprints.py`（`CAR.BYD_SONG_PLUS_DMI_22` 条目）。

来自他人实车（声明为宋 PLUS DMI 22，即本工程目标车型）的完整指纹，关键差异点：

| 帧 | 内容 | 与本工程已知事实的对照 |
|---|---|---|
| 0x351-0x353 (849/850/851) | 在 | 与本工程路测定案一致：Tang 式三帧航迹真实存在（bus1，~22.7Hz，见 `byd-radar-010x.md`） |
| **0x354 (852)**、**0x350 (848)** | 在 | 本工程 port 解析 4 槽当超集；vendor 解密只认 849-851 三槽（852 为开区间端点）。**他们抓到的量产宋有 852/848 帧**——不能证明有载荷，但值得对表 |
| **0x100-0x10A (256-266)** | 在 | 即本工程 10-18 定案的「状态广播家族」（10 地址跟车帧字段恒定、非裸目标报文，见 `byd-radar-reverse-report-2026-10-18.md`）。再次佐证该家族在宋上确实存在 |
| **0x418 (418)** | 在 | BSD_RADAR 在宋上存在 → `ret.enableBsm` 可用（vendor interface 同款判据 `0x418 in fingerprint`） |
| 0x374 (884) | **不在**宋22 指纹（Han/TangDMI 指纹在） | 与本工程结论一致：汉专属 MRR 帧，宋无 |

**注意事项**：他们的指纹 dict 把跨总线帧（0x374 属 bus2 侧、0x351 系列属 bus1、底盘帧属 bus0）混在同一个 dict 里，**bus 归属不可信**。使用前必须拿本工程设备 `/data/carrot/capture/byd_long_*.jsonl`（或归档 rlog.zst，V0 离线回放已验证可喂本仓 CANParser）做一次 (addr, bus) 对表：确认 0x350/0x352/0x354 在用户自己车上是否同样出现、0x354 有无载荷变化。

另注：Tang_DM 指纹含一条 29 位长地址帧（134117888 ≈ 0x7FE7A00），属诊断/UDS 类，勿当应用报文。

**→ 2026-10-21 对表已完成（5 段快评 + 全量 415 段，见 §7）。悬念全部落定：0x354/0x350 在你车上存在（bus1 ~30Hz）但 7 小时窗口载荷仍空；「仅指纹有」4 帧你的车全窗口从未发；长地址帧坐实为 UDS 混入。**

## 2. 价值②：KF 雷达流水线（0x351-354 航迹接口的工程模板）

文件：`opendbc_repo/opendbc/car/byd/radar_interface.py`（528 行，全仓库质量最高的文件）。

现状：只解析 **0x374 RADAR_MRR @60Hz bus2（汉/唐 Veoneer MRR）**，且仅处理 `TargetID==2`（前方单目标）；开关是环境变量 `BYD_RADAR`（`interface.py:25`），注释点名「official OP 有 mpc solver 问题，carrot/sunny/forg 才有效」——这是本工程 radarUnavailable=False 决策（radard 三层兜底）的第三方旁证。

其输入规格与宋不同（单目标 60Hz vs tang 三槽 ~22.7Hz + TRACK_ID≠0 门控），**不能照搬，但每一级滤波都可以映射**到本工程「逆向 0x351-354 布局之后」的航迹接口：

| 级 | 他们的做法 | 行锚 | 对宋航迹接口的映射 |
|---|---|---|---|
| 预处理 | per-target 中值5 + EMA（Δ<0.5 α=0.4 否则 0.2） | radar_interface.py:246 | 直接搬，按槽位换 target 键 |
| 伪速度 | 滑窗6帧 `(t,d)` 最小二乘斜率 | :269 | 宋槽有 REL_SPEED 原始量，但 10-11 已证 REL_SPEED 单位/符号待标定——**伪速度可作交叉验证源**（sunnypilot 版对 Han 用 LONG_DIST/COUNTER 差分同思路） |
| 滤波核 | 2 维 KF [d, vRel]，匀速模型，Q=σa²·[[dt⁴/4,dt³/2],[dt³/2,dt²]]，Joseph 更新 | :81-166 | 完整可借；`vLead = v_smoothed + v_ego` 的自洽写法与本工程 `_finite` 教训（HUD 档层 measured=False 防伪 vLead）同域 |
| 门控 | 马氏距离 md²>25 拒绝（1DOF/2DOF 各按 χ²） | :126,154 | 可直接做「跟车槽位抖动剔除」，比本工程 radard 远侧否决更前置一级 |
| 失效策略 | 连续拒绝5帧→KF 重置；无效测量只 predict 不 update，输出 measured=False 滑行 | :398-407,439 | 对位本工程「丢帧置 measured=False、no_var_cnt≥10 剔除」 |
| 输出平滑 | 按车速分档 LPF（<10 τ=.5 / <20 τ=.3 / 其余 τ=.2；滑行档更慢） | :417-424 | 可作 V2 标定 dRel 时的稳定化候选 |
| 清理 | invalid_count≥6 且 age>1.5s 删滤波器 | :240-241,513 | 可借 |
| 诊断日志 | 每帧 append `/data/debug/byd_radar_complete_data.log`（raw/kf/final/md2 全量） | :491 | **移植价值高**：V1 gap 遥测阶段同款格式，比现造轮子快 |

## 3. 价值③：BydModifiedStockLong——第三条纵向路线（V3 备选）

文件：`opendbc_repo/opendbc/car/byd/tuning.py`、`selfdrive/controls/radard.py:421-444`、`common/params_keys.h:133-135`、`system/manager/manager.py:182-185`。

本工程纵向谱系现在是两端：

```
厂商全控（回退姿态，「跟车准」最低代价）          OP 全控 + HUD 合成 lead（radarUnavailable=False，V1→V3）
        ↑ 已定案                                ↑ 刚移植（d5fc77c4），依赖 HUD 档层标定
        └──────── carrot2 的中间态在这里 ────────┘
```

**机制**：保留厂商 ACC 做纵向主人，OP 只在回播 0x32E 时按「前车距离 × HUD 档位」对 AccelCmd 乘缩放百分比：

- `K_ACCEL_BP = [40,50,60,70,80] m`，每档位一组 POS/NEG 曲线（如 4 档 `NEG=[1.0,0.8,0.7,0.7,0.7]`：40m 内不减弱减速、远处减到 0.7；1 档近距全 1.2/1.3 增强的「紧迫」语义）——tuning.py:15-27
- 距离来源：radard 特判 `tracks.get(2)`（汉前方目标 ID=2），门控 `|yRel|<1.5 ∧ dRel>5 ∧ |vRel|<20 ∧ vLead>0.1` 才采信雷达航迹，否则 `get_RadarState(md, 1.0, 0.0)` 回落——radard.py:421-444
- 三参数：`BydModifiedStockLong` / `BydAutoTuning` / `BydLatUseSiglin`（默认全 0，manager.py 注入）

**对本工程的意义**：本工程的 0x32E 本来就是回播链（`bydcar 纯回播`，AccControlActive∧AccReqNotStandstill 闸门，10-05 厂商对齐已落地）——**在回播链上对 AccelCmd 加 lead-distance 缩放因子，比翻到 OP 纵向的代价低一个量级**，且不吃 HUD 合成 lead 的标定风险。若 V3（alpha 跟停）不过关，此路线是现成的第二方案。注意其前提与本工程一致：需要有一个可信 dRel（他们的 `tracks.get(2)` 是汉真航迹；我们对应 HUD 合成 lead 或逆向后的 0x351 航迹）。

配套观察：`BydAutoTuning` 把 byd 塞进 `torqued.py ALLOWED_CARS`（:36,:75），即 carrot2 允许 BYD 跑活车力矩学习；`tuning.py` 的 siglin 三参数 [4.867,1.09,0.243]（本工程 Han/Song21 组是 [1.807,1.674,0.04]，interface.py:18-22）可当本工程 vendor 七参数 non_linear 曲线的一阶 sanity 对照，不当调参目标。

## 4. 价值④：底盘信号目录补缺（0x222/0x223/0x2B6 + 时钟）

文件：`opendbc_repo/opendbc/dbc/byd_han_dmev_2020.dbc`、`carstate.py:119,191-201(注释),205-219(注释)`。

> **✅ 2026-10-21 对表绿牌**：0x222/0x223 在你车 bus0 以 60Hz 存在、0x2B6 以 ~2.4Hz 存在、0x418 以 ~24Hz 存在；carrot2 的信号定义直解码物理合理，且 DATETIME 与同段 gps 墙钟**秒级对齐（车辆时区=UTC+8 本地时间）**。执行记录见 §7。遗留：yaw 量程/符号需弯道段复核。

本工程 `byd_general_pt.dbc` 缺、carrot2 DBC 已有完整定义（含 scale/offset，免逆向）的 BYD 家族帧：

| 帧 | 名称 | 用途 |
|---|---|---|
| **0x222 (546)** | YAW_RATE（含 `YawRate`−`YawRateOffset`） | 本工程 carstate 未接；paramsd yaw 融合、将来横向质量（vendor 10-13 SteeringRateEstimator 同域）可直接用 |
| **0x223 (547)** | AXAY（ESP 横摆加速度） | 同上，lat 验证素材 |
| **0x2B6 (694)** | DATETIME（YY/MM/DD/hh/mm/ss，2Hz） | 见下 |
| 0x1D0 (464) | WHEEL_SPEED 四轮 | 备选车速源（现用 0x121 仪表脉冲） |
| 0x11F EPS | `SteeringAngleRate` @100Hz | 本工程 carstate:176 已接（值同源），carrot2 用它做速率限幅（`USE_STEERING_SPEED_LIMITER`，值=132→64 km/h 插值，默认 False）——可作本工程限幅参考实现 |

**DATETIME 校时提案**（对治本工程痛点）：设备「时钟每次冷启动退回 7-28、capture 文件名带错日期」是反复踩过的坑（ byd_long_capture 归档全受其害）。carrot2 在 carstate.py:205-219 留有一套**注释掉的**「读 0x2B6 → `date -s` UTC 校时」实现（带 system_time_valid 判断）。思路可借：先确认宋 bus0 实抓有 0x2B6 且解值合理（对照手机时间一分钟即可判），再决定落点（loggerd 校时 vs 采集器打时间戳），不必照搬他们的 `subprocess shell` 写法。

**顺带**：carrot2 主开关横向语义（013cda0「按钮功能」）：`CS.latEnabled = BTN_TOGGLE_ACC_OnOff ∧ 速度允许`，VCruiseCarrot 的 `_lat_enabled` 在多处从 `CS.latEnabled` 同步（cruise.py 三处 hook），把「主开关=横向使能」做成单一事实源——与本工程「main-on latch 被事件打掉→僵尸会话」是同一问题的另一种解法，值得读不可 cherry-pick（两份 cruise.py 已分叉）。

## 5. 反面清单（勿参考/需警惕）

| # | 问题 | 证据 |
|---|---|---|
| 1 | **safety 力矩校验被实际禁用**：`if (steer_torque_cmd_checks(...)) { tx = true; }` 返回值被丢弃（tx 初值即 true），注释自认 `//false; disable this` | safety_byd.h:134-136 |
| 2 | **宋22/23 纵向是死的**：`EXP_LONG_CAR` 只含 Song_**21**（interface.py:152）→ alphaLong 不可用；acc_cmd 发送门 `longActive ∧ mrr_leaddist>3`，而 `mrr_leading_dist` 初始 0、仅 `BYD_RADAR` 环境且收到 0x374 才更新（carstate.py:191）——宋无 0x374，永假，AccelCmd 冻结。厂商纵向又被 fwd 屏蔽（ACC_CMD bus2→bus0 掐断）→ 宋的「纵向」实际无处可来 | interface.py:143-152, bydcan.py:96 |
| 3 | **按钮事件 copy-paste bug**：`btn_acc_set_reset = cp.vl["PCM_BUTTONS"]["BTN_AccCancel"]`（应为增减速度信号） | carstate.py:97 |
| 4 | specs 自认未填（values.py:72 注释 "likely be incorrect, developers please fill"），10 平台共用 Han 一个 DBC，宋系无专属 DBC | values.py |
| 5 | 替换相机架构（假 0x318）与本工程装机现实（厂商相机在线、ACC 会话属司机、协作式）冲突；采纳它=失去厂商会话/HUD/AEB 协作且他们宋路径并未跑通。**知道平行宇宙存在即可** | bydcan.py:111-156 |
| 6 | 零测试、零文档；SubMaster-in-carcontroller + `/data/debug` 每帧文件 append（card 路径同步磁盘写，本工程 card 路径纪律红线，勿学） | carcontroller.py:19,radar_interface.py:491 |
| 7 | SEAL 安全变体（0x1E2/0x1FC 角度转向、FLAG_SEAL 0x20）只在 C 侧，python 无对应平台——未完成的第三车型线 | safety_byd.h:34 |

## 6. 下一步候选（按性价比排序）

1. **指纹对表**：✅ 完成（5 段快评 + 全量 415 段，§7.2/§7.4）。四帧生死已判（车无）、本车指纹表已生成。**剩一项独立工作（本文档未做）**：把 full_census.json 的本车指纹回灌本工程 card 的 fuzzy-match，与 sunnypilot 移植版 fingerprints.py 做差异登记。
2. **航迹接口预读**：逆向 0x351-354 信号布局时，把 carrot2 KF 流水线（§2）作为滤波层起点；其 md2 门控 + measured=False 滑行 + 诊断日志格式直接可借。
3. **V3 备选路线登记**：ModifiedStockLong（距离×档位缩放回播 AccelCmd）记入 `openspec changes/byd-radar-synth-lead-port` 观察项，V3 不过关时启用。
4. **校时小探针**：✅ 存在性与解码合理性已证（§7.3，0x2B6=UTC+8 本地时间、与 gps 秒级一致）；剩「落点设计」（loggerd 校时 vs 采集器打戳，及 ±1 日跨 UTC 边界处理）。
5. **yaw 量程复核**：✅ 完成（§7.6：corr 0.90-0.97、斜率≈1.0、符号/量程/尺度三验通过；实测 ~50Hz；顺带独立旁证 SR=19.5）。剩实施项：carstate 接 `ret.yawRate`（出探索模式）。
6. **指纹回灌**（原 §6.1 尾巴）：✅ 验证完成（§7.7：活体单腿依赖 bus1，bus0 缺 5 常发帧，反事实已复现淘汰）。剩实施项：`fingerprints.py` 增补 0x41A/0x4BB/0x4D9/0x527/0x52A（+5 条件帧可选）——**改代码需退出探索模式**。

## 7. 对表执行记录（2026-10-21）

### 7.1 方法与数据源

- 数据源：`~/Documents/work/op/sunnypilot/docs_site/byd_logs_2026-10-05_07/realdata/`（415 段 rlog、5 条主路线；10-05~06 自家宋PLUS DM-i 实车，sunnypilot 当日运行日志）。
- 口径：rlog `can` 消息按 `(address, src)` 计数，**src≥128 的 TX/relay 镜像全部剔除**，只普查 RX；carrot2 指纹为跨总线合并 dict（已被 §7.2 长帧证据坐实），对表按地址级比集合、总线归属以实车普查为准。
- 采样：快评取 5 段（route2--40/120/200 + route3--3 + route5--30，各 60s，carState 全程 engaged≈跟车态）。
- 已知坑：rlog 里 can 事件的 logMonoTime 不可靠（当日设备时钟 bug 现场），Hz 一律用「帧数/段时长 60s」口径；`Corrupted events` 警告为 sunny schema 差异，无害（10-09 已证）。
- 脚本与产物（2026-10-21 已归档，gitignore 区）：`local_docs/byd/fingerprint_census_2026-10-21/{census.py,full_census.py,full_census.json,fp_song22.txt}`。

### 7.2 集合对表（5 段快评口径）

```
实车 RX：218 个 (addr,bus)，地址去重 209
carrot2 Song_22 指纹：205 地址（单 dict、跨总线）
交集 201
├─ 仅指纹有(4)：0x135 0x1A2 0x27B 0x2DD   ← 全量裁决(§7.4)：415 段≈7h 从未见，车无此帧
└─ 仅实车有(8→13，全量扩容见 §7.4)：0x41A 0x4BB 0x4D9 0x527 0x52A + 29位长帧 ×3
     · 0x527/0x52A(1319/1322) 在 carrot2 的 SONG_21 指纹里、0x4BB(1211) 在 QIN 指纹里
       → 属捕获条件差异，非车队/配置差异——你的车样本比他们 Song_22 那次更全
     · 29 位长帧 0x7FE7A00 / 0x1BFE7A1A / 0x1BFE7A1B：
       0x7FE7A00 = Tang_DM 指纹里那个 134117888（=他们把 UDS 诊断流量混进了指纹），实车同样存在
```

关键地址判定（帧数/300s；`变化字节` 来自逐字节 distinct 集，capped）：

| addr | bus | Hz | ∈fp | 变化字节 | 判定 |
|---|---|---|---|---|---|
| 0x350 | 1 | ~30 | Y | b0,b1 | **帧在、载荷空**——与 10-18 报告「track 全 0 填充不变」再证一致 |
| 0x351-0x353 | 1 | ~30 | Y | b0,b1 | 同上（样例 `427f00002000e000`，四帧同构） |
| **0x354** | 1 | ~30 | Y | b0,b1 | **你车上存在**，但无载荷变化；carrot2 指纹含 852/848 只是「量产宋有此帧」的双车旁证 |
| 0x100 | 1 | ~12 | Y | b0,b1 | 复现 9-28 登记（0x100≈10-12Hz 伴生） |
| 0x101-0x108 | 1 | ~30 | Y | b0,b1 | 「状态广播家族」复现 |
| 0x109/0x10A | 1 | ~30 | Y | b0,b1,b3,b6,b7 | 变化面更宽，与 byd-radar-010x 的 b7 presence/dRel 主源设计吻合 |
| 0x222 YAW | 0 | 60 | Y | b0,b1(→b6,b7 counter) | 存在，见 §7.3 解码 |
| 0x223 AXAY | 0 | 60 | Y | 同 | 存在 |
| **0x2B6 DATETIME** | 0 | ~2.4 | Y | b5（秒） | 存在，见 §7.3 |
| 0x418 BSD | 0 | ~24 | Y | b6,b7 | 存在（carstate 已接，无新增） |

### 7.3 解码验证（route3--3，60s，按 carrot2 DBC 定义直解）

| 帧 | 结果 |
|---|---|
| 0x2B6 首/末帧 | `(26,10,6,1,41,3)` → `(26,10,6,1,42,15)`（YY/MM/DD/hh/mm/ss 逐字节） |
| 同段 gps wallTime | 1791222064 s = **2026-10-06 01:41:04 (UTC+8)** → 与车辆帧 `10-06 01:41:03` **秒级对齐**；结论：车辆时间=本地(UTC+8)且准确——而设备自身时钟当时错显 7-28。**校时原料实锤**（落点见 §6.4） |
| 0x222 YawRate | min -0.012 / max +0.011 rad/s，全程≈0（直线段物理合理，**量程/符号未饱和验证**→§6.5） |
| 0x223 Ax | -0.02 → +0.93 m/s²（巡航微加减速，合理） |
| route2--40 段 0x2B6 | `1a0a05 15291f` = 26-10-05 21:41:31，同样落在真实时间带内 |

### 7.4 全量跑批结果（2026-10-21 完成）

415 段 × 8 进程，≈7 小时 RX 普查，**332 个 (addr,bus) 组合**；产物 `local_docs/byd/fingerprint_census_2026-10-21/full_census.json`（summary + 本车指纹全表：addr,bus → 帧数/长度/字节变化谱/样例帧，重跑 `full_census.py` 即可再生）。

- **裁决①「仅指纹有」4 帧**：0x135/0x1A2/0x27B/0x2DD 在 415 段全窗口**零出现**。carrot2 那台 Song_22 样车有、你的车没有（配置/封装或当年捕获混入他源）。处理：不入本车指纹；card fuzzy-match 时此为合法差异。
- **裁决② car_only 全量化 8→13**：
  - 常发慢播（全 415 段覆盖）：**0x41A(~1.2Hz)、0x4BB(~1.3Hz)、0x4D9(~2.4Hz)、0x527(~2.4Hz)、0x52A(~2.4Hz)**——carrot2 Song_22 那次捕获漏掉的，其中 0x527/0x52A 就在他们 SONG_21 指纹里，互证「宋系有帧、非你车独有」。
  - 低频条件帧（12/23/2 段不等）：0x180、0x299、0x40C、0x4A8；0x2BC 全程仅 5 帧（事件帧）。5 段快评漏检正因这些是条件触发。
- **裁决③ 0x350/0x354**：全窗口 742,452 / 742,450 帧（~29.8Hz），字节变化谱仍 **只有 b0/b1**——「航迹载荷空」对 7 小时覆盖成立为铁证。carrot2 指纹含 848/852 仅证帧存在。
- **附加发现**：底盘帧（0x222/0x223/0x2B6/0x41A 等）的 bus2 镜像**只出现在同样那 6 段**（各 ~55-2700 帧）→ 这 6 段即 10-18 引线并接实验期（bus0→bus2 relay），与当日「panda 端零新帧」记录互证。

### 7.5 附注（当日 TX 参考）

10-05 该设备上 sunnypilot 的 src≥128 地址集含 ~95 个（含 0x10D/0x10E/0x316/0x318/0x32D/0x32E/0x32F/0x3B0 及其余 relay 镜像）——注意 src≥128 混有 panda bus0→bus2 转发，**不等于 OP 主动发送**，分析须按 src 精确到 128+bus 拆（10-05 四趟已立此规矩）。

### 7.6 弯道 yaw 探针（§6.5 ✅，2026-10-21）

方法：route2 全段扫描 carState `Σ|ΔsteeringAngle|` 选转向丰富段（--0 低速大转角 TV=682°、--141、--211），按 carrot2 定义解码 0x222（`raw12b@0·0.002133−2.094 −（raw12b@12·0.002133−0.13）`），与运动学 yaw `v·tan(δ/SR)/WB` 做时移扫描相关（本工程 specs：SR=19.5/WB=2.765；平滑 21 帧；v>1.5 采样）：

| 段 | 相关系数 | 回归斜率 | 截距 | 量程（实车 yaw） |
|---|---|---|---|---|
| --0（低速转弯丰富） | **+0.93** | **1.03** | +0.001 | ±0.30 rad/s |
| --141（高速微调） | +0.90 | 0.83 | -0.003 | ±0.04 |
| --211（直线巡航） | +0.97 | 1.02 | -0.000 | ±0.01 |

**结论：0x222 的 DBC 解码在量程、符号（与转角同向，左正）、尺度（斜率≈1）三面全部验证通过**——carrot2 的 YawRate/YawRateOffset 定义可直接给本工程 carstate 接（`ret.yawRate` 现缺）。两个附注：① 实测帧率 ~50Hz（标称 60Hz 别信，接 carstate 频率表用 50）；② 斜率≈1 是 **SR=19.5 的独立旁证**（若真 SR=15 像 carrot2 那样设，斜率应≈1.3）——顺手给本工程 values.py 的 steerRatio 记一功。

### 7.7 本车指纹回灌验证（§6.1 验证部分 ✅，2026-10-21）

本工程活体 fingerprinting 是 **legacy 排除式**（`car_helpers.can_fingerprint` + `fingerprints.eliminate_incompatible_cars`）：候选=全品牌 37 个，逐帧在 bus0/bus1 排除（bus2 不参与），某 bus 唯一候选即定案（frame>100）。**10-08 台架那次「指纹匹配成功」实为 `FINGERPRINT` 环境变量强制指定，未测排除**。

三方集合校验：本工程 `fingerprints.py` 的 Song_22 dict 与 carrot2 的**逐地址全等**（205，同一社区来源）——即两个 fork 共享同一台「样车」的指纹，都≠你的车。

- **bus1（雷达轨）**：实车 bus1 帧集与 dict **零冲突**（0x350-354/0x100-0x10A 全在 dict 里，415 段全窗验证）。
- **bus0（底盘轨）**：10 个实车帧不在 dict（全 len=8）：常发慢播 **0x41A(~1.2Hz)/0x4BB(~1.3Hz)/0x4D9(~2.4Hz)/0x527(~2.4Hz)/0x52A(~2.4Hz)**（全 415 段覆盖）＋条件帧 0x180/0x299/0x2BC/0x40C/0x4A8。
- **真·时序模拟（原始到达序）**：3 个行驶段都在 frame 101 由 **bus1 抢先定案 BYD**、bus0 的违规帧没轮到出场——今天的成功是赛跑赢的。
- **反事实（雷达静默，只喂 bus0）：seg--3 在第 28 帧被 0x52A 淘汰 BYD → 失败**；另两段侥幸赢。即：**活体匹配单腿依赖 bus1 雷达总线，点火窗雷达若未上电/静默，bus0 必淘汰**（点火窗雷达是否有流量本档日志测不到——rlog 起点晚于 card 起来，属未证假设）。
- **修复方案（待出探索模式实施）**：给 `opendbc/car/byd/fingerprints.py` 的 Song_22 dict 增补 5 个常发慢播帧（+可选 5 个条件帧，排除式匹配对多余地址宽容：0x527/0x52A 恰在 carrot2 的 SONG_21 指纹里，属宋系家族合法帧）。补后 bus0 单轨可自给，双保险。carrot2 那份 dict 有同款缺陷（他们 bus1 还未必有雷达帧——他们宋无数据通路佐证）。
