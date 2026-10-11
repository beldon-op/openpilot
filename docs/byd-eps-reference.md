# BYD 宋 PLUS DM-i EPS（LKAS 执行器）参考手册

> 汇总自 16 个根因的实车调试全记录（`byd-song-plus-lkas-debug.md`）。所有结论以**本车实车 CAN 日志**为依据，参考实现（yysnet/opendbc、op_byd 厂商系统、mouxangithub）仅作旁证并标注验证边界。
> 本文是"现在时"快照：每条规则给出证据与复现次数，未定论项集中在 §11。最后更新：2026-09-29 深夜（根因 16 二次修正 `51b1a25a`：c 相位标志非许可，c 门死锁已撤，改请求包络 200 + 手轻武装门；§3.10/§4/§5/§6/§10 同步刷新）。

---

## 一、在系统中的位置

- DiPilot MPC（前相机，官方名"多功能视频控制器"）是原车 LKA/ACC 的大脑；**EPS 是唯一横向执行器**。
- EPS 的 LKAS 命令唯一来源 = bus0 的 **0x316（ACC_MPC_STATE）**。相机原发的 0x316 在 bus2，panda byd 固件 `fwd_hook` 阻断 bus2→bus0 的 0x316/0x1E2 → **OP 是 EPS 唯一的 0x316 提供者**（调试记录根因 1 实证：OP 不发时 EPS 端 SteerWarning=100%、"请检查多功能视频控制器"）。
- 启动阶段 pandad 尚未切入 byd safety 模式的 ~8s 内，相机 0x316 会短暂透传到 bus0（R10_000 实测：384 帧 @48Hz，之后归零）——不是持续桥接。
- 总线拓扑（实车 CAN 统计 + **2021 款维修手册官方端子表 P13**）：bus0 = **公有 CAN（底盘网）**：底盘/动力/车身 + EPS 反馈；bus2 = **私有 CAN（接雷达）**：仅 5 条（0x316/0x32D/0x32E/0x32F/0x432）；两总线地址交集为 0 → **必须全量转发 bus0↔bus2**（白名单饿死雷达，全 block 相机+雷达双饿死，根因 1 前置实验）。
- **发送方修正（手册 DTC `U023587`）**：0x32D/0x32E/0x32F 的发送方是**雷达 MRR**，不是相机——bus2 透传 = 透传雷达帧（ACC_CMD counter 自由递增由此解释）。
- **MRR 逐帧校验 MPC 的 0x316**（手册 DTC `C2F0882/83/86/87`：私有 CAN MPC 滚动计数器/校验值/信号无效/失去通讯）——根因 2"echo 字段不可缺省"的官方机理：我们替换 MPC 发 0x316 时雷达一直在当监工。
- 巡航进入官方硬条件（ACC 手册症状表）：主驾**安全带、车门、前舱盖开关** + OK 档电——根因 7 的官方出处。官方只承诺 LDW ≥60km/h，LKA 速度范围未公开（我们 2-23km/h 能控是 op_byd 实证能力）。

## 二、报文清单（频率为实车实测）

| 报文 | ID | 频率 | 总线 | 方向 | 作用 |
|---|---|---|---|---|---|
| ACC_MPC_STATE | 0x316 | 50 Hz | bus0（OP 发送） | OP→EPS | LKAS 命令：扭矩 / 会话模式 / 握手 |
| ACC_EPS_STATE | 0x318 | 50 Hz | bus0 | EPS→OP | 反馈：Prepared / **CruiseActivated（执行电源线）** / TorqueFailed / SteerWarning+err / 扭矩 |
| EPS | 0x11F | 100 Hz | bus0 | EPS→ | SteeringAngle / SteeringAngleRate（4°/s/bit，0-1020） |
| PCM_BUTTONS | 0x3B0 | 10 Hz | bus0 | 车身→ | ACC 按键（OP 伪造 resume；fwd 不阻断） |
| ACC_HUD_ADAS / ACC_CMD | 0x32D / 0x32E | 20 Hz | bus2 | **雷达 MRR→**（手册 U023587 确认） | ACC 状态 / 激活；真实转发（厂商曾整套伪造，我们不需要） |
| STEERING_MODULE_ADAS | 0x1E2 | — | bus0 | — | 角度命令（实验路径，宋 Plus 不发送，未启用） |

DBC：`opendbc_repo/opendbc/dbc/byd_general_pt.dbc`；固件侧安全模型：`opendbc/safety/modes/byd.h`。

## 三、0x316 命令帧协议（字节级）

| 字段 | 位定义 | 语义 |
|---|---|---|
| LeftLaneState / RightLaneState | 4\|2 / 34\|2 | 请求 LKA 时必须 =2（有车道目标）；idle 0/0 |
| **LKAS_Config** | 6\|2 | 1=ALARM，2=LKA，3=ALARM_AND_LKA；**3 = 出力会话许可** |
| SETME2/3/4/5/7、MPC_State、AutoFullBeam*、AlarmType、TrafficSign* | 各位 | **必须回显相机原发值**（不可缺省） |
| LKAS_Output | 16\|11 有符号 | 扭矩请求（厂商 p50=67 / p90=128 / max=193） |
| LKAS_ReqPrepare | 27\|1 | 握手请求 |
| LKAS_Active | 28\|1 | 激活位 |
| LKAS_State | 36\|4 | 0=关 / 1=待机 / 2=转向中 / 4=取消中；**用 1**（厂商模板） |
| Counter | 52\|4 | 0-15 循环 |
| CheckSum | 56\|8 | `byd_checksum(0xAF, dat)`——与相机原发帧逐字节对拍全部 MATCH |

**十条规则（全部实车字节级验证，括号为调试记录出处）**：

1. SETME_*/MPC_State 等回显字段不可缺省——从零构造 = 非法帧 → ADAS 域故障"check multifunction video controller"（根因 2，`1432de9`）。
2. 校验算法 0xAF 已对拍验证（调试记录 二.2）。
3. 握手时序：`ReqPrepare=1` → EPS 置 `LKAS_Prepared`（0x318 bit0，实测 38-50ms 应答）；**ReqPrepare 必须先于 Active**（调试记录 二.3：完全没有 prepare 请求则 EPS 永不 armed），但**不必等 Prepared 应答再发 Active**——厂商 3 帧 burst 后 50ms 无条件 Act=1（route 00000037 帧级实证），且**厂商全程 EPS `LKAS_Prepared=0` 依然出力**。Prepared 不是出力前提，其置位/清除语义见 §5/§11.5（c=0 拒绝状态下它照样置 1）。
4. LaneState 必须 =2，否则 EPS 不执行 LKA（调试记录 二.4）。
5. Config 组合合法性：`Config=1(ALARM)+LaneState=2` 是非法组合 → EPS 锁 SteerWarning、拒绝 armed；请求 LKA 时 Config=2（调试记录 二.5）。
6. **50Hz 连续流不可断**——断流 → SteerWarning 锁存 → 拒绝 armed → MPC 报"视频控制器"（根因 1，`8496a71` 无条件发送）。
7. **流形态许可 = LKAS_Config=3**。厂商模板：激活帧 `(State=1, MPC=0, Config=3, Active=1, Lane 2/2)`，待机帧 `(1,0,3,0, Lane 0/0)`；相机待机只发 1/2、从不发 3——echo 永远拿不到出力许可（根因 11，`da3c4e2`）。Config=3 只是流形态，**执行许可另需 EPS 侧 CruiseActivated=1（规则 10）**。
8. LKAS_State 用 1：6.12 曾改 2 是错误方向（state=2 帧会被接受回显但助力不介入，且包络变窄），厂商字节模板 State=1（根因 10 → 11 更正）。
9. **armed 会话（Config=3 + Active=1）的行为约束**：请求不得长时静默、不得与驾驶员对向——见 §6 锁存规则（根因 13/14）。
10. **`CruiseActivated`（0x318 bit1）= EPS 自己的会话相位标志（"正在执行"），不是许可、不是原车 ACC 状态（根因 16，两次改判）**：
    - 错误模型 v1（当许可门）：c=0 不武装 → **死锁**——c 只在接受会话后才置 1，我们等 c、EPS 等我们，双向等待（0000000d 两段 7218 帧 TX 全零，零控车）。
    - 正确模型（厂商帧级实证，route 7--12e）：**先出力、等激活**——厂商 act=1 且 c=0 的等待相位长达 2.86s（415 帧），等待期 demand 全程 ≤193 从不 rail；c 翻 1 = EPS 接受会话开始执行。
    - EPS 激活条件未完全逆向，驾驶员手轻是主导可观测量（0000000a：76.3s 手轻瞬间激活，手重时段全程不激活；108.8-109.5s 手轻仍不激活的例外存在）。
    - **教训：把"因"当"果"做门控 = 死锁**。等待相位的安全性靠请求包络（STEER_MAX_REQUEST=200）+ 武装手轻门（STEER_ARM_DRV_TORQUE=50），不靠等许可。厂商解密状态机读该位是**监视**相位（`is_steering_need_activate`/`is_steering_activated`），不是门控自己的输出。

## 四、会话生命周期（厂商编排，route 00000037 逐帧实测；我们已重写为同一架构，2c28a0cd09）

**总前提（规则 10 修正版）：会话照常流（prepare 突发 → Act=1 → 有界 demand ≤200），EPS 在自己的条件下激活（c 翻 1）后才真正出力**——等待相位（c=0、p=1）可长达 2.9s，靠请求包络与武装手轻门保证无害。**勿再按任何"许可位"门控会话流**（c 门死锁教训：0000000d 两段 7218 帧全零、零控车）。

| 阶段 | 厂商行为 | 我们的行为（现架构） |
|---|---|---|
| 怠速 | 不发 ReqPrepare；帧 `(State=1, MPC=0, Config=3, Active=0, Lane 0/0)` | 同左，无条件常开（含 c=0 时段；勿回退到"回显相机 Config"） |
| 进场 | engage 时 3 帧短促 prepare（60ms）→ **不等 EPS 应答** 50ms 后立即 Act=1 | 同左（3 帧 ReqPrepare 突发 lanes 2/2 → 无条件 Act=1）；**武装另需手轻**（\|drv\|<50，根因 15/16） |
| 激活 | 扭矩 0.24s 内 0→107（~9/帧@50Hz）→ 稳态 101-113 | 从 0 起步、DELTA 16/帧 限速爬升；对向时 driver-limit（ALLOWANCE=120）削幅——厂商 -153 vs +166 同款 |
| 稳态 | Config=3 + Act=1 + State=1 + Lane 2/2，**请求非零贯穿全程**，与驾驶员正面对抗不退让 | 同左（满权限 STEER_MAX=300）；armed-零 0.42s 背停（21 帧）是唯一我方额外退场 |
| 退场 | Act=1 保持下斜坡 52→0（~80ms）再切 Act=0 → **立即 re-burst 重连**（60-80ms，日志两次零投诉） | 同左；TF 硬切不复活；**SteerErrorCode≠0 退场后不重试**，等 EPS 清除且 c=1 |
| 等待相位（合法） | act=1 + c=0 最长 2.86s（415 帧），demand ≤193 从不 rail | 同左（≤200 包络）；旧架构在此相位 rail ±300 = R5 真因；c 门死锁（0000000d）是错误修复，勿复 |

## 五、0x318 反馈语义（50Hz）

| 字段 | 位定义 | 语义 |
|---|---|---|
| LKAS_Prepared | 0\|1 | 握手应答；**注意 Prepared=1 ≠ 会出力**——c=0 拒绝状态下照样置 1（根因 16），不能当执行判据；厂商 route 00000037 更是全程 Prepared=0 仍正常出力——置位条件在两车/两时观测不一致（我们频繁置 1），语义待深挖（§11.5） |
| **CruiseActivated** | 1\|1 | **EPS 自己的会话相位标志（"正在执行"）——非许可、非原车 ACC 状态（规则 10）**。接受会话后才置 1（0000000a：旧架构已武装 3.6s 后 76.31s 才置 1）；err=2 退场时与 p 一同清 0；与 OP 的 cruiseState.enabled 完全不同步。**勿门控于此位**（c 门死锁：0000000d 两段 7218 帧全零、零控车，`51b1a25a` 修正） |
| **TorqueFailed** | 2\|1 | **锁存故障**：EPS 放弃全部转向输入，**直到点火循环**（route 0000001c/20/22 + yysnet 同注 "EPS give up all inputs until restart"）；OP 收到即硬切退让 + UI "LKAS Fault: Restart the car to engage"（0000000a 实证：锁存后第二段全程 fault=True，重启车才清） |
| SteerWarning | 4\|1 | 见 §7；err=2 预警时 warn 同步置 1（0000000a 三次 err=2 均 warn=1） |
| SteerErrorCode | 5\|3 | **码表部分解码（0000000a + c9f1698c82）**：2 = ~0.5s 站下预警（可恢复，解除违规状态即清零，本日 3 次中 2 次自行恢复）；4 = 升级锁存，与 TorqueFailed 同帧出现（110.33s err=4+tf=1）→ 永久 LKAS Fault。**2026-10-11 二轮挖掘（415 段 b33616e596 语料）首次观测到 1/3**：err=3 639 帧=单一 episode ≈12.8s（seg 3e696e--0 t=79.2，c=1 会话健康、warn/tf 全程 0、自恢复回 0）；err=1 36 帧。两者形态良性，疑似"降级但续会话"档；语义仍需 EPS 分册。5-7 仍未观测 |
| **MainTorque** | 8\|12 有符号 | **执行侧扭矩，非无条件回显**——6.12/6.18 的"纯指令回显"结论被根因 16 推翻：c=0 拒绝窗口 demand -240 持续 4s 而 mt=0（回显说不成立）；执行时紧跟 LKAS_Output（相关 0.98，0000000a 实测 mt 略超 demand：-285 vs -240、+205 vs +198）。**mt==0 且 demand≠0 = "EPS 未执行"的可靠判据**（本次诊断即用它定位死会话）；是否等于电机实际出力仍存疑，但执行判据用途已定论 |
| ReportHandsNotOnSteeringWheel | 21\|1 | |
| SteerDriverTorque | 24\|12 有符号 | 驾驶员扭矩 raw 量程，见 §8 |
| Counter / CheckSum | 52\|4 / 56\|8 | XOR 校验（开源 port 说法，未字节验证；我们只收不发） |

## 六、锁存规则（TorqueFailed；armed 会话 = Config=3 + LKAS_Active=1）

第二次驾驶 6 处真锁存逐帧时序（`docs_site/hour_logs_2/latch_report.txt`，**旧架构时代**——触发状态（armed-零/对向让位）在现架构下已结构性消失，保留作 EPS 行为记录）：

| 案例 | 不良状态开始 | SteerWarning | TorqueFailed | SW→锁存 |
|---|---|---|---|---|
| Rf_015（ACC 重 SET 时驾驶员正打盘） | armed-零 −0.72s | −0.50s | 0.00s | 0.50s |
| R10_000（出力 24 时被抓盘，让位归零） | 零请求 −0.48s | −0.50s | 0.00s | 0.50s |
| R15_001（方向盘停在 −34° 时 armed） | armed-零 −0.71s | −0.51s | 0.00s | 0.50s |
| Re_014（armed 时驾驶员持盘 −13.5°） | armed-零 −0.71s | −0.51s | 0.00s | 0.50s |
| R13_003（armed 时驾驶员持盘 stq 110~150） | armed-零 −0.64s | −0.50s | 0.00s | 0.50s |
| R16_000（OP −70 vs 驾驶员 +46 对向） | 对向 ~−0.63s | −0.49s | 0.00s | 0.49s |

**新案例（根因 16，route 0000000a seg1，2026-09-29）——err=2 三击升级链**：
同一行程内三次 `SteerErrorCode=2 + SteerWarning`（t=78.07/103.99/109.79，均 ~0.5s 后解除或升级）：
前两次 EPS 自己退场（c 位清 0）→ 我方重连恢复；**第三次（取消 ACC + 60ms 内重 SET，驾驶员 +210 猛拉，我方 rail 的 +218 demand 仍在总线上）0.48s 后升级 `err=4 + TorqueFailed` 同帧出现 → 永久 LKAS Fault**。教训：err=2 时总线上残留的大幅值 demand（rail 后限速下坡要 0.33s+）是升级的催化剂——预防 = 不制造 rail（c 门），而非加快退场（固件 rate 18/帧 下坡是硬下限）。

**规则终版（2026-09-29 改判后）**：

| # | 触发条件 | 证据 | 现行防御 |
|---|---|---|---|
| R1 | **armed 会话请求 ≈0 持续 ~0.5s**（容忍度实测 ≤0.48s；一小时内 4 处 + 第二次驾驶 5 处） | 6.23、6.24；route 33 第 7 次锁存同属此类 | armed-零 0.42s 背停（21 帧@50Hz 退场+burst 重连）+ 武装手轻门；旧静默守卫 0.16s/engage 门已删 |
| R2 | ~~armed 会话请求与驾驶员对向~~ **改判（9-29 厂商实证）：对向本身不触发**——厂商 -153 vs +166 正面对抗零投诉；R16_000 真凶是旧让位逻辑制造的零请求/±20 振荡（会话犹豫，非对抗） | R16_000 重读 + route 00000037 对抗段 + 0000000a +284 vs -285 | **无防御 = 厂商语义**：正面出力不退让（对向跟随逻辑已删，勿复活） |
| R3 | 错标帧时代（Config=2/State=2）的合理性规则：\|转角\|>50° 后 ~10s、全锁速摆动 >150°/s、幅值 ~55+ | 路测 3-5；厂商 Config=3 下用到 193 且无门控也干净 → 错标帧规则 | **角度/速率门已整体删除**（根因 6.30 的 100s 静默来源）；大弯/泊车靠 c 门 + 手轻门放行 |
| R4 | 非会话类：断流（根因 1）、非法 Config 组合（二.5） | 调试记录 一/二 | 无条件 50Hz 发送 + Config 逻辑 |
| R5 | **等待相位 rail**（会话已流、EPS 未激活 → 开环 windup ±300 → 激活瞬间甩盘 → err=2 → 升级 err=4+TF）＋其错误修复（c 门）→ 死锁零控车 | 根因 16 两次实证：0000000a（rail 三击链）+ 0000000d（c 门死锁，7218 帧全零）；厂商等待相位 415 帧从不 rail | **请求包络 STEER_MAX_REQUEST=200**（rail 结构性不可能）+ 武装手轻门 \|drv\|<50（等待期通常 <100ms）+ **照常流会话等激活**（勿等许可） |

**恢复**：仅点火循环清除（0000000a 第二段全程 fault=True 实证，重启车后第二 route 正常）。锁存后每次 engage 只是向故障 EPS 重发 ReqPrepare、永不 armed——仪表症状为"绿框但方向盘零扭矩"。OP 侧 `TorqueFailed → steerFaultPermanent` 硬切 + UI 告警（`ddf29f8`）。

## 七、SteerWarning（0x318 bit4）

- 触发：armed 会话进入零请求后 **0~0.22s**（R10 的 SW 在归零前 0.02s——抓盘瞬间即触）；错标帧非法组合；**err=2 预警与 SW 同步出现**（0000000a 三次 err=2 均 warn=1，t=78.07/103.99/109.79）。
- **不变量：SW → TorqueFailed 恒 ~0.50s（6/6 + 0000000a 第三击 = 7/7，109.85→110.33 = 0.48s）**。
- **SW 自行清除案例已获得（2026-09-29，2/2）**：0000000a 前两次 err=2/SW（78.07、103.99），EPS 自退场（c 位清 0）+ 我方停止请求后 ~0.3s 内全部清零、无锁存——"SW 持续 0.5s 才锁"假设获实证支持，退场可解；但**清除的前提是违规状态真的解除**（第三次 rail demand 残留 → 未及清除 → 升级）。
- 会话退场（idle echo）后 SW=1 **不**进展为锁存：大量 disarmed 段 SW=1 长期存在、零锁存（实证）。
- stock LKA 类比：手扶 → SW → 相机退出会话 → SW 清除 → 无锁存（推断 + 上述 2 例支持，解释了原车 LKA 手扶即释放却从不锁存）。

## 八、驾驶员扭矩量程（SteerDriverTorque，raw ±2048）

| 状态 | 量程 |
|---|---|
| 脱手噪声 | <50（峰值，振荡） |
| 轻握 | 60-150 |
| 用力/对抗 | >150，剧烈 200-300（0000000a 实测峰值 +296，泊车对抗） |

- `STEER_THRESHOLD=80`（steeringPressed，5 帧防抖）。
- driver-limit（现架构内置于控制器，ALLOWANCE=120）按厂商语义执行：对抗时仍正面出力（厂商 -153 vs +166），武装时刻（burst→active）要求 \|drv\|<50（根因 15/16）——**只门武装，不门会话内**。

## 九、出力标定（厂商 Config=3 实跑，route 00000037）

| 参数 | 厂商实跑 | 我们（现架构） |
|---|---|---|
| 扭矩 | p50=67 / p90=128 / max=193 | STEER_MAX=300（values.json 真值；实际幅值由 driver-limit 与横向环决定，回放包络 ≤193 同厂商） |
| 帧间步进 | p99≈16（其固件限 17；我方固件实测限 18） | DELTA 16/16（厂商 p99 对齐） |
| 进场 ramp | ~9/帧（0.24s 到 107） | 从 0 起步同 DELTA 16 限速（无独立软启动层，已删） |
| steerRatio / mass | 19.5 / 1926 | 同（`84e2ae8`） |
| 横向模型 | factor 2.5 / friction 0.10 / delay 0.3 | 同 |

## 十、现行门控矩阵（2026-09-29 厂商会话架构重写后，代码 ↔ 规则）

**现存的仅 5 项**（厂商语义 + 两个我方背停，再无防御塔）：

| 门控 | 参数（values.py） | 防的规则 | 来源 |
|---|---|---|---|
| **请求包络** | STEER_MAX_REQUEST=200（厂商全程 max 193；ELF HIGH error=200） | R5 等待相位 rail/升级链 | 根因 16 修正版，`51b1a25a` |
| **武装手轻门** | STEER_ARM_DRV_TORQUE=50（只门 burst→active，不门会话内） | R1 变体（cold-arm，Rf_015/R15/Re_014/R13） | 根因 15 回归，`2c28a0cd09` |
| armed-零背停 | STEER_ZERO_EXIT_FRAMES=21（0.42s，实测锁存带 0.48-0.72s 内退场+burst 重连） | R1 | 根因 13/14 语义重写 |
| 刹车抑制 | 3 帧按下/5 帧释放 | R1 变体 | — |
| TorqueFailed/err 处理 | TF 硬切不复活 + steerFaultPermanent 告警；err≠0 退场不重试 | 恢复路径 | 路测 2 + 根因 16 |

**已删除（勿复活，全部有根因判决）**：~~CruiseActivated 许可门~~（当许可 = 死锁，0000000d 两段零控车）、静默守卫 0.16s（→0.42s 背停替代）、对向跟随（R2 改判：对抗合法，犹豫才是罪）、角度/速率门 + 大转角 10s 锁定（R3 错标帧规则 + 100s 静默来源）、角度 holdback（6.30 静默）、软启动（vendor ramp 即 DELTA 限速）、engage 需求 ≥6 门（并入手轻门语义）、让位曲线、follow 翻转。

**固件侧（fw_base 0.9.x）**：allowance 120 / rate 18/18 / 0xf3 心跳 `controls_allowed=engaged`（mismatch 死锁修复，`0cfd780854`）；bit1 电源线是控制器层门，固件不改。

## 十一、未定论 / 调参旋钮

1. **SW→锁存机制（部分定论，9-29）**：2 个清除案例（0000000a 前两次 err=2）支持"SW 持续 ~0.5s 才锁"；第三击表明**违规状态未真解除则清除无效**。残余问题：清除判定的确切边界（总线上残留 demand 算不算违规）——err=2 时立即零请求是否够，待实车复验。
2. ~~对向跟随参数~~ **已 moot**：逻辑整体删除（R2 改判）。
3. SteerErrorCode 码表：2=可恢复预警、4=升级锁存（与 TF 同帧）；1/3/5-7 未观测。
4. ~~角度/速率门放宽~~ **已 moot**：整体删除。
5. **LKAS_Prepared 语义（9-29 升级为独立疑点）**：三个互斥观测——①调试记录 二.3 时代"无 prepare 请求则永不 armed"（预备握手必要性成立）；②厂商 00000037 全程 Prepared=0 仍出力（应答非出力前提）；③我方 0000000a EPS 频繁置 1 且 c=0 拒绝时照样置 1（置位门槛极低）。综合：Prepared 只是"收到过 ReqPrepare"的应答位，与执行链路（§规则 10 的 c 位）完全解耦；退场后 ~0.2s 清除为单例观察。**勿把 Prepared 当任何门控输入**（现行代码未用，保持）。
6. armed-零容忍度实测区间 0.48~0.72s，EPS 内部阈值未知。
7. **EPS 激活条件未完全逆向**：手轻是主导可观测量（0000000a 76.3s 手轻瞬间激活；手重时段全程不激活），但有例外（108.8-109.5s 手轻仍不激活）。**未加"拒绝守卫"**：强行退场会误杀合法等待相位（厂商 2.86s 前例）；先观察实车。
8. MainTorque 是否等于电机实际出力（执行判据用途已定论，物理量纲存疑——mt 曾超 demand：-285 vs -240）。
9. 厂商 demand 包络 max=193 与 ELF HIGH error=200 的关系（我方控制器包络取 200）；STEER_MAX=300 保持为固件绝对限。
10. **0000000d seg1 回放 sign flips 46（阈值 40）**：录像期我方零输出，录到的 demand 是规划器追人手晃动的产物，疑似回放伪影；若实车出现 ~0.6Hz 翻向，下一杠杆 = demand 平滑/死区（厂商 ELF `byd_adjust_steer_torque` 嫌疑）。实车复验必看项。

## 十二、工具与证据索引

| 内容 | 位置 |
|---|---|
| 第二次驾驶 6 锁存逐帧重建 | `docs_site/hour_logs_2/analyze_latch.py` → `latch_report.txt`（本地，gitignore） |
| 修复回归（锁存时间线灌回控制器） | `docs_site/hour_logs_2/replay_latches.py`（**14/14 干净**，含 c 门/手轻门场景） |
| 真实 route 回放（灌新控制器查 armed-silence/flips） | `docs_site/hour_logs_2/replay_routes_new.py`（c9f1698c82 / 909633d7ed / 0000000a 全 CLEAN） |
| 0000000a 帧级分析脚本 | `/tmp/byd_0a/analyze*.py`（临时，重拉：rlog 在设备 realdata） |
| 厂商 c 位-出力相关性验证 | `/tmp/byd_0a/vendor_check.py` + `docs_site/op_byd_logs/`（route 7--12e：c=0 零出力） |
| 厂商系统工作日志（字节模板来源） | `docs_site/op_byd_logs/`（含 vendor_diff.py） |
| **官方维修手册**（总线架构/巡航进入条件/MRR-MPC 校验 DTC） | `docs_site/pdf/2021年款比亚迪宋PLUS DMi-01-维修手册-*.txt`（ACC + MPC 分册；SteerErrorCode 码表需 EPS 分册，暂缺） |
| 调试全史（根因 1-14） | `byd-song-plus-lkas-debug.md`（15/16 见 `byd-current-status.md`） |
| 当前状态快照 | `byd-current-status.md` |
| 部署/测试规程 | `byd-song-plus-ops.md`（测试规程 = 调试记录 6.22） |

根因速查：1 断流 / 2 SETME / 3 UDS 查询帧 / 4 controlsMismatch 容忍 / 5 按键 0x3B0 / 6 DMS / 7 安全带 / 8 AccState 双义 / 9 扭矩量程 300 / 10 LKAS_State / 11 Config=3 会话 / 12 对抗性助力 / 13 armed-零静默 / 14 守卫输掉比赛 + 对向残余 / 15 冷武装（角度门重开撞持盘，`d9eec5070e`）/ **16 CruiseActivated 相位标志误当许可（v1 c 门死锁 0000000d 两段零控车；正解 = 请求包络 200 + 手轻武装 + 照常流会话等激活，`51b1a25a`）**。
