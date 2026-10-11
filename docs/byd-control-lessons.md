# BYD 宋 PLUS 横向控制 · 三方实现对照与可吸收经验

> 汇总 2026-09-29/30 对三方实现的代码考古与官方维修手册消化的全部可吸收项。
> 配套阅读：EPS 协议层规则见 `byd-eps-reference.md`，当前状态见 `byd-current-status.md`。
> 本文是"候选经验库"：**每项标注吸收状态与触发条件**，动控制行为前先看 §5 决策树与纪律。
> 最后更新：2026-10-04（新增 §3.4 MM-X sunnypilot-pc 对照）。

## 一、资料源清单

| 源 | 路径 | 状态 |
|---|---|---|
| 厂商 op_byd 解密代码（PyArmor 运行时产物，100% 覆盖） | `/Users/wujiafu/Documents/op/cp_byd/docs/pyarmor_decrypted/opendbc_repo/opendbc/car/byd/`（`*.functions.py` 按代码对象可读；`*.values.txt` 常量全可读） | 已消化 §2 |
| yysnet 社区 port | `/Users/wujiafu/Documents/op/opendbc/opendbc/car/byd/` | 已消化 §3 |
| mouxangithub port | `/Users/wujiafu/Documents/op/mouxangithub/opendbc/opendbc/car/byd/` | 已消化 §3.3 |
| 官方维修手册（2021 款宋 PLUS DMi）ACC + MPC 分册 | `docs_site/pdf/2021年款比亚迪宋PLUS DMi-01-维修手册-*.txt`（OCR，正文可读） | 已消化 §4 |
| 官方维修手册 **EPS 分册** | **未获得** —— SteerErrorCode 码表仍缺，搞到即补 | 缺 |
| MM-X sunnypilot-pc（PC 运行 fork，BYD 在 submodule `MM-X/opendbc-sny`） | `/Users/wujiafu/Documents/op/sunnypilot-pc/opendbc_repo/opendbc/car/byd/`（yysnet 二手拷贝，2025-04~08 冻结） | 已消化 §3.4 |
| 🐬 mypilot 社区开源包（BYD 全系+吉利：厂商固件同源明文还原 + 调校包 + DBC 库） | `/Users/wujiafu/Documents/op/openpilot-mypilot-community-dbcall/` | 已消化 §3.5（2026-10-11；雷达 tangdm 布局已并入 lane-change §8.5.6） |
| 厂商固件 ELF（带符号） | `docs_site/op_byd`（panda.elf：BYD_STEERING_LIMITS=[300,18,18,…,46]、LOW=[300,9,9,46]、HIGH error=200、ALT=[150,50,50]） | 已入 EPS 参考 §9 |

## 二、厂商解密代码的发现（最权威参考实现）

### 2.1 扭矩限幅链（code objects 004/005/006，`carcontroller.py.functions.py`）

- `004_get_byd_torque_limits`：LOW/HIGH 两组限值按 flag 选择（对应 ELF 的 LOW=[300,9,9,46] vs [300,18,18,…]），含速度条件覆写（ELF 的 STEERING_TORQUE_LIMIT_SPEED=20 膝点）。
- `006_apply_byd_steer_torque_limits` 核心两段：
  1. **速度插值 allowance**：`np.interp(v, [50,120,200], [1,0.5,0])` 乘入限幅窗口——**对向/出力窗口随速度收窄**（50→全量，120→半量，200→零；自变量单位待确认，km/h 合理）。
  2. **手离盘计时清零**（`BYDHandsoffFix`）：连续时长比 `dt/LIMIT(~24s)` 分档 `0.9/0.8/0.7/0.5` × 速度门（120/80/50/THRESH）→ 主动清零——**抢在 EPS 抱怨（SW→锁存）之前自己优雅退场**。
- `005_..._basic`：标准 allow-window + rate ramp + 二次 clip。

### 2.2 EMA 低通滤波（code objects 039/040）

独立滤波类：`alpha = dt/(dt+tau)`，`val = (1-alpha)*prev + alpha*new`——厂商对某信号做 tau 型低通。**滤波对象未逆向**（嫌疑：横向 demand 或驾驶员扭矩）。若实车出现 dither，这是"厂商味"的平滑方案。

### 2.3 回显跟随（模块级 + update 各一处）

```python
if not (flags & X) or cond: demand = 0
elif (...) and abs(prev - cam_msg['LKAS_Output']) <= K * 2:
    demand = cam_msg['LKAS_Output']   # 直接采用原车相机自己的请求
```
"隐身"技巧：OP 与原车相机请求接近（≤K*2，K 未知）时发送相机原值，bus0 流与原车字节一致。**有意图冲突风险，记录不采纳**；但解释了厂商日志里部分帧与相机输出完全一致的现象。

### 2.4 flag 体系（`carcontroller.py.values.txt` 全可读）

- `BYD_TORQUE_WITH_DRIVER(1024)` / `BYD_LOW_TORQUE(8)` / `BYD_TORQUE_WITH_FACTOR(4/512)` / `BYD_FORCE_TORQUE_FIX(256)` / `BYD_TORQUE_REMATCHING_1/2/4`——不同车型切换"对抗/不对抗/限值重映射"模式。
- `ALT_HANDSOFF_CARS = (SEAL_06_DMI, SEAL_07_DMI_22, SEAL_07_DMI_24)`——手离盘计时清零仅海豹系启用；**宋不在列表**（我们 hands-off timer 降 P2 的第二个依据）。
- 角度路径 `MPC_LKAS_CMD_ANGLE`（带 JerkUpper/LowerLimit、SET_ME_3=3）仅 `BYD_ANGLE_CONTROL` 车型——**宋不涉及**。

### 2.5 engage cooldown / boot latch hold（carstate 层，2026-09-30 从 7--12e 实录解码）

厂商 carstate 自带 engage 状态机（disasm 名字表：`is_cruise_latch`/`prev_is_cruise_latch`/
`btn_acc_set_reset`/`btn_acc_cancel`/`engage_cooldown`/`lkas_btn`），**boot-mid-cruise 时把
cruiseState.enabled 压 ~15s 再放行**——边沿迟到但真，通用事件层零改动（他们的 car_specific.py
零 byd 改动）。实录（7--12e_0，雷达 ACA 全程 active）：3.18s latch=0/avail=0 → 16.74s
avail=1 → 17.54s latch=1（真边沿）→ 17.55s OP enabled 同帧。**已照搬**：我们
`byd/carstate.py` `BOOT_LATCH_HOLD_TIME=15`。反例教训：同一症状在 car_specific 层合成
pcmEnable（已撤）也能修，但层不对——**先问"厂商在哪层管这件事"再动手**。

## 三、社区 port 的发现

### 3.1 yysnet（`/Users/wujiafu/Documents/op/opendbc/`）——最有价值的一项

**转角速率 governor**（`carcontroller.py` update，`USE_STEERING_SPEED_LIMITER`）：
```python
rate_limit = np.interp(CS.out.aEgo, [8.3, 27.8], [132, 64])  # m/s→km/h: 30→100, 允许转率 132→64 deg/s
delta_rate = CS.steeringRateDegAbs - rate_limit
# 实际转率超限 → steerRateLim 夹紧 demand（0.005/帧收），低于限 → 0.25/s 缓慢恢复
new_steer_pu = np.clip(steer_desire, -self.steerRateLim, self.steerRateLim)
```
- 作用：demand 被车的实际转率"油门"约束——车转不过来时收油，防 dither/对抗失控，自恢复。
- **注意：作者默认 `USE_STEERING_SPEED_LIMITER = False`**——作者自己没敢默认开。适合做成 param 实车 A/B。
- **这就是根因 16 v2 遗留观察项（sign flips）的第一杠杆**。

其余参数（对照用，勿照抄）：`DELTA_UP=7/DOWN=10`（比厂商 p99=16 温和）、`SOFTSTART=6/帧（1s 到满）`、`DRIVER_ALLOWANCE=68`、**等 Prepared ack 才武装**（旧模型，已被厂商帧证据推翻——勿吸收）。

### 3.2 消化结论：不吸收项

| 项 | 来源 | 不吸收理由 |
|---|---|---|
| 等 Prepared ack 再武装 | yysnet | 厂商 3 帧 burst 不等 ack（route 00000037 实证）；Prepared 与执行链解耦（EPS 参考 §3.3/§11.5） |
| counter 首帧对齐车值 | yysnet | 厂商自由递增可行（route 00000037） |
| 软启动层 | yysnet | 厂商 ramp 即 DELTA 限速；软启动已随旧架构删除（勿复活） |
| Atto3 角度模式/jerk planner | mouxangithub | 宋走扭矩路径，不涉及 |

### 3.4 MM-X sunnypilot-pc / opendbc-sny（`/Users/wujiafu/Documents/op/sunnypilot-pc/`，2026-10-04 对照）

主体是"openpilot 在 PC 上跑"（metadrive 仿真 + 本地 segments 工具 + uiview）；BYD 部分是
`feat(car): Sync byd brand from yysnet`（2025-04）的**二手拷贝**，控制行为与 §3.1 同源，**代码无需 merge**。
但它是第一个可查证的"**同款车已有人控车**"案例，以下按事实/观察项记录：

**事实核**：
- `Adapter byd BYD_SONG_PLUS_DMI_21`（2025-04-02，子模块 f2c673cc）——**21 款宋 PLUS 有人跑通横向控车**。范围 = 扭矩路径横向 + **原厂 ACC 纵向**（`interface.py: EXP_LONG_CAR` 只含 Han/Tang，Song 的 `openpilotLongitudinalControl=False`）；OP-long 他们没做，我们 Phase2 在前。
- 他们的 panda `steer_req = LKAS_Active && EPS CruiseActivated`（`safety/modes/byd.h` 0x316 tx check）——**c 门在那台车上可用**。与根因 16（22 款 c 是会话相位标志、合法等待 2.9~7.7s、c 门版本 7218 帧全零死锁）不矛盾，反而提示 **EPS 的 CruiseActivated 行为可能分年款/分固件**。待回问情报：那台车具体年款、上电时序。
- 他们的坑（勿照抄）：`byd_init` 里 `use_song → byd_platform = TANG_DMI`，`SONG_STEERING_LIMITS` 是永不选中的死代码且注释 `//values to be check`——那台车实际吃的是唐的限速没校验过。

**吸收观察项（只登记，不动行为）**：
1. **多年款 fingerprints**（`car/byd/fingerprints.py`）：Song Plus DMI 21/23、Song Pro 22、Qin Plus 23、Yuan Plus(Atto3) 22 全量——将来扩年款/社区支持可直接摘，21/23 也可对照我们 22 款验证总线差异。
2. **fake 0x318 回发 MPC**（`carcontroller.py create_fake_318`）：他们伪造 EPS 状态帧糊住摄像头来消 fault，与我们"echo 摄像头原字段"是两条路线。我们方案已实证，此项仅作备案——若再遇 `check MFC` 类 fault 可对比。
3. **距离域纵向 jerk 表**（`values.py K_jerk_xp=[4,10,20,40,80]m` + lower/upper 插值，`acc_cmd` 里按 `mrr_leaddist`  interp）：按跟车距离限加加速度——纵向"速率 governor"拍板时的一个输入（我们现为时间域 comfort band）。
4. **PC 回放环境**：M 芯片 Mac 可直跑 + 本地 segments——与控车无关，但可能替代部分设备端 replay 验证。

### 3.5 mypilot 社区开源包（🐬 `/Users/wujiafu/Documents/op/openpilot-mypilot-community-dbcall`，2026-10-11 对照）

BYD 全系+吉利的社区资产合集（说明文档自称含"厂商商业固件 PyArmor 明文还原"=与我们 §2 cp_byd **同源重复**，附 `server_connect.py` 授权校验佐证厂商 license 逻辑）。**唯一硬增量已吸收**：`byd_tangdm_radar.dbc` 0x351-354 槽位布局 → 雷达 A+B 落码（专档 §8.5.6，commit 5c1066e254）。其余登记如下，均不动控制行为：

**吸收观察项（只登记）**：
1. **距离域纵向平滑表**（`byd2/tuning.py K_ACCEL_BP=[40,50,60,70,80]m × K_ACCEL_POS/NEG_1..4BAR` 加减速百分比）：与 §3.4-MMX 项 3（K_jerk_xp）同一思路的**百分比衰减版本**——OP-long 速率 governor 拍板时两版对照输入。
2. **HANDSOFF 周期微释放重接管**（`tuning.py HANDSOFF_ANGLE=[4,11,18]° HANDSOFF_PERIOD=[5,10,15]s`，治 D9/唐 离手久 EPS 退出）：与根因 18/守卫 C **同题不同解**（他们主动微释放再抢回，我们撤 Active 压重试）——仅作备选策略 #2 备案。
3. **社区版宋 22 NN 模型**（`lat_models/BYD_SONG_PLUS_DMI_22.json`，2026-01 训练、test_loss 0.041）：**不入库**——本仓库 sunnypilot NNFC 目录已自带宋 22 模型（2025-01 训练、loss 0.031，更优且原生接线）；社区副本归档 `docs_site/byd_mypilot_assets/`（若放入 NNFC 扫描目录会被 similarity 路由抢选，禁止）。
4. **功能面名单交叉印证**（高阶版 values.py）：`EXP_LONG_CAR`/`TORQUE_LAT_CAR` 含宋 21/22/23、`RADAR_CAR`=秦Pro/唐DM、`PT_RADAR_CAR`=汉/唐DMP/腾势/元——社区验证过"宋可跑纵向(原厂ACC)+力矩横向"，与我们路线一致；宋不在其任何雷达名单（§8.5.6 互证）。

**不吸收项**：
| 项 | 理由 |
|---|---|
| `BYD_Files/opendbc/safety/safety_byd.h` | 旧于我们移植版：无 echo-relay、无 LONGITUDINAL flag、checksum/counter 全 ignore；我们的含 safetyTxBlocked 看门狗 |
| `byd源码/carcontroller` TTC 预测刹车/防画龙/0帧起步、`carcontroller(2).py` 增强滤波 | 社区启发式，与"厂商行为为基准"原则冲突；厂商代码已 100% 还原，行为基准取厂商 |
| `BYD_Files/params.toml` LM_PARAMS | 全车系同一组 [2.2,2.5,0.145]，占位假数据 |
| `Bydsong(1).dbc`/`generated(1).dbc`/`byd大全.dbc` | `MSG_xxx`+raw 字节无命名清单；宋报文覆盖以 415 段 rlog 实测为证更硬，仅作将来挖掘底账 |
| `DISABLE_EPS_WARNING/TEMPORARY/PERMANENT` 三开关 | 屏蔽 EPS 故障与我们守卫方向相反（根因 18 教训=EPS 静默弃权必须硬停） |

**风险旗**：社区 RelSpeed 单位混乱（dbc 标 km/s、代码原样当 m/s 传）；我们按 KPH_TO_MS 落码——10-11 夜厂商 bytecode 证实 **Tang 分支 `vRel=REL_SPEED×KPH_TO_MS`**（专档 §8.5.6），换算方向与厂商一致，仅剩 dbc 的 factor/offset 数值待首帧真 IsValid=1 数据核对。

**待办（NNFF 互验）**：
- [ ] sunnypilot 原生 `NeuralNetworkLateralControl` 参数开/关，对同一批段/路测跑 **siglin（现状=厂商表）vs NNFC（本地宋22模型）** 的 demand 扭矩 A/B（离线 replay 先行，对照 `NON_LINEAR_TORQUE_PARAMS`；差异大再议路测）。归档社区模型仅备三方对比用。

## 四、官方维修手册（ACC + MPC 分册）可吸收项

### 4.1 架构确认（已写入 EPS 参考 §一/§二，此处存档出处）

- **官方端子表 P13**：`P13-5/11 = 公有 CAN（底盘网）`、`P13-6/12 = 私有 CAN（接雷达）` → 我们的 bus0 = 底盘网、bus2 = 私有 CAN（雷达域）。
- **`U023587` "MRR 发送端口错误（0x32D,0x32E,0x32F）"**：ACC_HUD_ADAS/ACC_CMD/ACC_AEB 的发送方是**雷达 MRR**，不是相机——bus2 透传 = 透传雷达帧；ACC_CMD counter 自由递增由此解释。
- **`C2F0882/83/86/87`**（私有 CAN MPC 滚动计数器/校验值/信号无效/失去通讯）：**MRR 逐帧校验 MPC 发的 0x316**——根因 2"echo 字段不可缺省"的官方机理，CAN 上互为监工（MPC 检查 EPS：U1017xx 系列）。

### 4.2 巡航/功能边界（官方口径）

- 巡航进入硬条件：主驾**安全带、车门、前舱盖开关** + OK 档电（根因 7 官方出处；仪表指示灯响应是检查项）。
- 官方只承诺 **LDW ≥60km/h**；LKA 速度范围未公开——我们 2-23km/h 能控是 op_byd 实证能力，勿当官方规格。
- MPC 融合功能：AEB/PCW、TSR（交通标志）、智能远光、LDW——与雷达深度耦合。

### 4.3 诊断资源

- MPC 分册 DTC 全表 = 通讯（BCM/ESC/SAS/EPS/ECM/组合开关/MRR）+ 标定（C1C2C46/47 在线/初始校准越界、C1C2C94 超时、C1C3254 无校准数据、C1C2A00/344 匹配参数）+ 电压/温度/遮挡（C1C2D97 摄像头遮挡、B100757 安装照未拆、C100319 挡风玻璃）——**无 LKA/EPS 故障码**。
- ACC 分册 MRR DTC 全表：标定类（C2F9A78 未校准、C130478/C130578 水平/垂直偏差、C2F9402 位置故障）、**产线模式 C2F0206、模型配置 C2F0381**（动配置/刷写时排查用）、ESP 侧供给（C130204 参考速度、C2F4201 偏航率、C2F4107 轮胎尺寸）。
- **SteerErrorCode 码表在 EPS 分册（未获得）**——弄到手册即补 EPS 参考 §五。
- 硬件：MPC 在前挡风支架 + 车道偏离保护盖（卡扣），UF39 保险，P13 插件——动硬件才用。

## 五、吸收决策树（挂接根因 16 v2 实车验证）

**v2 复验已完成（2026-09-30，route 0000000e/f，详见 `byd-v2-validation.md`）：整体能控、
零锁存、包络/手轻门均工作，但"什么都不加"不成立——下表第 2/3 行已实车触发。**

| 实车 v2 表现 | 动作 | 选项（按优先级） |
|---|---|---|
| 收敛、无 dither、手感正常 | ~~什么都不加~~（未成立） | — |
| 出现 ~0.6Hz 翻向 dither（sign flips 观察项坐实） | 上平滑/限幅杠杆 | ① yysnet 转角速率 governor（param 默认关，A/B）；② 厂商 EMA（tau 型低通） |
| 等待相位（c=0）demand rail 到包络、EPS 激活瞬间全额释放 | 抑制 windup | c=0 时冻结/衰减 demand（厂商环"几乎不积分全靠前馈"不 windup；**先做 NNLC 关/开 A/B**——NN 未标定是第一嫌疑） |
| ACC armed 时雷达 ACA 打摆（Notify=8，16/16 全在 armed 时段） | 修上游观察 | 无直接杠杆——先修振荡再复测；0x316 退场空洞是其下游（退场斜坡 vs controls_allowed 冲突），上游平息后仍现再动 |
| 高速段对抗吃力/过冲 | 收紧高速窗口 | 厂商速度插值 allowance（`np.interp(v,[50,120,200],[1,0.5,0])`，先确认自变量单位） |
| 长时间脱手引发 EPS 抱怨 | 自己先退场 | 厂商 hands-off 计时器（注意：宋不在厂商 ALT_HANDSOFF_CARS 列表 + 本车 hands 位恒 0，双重降级 P2） |

## 六、我方独有的反哺资产（三方代码里读不出）

- `CruiseActivated`（0x318 bit1）= EPS 会话**相位标志**（"正在执行"），接受会话后才置 1——非许可、非原车 ACC 状态；c 门死锁教训（0000000d 两段 7218 帧全零）。
- `SteerErrorCode` 部分码表：2 = ~0.5s 可恢复预警（SW 同步置 1）、4 = 升级锁存（与 TorqueFailed 同帧）。
- 等待相位（act=1 + c=0）合法且可长达 2.86s（厂商 415 帧），安全性靠请求包络（STEER_MAX_REQUEST=200）而非许可位。
- 武装手轻门 `STEER_ARM_DRV_TORQUE=50`（根因 15/16 的实车真相）。
- err=2 时总线上残留大幅值 demand 是升级催化剂——预防 = 不制造 rail，而非加快退场（固件 rate 18/帧 下坡是硬下限）。
- 本车 EPS `ReportHandsNotOnSteeringWheel` 恒 0（3 条 route 验证）= 该位未启用。
