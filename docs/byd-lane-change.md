# BYD 打灯变道移植方案

> 状态：**方案定稿 + T2/T3 已落码待路测**（2026-10-04）。T1 符号标定已离线完成（§四.1）。
> 实现与本文设计稿的差异：fork 内**不存在 NO_ACTUATION→latActive 传播链**（grep 零命中），
> 故案 A 以「selfdrived 第二实例同谓词出事件 + controlsd_ext 实例出让位」落地；
> L2 的 `=0` 语义落码时映射为 inf（等效厂商禁用且有中途 abort，§三 描述仍成立）。
> 分支：main-lon-tici。本文所有行号均已实测复核。

---

## §〇 结论速览

1. **厂商的「打灯变道」核心就是原生 openpilot DesireHelper 状态机**——厂商解密树 `op_byd/selfdrive/controls/lib/` 下**没有 desire_helper.py**（未修改、直接继承），其 `legacy_lateral_planner.py:9` 原样 import 上游 DesireHelper。拨转向灯 → 车速达标 → 司机同向轻转方向盘 → MPC 横向轨迹带入邻道，盲区有车则拦截。这套机制**本仓库已完整具备且架构更新**（state machine 在 modeld 内跑，desire 进 modelV2.meta）。
2. 厂商在原生之上只有**两条自定义**：
   - **①可配速度阈值** `dp_lat_lane_change_assist_speed`（MPH，替换原生固定 20 mph；=0 关闭辅助变道）；
   - **②assist-less 让位模式**（阈值=0 时生效）：打灯 + 司机同向带舵 → `CC.latActive=False`，openpilot 横向**完全退出**让司机手变道，灭灯解锁，期间仅弹 laneChange 提示。
3. BYD 侧**输入信号已全部就绪**：`opendbc_repo/opendbc/car/byd/carstate.py` 已解析 STALKS 左右转向灯（:229-230）与 BSD_RADAR 盲区（:235-236），`steeringTorque=ACC_EPS_STATE.SteerDriverTorque`（:151）。理论上现在就能用——**符号问题已离线排除**（§四.1：实测左=正，原生触发条件直接成立），唯一留白是旧日志无拨灯记录，留路测验证（§六 T4）。
4. 移植 = L1 验证（零代码）+ L2 阈值参数 + L3 让位模式，三条独立可回滚。

---

## §一 厂商实现还原（实证）

路径前缀 `/Users/wujiafu/Documents/work/op/op_byd/`（PyArmor 解密产物完整代码树）。

### 1.1 状态机 = 原生 DesireHelper（未改）

`selfdrive/controls/lib/legacy_lateral_planner.py:77-83`：

```python
# Lane change logic
lane_change_prob = self.LP.l_lane_change_prob + self.LP.r_lane_change_prob
self.DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob, self._dp_lat_lane_change_assist_speed)

# Turn off lanes during lane change
if self.DH.desire == log.LateralPlan.Desire.laneChangeRight or self.DH.desire == log.LateralPlan.Desire.laneChangeLeft:
    self.LP.lll_prob *= self.DH.lane_change_ll_prob
    self.LP.rll_prob *= self.DH.lane_change_ll_prob
```

（厂商给 DH.update 多传了第 4 参=速度阈值；原生是 3 参。状态机本体逻辑与上游一致：off→preLaneChange→starting→finishing，10 s 超时回 off。）

### 1.2 自定义①：可配速度阈值

`legacy_lateral_planner.py:38`：

```python
self._dp_lat_lane_change_assist_speed = int(self.params.get("dp_lat_lane_change_assist_speed", encoding="utf-8")) * CV.MPH_TO_MS
```

`controlsd.py:121-122`：

```python
self._dp_lat_lane_change_assist_disabled = int(self.params.get("dp_lat_lane_change_assist_speed", encoding="utf-8")) == 0
self._dp_lat_lane_change_assist_disabled_active = False
```

语义：参数=MPH 阈值（替代原生 20 mph）；**=0 时辅助变道整体禁用**并进入 §1.3 模式。设备上实际下发值未知（厂商 params 默认定义不在解密树内；本仓库 `docs_site/op_byd_data/values.json` 亦无此键）——实现时按「默认 20（=原生行为），0=让位模式」设计，不猜厂商在线值。

### 1.3 自定义②：assist-less 让位模式

`controlsd.py:691-703`（厂商注释自称 "rick - assist-less lane change"）：

```python
# rick - assist-less lane change
if self._dp_lat_lane_change_assist_disabled:
    # de-activate
    if not CS.leftBlinker and not CS.rightBlinker:
        self._dp_lat_lane_change_assist_disabled_active = False

    # activate
    if not self._dp_lat_lane_change_assist_disabled_active and CS.steeringPressed and \
        ((CS.steeringTorque > 0 and CS.leftBlinker) or
         (CS.steeringTorque < 0 and CS.rightBlinker)):
        self._dp_lat_lane_change_assist_disabled_active = True

    if self._dp_lat_lane_change_assist_disabled_active:
        self.events.add(EventName.laneChange)
        CC.latActive = False
```

要点：**锁存**（灭双灯才解锁，与 §1.5 的灯镜像独立）；激活瞬间 openpilot 横向退出，司机全程手动变道；提示复用 `laneChange` 事件。动机推断：BYD EPS 在司机对抗/低速场景有弃权-锁存史（根因16/17/18，见 byd-lateral-lifecycle.md），与其带着 LKAS 请求打架，不如打灯+带舵时干脆让位——**与我们路测实证的方向一致，值得完整移植**。

### 1.4 盲区拦截与事件（厂商=原生行为）

`controlsd.py:376-388`：preLaneChange 时 `CS.leftBlindspot/rightBlindspot` 命中方向 → `laneChangeBlocked`，否则 `preLaneChangeLeft/Right`；starting/finishing → `laneChange`。

### 1.5 变道期间灯镜像（厂商=原生行为）

`controlsd.py:730-733`：`laneChangeState != off` 时把 CC.leftBlinker/rightBlinker 置为变道方向（HUD 显示）。

---

## §二 本仓库现状：架构映射表

本 fork 为拆分架构（selfdrived/modeld/lateral MPC 分进程），厂商两参对照如下——**能力一一对应，无缺失**：

| 厂商（老架构） | 本仓库（新架构） | 差异 |
|---|---|---|
| DesireHelper 状态机（legacy_lateral_planner 调用） | `selfdrive/controls/lib/desire_helper.py:41-139`，由 **modeld** 调用（`selfdrive/modeld/modeld.py:348` `DH.update(sm['carState'], sm['carControl'].latActive, lane_change_prob)`，publish→`modelV2.meta.laneChangeState/Direction` :349-353） | 触发多了 sunnypilot ALC 免舵路径（§二.1） |
| 速度阈值 20 mph 硬编码（厂商改参数） | `desire_helper.py:10` `LANE_CHANGE_SPEED_MIN = 20 * CV.MPH_TO_MS` 硬编码 | L2 移植点 |
| controlsd 事件（§1.4） | `selfdrive/selfdrived/selfdrived.py:314-329` 逐字同款 | 已有 |
| events 定义 | `selfdrive/selfdrived/events.py:399-428`：preLaneChangeLeft/Right、laneChangeBlocked、laneChange（全 ET.WARNING） | 已有 |
| CC 灯镜像（§1.5） | `selfdrive/controls/controlsd.py:124-128` 同款 | 已有 |
| assist-less hack（§1.3） | **无对应 → L3 移植点**。理想落点 `sunnypilot/selfdrive/controls/controlsd_ext.py:36-45 get_lat_active()`：该钩子已为 BlinkerPauseLateral（`sunnypilot/selfdrive/controls/lib/blinker_pause_lateral.py`，拨灯+低速→暂停横向）而建，与厂商 hack 同语义域，加锁存分支即为仓库惯例写法；事件在 selfdrived 补 `laneChange`（§五.2 给了两案） | 缺 |

### 二.1 sunnypilot 对状态机的既有扩展（移植时必须共存，不改其行为）

- **ALC 自动确认变道**（`sunnypilot/selfdrive/controls/lib/auto_lane_change.py`）：`AutoLaneChangeTimer` ≥ NUDGELESS 且非刹车/非连续变道时，**免司机转舵**直接进 starting。默认 NUDGE=0（原生行为：需轻舵）。desire_helper.py:93 `(torque_applied or self.alc.auto_lane_change_allowed)`。
- **LaneTurnController**（导航路口方向盘向，desire_helper.py:62-64,124-127）。
- **BlinkerPauseLateral**（`BlinkerPauseLateralControl` 参数，拨灯且低于 `BlinkerMinLateralControlSpeed` → latActive 暂停）——**与 L3 的语义区别**：它=拨灯即停（不看舵向），L3=拨灯+同向带舵才让位且锁存到灭灯。并存矩阵见 §五.3。

### 二.2 BYD 车型侧

- 信号：`carstate.py:229-230`（STALKS 左右灯）、:235-236（BSD_RADAR 盲区）、:151 `steeringTorque`、:155-156 `steeringPressed`（|torque|>STEER_THRESHOLD，5 帧防抖）。
- 无车型能力位需要开：本 fork 的 LC 链路对全车型通用（原生 openpilot 亦然，无 `laneChangingEnabled` 之类 CarParams），BYD 不需要动 interface。
- 横向授权链：`controlsd.py:115-122` `CC.latActive = get_lat_active(...) and not steerFault* and (not standstill or steerAtStandstill)`——BYD torque 路径的根因16/17/18 守卫全在 carcontroller 内，与本链路正交。

---

## §三 移植设计（L2：可配速度阈值）

1. **参数**：`common/params_keys.h` 新增（模式照抄 :136 `AutoLaneChangeTimer`）：
   `{"LaneChangeAssistSpeed", {PERSISTENT | BACKUP, INT, "20"}},`
   单位 MPH，语义照厂商：`>0` 作为状态机速度阈值；`=0` 禁用辅助变道并启用 L3。默认 "20"=现状零行为变化。
2. **读取点**：`selfdrive/controls/lib/desire_helper.py` 的 `DesireHelper` 增加 `update_params()`（照 ALC 的 `auto_lane_change.py:63-64` 模式：初始化读 + 每 N 帧刷新），`update()` 内 `below_lane_change_speed` 的比较值由 `LANE_CHANGE_SPEED_MIN` 改为 `max(self._speed_threshold_mph * speed_factor, 0)`；**`=0` 时 `below_lane_change_speed` 恒真 → preLaneChange 永不进 → 状态机自然关闭**（厂商同款效果，无需显式 if）。
3. **UI**：厂商 dp_ 参数无设备端 UI。本仓库参数系统带 UI 注册文件时（sunnypilot 设置页 json）暂不注册——USB `params put` 可改即达厂商形态；若用户要 UI 再加，单列 todo。

## §四 移植设计（L3：assist-less 让位模式）

### 四.1 符号标定：**已完成（2026-10-04 离线，docs_site 存档 rlog），结论=无需修正**

`desire_helper.py:81-83` 与厂商 L3 激活条件同式：**左=steeringTorque>0**。经五层实证链确认 BYD `SteerDriverTorque` 原生即左=正：

1. `bydcan.py:74` `LKAS_Output = apply_torque` 原值上总线（CarController 内无取反）+ 实车跟线正常 → 总线请求系=左正；
2. 根因18 回显守卫 `abs(apply_torque_last − steeringTorqueEps)` 路测通过（carcontroller.py:148）→ `MainTorque` 回显=左正；
3. 日志实测（r00000011_1）：`MainTorque`×`SteeringAngleDeg`同号 2328/反 241 → 角传感器=左正；
4. 日志实测：`SteerDriverTorque`×角 同号——1f--1 **1279/0**、r00000011_1 **2300/263**（反号样本=回盘瞬态，物理正常）→ **司机力矩=左正，无需取反**；
5. 旁证：厂商 route 7--12e 分析「-153 against a +166 driver yank」（请求与司机力矩反号=对抗）与本系一致；`carcontroller.py:102-104` Guard A `demand*drv<0` 的「对向」语义成立（既有代码无符号 bug）。

勘误记录：初测「符号反了」是假象——`controlsd.py:89` 的 `self.curvature = -VM.calc_curvature(...)` 使日志 `controlsState.curvature` 字段与转向角**全局反号**（实测 0 同/5096 反，纯由该负号产生），该 fork 曲率约定与 LC 逻辑无关（desire_helper 不读曲率，只读 steeringTorque），移植勿被此字段误导。
转向灯字段在 1f--1 / R10_000 / r00000011_1 三段存档中恒 0（当时无拨灯动作，不能证伪解析）→ 保留路测清单（§六 T4）。

### 四.2 落码位置

全部在 **`sunnypilot/selfdrive/controls/controlsd_ext.py:36-45 get_lat_active()`** + 一个新锁存类（仿 `blinker_pause_lateral.py` 结构、同目录 `assist_less_lane_change.py`）：

```
class AssistLessLaneChange:
  update(CS) -> bool:
    if speed_threshold != 0: return False        # 仅 L2=0 时启用（参数复用）
    if not left and not right: latch = False     # 灭灯解锁
    if not latch and steeringPressed and ((torque>0 and left) or (torque<0 and right)):
      latch = True
    return latch                                  # True → get_lat_active 返回 False
```

`get_lat_active` 中排在 `blinker_pause_lateral` 之后、MADS 分支之前（两者优先级并列，命中即 `latActive=False`）。

### 四.3 事件（提示司机「正在让位」）

- **案 A（首选，实现时验证）**：在 selfdrived 加事件 `EventName.laneChange` 的 assist-less 分支——但让位判定在 controlsd 进程，跨进程传播依赖 `carControl.latActive` 回读：selfdrived 订阅 carControl，可判「`modelV2.meta.laneChangeState==off` 且 `not CC.latActive` 且 CS 拨灯」还原场景——**状态回环有 1 帧延迟，可接受**。
- 案 B（保底）：不弹事件，纯让位。功能完整性优先于提示，路测反馈再补。

---

## §五 风险与并存矩阵

1. **根因16/17/18 守卫交互**：辅助变道（starting）≈2 s、同向、低力矩（<120 允许带内），不触 c=0 守卫 A/B（要求 c=0 对向 >140 或 >9 s）、不触 24 s 窗；根因18 的 EPS 回显守卫对变道轨迹照常生效（预期内，静默弃权时本就该硬停）。L3 让位模式**主动退出横向**，与守卫方向一致（司机接管即零指令），无冲突。
2. **L2=0 时**：desire 机死 → `selfdrived.py:314-329` 的 preLaneChange/blocked 事件全部失去触发源（原生厂商同款，非 bug）；灯镜像（controlsd.py:124-128）同理。
3. **ALC 共存**（L2>0）：`AutoLaneChangeTimer`≥NUDGELESS 时拨灯免舵自动变道=厂商没有的超集能力，保持现状默认 NUDGE 即可，矩阵备注进 UI 文案待办。
4. **BlinkerPauseLateral 共存**：两者同开时，低速拨灯 → pause 先命中（不等舵向）；L3 在 ≥其速度阈值或带舵时接管。语义为「更保守的谁先命中谁管」，无对抗。
5. **standstill**：BYD `CC.latActive` 的 standstill 条件（controlsd.py:113-118）与变道无关，低速 <20 mph 本就被 L2 阈值挡在 preLaneChange 外。
6. **法规/心理**：厂商双模式全保留、默认走辅助（原生行为），让位模式仅 `LaneChangeAssistSpeed=0` 显式选择，风险面与厂商一致。

## §六 实现待办拆分（未来落码，按序、各自独立 commit）

- [x] **T1 符号标定**（§四.1）：**离线已完成**——docs_site 存档 rlog 实证 `SteerDriverTorque` 左=正、与 openpilot 约定一致，**不需要任何取反改动**；路测仅剩拨灯信号在场确认（T4 内含）。
- [x] **T2 L2 参数**（§三）**已落码 2026-10-04**：`params_keys.h` `LaneChangeAssistSpeed`(INT,"20")；`desire_helper.py` update_params 读参、`=0` 映射为 `inf`（v<inf 恒真=机器全关，含**中途改 0 会解除进行中的 preLaneChange**，与厂商一致且更干净）。单测 `test_lane_change_assist_speed.py` 5 例。
- [x] **T3 L3 让位模式**（§四）**已落码 2026-10-04**：新类 `sunnypilot/selfdrive/controls/lib/assist_less_lane_change.py`（厂商 :691-703 逐行对齐，含当帧激活/双灯灭解锁/跨灯保持锁存）；接线 `controlsd_ext.get_lat_active`（排在 blinker_pause 后、MADS 分支前=任何模式下都让位）；事件走**案 A**（selfdrived 第二实例同谓词→`EventName.laneChange`，两进程同一 carState 流一帧内收敛；`params_thread` 10Hz 刷参，与 mads 同钩子）。单测 `test_assist_less_lane_change.py` 9 例。
  - **本机跑法**（仓库只带设备端 .so，pytest 在本机 import Params 即挂，既有 sunnypilot 测试同样跑不了）：`docs_site/byd_lane_change_2026-10-04/run_host_tests.py`（stub params → 5 套件 **69/69**，含把三个历史套件首次跑通）。设备/CI 原生环境直接 `pytest` 两个新文件。
- [x] **T4 路测首测 checklist**（2026-10-05..07 部分完成，docs_site/byd_logs_2026-10-05_07/ 415 段 rlog/qlog）：
  - **拨杆灯位语义已定案（推翻 §1.4 的 5=左 猜测）**：`TURN_SIGNAL_SWITCH` 是**成对集合**不是单值——厂商 disasm 10706/10756：`leftBlinker = in (2,3)`、`rightBlinker = in (4,5)`；实车逐段回放 2886 帧/段确认 **右拨 = 1→4(0.3-0.4s)→5(保持)**、**左拨 = 1→2(0.4s)→3(保持)**（瞬态是拨杆机械经过位，与保持位同侧，故厂商把两侧各两值并集）。旧单值猜测 5=左把两侧整体反了。
  - **旧映射的实测后果**：左拨（raw 2/3，采样 32 段里共 100s）`carState.leftBlinker` **恒 0** → 左变道、UI 左灯、BlinkerPauseLateral、L3 让位全部对左灯失效；右拨的 60 次 `laneChangeStarting right` **全部落在 0.2-0.3s 瞬态上**（85 个瞬态→60 次发起，命中 71%），415 段里**没有一次长 hold 被识别成右灯**（>1.2s 的右 run = 0，左 run = 81）。
  - **待复测**：修完映射后的左灯 preLaneChange→带舵→starting→finishing、盲区 blocked（BSD 位已核对无异常：`RIGHT_APPROACH` 13.3% / `LEFT_APPROACH` 5.5% 帧占、`APPROACH`=两侧之或，与字节位一致）、L3 让位、低速 <32km/h 不触发。
- [ ] T5（可选）：UI 注册 + 英文/中文文案。
- [ ] **T6 拍板项（路测数据带出的两条，均属控制行为，先不动）**：
  1. **发起窗口被放大**：旧映射只在 0.3s 瞬态里发起（85→60，命中 71%）；映射修正后一次拨灯 = 连续 2-9s 的 `one_blinker`（左 hold 实测中位 2.6s、最长 20.3s），整段都是 preLaneChange 待命——**只想示意不想变道时，方向盘稍给力矩即开变道**的暴露时间放大 ~10 倍。候选（择一）：preLaneChange 需持续 N 帧才允许 starting / 提高 torque 门限 / 保持现状（与厂商一致）。→ **已拍板落码（§八.2 方案一"持续 clear 窗口"）**，顺带堵死后文瞬态发起与 BSD 10Hz 滞后。
  2. **灯不灭就再来一次**：`laneChangeFinishing → one_blinker → preLaneChange` 是 stock 语义，数据里已见 seg --51 38s 内 3 连发、--25 2 连发；叠加 §1.4 的方向翻转旧象（同一拨灯内 preLaneChange right→left 翻转 4 次、由此产生 2 次意外的 `starting left`）→ 修完映射后翻转消失，但**连续变道/变完再变回**的机制仍在。候选：starting 后要求灯重新来过（灭灯边沿）才允许下一次。

## §七 本文档的证据链

- 厂商：op_byd/selfdrive/controls/controlsd.py:121-122,376-388,691-703,730-736；lib/legacy_lateral_planner.py:9,38,77-83；lib/ 下无 desire_helper.py（未改实证）。
- 本仓库：selfdrive/controls/lib/desire_helper.py:10,41-139；selfdrive/modeld/modeld.py:220,348-353；selfdrive/controls/controlsd.py:115-128；selfdrive/selfdrived/selfdrived.py:314-329；selfdrive/selfdrived/events.py:399-428；sunnypilot/selfdrive/controls/controlsd_ext.py:14,21,34,36-45；sunnypilot/selfdrive/controls/lib/auto_lane_change.py:13-36,45,63-64,85-106；opendbc_repo/opendbc/car/byd/carstate.py:151-156,229-236；common/params_keys.h:136。
- 路测背景：docs/byd-lateral-lifecycle.md §七A/B（根因16/17、守卫 A+B）、byd-control-lessons.md、docs_site/byd_roadtest_2026-10-03/。
- §八审计：`tools/byd/lane_change_bsd_audit.py`（本仓库，host 可跑）对 docs_site/byd_logs_2026-10-05_07/realdata 全 415 段 rlog；carrotpilot 同源移植（op/carrotpilot 本地 clone，byd 目录注释逐字相同、同样 radarUnavailable=True 无 radar_interface）；params_snapshot_2026-10-07（AutoLaneChangeTimer=0=NUDGE、无 LaneChangeAssistSpeed 文件=默认 20、BlinkerPauseLateral=0）。

---

## §八 BSD 门控强化与后方来车审计（2026-10-10 拍板+落码）

### 8.1 缘起与审计事实

用户路测反馈"打灯变道又没有根据雷达判断后方有车才变道"。对 415 段存档 rlog 全量审计（§八工具），事实链：

1. **门控存在且在工作**：70 次 `laneChangeStarting` 发起帧，**同侧 blindspot=1 的 0 次**；`laneChangeBlocked` 事件 20 次。`desire_helper.py` 的 `blindspot_detected` 拦截（§二 stock 路径）不是纸面代码。
2. **但它只有一只 10Hz 的眼睛**：`BSD_RADAR` 0x418 的 LEFT/RIGHT_APPROACH（2-bit，值域实测 0/1/2，rare 2）是**全部**后方来车输入——415 段里 `RADAR_MRR` 0x374 **0 帧**（录制 bus 上根本不存在）；厂商 fork 同样只用 CS.blindspot（其 radar_interface.py 未解密、controls 层拦截=原生 BSD 位），carrot 移植同样 `radarUnavailable=True`。**结论：当前平台拿不到目标级"后方多远/多快接近"，BSD 位只覆盖"已进入盲区警告圈"——后方远处快速接近的车在 BSD 亮之前，系统视之为安全。**（**10-11 勘误：本条中"radar_interface.py 未解密"与"宋线束仅 bus0/2"两点已被 §8.5 推翻；对"后方"目标级数据的缺失结论仍成立。**）
3. BSD 帧占：carState blindspot-true 19.66%（(1,0) 左 44504 / (0,1) 右 58605 / (1,1) 17674 帧）；60+ 次瞬态（0.07-0.15s hold）右发起是旧映射 bug 产物，映射已修（839d871ae8）。

### 8.2 落码（`selfdrive/controls/lib/desire_helper.py`，全车型生效、BYD 设备实测）

| 变更 | 谓词 | 理由 |
|---|---|---|
| **①发起前 clear 窗口** | 新增 `LANE_CHANGE_CLEAR_TIME_MIN=0.5`：preLaneChange 中同侧盲区连续 clear 累计 `lane_change_clear_time ≥0.5s（容差 1e-6，防 10×DT_MDL 浮点和差一帧）`才允许 `(torque_applied or alc.allowed)` 进 starting；任一同侧命中→归零；off→pre、finishing→pre（连续变道）、pre→starting 三处入口都重置 | 堵 BSD 10Hz 采样滞后 + 拨杆瞬态发起（T6.1 候选一落地；0.3s 瞬态永远凑不满 0.5s 窗） |
| **②变道途中中止** | starting 状态下同侧 blindspot=1 → 回 **preLaneChange**、desire 归零（横向回中）、clear 窗重开；selfdrived 对 preLaneChange+盲区的既有谓词自动弹 `laneChangeBlocked` | 原 starting 全窗无保护（10s）；finishing **不**中止——过半再折回比继续完成更危险 |
| ③连续变道重检 | finishing→pre 也重置窗口 | 变完再变/变回必须重新确认后方 |

不动项：`LaneChangeAssistSpeed=0` 让位模式（状态机本就 off，厂商语义）；ALC/BlinkerPauseLateral 语义；carstate 信号（0x418 已正确解析，carstate.py:268-269）。

### 8.3 测试

- `test_lane_change_assist_speed.py`：新增 `TestBlindspotRearTrafficGate` 6 例（边缘有盲区永不起动 / 窗口中途命中重开 / 途中同侧中止回 pre 且保持方向 / 对侧不中止 / 中止后须满窗再起 / 连续变道重开窗）；原 nudge 例适配窗口；`test_lane_turn_desire.py` 集成例 10→12 帧。
- host 跑法（本机 pytest 缺 pytest-asyncio 触发 `--strict-config` 直接 bail）：`.venv/bin/python docs_site/byd_lane_change_2026-10-04/run_host_tests.py -o addopts=` → **75/75**（原 69+6）。

### 8.4 雷达挖掘待办（根治"后方远处来车"，设备上线后）

- [x] ~~T7 设备探针~~ **10-11 已被日志回放代答（§8.5）**：bus1 有流量（"仅 bus0/2"错误），前向 MRR 目标流候选 0x380-38f 已在档内；bus3 与"后方 SRR 目标流是否在可及总线"仍待设备探针补一答（`tools/byd/radar_bus_probe.py` 保留可用）。
- [ ] T8 找到目标流后：入 dbc→（可选 radar_interface/radarState 或新增车型侧信号）→ starting 前做距离+接近速度门控（参照 stock `can_change_lane` 语义，本仓库已无该函数，需自建）。**10-11 进度：前向 0x38x 挖掘与入库见 §8.5 待办；后方若证实无目标流，T8 退化为"BSD 档位语义精细化"（0x418 2-bit 的 1 vs 2）。**
- [ ] T9 复测 §8.2 三行为（路测）：①满窗才起 ②途中亮盲区=回原车道+blocked 提示 ③连续变道二次确认；顺带回归 10-09 拨杆集合判 2/3 左、4/5 右。
- 语义留白：0x418 2-bit 的 1 vs 2 档位含义未知（实测 2 值仅 0.05% 帧）；若路测证实"2=紧迫"，可把发起条件收紧为特定档位而 bool 保留中止。BSD 19.66% 帧占意味着正常跟车就可能常亮——路测若发现 0.5s 窗频繁延误发起，回来重定 `LANE_CHANGE_CLEAR_TIME_MIN` 或改用档位语义。

### 8.5 雷达回放挖掘：bus1 目标流发现（2026-10-11，用户确认档案含跟车记录后重挖）

语料=415 段 rlog；真值=0x32d ACC_HUD_ADAS 的 HasLead(bit9)/LeadingDistance(bit13-15, 0-7 档)——**301/415 段存在跟车**，其中 69 段"有/无前车"两状态各 >800 帧。回放扫描实证：

1. **bus1 有流量（勘误"仅 bus0/2"）**：~110 个地址，含 0x101-0x10a（~28Hz）、0x350-354（~30Hz）、**0x380-0x38f（16 帧槽 @15Hz）**、0x3c8-3ca、扩展帧 0x7fe7a00。
2. **0x350-354 = 空载心跳，非目标流**：payload 恒（0x350 恒 `00f0ffffff1f`、0x351-354 恒 `00002000e000`），首 2 字节为滚动 CRC/tick（byte1 同拍跨地址共享→单一发送 ECU）；跟车/无车两状态分布 AUC=0.5。**厂商 radar_interface.py 已反编译**（`cp_byd/.../byd/radar_interface.py.decompiled.py`：EnableRadarTracks 门控、字段 TRACK_ID/VALID/LONG_DIST/LAT_DIST/REL_SPEED），但**宋不走这两个分支**（0x374 全档 0 帧、0x351-354 恒空载）→ 自建接口不套厂商分支（10-11 拍板）。
   **10-11 三修（bytecode 定案，见 §8.5.6）**：分支集合从 `_create_radar_can_parser` 的 co_names 直接挖出——(849,852)=**{BYD_TANG_GAS, BYD_TANG_DM}**@CanBus.BYD_AUX(**bus1**)25Hz、884={HAN_EV/DM_20_21}@BYD_CAM(bus2)；**宋不在任一集合（厂商也不为宋解雷达）**。且此前"vRel=nan 厂商也没速度"系误读伪码分支：**厂商 Tang 分支 `vRel=REL_SPEED×KPH_TO_MS`**（仅汉 20/21 走距离差分 rate calc）、`yRel=clip(±5)`、track_id==0=空槽哨兵。
3. **~~前向目标流候选~~ → 0x380-0x38f 证伪（10-11 二轮位级检验）**：首轮"状态分离分"（0x380 LE@2=0.856 等）被 **92 个"档位≥3 档且各≥150 帧"段的逐档中位数单调检验推翻**——没有任何字段跨段一致随 0x32d 档位单调（同槽字段换个段就反号）；字节形态 = **b0/b1 严格滚动计数器（b1 恒 +17/帧 mod 256、同拍跨槽共享→单一发送 ECU）+ 逐槽常量（如 b3=0xFA 系、b2=0x80 系、b4=0xF8/0x80 系）+ 校验噪声**，是**状态/遥测表**特征，非目标列表。分离分是计数器新鲜度与档位的时段共现伪影。
4. **最终判定：宋的任何已录总线上都不存在目标级雷达流（前后向皆无）**，"日志回放解密目标流"此路不通。前向可用的只有 0x32D 融合摘要（HasLead+LeadingDistance 0-7 档，实测跟车段以 3-4 档为主）。与厂商 fork 只用 blindspot 位、carrot `radarUnavailable=True` 互证。工具留档 `tools/byd/radar_layout_mine.py`（引新总线后可复用）。
5. **入库落地（8.5.2 完成，形态修正）**：
   - `opendbc/car/byd/radar_interface.py`（新）：**0x32D 档位合成单目标 leadOne**（dRel=档位表值、yRel=0、vRel/aRel=NaN——与厂商同缺口），显式不套厂商 849/852/884 分支（宋不走那些 ID）；`values.py` dbc_dict 增 `Bus.radar: byd_general_pt`；`interface.py` 挂 `RadarInterface`。
   - **默认惰性**：`ret.radarUnavailable` 保持 True → RI 无 parser、liveTracks 不发布、控制行为零变更（v2 路测前不动控制原则）。
   - `RADAR_HUD_BARS_TO_M`（values.py）= 占位档位→米表（1:6 2:13 3:22 4:35 5:50 6:70 7:90m）。
   - 测试 `selfdrive/car/tests/test_byd_radar_interface.py` 6 例（真 CANParser 位序回归+惰性+清槽）；host 回归 **81/81**（75 基线+6）。rlog 回放段 113：3620 返回帧，lead→单点 100% 一致、0 错值、0 无中生有。
6. 待办：
   - [ ] 8.5.4 **路测标定档位→米**：人工测 1-4 档实际车间距（GPS/雷达实测对照），定稿 `RADAR_HUD_BARS_TO_M`；标定后翻转 `interface.py` 的 radarUnavailable 即点亮（无其他改动）。
   - [ ] 8.5.5 后方方向不变：可及总线无 SRR 目标流（曾疑 0x2e0-2e8 块，复核为跨总线统计串入，bus1 无此地址）；要么 0x418 档位语义细化，要么设备探针（`radar_bus_probe.py` 补 bus3 与后方 SRR 两问）后评估引线代价。

### 8.5.6 社区 mypilot 资产 + 槽位双路接口落码（2026-10-11 拍板 A+B）

`/Users/wujiafu/Documents/op/openpilot-mypilot-community-dbcall`（🐬 mypilot 社区开源，BYD 全系+吉利，明文源码+DBC）补齐了最后一块：

- **`byd2/byd_tangdm_radar.dbc`**：**0x351-354 = RADAR_TRACK_351..354**（MRR 4 轨迹槽 @25Hz，解析于 `CanBus.MRR=1`——正是宋的 bus1）：`LongDist 16|12@1- (0.06,-4)m`、`LatDist 28|10@1+ (0.05,-25.6)m`、`TrackID 38|6`、**`RelSpeed 44|10 (1,-512)`（dbc 标 km/s 显系笔误，社区代码原样当 m/s 传）**、`IsValid 56|1`。**用此布局反解宋的空载 payload `00002000e000`：LongDist=-4 下界、LatDist=-24 下界、IsValid=0——与"无有效目标占位帧"完全自洽**。据此修正 §8.5 第 2 条：宋 bus1 的 0x35x **就是**厂商 (849,852) 分支协议族，只是 MRR 从不向 bus1 输出有效目标；社区同样未把宋列入其雷达名单（`RADAR_CAR`=秦 Pro/唐 DM；`PT_RADAR_CAR`=汉/唐DMP/腾势/元 的 pt 域 0x374 `CID_NCBHOL`@60/100Hz；宋两者皆非）。
- 同包其他雷达资产：`byd源码/radar_interface.py`（低阶明文版，0x374 @MPC 总线 TargetID **1左/2前/3右**，vRel=NaN）；**`EnableExtRadar`=外接 GM Continental 雷达方案**（0x460 header+0x461-0x474，直接用 `gm_global_a_object` dbc）——社区给"无可达内置雷达"车型的硬件外挂路径，宋纵向重建的现成第三选项；`比亚迪3/Bydsong(1).dbc`=宋整车原厂 DBC 数百报文**无雷达帧**（佐证宋可及总线本无目标流，非厂商私藏）；`byd2/byd_generic_pt.dbc` 含混淆名版 0x374（`CID_XXQFOZ`，与我们 BO_884 位序一致+两个附加位）。
- **落码（A+B 完成）**：
  - **A**：tangdm 布局 verbatim（含 0x352 LatDist 负号怪癖）入 `byd_general_pt.dbc` BO_ 849-852。
  - **B**：`radar_interface.py` 双路——**track 优先（IsValid 过滤、RelSpeed 按 KPH_TO_MS 换算带 TODO、槽点 id 101-104 与 HUD 点 id=1 命名空间隔离、槽全失效即清）→ 无有效槽时回退 0x32D 档位合成**；track 总线静默不报 canError（仅 HUD 门控错误）。真帧一旦出现（引私有 CAN/AEB 工况/路测偶发），**零改动自动切真轨迹流并获得 vRel**。
  - 验证：host **86/86**（81 基线+5 track 用例）；段 113 回放：bus1 实测 7240 帧解码全 IsValid=0（社区布局交叉验证✓）、HUD 回退 3620/3620 一致 0 错值（与上一 commit 行为等价）；`radarUnavailable` 仍 True→设备零变更。
- **厂商 bytecode 定案（10-11 夜，`marshal` 直读 `radar_interface.py.parts/006/008*.pyc` 的 co_names——车型 frozenset 未被混淆）**：分支 = **{BYD_TANG_GAS, BYD_TANG_DM} → (849,852) @25Hz on CanBus.BYD_AUX**（`values.py.values.txt` 钉死 **BYD_AUX=1=bus1**、BYD_CAM=2）；{HAN_EV/DM_20_21} → (884) on bus2；**宋不在任一分支=厂商不为宋解雷达**（与"宋 0x35x 全空载"因果闭合：不是漏配宋，是宋的 MRR 真不往 bus1 发目标——"tangdm 是否适用宋"从推定升级为同家族实证，消费方式以厂商 Tang 分支为准）。
  - `_update` 全文还原：**Tang 分支 `vRel = REL_SPEED × KPH_TO_MS`**（汉 20/21 才走 `NonConstantCanSignalRateCalculator(60,16)` 距离差分）、`yRel = clip(LAT_DIST,±5)`、**`track_id==0` = 空槽哨兵**、失效链 `no_var_cnt≥INVALID_LIMIT→measured=False→NO_VAR_LIMIT→删点`。§8.5 第 2 条早先"vRel=nan 厂商没有目标速度"系误读 decompyle 伪码，**更正**（nan 仅 aRel/yvRel）。
  - 对齐落码：`_update_tracks` 增 tid==0 哨兵 + yRel clip±5；vRel 换算从"TODO 猜测"升级为"厂商同款"（dbc factor/offset 数值仍待首帧真目标核对）。复验：host **87/87**（+1 哨兵用例）；回放 4212 帧返回、按厂商口径可疑 track 帧 **0**、回退 3620/3620 不变。

### 8.5.7 byd_logs_2026-10-05_07 全量雷达复扫（2026-10-11 夜，415 段 6.9GB，工具 `tools/byd/radar_1005_scan.py`，数据 `radar_mine_1005.json`）

社区包对照驱动的定案级复扫——把 0x351-354 槽、0x374、bus1 全部高频地址块一次过筛（跟车帧 0.9M+）：

- **0x351-354（tangdm 槽）**：793,578 tick×4 槽（bus1 实测 742,452 帧/槽），**IsValid=1 计 0、TrackID≠0 计 0**；80 种 payload 仅 b0/b1 滚动计数器+状态位。即使同档案 HUD 有 903,258 个 lead 帧（60.8%），槽从不带目标——"AEB/跟车工况会填槽"假设在本档案无一次例外，**track 路只能靠引私有 CAN 点亮**。
- **0x374 RADAR_MRR**：**415 段×全总线 0 帧**。社区 `byd源码/radar_interface.py`（0x374 @MPC=bus2，TargetID 1左/2前/3右）对宋同样被其 `radarUnavailable=True` 死代码（§8.5.6 互证再强化）；唐 dbc 的 MSG_34x 布局系暴力拟合伪迹（range [1.0|1.0]/factor 100），不可采信。
- **bus1 新面证伪（本轮最大发现）**：冒烟直方图浮出两个从未挖过的连续块——**0x101-0x10A（@25Hz，与槽同 tick）与 0x340-0x34A（@14Hz）**，payload 多样性"RICH"。用 `radar_layout_mine.py --targets`（新增 --targets/--segs 旗标）对 8 个强跟车段做秩一致性：**无任何 u8/u16 字段达到 ≥8 种取值**——数据字节准常量，变异全部来自共享 b1 计数器；与 0x380-38F/0x3F8-3FF/0x301/0x396/0x397/0x350 同一签名（b1 每帧严格 +17 mod256）。**判定：宋 bus1 = 单 ECU 状态广播全家桶，无目标流，非漏挖**。这正是首轮 0x38x 新鲜度伪影陷阱的重演，工具 docstring 已记。
- **0x32D HUD 档位观察（喂 §8.5.4 标定）**：1,484,902 帧、lead 60.8%、321/415 段有 lead；**LeadingDistance 全档案只出现 1-4 档**（bar1 0.4% / bar2 9.7% / bar3 39.9% / bar4 50.0%），0/5/6/7 从未出现——`RADAR_HUD_BARS_TO_M` 的 5-7 是外推值，路测标定按 1-4 即可；档位切换全程仅 1,482 次（lead 期稳定，回退合成不抖）。
- bus1 另有 J1939 扩展帧 0x7FE7A00/0x1BFE7A1A/0x1BFE7A1B（~14Hz 诊断广播），与雷达无关，track 解析器地址集不命中即无影响。
- **落码侧微调（本 commit，注释级、零行为变更）**：`values.py` 档位表注记观测分布；`radar_interface.py` docstring 把"AEB duty state 可能填槽"降级为"415 段实证仅私有 CAN 引线可达"；`radar_layout_mine.py` VERDICT 扩两个新证伪块。
