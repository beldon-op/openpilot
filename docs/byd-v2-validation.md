# BYD 宋 PLUS DM-i · v2 实车复验报告（route 0000000e/f）

> 数据：2026-09-30 两段路测。route `0000000e`（4 seg，281s）+ `0000000f`（5 seg，302s），
> 设备 GitCommit=`9a46d312e4`（根因 16 v2 = `51b1a25a5f` 在跑），固件 fec63fda。
> 配置：`AlphaLongitudinalEnabled` 参数不存在 = **OFF（纵向 = 原车 ACC）**；NNLC=ON（未标定）。
> rlog + 分析脚本存档：`docs_site/byd_v2_validation/`（gitignore，本地保存）。
> 状态：v2 核心目标全部达成；部署路测完成（§八：症状2 数据实证修复、NNLC 排除、4s 接管延迟分解）。
> 最后更新：2026-09-30（含 §八 部署路测）。

## 一、v2 核心验证结论（对照 `byd-control-lessons.md` §五决策树）

| 验证项 | 结果 | 证据 |
|---|---|---|
| 整体能控 / c 门死锁解除 | ✓ | route e armed ≈234s/281s，其中 **72% EPS 真执行**（c=1）；route f 84% |
| 零 TorqueFailed 锁存 | ✓ | 9 seg 全程 TorqueFailed=0（v2 前的历史：10 次锁存） |
| err=2 不升级 err=4 | ✓ | 全程无 err≥2 记录（含 bounce 风暴时段） |
| 手轻武装门（根因 15） | ✓ 工作 | SET 后手扶盘：engage→burst 延迟 0.6-3.2s；手离开：0.02-0.06s 秒装 |
| 请求包络 200（根因 16v2） | ✓ 顶格即止 | armed 期间 MT max=200（= 包络），不再出现 ±300 rail |
| MT 分布 vs 厂商 | ⚠️ 偏热 | 我方 p50 61-73（厂商 52）/ **p90 159-180（厂商 115，1.5×）** / max 200（厂商 193） |
| sign flips（遗留观察项） | ✗ **坐实** | 见 §五 |
| ACC bounce | ✗ **新发现** | 见 §四 |

用户体感"整体有控车、效果不错"与数据一致（EPS 真执行占 armed 大头，MT 中位数与厂商同量级）。

## 二、症状 2 定案：boot-mid-cruise engage 边沿吞噬（第 2 段）

**现象**：ACC 已开 → 设备才启动 → 迟迟不控车，需 ACC 多次重新进入才能控车。

**机理**（`car_specific.py:215` + `carstate.py:144`）：engage 唯一入口 = `cruiseState.enabled`
**上升沿**（pcmEnable）。OP 启动前 ACC 已真活跃（AccControlActive=1）时，`is_cruise_latch`
第一帧即 True——边沿在 OP 醒来之前已经发生完，永不再来 → OP 永不 engage，无绿框。

**route f 证据**：开头 1.3→21.9s 共 **20.6s "ACC-on-OP-off"**（ACC 持续 on、selfdriveState.enabled
恒 False），21.9s 司机取消→重 SET 造出边沿后才 engage。

**"多次重新进入"的解释**：每次取消→重 SET 只造一次边沿；前几次失败 = 撞 selfdrived
就绪窗口（点火后 ~15s，ops 文档 §三.2 已知）。就绪后的重进一次即成功。

**修复实现——✅ 厂商同款方案（`opendbc/car/byd/carstate.py` `BOOT_LATCH_HOLD_TIME`）**。
初版曾在 `car_specific.py` 合成 pcmEnable（厂商从不改该文件，层不对）；后从**厂商自己的
boot-mid-cruise 日志**（7--12e_0，雷达 ACA 全程=1）解码出厂商真实机制并照搬：

```
厂商实录: 3.18s latch=0/avail=0（ACC 明明 active 却压着）→ 16.74s avail=1 → 17.54s latch=1
         （真边沿）→ 17.55s OP enabled 同帧接上
```

**厂商 = 在 carstate 层把 latch 压 ~15s（engage cooldown），让边沿"迟到但真"**——
canValid/就绪窗口全部就绪后边沿才发出，通用层零改动。我们的实现：CAN 启动后前 15s
`is_cruise_latch` 强制 False（raw latch 照算），到期自然放行 → 唯一一次真边沿。
hold 期间按 SET → 被吸收进 hold 结束的边沿；hold 期间取消 → raw latch 为 False 无边沿。
`cruiseState.available` 保持 raw 不压。
单测：`selfdrive/car/tests/test_byd_carstate.py` 3 场景（hold 时序/hold 中取消/无 ACC），
`pytest --noconftest -c /dev/null`。**待实车复验**：boot 时 ACC 已开 → ~15s 后应自动出绿框。

## 三、症状 1：绿框早、控车晚 = 三层延迟叠加（第 1 段）

绿框 = OP engaged（`selfdriveState.enabled`），到 EPS 真出力（c=1）之间实测三层：

| 环节 | 实测 | 性质 |
|---|---|---|
| ① engaged → 武装 burst | 手扶盘 0.6-3.2s；手离开 0.02-0.06s | 手轻武装门 `STEER_ARM_DRV_TORQUE=50`（根因 15），by design 但 UX 可感。与 deep-review 控车#1（STEER_THRESHOLD=80 落轻握带 60-150 → 轻握既 pressed 又挡武装）同族 |
| ② burst → EPS 接受（c=1） | 常态 0.2-1.1s，最长 2.8s | 厂商语义（等待相位）；**drv=0 时 EPS 也等 → 等待条件非纯手**，条件未逆向 |
| ③ 等待相位 windup | c=0 期间 demand **饱和在 200 包络** | 我方独有：等待相位占 armed 时间 16%（route f）-28%（route e）。厂商等待相位 demand ≤193 从不 rail（其环几乎不积分全靠前馈）；我们 rail 满 200 |

**风险**：③ 的 windup 在 EPS 激活瞬间全额释放。本两段未捕捉到明确的"rail 态激活猛给"
样本（234.5s 的 +23° 转角经 MT=0 证伪为司机打盘，非 EPS 释放），量级未定，但机理存在
——根因 16 v1（0000000a）的"c 翻 1 瞬间甩盘"就是同类形态，包络 200 已把最坏值从 300 压到 200。

## 四、新发现 A：ACC bounce 风暴 → OP enable 抖动 → 0x316 流空洞

两段路测最重要发现。**armed 时段雷达 ACA（ACC_CMD.AccControlActive，bus2 原发）以
60-110ms 打摆**，伴随 ACC_HUD_ADAS `Notify=8`；gas/brake/按键/steeringPressed 全零 = 非人为。
- 分布：route e 12 次 + route f 4 次，**16/16 全部发生在我方 armed 时段**；idle 时段零。
- 节奏：间隔 500→460→240→120ms **递增**（雷达渐怒），每次 ACA=0 持续 60-110ms。
- 厂商对照：3 条厂商 route（7--12e/6--ff9/5--44a）**零 bounce**；但厂商首个 armed 会话
  启动 6.8s 后 ACA 也让位过一次（40s 长让位，随会话结束恢复）——"LKA active 时 ACC
  短暂让位"疑为车的正常联动，厂商是低频长让位，我们被自己的循环放大成 2Hz 抖动。

**连锁反应（帧级实证，route e seg3 222.8-226.8s）**：

```
雷达 ACA 跌零 60-110ms + Notify=8
  → AccState 3→1，stock_acc_on=False → is_cruise_latch 跌零
  → pcmDisable → OP disable → controls_allowed=0
  → 固件拒发我方 Active=1 帧（byd.h：active=1 需 controls_allowed）
  → 0x316 流空洞：退场斜坡期间帧全被 REJECTED（src=192），4 处，
    最长 21 帧 / 0.42s 连发拒收 —— EPS 收不到优雅退场（斜坡+Active=0 收尾）
  → ACA 恢复 → latch 回 1 → pcmEnable 上升沿 → OP 重 engage → re-burst
  → 循环，直到雷达风暴自行平息（~4s）
```

route e 单段因 bounce 产生 **20 次 enable 循环**（0.3-0.6s 间隔的假"重新激活"）。
本次未升级为 err 的原因：0.42s 空洞 < 实测 armed-零锁存带 0.48-0.72s——**贴线幸免**，
再长一档就是 EPS 断流投诉（SteerWarning/根因 14 族）。

**触发源未定位**。嫌疑排序：①需求环振荡（§五，雷达/watchdog 对转向活动不满）→ 修振荡
可能连带消失；②armed-零背停的 0.45s 会话重启节奏可疑但时序不符（bounce 先于我方退场）；
③雷达对 0x316 内容的校验（我方 armed 帧与厂商逐字节同分布，含低频罕见模式，已排除内容差异）。

**修复方向（v3-5，先修上游再看）**：
- 上游：§六-1 振荡杠杆 + §六-2 NNLC A/B。
- 空洞本身：退场斜坡（~80ms，out>0 时 Active=1）与 controls_allowed 抖动冲突。
  选项 a：controller 侧 latActive 掉时 Active=0 硬切（放弃斜坡，违背厂商退场编排）；
  选项 b：固件放宽（controls_allowed 撤销后允许 Active=1→0 斜坡 N 帧）；选项 c：不动，
  待上游修复后复测。倾向 c → a 备选。

## 五、新发现 B：需求环振荡坐实（sign flips 观察项结案）

根因 16 v2 遗留观察项"0000000d 回放 sign flips 46>40（疑似零输出期规划器追人手的回放
伪影）"——**实车改判为真实存在**：

- 直道、方向盘静止（0.5s 窗口转角 range ≤0.6°）时，我方 LKAS_Output 在 ±50 摆动
  （0.5s range 42-76，route e seg3 222.5-227.5s）。
- armed 时段 EPS 转角速率 p90=96-160°/s、p99=272-288°/s、max=464°/s——历史"正常控车"
  max=129°/s、p90=55（根因 9 时代），现在**整个分布上移 2-3 倍**。
- MT p90 159-180 vs 厂商 115：出力整体偏热，与振荡互为印证。

§五决策树第二行触发条件达成。注意状态文档 C-4：**NNLC=ON 且从未标定**——先做
NNLC 关/开 A/B 再上 governor，避免两个变量混在一个路测里。

## 六、待拍板清单（v3 候选，按建议优先级）

| # | 项 | 改动面 | 说明 |
|---|---|---|---|
| 1 | ~~NNLC A/B~~ | — | **✅ 结案（§八.3）**：NNLC=OFF 基线下振荡/bounce/MT 热全部复现 → NNLC 排除 |
| 2 | ~~振荡杠杆：yysnet 转角速率 governor 或厂商 EMA~~ | 控制行为 | **降级为观察项（§九 二轮挖掘）**：10-05..07 语料 175 万直道 0.5s 窗 torque 换向 p50/90/99=**0/0/0**——09-30"直道 ±50 振荡"在厂商满权限重构+brake_yield 后未复现；仅当实车再现 ~0.6Hz 抖再启用 |
| 3 | ~~boot-engage（症状 2）~~ | — | **✅ 结案（§八.1）**：`d4160fd` BOOT_LATCH_HOLD 路测+帧级数据双确认，自动接车 16.3s |
| 4 | 等待相位 windup 抑制（c=0 时冻结/衰减 demand） | 控制行为 | **升为第一优先（§九）**：415 段实测 latActive&c=0 帧占 7.6%，其中 **|归一化需求|>0.8（封顶前 ≥240/300）= 34065 帧 ≈ c=0 期的 18.6%**；唤醒代价 engage→c=1 p50 1.95s/p90 9.3s/max 11.9s（n=140 唤醒）——排队蠕行每次出休眠窗都要重握手+吃 rail 释放。方案候选：把 brake_yield 泛化为 c=0 无条件衰减（不限刹车） |
| 5 | ~~0x316 流空洞~~ | — | **✅ 结案为口径伪影（§九）**：按"latActive 连续窗"重测=**0 洞**；历史 2~66s 大洞全部是停车/刹车休眠窗（b33616e596 设计内）跨窗误计。固件拒发假设不成立，无需动 byd.h |
| 6 | STEER_THRESHOLD 标定 | 参数 | 4s 接管延迟的第一层（§八.2：手扶盘武装延迟 0.8-2.3s，轻握带 60-160 再确认）；§九：语料内 OP 退出仅 13 次全为 gear/user，刹车不掉横向实证生效 |

纪律提醒：4 触碰控制行为，逐项拍板、单项路测（勿多项同改——根因 14-16 的教训）。

## 七、分析工具与数据（docs_site/byd_v2_validation/）

```
r0000000e_{0..3}.zst / r0000000f_{0..4}.zst   # 两段路测 rlog（设备 /data/media/0/realdata/）
scripts/
  analyze_engage.py   # 逐 engage 事件时序：enabled→latActive→burst→armed→EPS c=1 + 手扭矩
  scan_bounce.py      # 全程 ACA 跌零统计（bounce vs 长让位分类、armed 关联、Notify）
  fine_trace.py       # 帧级三路并显：我方 TX(src=128)/相机 0x316(src=2)/EPS 0x318
  dump_trans.py       # 状态变化行打印（latch/ACA/op/lat/c/tx 摘要）
  eps_exec.py         # EPS 执行判别：c + drv + MainTorque + 转角/转率
  exec_stats.py       # armed 时段 c=1 执行率 + MT 分布（支持厂商 sendcan 对照）
  dump_acc.py         # ACC 域信号 + 踏板/按键（排除人为）
  accel_trace.py      # 我方 ACC_CMD echo vs 雷达原帧 accel diff（OP long 离题验证）
  angle_trace.py      # 0.5s 桶：转角 range / 出力 range / 翻向数（§五 振荡证据）
  rej_timing.py       # REJECTED 0x316 帧时间 vs bounce/armed 时段（§四 流空洞证据）
  vendor_overlap.py   # 厂商 sendcan armed 会话 vs ACA 时间线（§四 厂商让位对照）
  diff316.py          # 我方 vs 厂商 armed 0x316 全字段模式 diff（§四 内容排除）
  rate_compare.py     # armed/idle 转角速率分布（§五）
```

用法：`PYTHONPATH=$PWD/opendbc_repo:$PWD .venv/bin/python scripts/<x>.py <rlog> [w0 w1]`。

**src 编码备忘（本次踩坑修正）**：can 流 `src` = bus 号 + 特殊位。
`0`=bus0 RX、`2`=bus2 RX、**`128`=bus0 TX loopback——我方 TX 与 fwd_hook bus2→bus0 转发副本
混在一起**（0x316@128=我方（790 转发被 block）；0x32D/E/F@128=雷达帧转发副本，非 OP 纵向！），
`130`=fwd bus0→bus2 转发副本（0x11F/0x318 等全量 1:1），`192`=固件拒收。
CANParser 按 `src==bus` 过滤（`parser.py:221`），跨 src 同址帧（0x316/0x32E 双源）必须分 parser。

重新拉取：`ssh comma@192.168.31.44 -i ~/Documents/op/menmen.key
"cat /data/media/0/realdata/<route>--<seg>/rlog.zst" > <route>_<seg>.zst`
（本地文件名含 `-` 会让 logreader 误判路由名，存成 `r{route}_{seg}.zst`。）

## 八、部署路测：v2 修复上车复验（route 10/11/12，NNLC=OFF；2026-09-30）

> 数据：2026-09-30 午间两段路测 + 15s 残段。route `00000011`（5 seg，323s）+ `00000012`
> （4 seg，222s）+ `00000010`（15s，未完成起步）。设备 GitCommit=`d4160fd`
> （症状2 修复 BOOT_LATCH_HOLD 上车）。**NNLC=OFF（用户关后开车，两次均确认）。**
> rlog/qlog/swaglog 存档：`docs_site/byd_roadtest_2026-09-30/`（gitignore，md5 20/20 校验）。
> 用户体感：整体效果偏好；症状2 已修复；重新开 ACC 后 ~4s 才接管（下文逐一对应）。

### 1. 症状2 = ✅ 数据实证（帧级，route 12 seg0）

ACC 开着上车 → 设备启动，全程无按键：

```
1.30s  ACA=1（雷达 ACC 真活跃） latch=0（hold 压住） op=0 | EPS c=1（boot 前旧会话）
7.66s  EPS 旧会话 c→0（退场），ACA 仍 1，latch 仍 0（压住中）
16.28s latch 0→1（hold 到期放行，真边沿）→ op 同帧 enabled → lat=1
16.37s burst P=1 → A=1 armed（out=16）—— engage 后 0.09s
20.36s EPS c=1（+4.08s 等待相位）
```

生效时长 16.28−1.3 ≈ **15s，与 BOOT_LATCH_HOLD_TIME 一致**。两路由
ACC-on-OP-off >2s 长段 = **0**（v2 route f 为 20.6s）。症状2 结案。

### 2. "重开 ACC ~4s 才接管" = 三层分解确认（18 次 engage 逐事件时序）

| 层 | 环节 | 本次实测（NNLC=OFF） | v2（e/f） | 厂商 |
|---|---|---|---|---|
| ① | engage→armed（手轻武装门） | 手扶盘（pressed=1）0.82-2.29s（5 例）；手离开 <0.1s | 0.6-3.2s / 0.02-0.06s | — |
| ② | armed→EPS c=1（等待相位） | 干净 boot 4.08s；重 SET 0-7.7s；**两个 7s 案例均邻 bounce** | 0.2-1.1s 常态，max 2.8s | max 2.86s（415 帧） |
| ③ | 等待期 windup | r11 212.33s wait 中 out rail 200；弯道 out ±170 拉锯 | rail 200（包络封顶） | ≤193 从不 rail |

engage→EPS c=1 全程 1.6-7.7s、中位 ~4.2s = 用户"4 秒左右"✓。要点：
- 两个 6.7/7.7s 案例（210.4s / 230.1s）帧级：armed 后 out 恒 16，EPS 握 `LKAS_Prepared=1`
  不放、c 迟迟不翻——**EPS 显然不等 out 幅值**；激活条件仍未逆向
  （eps-reference §11.7：手轻主导但有例外）。等待期越长，③ 的 windup 越肥。
- 另有 bounce 假取消→enable 抖动加重感知（r11 99.8-100.2s 0.4s 内 3 次 enable 边沿）。
- 杠杆对应：① → 拍板 #6（STEER_THRESHOLD）；③ → #4（windup 冻结）；感知抖动 → bounce 修复。

### 3. NNLC=OFF 基线 = 振荡/bounce/MT 热全部复现 → NNLC 排除（拍板 #1 结案）

| 指标 | v2（e/f，"NNLC=ON"） | 本次（11/12，NNLC=OFF） | 厂商 |
|---|---|---|---|
| armed 转角速率 p90 / p99 / max | 96-160 / 272-288 / 464 | **208-240 / 464-800 / 960-1488** | — |
| \|MT\| p50 / p90 / max | 61-73 / 159-180 / 200 | 60-63 / 166-200 / 200 | 52 / 115 / 193 |
| ACA bounce | 16 次（16/16 在 armed） | 10 次（r11 8 + r12 2；3 次在 op-on，其余 armed 帧） | 0 |
| 0x316 空洞最长 | 21 帧 / 0.42s | **25 帧 / 0.45s**（r12 169.2s，bounce 后 40ms） | 0 |
| sign flips | 坐实（直道 ±50） | 仍在（r11 126.1s：ang≈-5°，out -29↔+33 /0.5s） | — |

- **NNLC 不是振荡/MT 热的根源**——OFF 下全部复现，转角速率尾段反而更差
  （弯道需求拉锯 + windup 释放：r12 108-112s 弯道 out -183↔+31）。
- caveat：e/f 侧 NNLC 是否真生效，日志 `lateralTorqueState.error/f` 分布呈 stock 路径特征
  （lataccel 量级 ≤0.94，非扭矩空间），无法强确证；但两种解读结论一致：
  **振荡根源在 stock 扭矩环+我方前馈，不在 NNLC** → 拍板 #2 升第一优先。
- bounce→enable 抖动→0x316 空洞链条完整存活：空洞全部与 bounce 相邻
  （±40ms 内），最长 0.45s vs armed-零锁存带 0.48-0.72s——仍贴线（拍板 #5 维持"上游修完再复测"）。

### 4. 数据备忘

- route 11 前 3 seg / route 12 seg0 录制时设备时钟未同步（mtime 显示 2025-07-02），
  logMonoTime 为开机相对时（route 基准 83.1s/84.3s）；分析用段内相对时即可。
- route 11/12 各自末段尾带 corrupted tail（logger 未收尾，设备端固有，非拉取损伤）；
  `q00000012_3.zst` 设备端即空（rlog 完整，无影响）。
- `exec_stats` 的 armed 计数含 CANParser stale 高估（只看相对占比）；MT 分布不受影响。
- 脚本沿用 `docs_site/byd_v2_validation/scripts/`，副本在本目录 `scripts/`
  （`analyze_engage.py` 已改路径）。

## 九、二轮全量挖掘（2026-10-11，docs_site/byd_logs_2026-10-05_07 全 415 段，build=b33616e596）

工具：`tools/byd/log_mine.py`（单 pass，host 直接跑，`参数=目录 [段数上限]`）。语料=8.2h onroad（含排队蠕行）。

| 项 | 结果 | 判定 |
|---|---|---|
| 直道换向振荡（拍板 #2） | 175 万直道 0.5s 窗 flips p50/90/99=0/0/0 | **降级观察**（09-30 振荡在满权限重构+brake_yield 后未复现） |
| 等待相位 windup（拍板 #4） | latActive&c=0 占 7.6%；其中归一化需求 >0.8（封顶 240/300 前）=34065 帧≈c=0 期 18.6%；唤醒 engage→c=1 p50=1.95s / p90=9.3s / max=11.9s（n=140 唤醒，多为蠕行出休眠窗） | **升第一优先**：brake_yield 泛化为 c=0 无条件衰减可同治"接管晚+释放猛" |
| 0x316 流空洞（拍板 #5） | latActive 连续窗内 **0 洞**；历史 2~66s 大洞全是休眠窗跨窗误计（测量伪影） | **结案**，固件不动 |
| OP 退出归因 | active→false 全语料仅 **13 次**，全部 wrongCarMode/reverseGear/userDisable；刹车不掉横向（BYD selfdrived 豁免）实证生效 | Mads=0 下体验已达标，勿被"参数=0"误导去开 Mads |
| SteerWarning / TF | warn episode 16 次（≈0.5/h，多在低速+对向手力）；TorqueFailed=**0** | 守卫栈（warn 硬停+守卫 C+brake_yield）有效 |
| EPS err 码 | 首次观测 err=3（639 帧单 episode，c=1 健康自恢复）与 err=1（36 帧）；2=179 帧、4=0 | 码表已更新 eps-reference |
| 速度标定 | vEgo/GPS p50=0.999 mean=1.007（10/9 ✓）；vEgoCluster/GPS p50=1.085（1.086 ✓） | 结案 |
| PEDAL | 869 次油门按压仅 16 次 <0.15s 短点（≈2/h） | 0.05 阈值达标 |
| BSD 0x418 | true 19.6%；**静止 0 帧**（RCTA/开门不污染）；**value≥2 共 304 帧全在 ≈22m/s(80km/h) 行驶** | "2=行驶中紧迫档"成立——§八.4 T8 门控升级的弹药；road-test 对照镜灯确认 |
| cruiseMismatch | 1334 事件帧（≈163/h，多为 NO_ENTRY 场景非掉控） | 观察，#4 落地后复看 |
| 拨杆变道 | preLC left=64 / right=85、blocked=20、laneChange=814 | 语料为旧映射期；§八 新门控待 T9 路测 |
