# BYD 宋 bus1 雷达逆向 — 整体报告（2026-10-18）

> 一句话：**宋的 bus1 确实有雷达原始数据**——0x101-0x10A 是 Continental ARS410 家族的目标帧，
> 其中 0x109 b7 是领先 HUD 档位的连续前向距离（候选缩至 feet 0.3048 vs 0.25，1.0/0.5/0.45 已被 13 起跟停事件硬上界排除）、0x10A b3[7:5] 是目标存在位、
> b0 已破译为 SAE J1850 CRC-8；但**没有相对速度场**，完整 track 语义仍不可覆盖。
> 六帧已入 dbc（HYPOTHESIS，无任何在车实例化），控制行为零变更。
> 细节定案见 `docs/byd-radar-010x.md`，本报告为全程汇总。

## 1. 起因与结论

- 起点问题：分析 `docs_site/hour_logs`（2026-09-28 深夜首次 40 分钟连续控车的 4 段全量 rlog）的 can1 有无有效雷达数据。
- 首轮结论（按既有 tangdm 布局判）：0x351-354 槽位恒空、0x374 零帧 →「无有效目标流」，与 10-11 定案一致。
- 翻案：抛开 IsValid 与已猜布局做字节级挖掘后，**0x101-0x10A 被证明携带真实目标信息**，
  推翻 10-05_07 复扫「bus1 新面 0x101-0x10A=状态广播/计数器伪影」中涉及 0x109 的部分。

## 2. 方法演进（五个阶段，工具全部入库 tools/byd/）

| 阶段 | 方法 | 工具 | 产出 |
|---|---|---|---|
| ① 字节级挖掘 | 0x32D HUD 档位当监督,对 bus1 全地址×全字节算互信息 MI | radar_re_mine.py | 命中 0x109 b7 MI=1.28bit,两段独立单调 |
| ② 跨段验证 | 416 段 rlog 逐段档位序检验(防新鲜度伪影) | radar_0109_validate.py | 严格单调 102/113=90.3%,c0 归零 96.3% |
| ③ 对抗审查 | 三路独立证伪:时间假象/HUD 回声/单位自洽 | radar_echo_probe/checks.py, radar_unit_check*.py | 计数器说死、回声说死(8/10 翻转 b7 先行)、米制被疑 |
| ④ 监督反解 | can2 为标(档位/时距带+vEgo),单/双字节×LE/BE×符号全变体回归 | radar_sup_mine.py | 0x109 b7 唯一跨段稳定;无 vRel;b0 确定性函数;b6/b7 平行 |
| ⑤ 钉定收口 | 静止逼近硬上界+翻转回归+CRC 试打,486 段全量+独立日 | radar_unit_pin.py + radar_unit_aggregate.py + radar_415_diversity.py | u≤0.467(米制否)、CRC 破(100%×2 组)、多样性终检成立 |
| ⑥ 跟停等式 | 13 起 ACC 跟停事件(standstill+驻停平台+存在位),逐窗 u≤D/Δb7(前车不可倒车) | radar_stop2lead.py (/tmp/radar_stop2lead.json) | **u≤0.42-0.44 → 0.5/0.45 判死,候选缩至 feet vs 0.25** |

方法论教训两条：**单调性检验必须跨段**（同段内新鲜度/时间共线都会造出伪相关）；
**单位标定必须有物理恒等式锚**（前车静止时 `−d(dist)/dt = vEgo`），日志内即可完成，不必等路测。

## 3. 最终字段定案

| 字段 | 定案 | 置信 | 关键证据 |
|---|---|---|---|
| 0x109 b7 | **连续前向目标距离**，u∈[0.25,0.44] m/计数，**候选缩至 feet(0.3048) vs 0.25 二选一**（④b τ 阶梯旁证 feet）；0=无目标哨兵，**247=丢失边缘瞬态码非持续超量程（④a 翻案），连续上限 246** | 量高/值未钉 | 单调 90.3%×2 组归档、distinct 240、领先 HUD 翻转 309/309、**13 起跟停事件硬上界 u≤0.42-0.44**、驻停平台 15c→feet 4.6m/0.25 3.7m、τ@feet={0.89,1.46,2.01}s≈工业阶梯 |
| 0x10A b3[7:5] | **目标存在位**：111=前向可见目标 | 高 | 全 384 位唯一干净分离：lead 96.5% vs no-lead 0.0% |
| 全 6 帧 b0 | **CheckSum = SAE J1850 CRC-8**（poly 0x1D/init 0xFF/无反射/xorout 0xFF，载荷 b1..b7） | 确凿 | 2933 唯一载荷 100% + 独立日 5 帧型 80100/80100 |
| 全 6 帧 b1 | AliveCounter 滚动计数（低 nibble 每帧 +1） | 高 | 跨帧恒 +1 规律 |
| 0x101/0x105 | 同一帧的**纯传输重发**（含 b0 全等 99.7-100%）；b7 是另一路**表状量化**信号：远距钉 0xD8、近带簇 29-34、时距假说证伪（④d） | 重发高/含义低 | 单调仅 24.8%、class-0 均值 30.6 非零、非钉值 corr(b7_109)≈0.60、corr(b7/vEgo)=-0.318 |
| 0x109 b6 | **纯二值状态 {0x7F,0xFF}，不含距离**（④c：粗细分解/平行递减证伪） | 中高 | 全库 676035 帧无第三值，仅 bit7 之差；lead 0xFF 49.8% vs class-0 1.9% |
| 0x102/0x106 | 传感器健康 A/B 对（b4 慢量 ~152/~103） | 角色中/义 UNKNOWN | 与 lead/no-lead 无关 |
| vRel | **不存在**于这 6 帧 | 高 | 全体字节对 −db7/dt 监督无一 \|r\|>0.4（486 段） |

**重要推论**：0x109 帧存在 10 段「ACC 未跟车但 b7/存在位非零」的真实目标（HUD 路看不见的增量）
→ 下游窗过滤规则=**存在位 AND 档位**双条件，只看 HUD 会丢真实目标。
④f 复核：假警尾段 11/11 ghost 窗 b7>0，原「存在位卡死 4 段」=静止前车驻停平台等真实可见目标——过滤规则收敛为 **presence==7 AND b7>0**。

## 4. 与既有定案的修订关系

- **维持（但 0x38x 已重开）**：0x35x 槽恒空、0x374 零帧、"宋无完整 track（无 vRel）"。**10-18 晚追加**：bus3 探索（`docs/byd-radar-rear-bus3-2026-10-18.md`）发现 0x380-0x38F/0x340s/0x3C8s 一族 **67.16ms + J1850 CRC-8 100%** 的多地址帧组（3935 万帧），段内与 0x109 b7 强相关（r=0.78-0.87）且**槽位随段动态分配**——原"状态表证伪"所用的跨段档位单调性检验对动态 track 表天然失效，当晚翻案完成并销账（`docs/byd-radar-slot67-2026-10-18.md`）：确为多目标槽表——全局周期计数器+J1850 CRC+全表广播，块 C（0x380+4k，6 对象×4 帧）F0 帧 x=((b3&0xF)<<8)|b2 12bit，窗内同对象回归 slope≈8×0x109 b7（1/8ft 刻度，若 b7=0.25m 则 1/32m，与 b7 单位之谜绑定），三设备独立复现，ACC 领先车 75% 在槽 1、18% 槽 2；块 B（0x340+3k，近程 1dm 粗表，lead<25m 偶合）。**负结果=relV 判死**：325 个 lead 门控窗内 F1-F3 全字节变体对关闭速度 |r|≤0.10——公开 CAN 无相对速度，track 缺口维持原结论；y/state/D 块语义开口，未入库（纪律：不确定不编造）。
- **后向定案**：日志无 src==3 流量（415 段全局，src 集合 {0,1,2,128,130,192}，后三者=panda LOOPBACK/REJECTED 回声）；0x418 实测 20Hz、b6/b7 为低位恒 0xF 的滚动计数器、798 次 L/R onset 无字节响应——**后方目标流在日志可达数据中不存在**，与手册"前 MRR 走 MPC 私有 CAN、两侧加塞=MPC 视频融合"互证。详见专档。
- **缩小**：§8.5.7「bus1 0x101-0x10A 皆状态广播」→ 仅对 0x102/106/10A 状态字节成立，0x109=距离。
- **新增**：`RADAR_HUD_BARS_TO_M{2:13,3:22,4:35}` 不可作米标定真值——HUD 档位是**时距带**
  （类内 corr(b7,vEgo)=0.70-0.95），8.5.4 档位→米标定的前提本身要重做。
- **社区 ARS410OD dbc 定位**：地址段 0x20-0x70 与宋 bus1 不相交，不可直套；
  其 CheckSum/AliveCounter 惯例是本次 CRC 破译的钥匙，两段式对象结构支持 0x109+0x10A 同目标配帧假说。

## 5. 落码与产物（commit ec0d3d04e7，已推 origin/dev-tici）

- **dbc**：`byd_general_pt.dbc` +BO_ 257/258/261/262/265/266（RADAR_DIST_109 `LongDist_109:63|8@0+ (0.3048)` + HYPOTHESIS CM_、RADAR_PRESENCE_10A `Presence_10A:31|3@0+`、CheckSum/AliveCounter、其余 MSG 占位零编造）。
- **控制**：radar_interface.py / values.py / 双路 track+档位回退 **一字未动**（路测门禁纪律）。
- **测试**：`selfdrive/car/tests/test_byd_radar_010x_dbc.py` 10 例（真 CANParser 位序/邻字节/重发一致性/地址冲突）；byd 套件 38/38（host 须 `pytest --noconftest -o addopts=`）。
- **工具**：10 个挖掘/验证脚本入库 `tools/byd/`（见 §2 表）。
- **文档**：本报告 + `docs/byd-radar-010x.md`（定案草案+补录+风险台账）。
- **数据**：`/tmp/radar_1005_1005.json`、`/tmp/radar_415_diversity.{txt,json}`、`/tmp/up_*.json`、`/tmp/radar_unit_pin.json`。

## 6. 风险台账（10-18 晚更新）

| # | 风险 | 状态 |
|---|---|---|
| 1 | 单位 0.3048 vs 0.25 未绝对钉 | **销(2026-10-19 owner 拍板)**——**u=0.25 m/count**(回忆实际跟停驻停距 15c≈3.75m 在 0.25 侧);feet 口径的 τ 阶梯形状旁证随之降级({0.71,1.02,1.4}s 偏小但通);dbc LongDist_109 factor 0.25+PINNED、[0|61.5]、测试 10/10;连带钉死:块 C x12=1/32m(量程 128m)、块 B=1dm、b7 极值 9c=2.25m/246c=61.5m |
| 2 | 0/247 哨兵语义 | **销（④a+④f）**——0=无目标已证；247 翻案=246↔0 丢失边缘瞬态码（全库仅 48 帧、episode 全 ≤0.5s），按「246 连续上限、≥246 不可信」处理；「存在位卡死 4 段」亦翻案为真实静止/可见未跟目标 |
| 3 | 多样性证据薄 | **销**——415 段+hour_logs_2 独立日复验完成 |
| 4 | 无 vRel | **维持（已定量）**——track 原语义不可覆盖；档 3 路（差分 vRel）**已由 radar_dvrel_quality.py 定量（2026-10-19）**：b7+0.4s med 滤波后稳态噪声≈0（281 窗）、13 起跟停事件跟踪 r=0.56-0.90/slope≈0.5-0.7（受 u 二选一换算）、检测延迟 0.24s（1.0s 窗→0.56s）——**够 RadarGapMonitor 级门控/预警用（与其落码选的 b7 平台斜率 0.2/s 等效互证），不够纵向 MPC 闭环**；块 C 12bit x 量化地板 8 倍优（0.57 m/s）但帧间抖动 ±4 LSB 使 med 滤波后仍 ~0.5 m/s（原始槽表抖动 vs b7=融合平滑值，差分应优先用 b7）；vLead/vRel 字段本身经 hour_logs 三通道补测穷尽判死（docs/byd-radar-hourlogs-vlead-2026-10-19.md） |
| 5 | b0 CRC UNKNOWN | **销**——SAE J1850 CRC-8 已破并双组泛化 |
| 6 | 0x101 语义 | 开口（④d 具体化）——表状量化、远距钉 0xD8、时距假说证伪；第二槽语义待建模，不可作 0x109 回退 |
| 9 | HUD_BARS_TO_M 依赖路径 | 开口——档位=时距带的结论尚未回写 values.py 注释（纪律：路测前不动） |

## 7. 下一步

1. **跟停标定**（唯一硬开口）：10-05 归档的 13 起跟停已把候选缩到 **feet 0.3048 vs 0.25**；下次路测实测一次跟停驻停距即可定夺（15c 平台→4.6m=feet / 3.7m=0.25）。247 观察项已被④a 解答（丢失边缘瞬态码），路测顺带确认 246 顶值场景即可。
2. **T8 落地**：BSD 发起前 clear 窗/starting abort 可升级用 0x109 连续量——**只做单调序与相对阈值**（比例于档位穿越提前量），米制阈值等①钉死后引入；过滤用存在位+档位双条件。**④e 设计数字已齐**（100% 领先、丢失路同拍、静态阈不可用），④f 过滤规则=presence==7 AND b7>0；**当晚 ⑤ 已落码**（radar_gap.py+CarState gap parser+capnp @61/62+desire_helper 三级 `BydRadarGapGate` 默认 0=行为零变更；415 段回放验收=误阻塞 8.3%、closing 预警 33%/23%、提前中位 4.8s；测试 18 例、runner 105/105），路测开 gate1/gate2 见 docs/byd-radar-010x.md §⑤。
3. **设备探针**（T7 残余）：**当晚 bus3 日志侧已定案**（`docs/byd-radar-rear-bus3-2026-10-18.md`：415 段 src 集合无 3；0x418=20Hz 纯结果位；后向数据源在公开 CAN 不存在，车侧为 MPC 视频方案）——`radar_bus_probe.py` 上机只剩"物理 bus3 接线可达性"一问可销，优先级降级；67ms 槽表专项**当晚已完成**（`docs/byd-radar-slot67-2026-10-18.md`：多目标表成立、块 C x=8×b7 细版、**relV 判死**→ track 缺口确认无法从公开 CAN 补，MPC 私有 CAN 引线（§6 风险#5/手册 P13 引脚）成为 raw track 唯一路径）。
4. 0x101/0x105 的"另一路 b7"与 0x109 b6/b7 平行递减（段偏置 -6/-12）继续建模（粗细分解 or 双目标）。**④ 已收口一半**：b6 纯二值、平行递减/粗细分解证伪（风险#8 销）；0x101 表状量化形态已明（远距钉 0xD8、时距假说否），第二槽语义（预测目标/旁车?）仍开口。**⑦ 全收口**：b3 判非横向（横移窗回归 slope 0.0 LSB/m）——分道输入死路，T8 level-2 新目标语义即终态。

## 7b. 当晚续探（④，10-05_07 全帧缓存六项，工具 radar_seg_extract + 5 分析）

前置 `radar_seg_extract.py` 一次解 415 段全 8 字节（0x109 帧 742451，/tmp/seg1005.pkl 132MB，0 解析错误），五项分析全部离线复用：
1. **④a 247 翻案**：非持续超量程——全库 48/742451 帧、10 episode 全 ≤0.5s、集中 246↔0 丢失边缘；连续上限 246（feet=75.0m）。1-8 空洞 539 个进入帧 0 命中复现。
2. **④b τ 阶梯**：档位边界 b7/vEgo 池化中位 {2.92,4.79,6.61} → τ@feet={0.89,1.46,2.01}s ≈ 工业 ~{1.0,1.5,2.0}s 阶梯、0.25 下不成套——**feet 旁证非定夺**（阶梯形状 u-不变）。
3. **④c b6 定分**：纯二值 {0x7F,0xFF}（676035 帧无第三值），粗细分解/平行递减证伪，风险#8 销。
4. **④d 0x101**：表状量化（远距钉 0xD8、近簇 29-34），时距假说证伪（corr=-0.318）——不可作 0x109 回退（结论加硬）。
5. **④e T8 统计**：b7 越旧/新平台中点**领先档位翻转 309/309(100%)**，翻转时刻仅走完平台 37-59%（提前量常超 8s 窗，判据须自适应回看）；丢失路 b7==0 与 HasLead 同拍（med 0.02s，clear 窗"消失"方向 b7 无提前量）；捕获路 b7 领先 ≥10s；单一静态阈误穿 29-60% → 迟滞+去抖必须。
6. **④f 存在位翻案**：fp>5% 的 11/11 ghost 窗 b7>0、CRC/帧率无恙；「卡死 100%」段 --234 = vEgo 恒 0 + b7=15 驻停平台（静止前车真实可见）——卡的是 HUD 不是雷达；过滤规则收敛 **presence==7 AND b7>0**。

## 8. 复现

```bash
# ①挖掘  ②跨段验证  ③回声  ④监督反解  ⑤钉定
.venv/bin/python tools/byd/radar_re_mine.py docs_site/hour_logs/radar_rlogs
.venv/bin/python tools/byd/radar_0109_validate.py docs_site/byd_logs_2026-10-05_07/realdata --workers 10
.venv/bin/python tools/byd/radar_415_diversity.py   # 见脚本头注释
.venv/bin/python tools/byd/radar_sup_mine.py extract && .venv/bin/python tools/byd/radar_sup_mine.py fit
.venv/bin/python tools/byd/radar_unit_pin.py docs_site/hour_logs_2 --pattern 'rlogs_flat/*.zst' --json /tmp/up_hl2.json
.venv/bin/python tools/byd/radar_unit_aggregate.py /tmp/up_hl2.json /tmp/up_hl.json /tmp/up_1005.json
.venv/bin/python tools/byd/radar_stop2lead.py docs_site/byd_logs_2026-10-05_07/realdata --workers 10   # 13 起跟停,候选缩至 feet vs 0.25
# ④ 续探:一次提取缓存,五项分析复用
.venv/bin/python tools/byd/radar_seg_extract.py docs_site/byd_logs_2026-10-05_07/realdata --workers 10 --out /tmp/seg1005.pkl
.venv/bin/python tools/byd/radar_247_census.py      # /tmp/radar_247.txt  247 翻案
.venv/bin/python tools/byd/radar_tau_fit.py         # /tmp/radar_tau.txt  τ 阶梯(feet 旁证)
.venv/bin/python tools/byd/radar_b6_model.py        # /tmp/radar_b6.txt   b6 二值/0x101 表状
.venv/bin/python tools/byd/radar_t8_lead.py         # /tmp/radar_t8.txt   翻转领先 309/309
.venv/bin/python tools/byd/radar_presence_fresh.py  # /tmp/radar_pf.txt   假警=真实可见目标
# 测试
.venv/bin/python -m pytest selfdrive/car/tests/test_byd_radar_010x_dbc.py selfdrive/car/tests/test_byd_radar_interface.py selfdrive/car/tests/test_byd_carstate.py -q --noconftest -o addopts=
```
