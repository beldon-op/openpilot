# BYD 宋 hour_logs 控车段 relV/vLead 补测 + 距离族跨固件复验(2026-10-19 第三轮专项)

> 一句话:**距离族在 9-28 旧固件上独立复验成立(0x109 b7 + 67ms 块 C x + 头帧 0x301 伴生距离量),67ms 槽表固件时间线前推至 9-28(0x3E4-0x3FF 恒值占位帧已在广播);但 vLead/vRel 三通道全灭**——旧 relV 判死所用的「关闭速度」监督在稳态跟车段方差退化 ~0、存在漏检绝对 vLead 字段的盲区,本轮用 vEgo 全量/去趋势 vEgo/驻停窗三条新监督通道补测 4 段 × 110 地址 × 2420 字段变体,**无任何稳定速度族**(最强候选 0x381/b3 窗内 r=−0.87~−0.97 定性为时间趋势混杂:slope 每窗 −45/−21/−13/−8 不守恒、r(t)≈r(v)、r(aEgo) 弱、与 0x10A b3 不同源)——「公开 CAN 无相对速度」从单监督盲区升级为**三通道全灭**,与 slot67 专档 §3 互锁。控制/dbc 零变更。

## 1. 动机与盲区补测设计

- 既有 relV 判死两轮(0x10x 486 段:对 −d(b7·u)/dt 无 |r|>0.4;slot67 325 窗:|r|≤0.10)的监督**全部是关闭速度**。稳态跟车时关闭速度≈0、方差退化,任何**绝对 vLead 字段**(ARS410 类雷达 vRel/vLead 双发)对该监督系统不可见。
- hour_logs 4 段恰是长稳态跟车(route04 seg07-16 连续 10.6min、route08 8 段 10.5min、route09 14min 66-72km/h、route0a 2.3min)——是 vLead 盲区假说的**最理想测试床**:若某字段编码 vLead,稳态跟车窗内 r(f,vEgo) 应极高。
- 五通道监督(全部字段变体:单字节 ×8、相邻字节对 LE/BE u16/i16 ×28、12bit 槽装配 ×3):
  | 通道 | 监督 | 检验对象 | 伪影判别 |
  |---|---|---|---|
  | r_dist | corr(f, b7) @lead 窗 | 距离族复验 | — |
  | r_vlead | corr(f, vEgo) @lead 窗 | vLead(稳态 vLead≈vEgo) | 受 vEgo 时间趋势污染 |
  | r_hf | corr(f, 去趋势 vEgo) @lead 窗 | vLead 高频分量 | **计数器/慢积分量在此通道归零** |
  | r_still | corr(f, vEgo) @驻停 lead 窗(窗内去均值) | vRel(驻停目标 vRel=−vEgo,期望 r≈−1) | 窗内 slope 守恒性 |
  | r_dot | corr(df/dt, −vEgo) @驻停窗 | 距离场速率一致性 | — |
- lead 门控 = presence==7 AND 0<b7≤245(既有定案过滤);驻停窗 = b7 ±2s 平坦(≤1c)+vEgo>0.8。

## 2. 正结果:距离族跨固件复验 + 槽表时间线前推

- **9-28 固件 67ms 槽表族完整在播**(全 89 地址 14.9Hz 广播、0x3E4-0x3FF 恒值占位帧活跃,10-05/07 上它们多为静默)——槽表非 10 月新固件引入,至少 9-28 已存在;两者帧率/块结构一致。
- **块 C 对象#1 距离复验**:0x380 x12lo((b3&0xF)<<8|b2)对 b7 全局窗 r=+0.744(f4_16+fa_2,2695 lead 帧);slope=5.24 低于 10-05/07 的 7.9-8.6——**动态槽位跨窗切换稀释**(hour_logs lead 窗连续 10+ 分钟、槽位重排机会多),与「同对象锁定窗 R²>0.9、slope≈8」证据链不冲突,窗内同对象回归判据维持 slot67 专档原文。
- **新发现:头帧 0x301/b67LE 携带伴生距离量**(r_dist=+0.724,f4_16 单段)——头帧的 b6b7 对与 b7 相关,候选块 C 汇总/最长目标回显,登记开口,不入库。
- 块 B 0x341/b7 r=+0.568(近程粗表在 hour_logs 亦有耦合,稀疏窗样本少维持原定性)。
- 0x109 b7/presence==7 与 HUD HasLead 一致性再验(f4_16 lead 1122 帧、fa_2 1573 帧;HUD HasLead=1 帧更多=HUD 平滑,雷达原始更敏感,与 415 段结论同向)。

## 3. 负结果:relV/vLead 三通道全灭(本轮核心)

- **2420 变体 × 5 通道扫描无任何稳定速度族**:r_vlead 通道 top1 是 0x109/b7 自身(r=+0.843,伪相关=跟车距离与 vEgo 共变)、r_hf 通道 top1 |r|=0.672 且仅 1 段(0x34a/b23BEs,复查为窗内时间趋势)、r_still 窗内去均值后无 |r|>0.7 且 slope 守恒的候选、r_dot 全 |r|≤0.32。
- **最强候选 0x381/b3(块 C 对象#1 F1 帧)定性为时间趋势混杂**:
  - fa_2 驻停窗内 r(vEgo)=−0.93/−0.89/−0.98/−0.97 非常诱人(方向恰是 vRel=−vEgo);
  - 但逐窗 slope = −44.8/−21.0/−12.8/−7.7 LSB/(m/s) **不守恒(6 倍漂移)**——真 vRel 刻度必须稳定;
  - 且窗内 r(t)(对纯时间)与 r(v) 大小相当(+0.99/−0.90/+0.96/−0.96)、r(aEgo) 弱(≤0.2)、与 0x10A b3 同源性散乱(−0.85~+0.58);
  - → b3 是「随时间单调漂移的慢变量」(与 ⑦ 对 0x10A b3 的「缓增量、慢偏置类」判定同构),**非 vRel**。
- **结论升级**:「公开 CAN 无相对速度」原判死只覆盖关闭速度监督(存在理论盲区);本轮补上 vEgo 全量/去趋势/驻停窗三通道后仍全灭,**盲区假说证伪,判死从单一监督升级为穷尽四类速度语义监督**(关闭速度/vEgo 绝对/高频/驻停锚)。track 缺口唯一近似维持:差分 0x109 b7(风险#4 原定性,噪声大仅备选;hour_logs closing 事件仅 f4_16 125 帧、样本不足以再标定)。
- **推论维持**:厂商 MPC 从私有 CAN 取 MRR 多普勒、公开网只发距离族(0x109 粗/块 C 细/块 B 近程)——信息边界是设计使然。

## 4. f8_8/f9_12:自由巡航基线段(新角色登记)

- 两段(10.5min 8-14m/s、14min 0-9m/s 高速)全程 presence==0、b7 恒 0、HUD HasLead 恒 0——**无 lead 自由巡航**,不能用于目标字段挖掘,但天然是 **free-space 基线**:雷达目标通道(0x10x、67ms 族)在无目标时全静默复验成立;后续伪影审查(虚警/恒虚警检出率)可用这两段作对照。

## 5. 新地址登记(9-28 固件在播、既有家族之外,census 全列)

| 地址 | 频率 | 内容速查 | 判定 |
|---|---|---|---|
| 0x100 | 10.0Hz | b0 计数器+b1/b3 慢变+b7 恒 FF,uniq~110 | 5 通道无监督信号,登记不动 |
| 0x103/0x104/0x107/0x108 | 25Hz | 0x10x 家族伴生帧,uniq=80,与 0x101/0x105 重发结构不同 | 语义开口,已扫无信号 |
| 0x302 | 12.5Hz | 头帧 0x301 伴生,uniq=16 | 语义开口 |
| 0x350-0x354 | 25Hz | tangdm 布局槽位**非恒空**(熵 1.58,uniq 80/16),但内容与 0x10x 不同源(帧差 64/96 LSB)、track 字段全 0 填充维持 | 「0x35x 私有槽无目标流」结论不变 |
| 0x569 | 1.0Hz | 低熵慢帧 | 登记 |
| 0x7fe7a00/0x1bfe7a1a/b | 25-30Hz | panda 回声/内部地址族(src 集合 {128,130,192} 同 52ad 提交) | 非车总线帧,不追 |

## 6. 与既有定案/落码的关系

- **slot67 专档 §3(relV 判死)**:本轮为它补上理论盲区(vLead 对关闭速度监督不可见)的检验,vEgo 三通道全灭后判死穷尽化——风险台账 #4 维持,描述可由「无 vRel」补强为「无 vRel/vLead(四类监督穷尽)」。
- **010x 专档「无 vRel」条目**:同上补强,不动字段假说表结论。
- **RadarGapMonitor(⑤ 已落码)**:块 C x 理论上可提 gap 分辨率——单位与 b7 绑定未钉、门禁纪律不动,维持原文登记。
- **dbc**:零变更(无新可入库字段;0x301/b67LE 伴生距离量未达「不确定一律 MSG 占位」的入库增益线)。

## 7. 复现

```bash
# 布局:fault_rlogs 平铺 zst 软链到 census 期望结构(一次性)
for s in f4_16 f8_8 f9_12 fa_2; do
  mkdir -p docs_site/hour_logs/radar_rlogs/$s
  ln -sf ../../fault_rlogs/$s.zst docs_site/hour_logs/radar_rlogs/$s/rlog.zst
done
# 全 bus1 census(地址/频率/熵 vs 已知家族)
.venv/bin/python tools/byd/radar_hour_census.py census docs_site/hour_logs/radar_rlogs
# 全帧+监督缓存(vEgo/aEgo/HUD) 4 段 ~9MB
.venv/bin/python tools/byd/radar_hour_census.py extract docs_site/hour_logs/radar_rlogs /tmp/hour_bus1.pkl
# 五通道监督挖掘(本文 §1 表,输出 §2/§3 全部数字)
.venv/bin/python tools/byd/radar_hour_mine.py --top 20
# §8 hour_logs_2 扩展轮(67 段,平铺 rlogs 软链同法;支持 workers)
.venv/bin/python tools/byd/radar_hour_census.py extract docs_site/hour_logs_2/radar_rlogs 10 /tmp/hl2_bus1.pkl
.venv/bin/python tools/byd/radar_hour_mine.py --cache /tmp/hl2_bus1.pkl --top 12
```

## 8. hour_logs_2 扩展轮(2026-10-19 夜,67 段全量 672M 独立语料,面向「雷达支持纵向控制还缺啥」)

> hour_logs_2(9-28 晚静默守卫验证会话,路由 0e-16,夜间驾驶+锁存重启场景)此前仅做过 10 段 5 帧单位复验;本轮首次全 bus1 挖掘(110 地址与 hour_logs 完全同集,同固件),回答纵向控制缺口清单。

### 8.1 relV/vLead 判死第三次独立复验(维持)
- lead 富集段 R15_001/002、R10_000、Rf_013、R13_001 等 5 通道全字段扫描:top 信号全部可定性——0x7fe7a00/b1 对 vEgo r=−0.83 为 panda 回声帧(非车总线,52ad 已判不追);0x344/b7 VREL 通道 r=+0.845 为跨窗趋势伪影(正号,期望 −1,单段);0x341/x12lo hf r=+0.606(3 段)为距离场的物理间接相关(vEgo 残差→closing 残差→距离残差);0x34a/b4 vEgo 全量 +0.764 但去趋势 −0.610 符号翻转=vLead 假说要求同号,排除。
- **三语料(10-05/07 415 段、hour_logs 4 段、hour_logs_2 67 段)× 四类速度语义监督全灭,判死终局。**

### 8.2 presence 枚举语义细化:非零即有效,但 7 才是「领先目标输出」
- 全 67 段统计:pres∈{2,4,5,6,7} 共 4306 帧 b7 有效性(0<b7≤245)**100%**,pres=0 时 0.01%(108052 帧)——**「存在位非零 ⇒ 距离有效」跨语料定律**(hour_logs 有 pres=1 共 3 帧同律)。现行 pres==7 门控在挖掘层面漏掉 +46% 有效数据(2957→4306)。
- 但 Rf_013 同段 HUD 同档内:pres=5/6 的 b7 均值 167/186 vs pres=7 的 103——**同时段系统性不同 ⇒ presence 是多目标选择器状态**(7=ACC 锁定领先目标的输出距离;5/6=其它可见/候选目标;迁移链 0→6→7、7→0、5↔6、4↔5 支持状态机/选择器两读,歧义未分)。
- **控制语义维持 pres==7**(b7 在非 7 态可能属于别的目标,并入会引入非领先距离);登记项:RadarGapMonitor level2「新目标出现」可从「pres 0→7」提前为「pres 0→非零」沿(更早的新目标进入视野信号,窗口更灵敏,路测后决定)。
- HUD-lead 而 pres=0 的窗(R15_004 457 帧、R15_001 287 帧等):HUD 平滑滞后/视频融合保持窗,与既有「雷达原始更敏感」定案同向。

### 8.3 纵向控制缺口清单(本轮结论汇总)
| 项 | 状态 | 来源 |
|---|---|---|
| dRel 距离 | ✓ 0x109 b7(25Hz)+块 C x12(15Hz 8×细分)+块 B(近程) | 三语料复验 |
| 目标存在/有效性 | ✓ presence 非零 ⇒ b7 100% 有效;7=领先输出 | §8.2 本轮 |
| standstill/驻停 | ✓ b7 平台+存在位(13 起跟停事件) | radar_stop2lead |
| 目标丢失边缘 | ✓ 0=无目标、247 瞬态、246 连续上限 | ④a/④f |
| vRel 原始多普勒 | ✗ 三语料×四类监督穷尽判死 | §8.1 本轮 |
| vRel 派生 | ✓ b7+0.4s med:稳态噪声≈0、0.24s 延迟(门控/预警级够用,MPC 闭环边缘) | radar_dvrel_quality |
| vLead/aLead | ✓ vLead=vEgo+派生 vRel;aLead 由 vLeadFilter 滤波导出(openpilot 惯例,无需原始字段) | — |
| **u 单位** | ✓ **已钉 0.25 m/count(2026-10-19 owner 拍板,回忆实际跟停驻停距 15c≈3.75m)**;dbc factor+PINNED、测试 10/10;连带块 C x12=1/32m、块 B=1dm、b7 极值 9c=2.25m/246c=61.5m | 风险#1 销 |
| 目标 ID/多目标 | 部分:槽表动态槽位无稳定 ID,ACC 领先 75% 在槽 1 可作启发式;presence 选择器语义歧义开口 | §8.2 |

**一句话:纵向控制的数据侧已齐——距离/存在性/驻停/丢失全复验,vRel 可派生(门控级),真正卡脖子的只剩 u 单位一次路测标定;原始 vRel 缺口确认由 MPC 私有 CAN 设计使然,公开总线挖掘到此收口。**

### 8.4 布局与工具变更
- hour_logs_2 软链布局:`rlogs_flat/R{route}_{seg}.zst` → `radar_rlogs/<seg>/rlog.zst`(同 hour_logs 法);软链目录不入库。
- `radar_hour_census.py`:load_all 加 Pool 并行 + `extract <logdir> [workers] <out.pkl>` 参数序;`radar_hour_mine.py`:加 `--cache`。控制路径与 dbc 零变更。


数据:/tmp/hour_bus1.pkl(4 段全 bus1+监督)。工具:radar_hour_census.py(census+extract)、radar_hour_mine.py(5 通道挖掘)入库 tools/byd/。控制路径与 dbc 零变更。
