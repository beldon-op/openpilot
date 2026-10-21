# BYD 离线路线图（2026-10-21 制定）

> 制定背景：车与设备均不可用（无法点火），但 415 段实车语料 + 普查产物在本地。
> 本文是「车回来之前干什么、回来之后按什么顺序干」的单一事实源；每过一个门禁就更新状态节。

## 1. 持仓与卡点

```
变更/线程                        离线 ──────────── 设备 ──────────── 车
byd-offline-prep                B ✓(基线清零)      —                 —
                                A ✓(结论:表未标定,保留) —            —
                                F/C/D/E ✓(四篇 docs) —                —
byd-bench-golden-replay(新)     全部可离线做掉 ✓   复核可选           —
byd-fingerprint-yaw-hardening   10/13             6.1 pull+重启      6.2 fingerprinted 日志确认
byd-radar-synth-lead-port ✓     代码完            —                  V1 gap 遥测 → V2 dRel 复核/标定
                                                                          → V3 alpha 跟停
打灯变道(3035fcf0)               E ✓(BSD 预验)     —                 路测
校时(prep-C 出设计)              设计稿 ✓          实现可在车外写     验证冷启动时钟
paramsd yaw 健康(a551f080 喂料)  D ✓(先验画像)     台架可看            路测首日观察
```

> **部署检查单（下次 OTA 内容 + 待办）**
> - 设备下次 OTA 包含：`a551f080`（活体指纹+yawRate）+ `d5fc77c4`（合成 lead）+
>   **本变更 byd-offline-prep**（B 基线修复 `07ba8cbe` + A `RADAR_HUD_BARS_TO_M` 注释加固）。
> - **点火验证仍待**（byd-fingerprint-yaw-hardening 侧，**不在本变更范围，此处不勾**）：
>   - [ ] 6.2 点火一次，日志确认 `fingerprinted=BYD_SONG_PLUS_DMI_22, source=can`
>   - [ ] 6.3 路测首日 paramsd liveParameters 健康 + bus1 点火窗帧数登记
>   - 以及 byd-radar-synth-lead-port 的 V1/V2/V3（车相关，见 §3）。
> - 本变更**不新增任何生产路径行为**（B 只动测试脚手架，A 只动注释；C/D/E/F 是 docs），
>   故 OTA 风险面与 a551f080/d5fc77c4 相同，未额外扩大。
> - **进车前置（新增，byd-bench-golden-replay）**：执行人在**自己机器**上
>   `.venv/bin/python scripts/byd_bench_gate.py` 得 `PASS(exit 0)` 方可 OTA。
>   `SKIP(3)`≠过（钉定语料不在本地——语料在 `corpus_base` 路径，别台机器需先取料）；
>   `REGRESSION(1)` 拦下并归因；`--accept` 必须带 drift_notes 出处。详见 docs/byd-replay-bench.md「门禁」。

结构性事实：**离线侧持续产出，下次 OTA 的风险面在越攒越大**（40ea446e 合并 + 合成雷达链 + 指纹加固将一次性进车）。BYD 红线：card 崩 = 0x316 断流 = EPS 锁，一次上车事故的成本 > 全部离线工作。所以路线图的第一原则是：**先把「进车前的验证」制度化（golden replay 门禁），再扩大离线建设**。

## 2. 候选与决策

| # | 候选 | 状态 | 价值 | 备注 |
|---|---|---|---|---|
| 1 | **byd-bench-golden-replay** 门禁化 | **已完成（2026-10-21，golden+gate 入库）** | ★★★★★ | bench 能力已入库（`scripts/byd_replay_bench.py`，M4 与设备逐位一致）；缺的是版本化 golden + 自动判定。golden 现值仍是 d5fc348 时代散文数字，HEAD 之后全链输出已「有意变化」，必须重立基线并记录每处差异出处 |
| 2 | ModifiedStockLong 原型（V3 备胎） | 定级=备胎（非升格） | ★★★ | prep-A 结论：HUD 表未标定**不影响 V2/V3**——那两级依赖 0x109 **b7 主源**（离线 dry-run 证形状可用，measured=False 闸使错标的 HUD 表从不进纵向）。备胎保持「V3 不过关才启用」，未因 A 结果升格。已登记 observations.md |
| 3 | 路测门禁会话计划 | prep 收尾时立项 | ★★★ | V1+6.2+paramsd+BSD 可并趟；车时最稀缺，编排值得半天 |
| 4 | V1 遥测报告生成器 | 随 #3 | ★★★ | 格式先定，日志回来即出报告 |
| 5 | 0x100 家族深挖 / specs 归档卫生 | backlog | ★★ | 不腐烂 |

依赖：`prep-A 拟合结果 → #2 定级`；`#1 完成 → 下次 OTA 有门禁`；`#3 依赖 prep-E/C 的产出物`。

## 3. 车/设备回来后的执行序列（现行版）

```
设备回桌：  6.1 pull/scp(=a551f080+prep-B+…) ──► golden bench 复跑(#1 产物) 过 → 待命
车回来：    趟1: 点火 6.2 fingerprinted 确认 + V1 gap 遥测(30min 跟车间隙)
            趟2: V2 dRel 复核(若 A 拟合达标只做验证；否则现场标定) + paramsd yaw 观察
            趟3: V3 alpha 跟停 + BSD 行为复核 + 校时验证(若已实现)
任意趟爆雷: 退路顺序 = 关 radarUnavailable(翻标志回 True) → 回退厂商纵向姿态 → ModifiedStockLong(#2 若建成)
```

## 4. 开放问题

- 设备大概何时回桌？若早于车：#1 验收加一档设备侧 process_replay 对照（文档已证两平台逐位一致，成本≈0）。
- 趟1 是否并 6.2 与 V1（省车时 vs 故障归因难度）——倾向并趟，fingerprinted 事件有独立日志行可分归因。

## 5. 状态日志

- 2026-10-21：路线图制定；prep-B 完成（另 session 勾选 1.1-1.3）；#1 提案 byd-bench-golden-replay 创建。
- 2026-10-21（本 session）：**byd-offline-prep 全部离线任务完成（18/18）**：
  - B 基线修复 `07ba8cbe`（-k byd 6 失败清零，透传语义核对=无真实回归）。
  - A 对撞回归：**四档全部不满足更新判据**（跨路线偏差 72-105%、vEgo 单调漂移、
    dry-run HUD 残差中位 −14.75m）→ 表值保留 + 「UNCALIBRATED」注释 + docs/byd-radar-010x.md ⑪节；
    「档×速」二维升级提案登记 observations.md（低优先，measured=False 闸使其不进纵向）。
  - C/D/E/F：`docs/byd-datetime-clock-*.md`（校时原料 414/415 段≤10s，须带时区日切规则）、
    `docs/byd-yaw-axay-dataset-*.md`（50Hz 钉死、|bias|<0.5°/s 达 83%）、
    `docs/byd-bsd-quality-*.md`（flicker 0.5%、静止/开门零误报、误放不可离线测）、
    observations.md（ModifiedStockLong + V1-V3 不归档）。
  - 部署检查单见 §1；#2 定级下调为「备胎（非升格）」。
- 2026-10-21：**#1 byd-bench-golden-replay 完成**：golden 版本化
  （`scripts/byd_bench_golden.json`，钉定双段 sha256+全精度指标+归因表）+ 门禁
  `scripts/byd_bench_gate.py`（PASS/REGRESSION/FLAKY/哈希不符=1/SKIP=3，--accept 强制归因）；
  散文基线自此由 golden 取代。归因表：全链 43.851→43.843 归 `d5fc77c4`（抽查复跑坐实）；
  card-only 逐位复现旧基线。**实现揭示并修复测量具失效**（design D3.1：40ea446e 后
  Mac 垫片顺序失效 + LateralStartupGate 台架永不导致两档链退化逐位相同；经批准在
  scripts/ 内修复，不碰控制路径）。
  遗留登记（5.3）：钉定语料在 `corpus_base` 本地路径、不在仓库——**别台机器跑出 SKIP(3)
  不等于过**，OTA 检查单执行人须在自己机器上取料并跑绿一次。
