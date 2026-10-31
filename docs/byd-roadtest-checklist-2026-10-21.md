# BYD 路测门禁检查单（V1/V2/V3 + 随行观察项）

> 用法：每趟车前对照本单，项 = **动作 → 看什么 → 判据 → 结果记哪**。
> 判据数字全部来自离线定案（docs/byd-radar-010x.md、byd-roadmap-offline-2026-10.md §3、
> byd-lateral-vendor-model.md 末节、byd-lane-change.md T4/T9），车上只验证、不即兴改码。
> 结果统一回填：本单勾选 + `docs/byd-radar-010x.md` 状态行 + roadmap §5 状态日志 +
> `openspec/changes/byd-radar-synth-lead-port/observations.md`。**V1→V2→V3 顺序不许跳**；
> 任意趟爆雷走「退路」节，不要现场修控制路径。

## 每趟必做（先于一切检查项）

- [ ] 上车前 `ssh` 设备确认 `pgrep -f byd_long_capture` 活着 + `/data/carrot/capture/` 有当日 jsonl（开机自启靠 continue.sh→watch；手动补拉 `cd /data/openpilot && PYTHONPATH=/data/openpilot setsid nohup /usr/local/venv/bin/python3 /data/carrot/tools/byd_long_capture.py >>/data/carrot/capture/capture.log 2>&1 &`）
- [ ] UI 健全性两件套（无摄像头设备前提）：不报 Communication Issue、角标显示 **DM OFF 而非恒显 DM CHECK**
- [ ] BYD 红线：行车中**不做任何让 card 停发的动作**（重置校正/升级/onroad 参数生效重启）——断 0x316 = EPS 锁 TorqueFailed，需熄火解锁（docs/byd-control-lessons.md）
- [ ] 停车搓 D/R 前先按 CANCEL（厂商会话僵尸态规避，第五趟定案）
- [ ] 收车：jsonl 切段按 `"k":"start"` 行；设备断电前拷走；文件名日期可能因冷启动时钟错误，别按名归类

## 趟 0 —— 进车前置（不用车，本机/设备侧）

- [ ] **bench 门禁**：`git commit+push` 工作区三件（bench_gate/golden/replay_bench 改动）；执行人**在自己机器**上取语料（corpus_base 路径）后跑 `.venv/bin/python scripts/byd_bench_gate.py` 得 `PASS(exit 0)` 方可 OTA。`SKIP(3)≠过`；`REGRESSION(1)` 拦下归因；`--accept` 必带 drift_notes 出处
- [ ] 设备 6.1：pull/scp 到含 `a551f080 + d5fc77c4 + 07ba8cbe + 992d6d64`（github 直连不通则 scp；auto_update 关）
- [ ] **重部署采集器**（记忆里设备 /data/carrot/tools 曾丢失）：`scripts/byd_long_capture.py` + `byd_long_capture_watch.sh` 上设备，验证 continue.sh 自启生效
- [ ] 参数初态核对：`BydRadarGapGate=0`、`AlphaLongitudinalEnabled=0`、`BydRadarDistScale` 缺省（0.25）、`DriverMonitoringEnabled=0`、`BydBlinkerStalkSwap` 按现状

## 趟 1 —— 点火 + 30min 跟车间隙（V1 gap 遥测 + 指纹/yaw 6.2/6.3 + 钉 u）

行为全部零变更（gate=0、stock 纵向），本趟纯观察取证。

- [ ] **6.2 指纹**：点火一次，日志确认 `fingerprinted car_fingerprint=BYD_SONG_PLUS_DMI_22, source=can`（**非** fixedFingerprint）；失败≠退车，记录 source 值回来归因
- [ ] **6.3 paramsd**：路测首日 liveParameters 健康（0x222 yawRate 已接线，实测 50Hz、|bias|<0.5°/s 达 83%），无新增 onroad alert
- [ ] **bus1 点火窗帧数**：byd_long_capture 记 0x109/0x10A/0x351-354 点火到上电完成窗的帧数（design 待解决问题 1）
- [ ] **V1 gap 遥测对表**：jsonl 里 `radarGapClosing`/`radarGapNew` 密度与离线预测对表——提前量中位 4.8s、closing 预警 22 次/h、新目标 41 次/h、稳态误阻塞 ~8%；**量级差一个数量级=解析或信号漂移，停下来回来查**
- [ ] **钉 u（一次跟停即可定夺）**：安全跟停后实测车前距（真实尺子/地砖）：驻停平台 b7=15c 对应 **≈3.7m → u=0.25 成立**；**≈4.6m → u=feet**，回来改 `BydRadarDistScale`（radarcan 重启生效，不动 CarParams）+ dbc factor + 测试同步
- [ ] 危险形态观察：「自车在动 + b7 冻结」（全库 0 例的 ghost 形态）——出现即截图/记时间戳，这是新证据
- [ ] **厂商纵向姿态体检**（跟车 20min）：顿挫应已消失（catch-up/MB 两修）；接管/交还瞬间无 ±0.5+ 阶跃体感；「ACC 功能受损」若再现，记录是否伴随 rns=0+车速>1 的 0x10 僵尸帧（兜底告警议题的实据）

## 趟 2 —— V2：stock ACC 下 dRel/vRel 同屏 + 观察项 a/b/c + 横向/变道随行

`AlphaLongitudinalEnabled=0` 不动（行为中性，纯读取雷达链）。

- [ ] **V2 主判据**：swaglog/开发者面板看 `radarState.leadOne.dRel` 与真实车距同屏对照（已知车距场景 2~3 次：跟停驻停、低速跟、40+ 巡航）；b7 主源形状应 ±1~2m 内跟随
- [ ] **观察项 a**：统计 `leadOne=None` 占比，与翻转前（纯 vision）比——b7 246↔0 丢失边缘 + vision 低置信时 leadOne 可短暂 None（本仓 `vision_only_lead_allowed` 强可见门所致）
- [ ] **观察项 b**：BYD 点从不设 aLead/jLead（快照缺失读 0.0 喂 LeadDynamics），看 `aLeadK` 曲线是否呆滞/跳变
- [ ] **观察项 c**：HUD 档层丢弃生效——liveTracks 有档位点（UI 显示用）、radarState 不消费（measured=False 闸）
- [ ] HUD 错标登记：UI 距离读数 vs 真实（占位表已知系统性偏大、档=时距带）——偏差样本顺手记进「档×速」二维表提案，**不当门禁**
- [ ] **横向 A+B 五点**（docs/byd-lateral-vendor-model.md）：①直道 0x316 幅值比旧版小且无左右摆 ②缓弯左强右弱不对称感符合预期 ③<29km/h 无增益突跳 ④EPS 会话接受瞬间无可感台阶（有→记下来，steer_mode 换带滞回版） ⑤capture 里 torqueState desired/actual 齐全
- [ ] **变道 T4**：拨灯信号在场确认（左=集合 2/3、右=4/5 回归）；旧日志无拨灯记录只欠这条
- [ ] BSD：记录 0.5s clear 窗是否频繁延误发起（BSD 占 19.66% 帧）——高频则回来重定 `LANE_CHANGE_CLEAR_TIME_MIN` 或改用 0x418 档位语义

## 趟 3 —— V3：alpha 纵向跟停 + GateGate 升档 + 变道 T9

- [ ] **开关方式**：`AlphaLongitudinalEnabled` 停车熄火后改、下次点火自然生效（**禁** onroad cycle，红线）；先决条件=趟 2 全绿
- [ ] **V3 主判据**：合成 lead 进 MPC 后的跟车/减速/跟停收敛；重点=**lead 瞬变**（前车急加/急切）时 vRel 0.24s med 窗延迟的体感（稳态≈0 / 跟停 r=0.56-0.90 为离线参考）
- [ ] standstill→起步走 `LongCtrlState.starting` 的 ResumeFromStandstill 脉冲正常（会话内属司机语义）
- [ ] **GateGate 升档**：`BydRadarGapGate=1`（clear 窗+禁发起）先跑一趟看阻塞体验与告警时机；满意再 `=2` 验证「新目标出现且逼近→starting abort」语义，并确认「已有前车逼近**不**退」是期望行为
- [ ] **变道 T9 三行为**（docs/byd-lane-change.md §8.2）：①满窗才起 ②途中亮盲区=回原车道+blocked 提示 ③连续变道二次确认
- [ ] TOO_CLOSE→−4 急刹映射仍**不开**；只观察 CloseWarning 位触发场景，攒实据
- [ ] 校时验证：仅当 prep-C 设计已实现才跑（冷启动时钟对齐 0x2B6）；当前状态=设计稿，**跳过**
- [ ] C 档（friction_enhancer）裁决：趟 2/3 横向若有「不平衡感」证据→立项移植，否则销账

## 趟 2/3 随行 —— 槽表层（BydRadarSlotTable）V2 观察项

多层链第 4 层（bus1 67ms 多目标槽表，openspec `byd-radar-multitarget-port`）。
默认 level=1（显示级）：槽表点进 liveTracks（measured=False，radarState 不消费）——
行为与 b7-only 逐字节一致，本节纯观察。**控制消费二段（level=2）只在前项全绿后启用。**

- [ ] **默认零变更回归**：level=1 下 `radarState.leadOne/leadTwo` 候选与 b7-only 现状一致（离线 diff 报告先行，车上复核 leadOne 无新增切换）
- [ ] **leadOne/leadTwo 切换频率**：开启 level=2 前先统计 level=1 显示流里 trackId 201+ 的生命周期（槽位重排 generation 跳变率）——抖动大则先调 `PAIR_MAX_JUMP_M`/`TRACK_MISS_CYCLES`，不带着闪烁上控制
- [ ] **cut-in 捕获时延**（level=2）：邻车道逼近目标从进表到成为 leadTwo 候选的时延，对照 vision 确认时刻
- [ ] **track 抖动观察**（level=2）：dRel 序列 vs b7 同目标对照（1/32m 细分辨率），差分 vRel 的 0.47 m/s 量化台阶是否被 KF 平掉
- [ ] **横向字段状态**：yRel 仍为 NaN（未定案隔离）；阶段一定案入库后本项升级为「yRel vs 视觉 leadsV3 y 同屏」
- [ ] **回滚演练**：`BydRadarSlotTable=0` → radarcan 重启 → liveTracks 无 201+ 点、radarState 与 b7-only 一致（零残留）；确认切换未触发 CarParams 重生成（红线）

## 退路（任意趟爆雷，按序，不即兴）

1. **雷达链出事**（假 lead 进控制/leadOne 异常）：`radarUnavailable` 翻回 True——interface.py:141 一行，是回退到 d5fc77c4 前零变更状态的**唯一开关**
2. **纵向出事**：关 `AlphaLongitudinalEnabled` → 回厂商纵向姿态（「跟车准」最低代价方案）
3. 两级退路都不够：ModifiedStockLong（roadmap #2 备胎，届时才建）
4. EPS 已锁（LKAS Fault）：熄火解锁 + docs/byd-panda-recovery.md；回滚后 `TestBydApplyNeverRaises` 必须先过再上车

## 收车后

- [ ] 数据回填：jsonl+rlog → 本机；V1 报告按 roadmap #4 格式（可先定生成器再攒数据）
- [ ] 更新三处：010x 状态行（u 定夺/V 级通过与否）、roadmap §5 日志、observations.md
- [ ] **归档纪律**：V1/V2/V3 全过之前，`openspec/changes/byd-radar-synth-lead-port` 不归档

## 参数速查

| 参数 | 位置 | 生效方式 | 现值 |
|---|---|---|---|
| `BydRadarGapGate` | desire_helper_byd 三 hook | card 重启（下次点火） | 0（关） |
| `BydRadarSlotTable` | radarcan.py 注入 RI，clamp [0,2] | **重启 radarcan** | 1（显示级；2=控制消费二段） |
| `BydRadarDistScale` | radarcan.py 注入 RI | **重启 radarcan**，clamp [0.1,0.5] | 0.25（钉定待实测） |
| `AlphaLongitudinalEnabled` | interface `_get_params`（safetyParam bit2） | 下次点火；禁 onroad cycle | 0 |
| `DriverMonitoringEnabled` | selfdrived | — | 0（无摄像头） |
