# BYD 校时调查：0x2B6 DATETIME 对齐（2026-10-21，byd-offline-prep C）

> 治「设备冷启动时钟退回 7-28、capture 文件名带错日期」。只出设计，不实施。
> 脚本 `local_docs/byd/offline_prep_2026-10-21/offline_stats.py --stream clock`，
> 产物 `stats_clock.json`（415 段全量）。解码口径沿用
> `docs/byd-carrot2-reference-2026-10-21.md` §7.3。

## 1. 信号事实（本车实测）

- 帧：**0x2B6 (694)** bus0，字节 0-5 = `YY MM DD hh mm ss`（2000+YY，**UTC+8 本地时间**），~2.4Hz。
- 字节变化谱 `....xx..`（census `full_census.json` 证 b4/b5 即 min/sec 会动、b0-b3 慢）。
- 有效性门：`0≤YY≤99 ∧ 1≤MM≤12 ∧ 1≤DD≤31 ∧ hh≤23 ∧ mi,ss≤59`（异常帧丢弃）。
- 6 段（10-18 引线并接实验期）有 bus2 镜像 `694,2`（109 帧）；分析按 bus0 为准。

## 2. 对齐结论（0x2B6 UTC 化 vs 同段 gps wallTime）

配对方式：按 logMonoTime 时序，每帧 0x2B6 减去「最近一帧 gps `unixTimestampMillis`」（UTC）。

| 表观差 \|veh−gps\| | 段数 | 占比 |
|---|---|---|
| ≤1s | 83 | 20% |
| ≤3s | 186 | 45% |
| ≤10s | 144 | 35% |
| >10s | 2 | 0.5% |

- 全库中位 **−1.5s**（负=车辆 DATETIME 略慢于 gps；gps 1Hz 采样 + 车辆秒截断的量级）。
- **414/415 段 ≤10s** → 0x2B6 作为校时**原料成立**：车辆 RTC 比设备 Linux 时钟可信得多。
- 两个 >10s 离群：`00000001--0`（−10.5s，轻度）与 `00000004--0`（+3.46e8s=约 11 年，实为
  该段 gps `unixTimestampMillis` 退化成 epoch 零附近的**坏 gps**，非车辆时钟问题）。
  → 落点实现须带 **gps sanity 门**（diff>1h 判定 gps 坏，不采用）。

## 3. 日切规则（关键，跨日样本表观）

**162/415 段**存在「本地日≠UTC 日」的帧（共 23097 帧）——即本地时区 00:00–08:00 区间，
UTC 仍是前一天。设备 epoch 是 UTC：

- 校时**必须带时区规则**：`epoch_utc = timegm(YY,MM,DD,hh,mi,ss) − 8*3600`，
  不能把 UTC+8 的年月日时分秒直接当 UTC 塞进 `settimeofday`（否则整表偏 +8h，
  在 16:00–24:00 本地时段会把日期推到次日）。
- 单帧只有秒级分辨率（无亚秒/无毫秒），校时精度上限 = ±1s + gps 采样延迟；
  够「文件名日期/日志归档」用，**不够**任何需要亚秒同步的用途（如 CAN↔camera 对齐——
  那仍走 logMonoTime）。

## 4. 落点候选对比（只设计，不选）

| 候选 | 描述 | 优点 | 风险 |
|---|---|---|---|
| **(A) loggerd 校时** | loggerd 启动读 0x2B6 若干帧中位数 → 校正系统时钟 | 全局受益（文件名+所有下游） | 改系统时钟影响已排队日志的 logMonoTime↔wallTime 映射；需防回拨 |
| **(B) 采集器打戳** | byd_long_capture 归档时用 0x2B6 覆盖文件名日期，不碰系统时钟 | 零副作用、可逆 | 只治文件名，rlog 内部 wallTime 仍错；paramsd 等消费方拿不到 |
| **(C) carState 透出** | carstate 解 0x2B6 → `carState.datetimeUtc`（新字段），下游各取所需 | 与本工程分层规范一致（BYD 逻辑独立）、供 (A)/(B) 复用 | 需加 capnp 字段换号；单点无消费者则空转 |

倾向：**先 (C) 透出做事实源，再 (B) 消费（低风险先落地）**；(A) 需单独评审时钟回拨安全性。
三者都不照搬 carrot2 的 `date -s` **subprocess shell 写法**（红线：card/采集路径禁 shell、
禁同步磁盘写；`system_time_valid` 判断思路可借，落地用 raw params + monotonic 护栏）。

## 5. 风险清单

- ❌ `subprocess.run(["date","-s",...])`：注入面 + 阻塞 + 时钟跳变打断实时线程。
- ❌ 无条件校时：坏 gps / 车辆 RTC 未跑起来（首帧 00:00:00）时会把设备时钟打飞；
  须 gps sanity 门（§2）+ 首帧哨兵值过滤 + 「只前进不回拨超过阈值」护栏。
- ⚠ 时区硬编码 UTC+8：车辆跨时区（出境/测试异地）即失效——本工程车辆固定国内，
  可接受，但实现里应把偏移做成可配 raw param 而非字面量。
- ⚠ 2.4Hz 无亚秒：秒级抖动 ±1s，别指望它做帧同步。

## 6. 验证方法（设备回桌后）

冷启动 → 记录设备 `date` 错值 → 跑 (C)+(B) 原型 → 比对新生成 capture 文件名日期
与手机时间；rlog 内 `carState.datetimeUtc` 与 gps wallTime 差应在 §2 分布内复现。
