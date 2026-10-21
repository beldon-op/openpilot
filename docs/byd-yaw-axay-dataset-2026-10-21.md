# BYD yaw/AXAY 数据包（2026-10-21，byd-offline-prep D）

> a551f080 刚把 0x222 YAW_RATE 接进 carstate（`byd/carstate.py:185-190`，
> `ret.yawRate = YawRate − YawRateOffset`，零编码哨兵置 0）。本包为 paramsd 上车前
> 备好 0x222/0x223 的数据画像。脚本 `offline_stats.py --stream yaw`，产物 `stats_yaw.json`。

## 1. 帧率口径复核（钉死 50Hz）

- 0x222 实测 **中位 50.0Hz，p05=49.9 / p95=50.1**（362 段）→ carstate 频率表用 50
  （`byd/carstate.py:326` 注记成立），**别信标称 60Hz**。

## 2. 表观 yaw bias（0x222 vs livePose 融合 yaw）

直线窗口判据：`|steeringAngle|<2° ∧ v>3m/s ∧ |aEgo|<0.5 ∧ |steeringRateDeg|<5°/s`。
残差 = `0x222 yawRate − livePose.angularVelocityDevice.z`（插值到 0x222 的 50Hz 网格，转 °/s）。
全库累计直线段 **11227s（中位 31.1s/段）**。

| \|表观 bias\|（各段 resid 中位数） | 段数 | 占比 |
|---|---|---|
| <0.05 °/s | 72 | 20% |
| <0.1 °/s | 130 | 36% |
| <0.2 °/s | 230 | 64% |
| **<0.5 °/s（paramsd 期望线）** | **301** | **83%** |
| ≥0.5 °/s | 61 | 17% |

- 全库 bias 均值 **−0.086 °/s**（略负=0x222 系统性偏左一点点，量级可忽略）。
- 读法：livePose 已是**去 bias 的融合输出**，故「0x222 − livePose」≈ paramsd 需要
  从 0x222 侧扣掉的 bias 代理量。83% 段落在 0.5°/s 内 → **0x222 静态/低速零点健康**，
  适合作 paramsd 的第三传感输入。
- **61 段 ≥0.5°/s 不作失败判**：多为该段 paramsd 尚未收敛（首几分钟）、或直线窗里
  gps 航向不可用致 livePose 退化。上车 paramsd 观察时须区分「0x222 自身偏」与
  「livePose 未收敛」——本离线包只给分布，不给逐段归因（归因要 paramsd 日志）。

## 3. 0x223 Ax vs KF aEgo（仅报告，不判定）

- **相关系数中位 0.47**（361 段），Ax 对 aEgo **斜率中位 0.31**（≪1）。
- 弱相关/低斜率是**预期内**，不作质量结论（design D4 明示仅报告）：KF aEgo 是重平滑
  + 滞后的估计，0x223 Ax 是原始 ESP 纵减（含坡度项 + 自身 offset），二者时域结构不同；
  斜率 0.31 说明 Ax 的 DC/坡度偏置占比大，不能直接当 aEgo 用。
- AXAY **在 carstate 无消费者**（`byd_general_pt.dbc:546-547` 注册了但「NO consumer yet」）。
  本包只是画像，接线与否属未来变更。

## 4. 口径与边界

- 解码沿用 carrot2 §7.3/§7.6（0x222 raw12b@0·0.002133−2.094 减 raw12b@12·0.002133−0.13；
  0x223 raw12b@0·0.027167−21.593），**左正**符号与转角一致。
- 362/415 段有 livePose 有效直线窗（余 53 段：过短/无直线段/livePose 未跑）。
- **量程/符号**已在 10-21 弯道探针三验（§7.6 corr 0.90-0.97、斜率≈1）——本包补的是
  **零点 bias 分布**，不是重新验证符号。

## 5. paramsd 上车观察值（给路测第一天用）

- 期望 `|paramsd yawBar` 与本文 −0.086°/s 同量级；若 >0.5°/s 持续，先查 livePose 收敛
  再怀疑 0x222。
- 50Hz 口径务必对齐：paramsd 若按 60Hz 假设采样会引入 ~17% yaw rate 尺度误差。
