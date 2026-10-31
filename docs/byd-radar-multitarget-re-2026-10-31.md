# BYD 宋 bus1 67ms 多目标槽表逆向定案（2026-10-31，openspec byd-radar-multitarget-port 阶段一）

> 一句话：**真帧载波+带内单比特探针打通**（288 探针/2274s/确定性 0 漂移），块 C 距离、
> 块 C-F2 closing、**块 D 逐目标速度**三组字段达成「探针阶梯 + 厂商 rd_spec_table + 实数据
> R²=1.0000 回归」三源闭环入库定案；横向 y 定位到候选位段但受引擎学习标定投影污染，
> 按 HYPOTHESIS 隔离；**旧 1/32 刻度模型退役**（锚不成立），引擎刻度 0.062319 接管。

## 0. 方法（D1 落地）

工具链（全部入库 `tools/byd/`）：`radar_probe_dump.py`（Mac 侧 rlog→载波二进制，含 carState 上下文）→
`radar_probe_run.py`（设备端：版本门 CORE_REVISION=dbc-core-20261030.1 + .so sha256 存档 +
J1850 CRC 自检 + 确定性双基线 + 单比特探针 + 按帧型批量 + 断点续跑）→ `radar_probe_solve.py`
（布局反解 + rd_spec_table 对拍）→ `radar_probe_verify.py`/`radar_probe_fields.py`
（引擎输出 vs wire 位段仿射回归）。探针产物存档 `tools/byd/probe_table_2026-10-31.json`（288 探针）。

踩坑记录（复现必读）：
1. **白名单必须过滤**——生产路径 `accepts()` 在 decode 之前；裸喂 0x351-354 心跳会污染引擎槽状态（停车段伪航迹之源）。
2. **`begin_pass()`（choose_shared+reset+choose_vehicle）每 pass 必调**——漏掉则引擎无车辆身份（hint 0），永不出真航迹。
3. **引擎有真实时钟保鲜门**——10-05 的原始 logMonoTime 必须 monotonic 重映射（每 pass 锚定 now−窗长）；原始时间戳=远古→全 stale→残迹输出。
4. **停车暖机段（前 ~5s）是引擎 decoy 区**：输出随机伪航迹、跨进程不确定——但被行车区完全隔离（5s 后逐字节确定），diff 一律 `--diff-after 10` 跳过。
5. **翻转键 = 每地址帧计数器**（不是 win 全局索引）；**翻转窗 ≥20 帧**（flip_k=3 被引擎平滑吞掉，无 delta）。
6. C-F0 距离位扰动会**挂起该槽目标**（flags→empty 20 帧后恢复）——引擎对距离帧扰动有硬门；closing/vr 位扰动则被接受并实时反映。

## 1. 定案（三源闭环，可入库）

判定标准：探针 delta 阶梯（scale×2^k 幂律）+ rd_spec_table 同构记录 + 引擎输出 vs wire 位段
OLS **R²≈1.0000**（引擎解码即 wire 的仿射函数，等价于拿到解码式本身）。三源同时满足 → PINNED。

| 字段 | 帧位 | 解码式（引擎语义） | 证据 |
|---|---|---|---|
| 距离 d（块 C，每对象行 F0） | bits 16-26（11 位，b2=低字节，b3 低 nibble=高 4 位；**bit27=状态/溢出位，置位即 kill**） | `d = 0.062319·x − 1.671 m`，量程 [0, 125.9] | 阶梯 0.062319×2^k（bit16-26）+ **spec g1/sig0 (16\|11, 0.062319, −1.671) 精确一致** + seg--12 引擎 slot0 回归 R²=1.0000（n=221） |
| closing（块 C，F2 帧 b2） | bits 16-23（8 位） | `closing = 0.015106·raw`（offset≈0） | 阶梯 + **spec g1/sig14 (16\|8, 0.015106) 精确一致** + 回归 R²=1.00000 |
| 速度 vr/va（块 D 行 0 F0） | bits 16-26（11 位） | `vr = 0.01439·raw − 12.56 m/s`，量程 ±16.9 | 阶梯 0.01439×2^k（bit16/20/24）+ 回归 **R²=1.00000**（0x3C8 @16\|11）；spec 表无对应记录（见 §3） |

重大语义修正：**块 D 不是独立目标表，是同一对象表的属性/速度帧**——D 行 0 F0 的位段
仿射映到 C 行 0 所追踪目标的 vr（R²=1.0），7 行 D 对 6 行 C 的富余与属性帧扩展一致。
slot67 时代"远距杂波/静态/车道"三猜作废。

**旧 1/32 刻度退役**：`d=x/32` 依赖"x12 对 b7 固定 slope 8"锚；2026-10-31 滑窗重测显示该
斜率随场景漂移（seg--142 各窗 0.10-0.19 b7/x，R² 0.71-0.99），锚不成立。引擎刻度
0.062319（≈2/32，含厂商标定因子 0.9971 与 −1.671m 偏置）为三源事实。**物理米制终验留给
V2 跟停实测**（对槽表目标直接量尺，一次性钉死；u=0.25-vs-0.5 张力随之消解，不影响解码）。

## 2. HYPOTHESIS（候选已定位，未过三监督，禁止入库/输出）

- **横向 y（块 C F0 bits 35-45，b4/b5）→ 三监督裁决：不定案，维持 NaN 隔离**（2026-10-31 当日跑分）：
  - S3 槽位一致性 **PASS**：锁定窗 per-cycle |Δd| 中位 = 0.0623m（恰为量化地板 1 count，b7 形态）；
    cross-slot 近重复 14 帧 → 槽索引有 churn，配对必须 distance-gated（与解码器实现一致）。
  - S2 转向补偿 **弱通过/定向**：引擎 y 对 ψ̇·d 回归 R²≈0.002、斜率≈0 → 引擎 y 为**自车系
    （已转向补偿）**——正是分道带需要的坐标系；但 seg--12 弯道样本薄，静态伪量假设未能强拒绝。
  - **S1 vision 对拍 FAIL（判死）**：415 段全量（297 段有效配对），wire bits35-45 仿射解码
    （探针回归系数）对 modelV2.leadsV3[0].y：**逐段相关 p50=−0.16（≈0/负）**、|Δy| p50=0.57m、
    p90=1.69m、最差段 6.4m——与视觉横向无任何一致。三过二不成立（S3✓ S2± S1✗）。
  - 教训：引擎 y 的单段 R²=0.9887 仿射拟合是**学习标定态的段内巧合**，不跨段泛化；位段
    （F0 b4/b5）语义与横向不符（疑为方位角/宽度类属性）。横向的出路回到 FRS-OD 引线
    （ARS410OD 81 地址）或 D 块深挖，本轮不接。
  - 工具：`tools/byd/radar_sup_{s1,s2,s3}.py`（S1 = 415 段全量跑分，可复现）。
- **C-F0 b7 字节**：对 vr 有物理相关（R²=0.9888，非仿射）——疑似伴生速度粗量，未定案。
- **C-F1/F3、0x301**：探针 288 位全静默（无任何输出位响应）。F1/F3 疑似行内计数器/ID；
  **0x301 b67LE 伴生量检验=阴性**（翻转任一位不影响解码输出）。
- 块 C F0 扰动 → 该槽目标挂起 20 帧后恢复（引擎距离连续性硬门）——装配器不需要复刻。

## 3. 与 rd_spec_table 的出入

- g1 = 块 C 行模板首次坐实（sig0=距离、sig14=closing 双双精确命中）。
- 块 D 速度记录在 spec 表**无同构项**（91 条中无 (11, 0.01439)）——probe+回归双源 vs spec 缺席，
  按纪律以三监督收尾（候选速度场价值极高：量化地板 0.0144 m/s，比距离差分的 0.93 m/s 细 65 倍）。
- 组→消息映射（20 个提取器调用点）仍开放，但 g1 已由双记录钉死为块 C。

## 4. 对既有定案的影响

- **"公开总线无原生 vRel"结论修订**：0x10x/块 C 确无逐目标速度，但**块 D 载有引擎解码为
  速度的 wire 字段**（flags bit256 仍标注"估计"语义，与 FRS-OD 原生多普勒的差别待 V2 判读）。
  设计 D4"槽表点不做主 vRel 源"维持 v1 不变（差分实现已入库），**D 速度场解码列为三监督
  后的第一优先升级**——若 S1/S2/S3 通过，槽表 vRel 直接取代差分。
- slot0/slot1 在 seg--12 40-72s 活跃时 **b7≡0、presence≡0**：槽表与 b7 链互补（多目标价值实证）。
- 槽代数/装配纪律全部由解码器实现并测试覆盖（13+6 用例）：CRC/计数器/空槽/edge/generation。

## 5. 复现

```bash
# Mac：载波提取（任选段；seg--12 为探针基线段）
.venv/bin/python tools/byd/radar_probe_dump.py <segment_dir> --out /tmp/probe_carrier.bin
# 设备（ssh comma@192.168.3.146，文件在 /data/media/0/probe/ 持久区）：
python3 radar_probe_run.py probe_carrier12.bin --window-len 73 --diff-after 10 \
    --flip-at 45 --flip-k 20 --batch --bits 16-63 --resume --dump-baseline \
    --out /data/media/0/probe/probe_table.json
# Mac：反解 + 对拍 + 物理回归
.venv/bin/python tools/byd/radar_probe_solve.py probe_table.json --out /tmp/probe_diff.md
.venv/bin/python tools/byd/radar_probe_verify.py probe_table.json probe_carrier12.bin
.venv/bin/python tools/byd/radar_probe_fields.py probe_table.json probe_carrier12.bin
```

版本门基线：VERSION=v72 / CORE_REVISION=dbc-core-20261030.1 /
so sha256=cd68a1018e576c223cab0dd5c9ed1e4dd9201cee6fc24ac3f45c8ca03b1b2bda。

## 6. 415 段离线回放（task 2.7，2026-10-31）

`tools/byd/radar_multitarget_replay.py`（缓存 `radar_multitarget_extract.py`，415 段/0 错误）：
- **控制可见性 diff：IDENTICAL**——RI level 0 vs level 1 的 measured=True 点序列在
  94,747 ticks × 20 抽样段逐字节一致（显示级零变更保证的实证；报告 `tools/byd/mt_replay_diff_2026-10-31.md`）。
- 可用性（40 段统计）：40/40 段有活行、265k 解码点、**并发目标峰值 8-10**（块 B 近程 + 块 C 远程两级同时活）、
  dRel 0.1-125.9m 全量程；distinct trackId ≈82/段（generation churn——V2 观察项的量化基线）。
