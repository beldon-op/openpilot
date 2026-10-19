# BYD 宋 后向/“bus3” 探索定案(2026-10-18,byd_logs_2026-10-05_07 全量 415 段)

> 一句话:**日志里不存在 bus3 流量(src==3 全局 0 帧),车侧也没有可达的后向目标流——宋 PLUS DM-i 22 的后方输入只有 0x418 结果位,且本档证明 0x418 里没有隐藏距离字段**;意外收获是 bus1 上被旧结论误杀的一族 **67.16ms + SAE J1850 CRC 多目标槽表**(0x301/0x340s/0x380s/0x3C8s,89 地址 3934 万帧),它对后方事件无响应、但对前向跟车强相关——**前向 raw 目标流的候选,值得开新一轮逆向**。工具全部入库 `tools/byd/`,控制路径零变更。

## 1. 背景

`docs/byd-lane-change.md` §8.5.5 留了两问:①bus3 是否可及;②后方 SRR 目标流是否存在。`tools/byd/radar_bus_probe.py` 是设备侧答案(T7,待上机)。本档用 415 段日志(6.9G)先把**日志侧能答的**全部答掉。

## 2. 总线普查:没有 bus3,也没有别的

`tools/byd/radar_bus3_census.py`(415/415 段,workers=10,解析 0 错误):

- 全部帧的 src 集合 = **{0, 1, 2, 128, 130, 192}**;**src==3(及 131/193 等组合)= 0 帧**。
- 128/130/192 不是隐藏总线:panda 驱动 `panda/python/__init__.py:66-74` 定义 `bus+=128 returned / bus+=192 rejected`,厂商 `values.CanBus` 枚举完全一致(**BYD_MAIN=0, BYD_AUX=1, BYD_CAM=2, LOOPBACK=128, REJECTED=192**)。即 128/130 = OP 自发送帧在 bus0/bus2 的回声(0x316/0x32d/0x32e/0x32f/0x432/0x20f=TX 集),192 = safety 拦截记录(与 carcontroller「blocks stock frames bus2->bus0」注释吻合)。7a4b3e696e 单段无回声(未控车段)。
- 双录现象:bus0 与 bus2 大量同 ts 同载荷帧(网关桥接),0x418 主体在 bus0、bus2 仅 6 段 1096 帧回声。
- **结论:当前接线(panda 三车道入车)能见的全部流量就是 census 里的集合;后向目标流若要出现,只可能在设备探到的物理 bus3 通道上(日志侧无法证伪接线可达性,T7 保留)。**

## 3. 后向唯一数据源 0x418 的字节级定案:没有隐藏距离

`tools/byd/radar_bsd418_mine.py` extract+analyze,593915 帧(415 段,bus0,实测 **dt=50.0ms=20Hz**,更正 `docs/byd-lane-change.md` §八 多处"10Hz"表述——发起窗 0.5s clear 窗设计数字不受影响,反而更充裕):

| 字节 | 实测 | 判定 |
|---|---|---|
| b0 | 恒 0xFD | 非 J1850 CRC(命中率 0.00%,与 0x10x 家族不同源/无校验) |
| b1 | 0x80 基底 + L/R 位 | **LEFT_APPROACH=b1[1:0]、RIGHT_APPROACH=b1[3:2](值域 0/1,rare 2)——就是既用的两个活位** |
| b2 | 0x55/0x56 双值,b0 位与 L/R 同拍 | 状态伴生位 |
| b3 | 99.8% 恒 0,偶发 1/2/4 | 低频状态位(语义开口,无监督可对齐) |
| b4/b5 | 恒 0xF0 / 0xFF | 常量 |
| b6 | 16 值均匀轮转,**低 nibble 恒 0xF** | **AliveCounter(4bit 滚动)** |
| b7 | 157 值,**低 nibble 恒 0xF**,798 次 onset 窗内 dur_med p50=130、pre/post 无趋势,corr(vEgo)=0.015 | **滚动码,非距离**——「b7 distinct=81 可能藏接近距离档位」假说证伪 |

798 次 L/R onset 的窗口分析:除 b2 伴生位外**无任何字节随事件系统性变化**→ **0x418 只携带盲区警告结果,不含目标距离/速度量。后方"多远/多快"在日志可达的一切数据中不存在。**

## 4. 车侧拓扑:后向为什么不可能有(厂商+手册交叉证据)

- **维修手册(MPC 分册,P13 引脚表)**:MPC 只有两路 CAN——**私有 CAN(接前 MRR 雷达)** + 公有 CAN(底盘网)。OP 接的是公有网 → 前雷达原始流物理不可见(这也给「0x35x 恒空/0x374 零帧」一个硬件层解释:那是 MPC 转发层的空槽,原始流根本不在这张网上)。
- 同一手册把「识别**两侧**并线加塞车辆」列为 **MPC 视频融合**职责;2021 款宋 PLUS DM-i 的 ACC/MPC 分册**没有任何后方毫米波雷达章节**(无 SRR/角雷达分册)。结合 §3(0x418=结果位):**本车后向感知大概率是摄像头/BCM 域,不存在一条没接的"后雷达 CAN"——bus3 探索从"接线可达性问题"降级为"数据源不存在"。**
- 厂商树(`docs_site/pyarmor_decrypted_opfull_2026-10-08`):`BYDFlags.BYD_REVERSE_BSM_CAN=4096` 只挂在 **SONG_PLUS_DMI_23**(carstate disasm 中 `('BSM',20)` 即 0x418 帧,20Hz,与实测吻合);宋 22 无此翻转。厂商全系 BYD 也只认 3 条 CAN(buses 0/1/2)——没有任何"后雷达专用总线"的代码痕迹。

## 5. 意外发现:bus1 67.16ms J1850-CRC 多目标槽表(前向;当晚已翻案,见 §5 末指针)

census 的 CRC 列(修正 `crc_n` 聚合 bug 后)扫全 fleet,**J1850 CRC b1..b7 命中 100% 的帧 = 3935 万,不止 0x10x 六帧**:

- 0x301/0x340-0x34A(9 活) + **0x380-0x38F 全 16** + 0x390-0x397(部分) + 0x3C8-0x3E3(28 活) + 0x350-0x354(5,载荷恒空)+ 0x302/0x100/0x569 —— 共 **89 地址 @67.16ms(14.9Hz,典型 MRR 扫描帧率)**,周期/相位完全一致(442222f=2×442221)。
- 其中 53 个地址载荷字节活跃(静帧=空槽占位,如 0x103/0x104/0x35x 之于一对目标槽)。
- 段内监督(`tools/byd/radar_slot67_mine.py`,跟停事件段):存在与 **0x109 b7 强相关的槽字节**(seg108:0x380 b2 r=0.836;seg110:0x340 b3 r=0.868、0x343 b3 r=0.818;seg100:0x344 b4 r=0.782),**但活跃槽随段变化=动态槽位分配**——track 表行为,不是单一物理字段。
- 后方相关性检验(`tools/byd/radar_slot67_rear.py`,0x418 onset±5s vs 同段对照,live/new 双指标):diff=+0.43(z=0.2)/+0.57(z=0.5)——**对后方接近事件无响应**,与"前向 MRR 目标表"一致。
- **与既有定案的关系**:10-11 用"跨段 HUD 档位单调性"把 0x38x 证伪成"状态表"——该检验假设单一字段,对动态 track 表天然失效;0x351-354"恒空"与 census(槽位有帧、载荷全零)兼容。**本档不直接翻案**(那是又一轮 0x10x 规模的逆向:槽配对结构/ObjID/dist/relV/angle/valid 位),但登记为**下一优先挖掘对象**——若证实含 relV,将补上 0x10x 无 vRel 的 track 缺口;若只是 MPC 前融合目标,也远强于单一 0x109。
- 纪律不变:**未定案前不入库、不实例化、不动控制**;路测门禁不变。
- **当晚追加:本专项已完成** → `docs/byd-radar-slot67-2026-10-18.md`。翻案成立(确为多目标槽表,块 C x=12bit≈8×b7 细版、三设备复现、ACC 领先车 75% 在槽 1);**但 relV 全字段监督 |r|≤0.10 判死——公开 CAN 无相对速度**,track 缺口不变,dbc 不入库、控制零变更。风险表 #4 由此销账。

## 6. 风险/待办

| # | 事项 | 状态 |
|---|---|---|
| 1 | bus3 物理可达性 | **日志侧定案:无流量**;设备端 T7(`radar_bus_probe.py` 已覆盖 bus3)上线跑一次即永久销账 |
| 2 | 后方目标流 | **定案:数据源在公开 CAN 上不存在(0x418=纯结果位+硬件为 MPC 视频)**;变道门控维持现状(0.5s clear+abort),**不要等 bus3** |
| 3 | 0x418 b3 低频状态位语义 | 开口(0.2% 帧,无监督线索),优先级低 |
| 4 | **67ms 槽表翻案/逆向** | **已销账**(当晚完成,`docs/byd-radar-slot67-2026-10-18.md`):多目标表成立(块 C x=12bit≈8×b7,1/8ft 刻度,三设备复现,lead 75% 在槽 1);**relV 判死**(F1-F3 对关闭速度 \|r\|≤0.10);块 B=近程 1dm 粗表;F1-F3/D 块语义开口;y/state 未解前不入库 |
| 5 | MPC 私有 CAN 引线 | 手册已给引脚(P13-5/6 公有、P13-11/12 私有、P13-1 IG1、P13-2 GND):前向 raw 流终极解,代价=破线/转接器,先评估 67ms 槽表够不够用 |
| 6 | lane-change 文档 10Hz 表述 | **20Hz 定案**(20Hz 双源:census dt=50.0ms + 厂商 ('BSM',20));§八历史叙述保留原文,§8.5.5 本档勘误 |

## 7. 复现

```bash
.venv/bin/python tools/byd/radar_bus3_census.py docs_site/byd_logs_2026-10-05_07/realdata \
    --workers 10 --out /tmp/bus3_census.json          # 全 fleet census+CRC 扫描(~40min)
.venv/bin/python tools/byd/radar_bsd418_mine.py extract docs_site/byd_logs_2026-10-05_07/realdata --workers 10
.venv/bin/python tools/byd/radar_bsd418_mine.py analyze               # 0x418 字节定案
.venv/bin/python tools/byd/radar_slot67_mine.py 00000002--d3da5795a9--108 ...   # 槽表段内监督
.venv/bin/python tools/byd/radar_slot67_rear.py --max-segs 24 --sample 40       # 后方 onset 响应检验
```
