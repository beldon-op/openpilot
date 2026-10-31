# BYD 完整雷达方案逆向与验证（2026-10-31）

对象：厂商仓 `/Users/wujiafu/Documents/work/op/carrotpilot/byd/byd`（分支 merge-byd2027-agnos，pyarmor 构建 2026-10-27）+ 台架设备 `comma@192.168.3.146`（/data/openpilot 同源构建，sunnypilot_chn 2025.002.000）。
结论先行：**这套 fork 的"雷达方案"是两套独立通道**——

| 通道 | 实现 | 覆盖车型 | 状态 |
|---|---|---|---|
| ① 厂商混淆 CAN 雷达（opendbc 层） | `opendbc/car/byd/radar_interface.py`（pyarmor+BCC） | 仅 唐 TANG_GAS/TANG_DM（0x351-0x353@bus1）、汉 HAN_EV/DM_20_21（0x374@bus2） | 2026-10-29 已逐行重建（`local_docs/vendor_decrypted_2026-10-29/reconstructed/radar_interface.py`），本次设备实测 38 平台真值表与重建 100% 一致，无任何新增 |
| ② object_stream_sp 影子雷达（SP 层，明文 Python + 受保护 .so） | `selfdrive/controls/lib/object_stream_sp/` + radard 重写版 | **宋PLUS DMI_22 / EV_21_22 已映射**（另有唐 18/19/20、D9 等布局） | 显示层已用宋真实路测数据验证可用；控制层被"速度非原生"门控锁住（详见 §5） |

宋PLUS 的雷达走 **通道②**。厂商混淆层对宋返回 rcp=None 是事实，但与本结论不矛盾——两套并行。

---

## 1. 架构与数据流

这套 fork 已把 stock openpilot 的 radard 拆散：

```
CAN ──┬─► card.py ─ RI=interfaces[fp].RadarInterface(CP,CP_SP)   ← 厂商混淆层（①唐/汉）
      │        └─ RD 非空时发布 liveTracks
      └─► radard.py（重写版，不用 RadarInterface）
               ├─ sm['liveTracks']（①的输出，宋为空）
               ├─ sm['modelV2']（视觉 leads）
               └─ ExternalRadar(CP)  ← object_stream_sp（②）
                        ├─ 独立订阅 can（只收 src==1），喂 UnifiedShadowRadar
                        ├─ current_leads/display_leads → 20Hz 写 /dev/shm/sp_external_radar.json
                        ├─ apply_primary：RadarShadowFusion=1 且配对+标定通过时替换 radarState.leadOne
                        └─ write_status → RadarShadowStatus/RadarCalibStatus 参数 + /data/media/0/radar_unified_sp/
```

- radard 不检查 CP.radarUnavailable；radarUnavailable 只剩 sunnypilot dec.py 和 radarErrors 消费。
- radard 仅 onroad 启动；台架（不点火）上整条链不跑，故设备上无任何运行痕迹（RadarShadowFusion=0 之外无状态文件）。

## 2. 通道①要点（详见 10-29 产物 README）

- 38 平台真值表（设备热态实测，`get_params` 走设备真实参数 + `RI(CP,None)` 实例化）：
  radarUnavailable=False 仅 6 家：TANG_GAS、TANG_DM、HAN_EV_20_21、HAN_DM_20_21（有 parser）、HAN_EV_22、HAN_DMI_22（**无 parser**，radarUnavailable=False 但 update 走基类空数据——厂商配置不一致，无实际效果）。
- `_get_params`(bcc_345) 常量池含 `radarUnavailable = fp ∉ (TANG_GAS, TANG_DM)` 逻辑段；池截断（<+61 more>）。
- 全平台 `CP.enableBsm=False`（含宋）；BSM(0x418) 订阅由 `enableBsm` 门控（carstate bcc_677/755），当前赋值点在截断池里。注意：**0x418 也不在通道②解码白名单里**——BSM 不属于这套前向方案。
- `_matrix.so` 另注册 gwm_haval_h6_pt / gwm_tank_500_pt DBC（厂商在做长城）。

## 3. 通道②文件构成

```
selfdrive/controls/lib/object_stream_sp/
├── stream.py        1706 行明文：标定引擎/航迹/primary/上传器/UnifiedShadowRadar
├── adapter.py        159 行明文：ExternalRadar，radard 的接线层
├── decoder.py          2 行：from .stream import UnifiedShadowRadar, is_supported
├── libobject_stream.so  56KB aarch64 stripped：受保护 CAN 解码器（本次已逆向，见 §4）
└── VERSION            v72（CORE_REVISION 'dbc-core-20261030.1'，2026-10-30 构建）
UI（明文 C++）：
├── selfdrive/ui/sunnypilot/qt/offroad/settings/external_radar.h   设置面板"雷达融合"
└── selfdrive/ui/sunnypilot/qt/onroad/external_radar_overlay.h    行车 3D 投影叠加（注释：改编自 owner 的 CP 源 DrawCarrot::drawRadarInfo()）
```

代码内自称："Shared protected radar core"、"Vehicle identity disambiguates shared CAN layouts as authorized by the owner"、"Original DBCs and field layouts never ship in this module"。`is_supported()` 恒 True（准入是数据驱动，非车型白名单）。

## 4. libobject_stream.so 逆向（本次核心成果）

导出 API（.dynsym 30 个符号，代码区 0xd10-0x8ea4，其中 0xd10-0x7fff ≈28KB 为无符号内部引擎，导出层 0x8008-0x8e64 为壳）：

| 函数 | 作用 |
|---|---|
| rd_init(key,len) | 2 字节密钥初始化；置 enabled、生成运行时反 dump 掩码 |
| rd_auto_accepts(addr) | 白名单判定（src==1 在 Python 层保证） |
| rd_auto_feed(addr,data,len,ts,v_ego,out,max) | 喂帧→NativeTarget[]（d/y/vr/va/a/vl + flags + 32 detail 浮点）；入口校验后尾跳内部引擎 0x4880 |
| rd_auto_choose_vehicle(name) | 指纹→族提示（0-8） |
| rd_auto_choose_shared / rd_auto_profile | 共享布局选择 / 当前 profile |
| rd_auto_basis/geometry/measurement_sd/velocity_bounds/counter_step | 位置投影与速度估计支撑（profile 2 专用） |

### 4.1 密钥与反 dump（rd_init @0x8048）

- 密钥仅 2 字节，Python 侧 `_rd_key() = sha256(b'')[:2] = e3 b0`（**通用包，非设备绑定**）。
- `srand(time(NULL) ^ &key)` → 三次 rand 生成两个运行时掩码：rand16 存 0x1e6fec（白名单掩码）、rand32 存 0x1d5b0。
- 三个内存 XOR 循环：8KB 主表区（0x1b620-0x1d620）^= rand32^key32；白名单区（0x1b0e0-0x1b618）^= rand16^key16；64B 配置块（0x1b0a0）用密钥车道解码。
- **文件态即明文的补集**：`file_u16 = 原值 ^ 0xb0e3`（key16 小端）。引擎运行时以 `[0x1d5b0]^key32` 等重构。⇒ 整个"受保护"数据可离线静态还原，密钥就是 sha256(b"") 的前两字节。

### 4.2 地址白名单（668 项，已全解）

file 0xb0e0-0xb618，668 个 u16，`^0xb0e3` 后为全部已实现布局的接受地址并集（覆盖低地址段 0x1-0xd7 = 其他品牌、0x101-0x10f、0x200-0x218、0x300-0x304、0x380-0x397、**0x3c8-0x40c**、0x410/420/.../0x4f0 步进 0x10、0x500-0x5b7、0x5c0-0x740 等）。宋实测接受 96/110，**不含 0x351-0x354（心跳槽）与 0x418（BSM）**——与 10-29"宋槽无航迹数据"结论互证。

### 4.3 指纹映射（rd_auto_choose_vehicle @0x89a0）

归一化：大写化 + 非字母数字→`_`，前缀 `CAR_` 后与 rodata 别名表 strcmp：
- **宋族 16 别名**（0x8ed0-0x9038）：DMI_21/22、5G_DMI_22、DMI_2021/2022、DM_I_21/22/2021/2022、EV_21/22、EV_21_22、EV_2021/2022/2021_2022 → family_hint=2（宋 DMI_22/EV_21_22 实测命中；DMI_23、SONG_L、SONG_PRO 不在表 → hint 0，落"共享布局待甄别" candidate [2,8]）
- **唐族 12 别名**（0x9040-0x913e）：TANG_DM_18/19/20、_18_19、_18_20、2018/2019/2020、2018_2019、2018_2020、TANG_GAS → hint=8
- profile 谱系（stream.py `_calib_file` schema 命名）：**p2=core60（宋）**、p8=core69_tang_axis、p6=core65_d9（腾势 D9）、p5=core68_tang_fd_v2、其余 core59。

### 4.4 flags 位语义（修正此前"decoy=授权锁"的猜测）

引擎内 flags 由"逐字段是否有限"装配（0x635c 附近：`fabs`→`fcmp 0x7f7fffff`→`orr`，bits 8/0x20/0x40/0x80/**0x100**/0x200 各对应一个计算值）。结合 Python 侧消费（stream.py 1399-1440）：

- bit1/2/4/8/16 = vRel/vAbs/accel/vLat 数值可用；bit32 = 原生 track id；bit64 = age；bit128 = 候选加速度
- **bit256 = 该目标速度是解码器估计值（非原生多普勒）**，Python 命名 'decoy'（display_velocity_source 三态：estimates['source'] / 'decoy' / 'native'）；bit512/1024/2048/8192/16384/32768/65536/131072/262144 各有语义（相对加速度/槽标记/闭合率/空槽/速度饱和/静止/硬复位等）
- 32 个 detail 字段名（_DETAIL_NAMES）：headingDeg、azimuthRad、width、left/rightEdgeRad、normalizedRangeRate、nativeRelativeVelocity、radarEgoSpeed、radarYawRateDeg、quality0-5、positionMaturityX/Y、trackMaturity、motionState、velocitySaturated……即 ARS410 类物体属性。**宋的记录 detail_mask=0**（一个 detail 都没有）。

### 4.5 主数据表（8KB @file 0xb620，已完全结构化 ★10-31 深夜补完）

引擎索引公式（0xde8-0xe0c）：`(信号序号×42 + 消息基址) × 6 个 u32` → **8 组 × 42 记录 × 24B = 8064B 恰好用满**。记录格式（所有 u32 先 ^ key32=0xb0e3b0e3）：

```
[u32 bit_start, u32 bit_len, u32 flagC, u32 sign_extend, f32 scale, f32 offset]
```

信号提取器（0xde8）：LSB 位序抽取 1..64 位、起始位 < dat_len×8、sign_extend 位做符号扩展、flagC=0 走无 scale 分支；每条消息的记录基址由引擎内该消息的 handler 传入。91 条 length>0 的记录已全解，存 `local_docs/rd_spec_table_2026-10-31.json`。要点：

- **group5 疑似宋 0x3c8-0x3ff 物体块模板**：出现 start>64 的多帧聚合位段（411|13、440|8、96|10…）+ 距离型 (48|12, 0.041, -2.42≈168m 满量程) + 速度型 (64|14, 0.0094, -75.2≈±78m/s) + 两个 6bit ID 型 (32|6、170|6)。
- group7 与唐槽布局高度相似（16|12 距离、56|6 ID、41|1 VALID），但 scale 与 10-29 行为学估计值不同（0.05 vs 0.06、-12.5 vs -3.0）——以本表为准，10-29 值当时是估计。
- group2 是 6 槽×6bit ID 的簇消息模板（7/15/23/31/39/47|6 等距排列）+ 55|4、51|4 状态位，typical 雷达簇/状态帧。
- 组→profile 的最终映射仍未定（引擎 28KB 内每消息 handler 传基址，需逐 handler 标注），但表本身 100% 可解，保护机制等于零。
- 经验法补充（10-31 深夜两次尝试，结论写给后续）：①全零帧探针无效——引擎带跨帧槽状态，喂零返回的是残留航迹（所有地址同值 d=47.06/y=-0.48/vr=0.39），不产生新记录；②单比特探针同样被状态演化污染（需在每次探针间 `rd_auto_reset()` 并重锁 profile 才干净）；③静态法：信号提取器 0xde8 的调用点约 20 处（0x4074-0x6c64），基址经寄存器间接传入（`madd w0, w0, 42, w1` 的 w1），需按调用点做控制流标注——机械工作，方法已明确。

### 4.6 宋实测（用户 10-05 路测 rlog，段 00000001--ec0323b5d6--0/--12）

- bus1 130,933 帧/72.6s → 接受 110,669 帧，解码 613,451 记录，稳定 6 条并发航迹（dRel 12.7~73m、yRel±1.9m、vRel -4.6~-6.6、vAbs 有值、radar=True）。
- `current_leads()` 必须显式传 now_ns（内部 freshness 用 monotonic；离线喂 logMonoTime 会全体过期——测试脚本坑，非产品 bug）。
- 全部记录 flags=263（bit1+2+4+256）且 detail_mask=0：**位置真实、速度为估计**（宋 wire 无逐目标原生多普勒/无原生 id）。
- primary 端到端：vision（prob≥0.7）喂入后 state 走到 waiting_radar，因 eligible 要求 `!flags&256 && flags&32 && details.nativeRelativeVelocity/normalizedRangeRate` —— **宋当前 wire 数据永远不满足 → 控制级锁定是数据属性，不是待调开关**。

## 5. 控制级（fusion）全链门控

`apply_primary`（radard leadOne 替换）需同时满足：

1. `RadarShadowFusion=1`（UI 开关会先校验：标定 accepted + 数据 3min 内新鲜 + 二次确认弹窗）
2. driving context：D 档、系安全带、关门、无转向灯、|yaw|≤0.03
3. primary 锁定：视觉 prob≥0.7 与雷达轨 8 帧连续配对（|y|≤1.5m 横向走廊、距离门 2.5-6m、|ΔvRel|≤2、唯一性校验、cut-in 立即让位逻辑）
4. 标定验收：profile2 走 `_GeometryLearning`（position-angle-20261028-v1：20000 moving/2000 range/200 cross/100 holdout 样本、跨检验 |c0-median(cross)|<8、holdout 距离中位误差<0.4m 横向<0.3m、两次稳定估计、可持久化/恢复）；其他 profile 走 `_LiveCalib`（候选轨+锚定轨，8-60m/|y|<0.8 配对）
5. `target.trackId ∈ eligible_ids`：需要 §4.6 所述原生级记录（宋当前不满足 → fusion_state='blocked_protocol'）

显示层无以上门槛：current_leads/display_leads 直接出（UI 预览 + 设置面板"最近雷达目标"）。

## 6. 标定与速度估计组件

- `_LiveCalib`：候选轨每帧加权递推（残差门控软排除）+ 锚定轨慢速升级（2000 帧满分/333 帧降级迁移），阈值 _TIGHT=0.6m/_LOOSE=1.8m 等；输出 dist_offset/lat_offset + MAE + progress（UI 只升不降 best_progress）。
- `_GeometryLearning`（仅 p2）：把雷达"码字+基距"经 .so 投影（rd_auto_basis/geometry）拟合安装角 c0 与距离偏置 d_off，投影需 `RadarY87ApplyCalibration≠0` 才应用到记录。
- `_RelativeVelocityKF`：距离+原生多普勒双观测 KF；**多普勒饱和时**用 `d_rel × normalizedRangeRate` 估计（'y87_saturation_kalman'）。
- `_DisplayMotion`：仅 profile2，为显示提供速度估计（原生优先、饱和用 KF）。

## 7. 遥测（注意点）

`_Uploader`（每小时一次，daemon 线程）：POST `http://180.103.127.9:9003/radar/upload`，表单 `filename=radar_sp-<fp>_<sha256(devID)[:16]>.dat`，content 为 XOR-SHA512 流加密 JSON：platform/ver=v72/fp 明文/标定进度/偏移/MAE/样本数/reset_count 等（**无原始 CAN**）。设备 ID 取 `/data/openpilot/dump.txt`（根目录那个 base64 文件就是设备标识）→ DongleId → IMEI → androidboot.serialno。服务器仅回 "OK"；**本仓代码无下行/授权回发逻辑**（"CP receiver"在服务端，本机/设备均无其痕迹；CP 版路径 /data/params_cp、/data/media/0/radar_unified 不存在）。

## 8. 参数清单（/data/params/external_radar_sp/）

| 参数 | 写方 | 作用 |
|---|---|---|
| RadarShadowFusion | UI | 融合进 leadOne（总开关，默认 0） |
| RadarAutoCalib | UI（默认开） | 自动采集标定配对 |
| RadarCalibReset | UI 按钮 | 置 1 触发重置，雷达进程清空后写回空 |
| RadarY87ApplyCalibration | —（默认开） | 是否把 p2 位置投影应用到记录 |
| RadarApplyInstallationCalibration | —（默认关） | 非 p2 的 _LiveCalib 偏置应用 |
| RadarShadowStatus / RadarCalibStatus / RadarFusionStatus | 雷达进程 | 状态/标定进度（UI 1s 轮询、3min 新鲜度） |

## 9. UI 细节与已知瑕疵

- offroad 面板（settings.cc 注册为"雷达融合"）：雷达接收状态 / 最近雷达目标 / 自动标定开关 / 补偿数值 / 标定进度 / 重置 / 融合开关 / 融合状态。
- onroad 叠加（model.cc:83 调用）：≤64 目标投影进 3D 场景，primary 绿、远离橙、接近红，带速度/距离/横向标签，250ms 新鲜度门。
- 瑕疵：UI 融合态文案匹配 `locking:N/10`，而 `_PrimaryTarget.REQUIRED=8`（fusion_state 输出 `locking:N/8`）→ 锁定中会显示"融合状态未知"。

## 10. 宋PLUS 控制级解锁路径（open items）

宋 wire 缺原生逐目标多普勒与原生 id（flags 263、无 details）——两条路：

1. **等 .so 更新/授权**：8KB 主表与 668 地址白名单是全品牌并集，说明引擎框架支持多种布局；若后续 .so 为宋布局启用多普勒字段提取（或厂商在私有流里给出），bit32/bit256 语义自动满足。主表格式已可静态解（§4.5），可跟踪 .so 版本变化（CORE_REVISION/TELEMETRY_VERSION）。
2. **FRS-OD 私有 CAN 引线**（byd-lane-change.md §8.5.10 原方案）：ARS410OD_PrivateCAN.dbc（0x2B-0x5B FRS_Obj_1..64 Part1of2/2of2）有完整原生多普勒。**已核对：FRS-OD 全部 81 个 BO_ 地址（0x2B-0x5B）均在 668 项白名单内（81/81）**——即把 FRS-OD 帧引到 src==1 总线后解码器立刻接受，无需改 .so。待验证项：①FRS-OD 流经数据驱动 auto-profile 识别到哪个 profile（低地址段与吉利/长城布局重叠，dbcall 吉利版同为 ARS410 FRS-OD，大概率命中已有布局）；②该布局是否输出原生多普勒+原生 id（bit32/bit256 清位）；③**与宋自带 bus1 物体块（0x3c8-0x3ff）同总线时的 profile 抖动**——Python 层 `profile != _active_profile` 会清空全部航迹，两股流并存可能互踢，需实测或设法让 FRS-OD 独占判定。本地 ARS410OD_PrivateCAN.log 仅为 CANdb++ 转换警告（145B），无帧可用，需实车/车友抓包（宋需自车抓包定案不变）。
3. 显示层现在就能用：实车 onroad 即出 6 目标级航迹 + UI 投影；对 ACC 无影响（被动）。

## 11. 复现

- 设备环境：`PYTHONPATH=/data/py_offline_site/cp312-aarch64:/data/openpilot`（numpy/pycapnp/pycryptodome/pyzmq/setproctitle/smbus2 已补装，删新增项即还原）。
- 喂数脚本（设备 /data/tmp/）：`feed_decoder.py`（静态时基）、`feed_decoder2/3.py`（monotonic 重映射 + 逐地址统计）、`feed_e2e12.py`（vision+CAN 端到端）、`feed_details.py`（detail dump）、`feed_identity.py`（跨身份 flags 对照）；数据 `song_bus1_frames.bin`、`song12_bus1.bin`、`song12_vision.jsonl`（本地 /tmp 同名）。
- .so 逆向工件（Mac /tmp/objsod/）：dis.txt（全量 objdump）、rodata 字符串表、白名单/主表解码脚本；**信号 spec 全表已存 `local_docs/rd_spec_table_2026-10-31.json`**（8 组×42 记录，含 schema 说明）。
- 探针脚本（设备 /data/tmp/）：fingerprint_groups.py（全零帧——结果无效，留作反面记录）、bitprobe.py（单比特——被槽状态污染，需 reset 隔离重做）。
- 真值表脚本：本地 /tmp/probe_all.py（38 平台 get_params+RadarInterface）。

---

## 12. 自研落地（同日续篇，openspec `byd-radar-multitarget-port`）

以本文档的 .so 逆向为"标准答案"，自研开放解码器当日落地并入库（carrot-byd 分支）：

- **对拍 harness**（`tools/byd/radar_probe_{dump,run,solve,verify,fields}.py`）：真帧载波 + 带内单比特
  探针在设备端打通，288 探针/确定性 0 漂移；D1 方法论验证成立（§4.5 的"探针被槽状态污染"结论
  由白名单过滤 + 每 pass begin_pass + monotonic 重锚 + flip_k≥20 四项修正解决，坑详录
  `docs/byd-radar-multitarget-re-2026-10-31.md` §0）。
- **三源定案**：块 C 距离（`d=0.062319·x−1.671`，11 位，bit27=状态）、块 C-F2 closing
  （`0.015106·raw`）、块 D 逐目标速度（`0.01439·raw−12.56`）——探针阶梯 + 本表 g1/sig0、g1/sig14
  + 引擎输出 R²=1.0000 回归。g1=块 C 行模板坐实；**块 D=同对象表的属性/速度帧**（语义修正）。
- **旧 1/32 刻度退役**：x↔b7 斜率被证实场景依赖，引擎刻度接管（物理米制 V2 跟停终验）。
- **自研解码器**：`opendbc/car/byd/radar_multitarget.py`（纯 Python 周期装配 + CRC/计数器/槽代数），
  radar_interface 第 4 层（trackId 201+，`BydRadarSlotTable` 参数 0=关/1=显示/2=控制），未定案字段
  NaN 隔离；T8 分道化（gate=3，yRel 迟滞分类）随行落地。
- 本表未覆盖项：块 D 速度记录无 spec 同构项（probe+回归双源）；横向 y = 引擎学习标定投影，
  离线不可复原，HYPOTHESIS 隔离待 V2。
