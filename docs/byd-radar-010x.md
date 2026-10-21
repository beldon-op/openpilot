> **状态(2026-10-18,当日七次更新:挖掘→复验→跟停收口→夜间全帧续探④→T8 落码⑤(BydRadarGapGate 默认 0,行为仍零变更)→后向/bus3 定案⑥(docs/byd-radar-rear-bus3-2026-10-18.md)→67ms 槽表专项收口(docs/byd-radar-slot67-2026-10-18.md:多目标表成立、x=8×b7 细版、relV 判死、未入库)→b3 非横向判定⑦)**:逆向定案草案;六帧已入库 byd_general_pt.dbc(**HYPOTHESIS 标注、无任何在车 CANParser 实例化**);b0 CRC 已破(SAE J1850);单位候选已收口为 feet 0.3048 vs 0.25 后,**2026-10-19 owner 拍板钉死 u=0.25 m/count**(回忆实际跟停驻停距在 0.25 侧:15c→3.75m;feet 旁证 τ 阶梯形状随之降级;dbc factor 已改 0.25+PINNED,测试同步,host 10/10),路测实测驻停距定夺,控制行为零变更(路测门禁不变)。数据=docs_site/hour_logs(2026-09-28 控车 4 段全量 rlog)+ docs_site/byd_logs_2026-10-05_07(415 段)+ 当日 415 段全量复跑(见「定案补录」)。工具=tools/byd/{radar_re_mine.py(MI 挖掘),radar_0109_validate.py(416 段跨段检验,/tmp/radar_0109_1005.json),radar_415_diversity.py(415 段多样性/回声终检,/tmp/radar_415_diversity.{txt,json}),radar_sup_mine.py(can2 监督拟合),radar_echo_probe/radar_echo_checks.py(回声审查),radar_unit_check*.py(单位审查),radar_unit_pin.py+radar_unit_aggregate.py(486 段静止逼近/翻转回归/CRC 试打,/tmp/up_*.json),radar_stop2lead.py(13 起跟停等式标定,/tmp/radar_stop2lead.json),radar_seg_extract.py(全帧缓存 8B×六帧→/tmp/seg1005.pkl,配套 radar_seglib.py),radar_247_census.py(/tmp/radar_247.txt),radar_tau_fit.py(/tmp/radar_tau.txt),radar_b6_model.py(/tmp/radar_b6.txt/.json),radar_t8_lead.py(/tmp/radar_t8.txt),radar_presence_fresh.py(/tmp/radar_pf.txt),radar_t8_replay.py(T8 落码验收回放,/tmp/radar_t8_replay.txt),radar_b3_mine.py(/tmp/radar_b3.txt),radar_b3_lat.py(二阶横向回归,/tmp/radar_b3_lat.json);实现=opendbc/car/byd/radar_gap.py+carstate/interface 接线+desire_helper 门控+test_byd_radar_gap.py}。本文推翻 docs/byd-lane-change.md §8.5.7 中「0x101-0x10A=状态广播」对 0x109 b7 的判定,其余维持。

# BYD 宋 bus1 0x10x 雷达字段逆向定案草案

## 结论一句话
0x109 b7 是领先 0x32D 档位的前向目标连续距离量(单位候选已收口为 feet 0.3048 vs 0.25:13 起跟停事件硬上界 u≤0.42-0.44 排除 1.0/0.5/0.45;415 段全量复跑后多样性/回声终检全面成立,见定案补录),10-05_07"bus1 新面=状态广播/计数器伪影"的表述被推翻;0x101/0x105 为原始 MRR 第二目标槽的纯传输重发,0x10A 为存在/状态伴生帧——bus1 并非全假,但仍无 vRel,track 路数据源缺口不变。

## 定案补录(2026-10-18 全量复跑 + dbc 入库)

### ① 415 段全量多样性/回声终检(tools/byd/radar_415_diversity.py,Pool workers=10,415/415 段解析 0 错误)
- **全局 distinct(0x109 b7)=240**(非零 239,值域 0,9..247,1-8 空洞;分类帧 740651)——回声假说预测 ~3-6 档,在 415 段全集上被彻底证伪。
- **H(b7|HUD 档位) 熵**:全 415 段中位 **3.312 bits**(p25=0.332、p75=4.453);仅跟车段(HasLead≥10 帧,n=320)中位 **3.815 bits**(回声应≈0;94 个无跟车段中位≈0)。
- **恒档 hold(≥2s)内 b7 逐帧变化率**:跟车档均值 n=314 中位 **9.99%**(p25=5.49/p75=17.18/max=34.39);class-0 hold n=244 中位 **0.0000%**(mean 0.332%)——约两个数量级差,原 2 段证据(f4_16/fa_2 8.7-14.6%)恰落带内且被分布强化。
- **0x10A b3[7:5]==111 与 lead 符合率**:415 段中位 **98.62%**(p75=100%);P(111|lead) 中位 95.36%;**假警中位 0.00%**(p75=0.13%),但尾部 15 段 >5%、其中 **4 段卡死 100%**(d3da5795a9--180/--234、8801cc6fda--0 等)——存在位过滤必须再配新鲜度,不能单看值。
- **「b7 只在跟车段非零」严格意义不成立、方向成立**:跟车段 320 个中 98.1% 有非零 b7(distinct 中位 64);无跟车段 94 个中 84 个(89.4%)整段恒零,**10 段反例**(d3da5795a9--142/143/148、7a78691537--0/1/62/64/65/66/67)非零帧 8830/164960=5.35%;全档非零 b7 落在跟车段外的比例 **1.97%**;class-0 窗内残差 10860/290811=**3.73%**(与 hour_logs 的 3.7% 瞬态完全吻合)。
- **反例机制**(--142 实测):b7=41 **平稳保持 62s** 而 HUD 全程 HasLead=0,且这些段 0x10A 存在位同样点亮(fp 16-70%)→ b7/存在位反映**原始雷达可见目标**(ACC 未跟但雷达可见),建模 c0 残差/247 哨兵的窗过滤必须同时用 0x10A 存在位而非仅 HUD 档位。

### ② 单位 + CRC:重跑完成(agent 停滞版作废,`tools/byd/radar_unit_pin.py`+`radar_unit_aggregate.py`,486 段全量)
- **b0 CRC 已破(风险#5 销账)**:**SAE J1850 CRC-8** = poly 0x1D / init 0xFF / 无反射 / xorout 0xFF,载荷 b1..b7(线序)。hour_logs 2933 个唯一载荷 **100%**;独立日 hour_logs_2 10 段 × 5 帧(0x101/102/106/109/10A)**80100/80100=100%**。ARS410 dbc 的 CheckSum 惯例即为线索来源;此破译同时佐证 0x10x=Continental ARS410 家族。
- **u(米/计数)——米制被否,feet 幸存但绝对值未钉**:
  - 逐帧硬上界 `u ≤ vEgo/|db7/dt|`:最紧可信窗(17.2s 停车逼近,drop=79,ve=2.5m/s)给 **u ≤ 0.467**(短窗 0.231 属量化噪声偏置,弃用);监督拟合 0.269 m/计数 r²=0.69 与之一致。
  - 档位翻转回归(486 段,965 翻转):2-3 斜率 2.85 c/(m/s)、3-4 斜率 4.06 → 换算时距带 tau:米制=2.85/4.06s(物理不可能,ACC 无此档位)、feet=0.87/1.24s + 截距 3.7m(正常)、0.25m=0.71/1.02s(亦通)。**结论:1.0m 排除,0.3048(feet)与 0.25 均幸存**,dbc 暂以 0.3048+HYPOTHESIS;绝对值仍差一档区分,已知车距路测保留为门禁(一次跟停即可分辨 0.3048 vs 0.25:41c→12.5m vs 10.3m)。
- **hour_logs_2 独立复验(风险#3 补强完成)**:逐段跨档单调 **104/115=90.4%**(与 hour_logs 90.3% 吻合,非新鲜度伪影);0x10A 存在位 P(111|lead)=85.2% vs P(111|no-lead)=2.2%。

### ②-补 跟停(stop-to-lead)事件挖掘:硬上界收紧到 u≤0.42-0.44(`tools/byd/radar_stop2lead.py`,10-05 全档 415 段)
- 判据:standstill≥2s + 驻停时 b7 平台(5-80c)+ 存在位 111 + HasLead≥0.9 + 接近段 drop≥10c;命中 **13 起**。
- 硬上界性质:任何窗 `u ≤ D_L/Δb7_L`(前车不可倒车)。最紧可信值:**--47 L=4s 窗 u≤0.414**(dlt=8c,含 ±1c 舍入余量→0.44);**--35**(13s,76→16c,D=27.3m,R²=0.9993、残差 0.4c 的教科书级近静止曲线)全窗值 0.45 亦被该上界压住(即其前车最后段有 ~8% 蠕行)。
- **0.5/0.45 就此判死**(上界 0.42-0.44 达不到);1.0 早已死。**幸存候选缩到二选一:feet 0.3048 vs 0.25**。
- 驻车间距分布佐证:13 起停驻 b7 平台 med=15c(11-17)→ u=feet 时 4.6m(典型 ACC 驻停距)/u=0.25 时 3.75m——两者皆合理,日志内无法再分,分辨器=路测实测驻停距(4.5m≈feet,3.7m≈0.25)。
- hour_logs_2(9-28 晚)0 事件——该会话锁存多、真跟停少,与 README 相符。

### ③ dbc 入库(本文件配套落地,控制路径零变更)
- opendbc_repo/opendbc/dbc/byd_general_pt.dbc 追加六帧,BO_ 十进制与 0x351 块同风格:**257 RADAR_RETX_101 / 258 RADAR_HEALTH_102 / 261 RADAR_RETX_105 / 262 RADAR_HEALTH_106 / 265 RADAR_DIST_109 / 266 RADAR_PRESENCE_10A**(角色_帧号命名;信号后缀选 **_109/_10A** 与 `_tang` 区分,未用 _mrr——0x35x 槽同属 MRR,用 _mrr 会混淆来源)。
- 字段规则(严格按定案,不确定一律 MSG 占位):公共 b0=`MSG_10x_CheckSum`(7|8@0+,**算法已破=SAE J1850 CRC-8,见②**)、b1=`MSG_10x_AliveCounter`(15|8@0+,建模两者均跳过);`LongDist_109 : 63|8@0+ (0.3048,0) [0|75.3] "m"` 并 CM_ 注明 **HYPOTHESIS: 未路测钉死**;`Presence_10A : 31|3@0+ (1,0)`(b3[7:5]);0x101/0x105 纯重发、0x102/0x106 传感器健康对、其余全部 `MSG_<帧号>_B<n>` 原始占位(factor 1/offset 0),**无编造解析式**;ObjValid(b6[7]) 置信中、按规则未设,留在 MSG_109_B6。
- 位序说明:草案书写的 `56|8@0+` 在本仓 CANParser 与 DBC 标准(Motorola MSB 起始位)下对整字节 7 不成立(56|8@0+ 起点为 b7 的 bit0、跨字节出界),整字节 7 的正确写法为 **63|8@0+**;Presence 的 31|3@0+ 即 b3[7:5]。回归测试逐字节钉死这一点。
- **无任何消费端改动**:radar_interface.py / values.py / carstate / interface.py / 控制路径一字未动,在车 parser 不实例化 257-266;行为零变更、路测门禁不变。
- 回归测试:`selfdrive/car/tests/test_byd_radar_010x_dbc.py`(真 CANParser+样本 payload:LongDist_109=b7×0.3048 且不随 b0-b6 变、Presence_10A=b3[7:5] 且邻字节不漏、重发帧同载荷全信号一致/b0 单比特隔离、六帧 b0/b1 公共层、MSG 占位原始性、全 byd 车型 pt+radar 两路 DBC 实例化+无地址冲突)。当日 byd 主机套件 **38 passed**(新增 10 + test_byd_radar_interface 11 + test_byd_carstate 17);主机运行需 `--noconftest -o addopts=`(仓内 params_pyx.so 为设备 aarch64 产物,macOS 无法加载,同 run_host_tests.py 原理)。

## ④ 夜间续探(2026-10-18,全帧缓存 /tmp/seg1005.pkl,六项分析)

前置:`radar_seg_extract.py` 一次性解 415 段(0 错误,0x109 帧 742451,六帧全 8 字节+HUD 档位类+vEgo,pickle 132MB),以下全部离线复用不再读 6.9G。

### ④a 247 翻案:**不是持续超量程钉死,是丢失边缘瞬态码**(风险#2 全销)
- 全库 b7==247 仅 **48/742451 帧**(尾段 235-247 中占 1.5%,比 246 还稀);真实饱和钉死应在顶值堆积,形态相反。
- 连续 ≥2 帧的 247 episode 共 10 段,**全部 ≤0.5s**,全部集中在 d3da5795a9 高速段(class 4、vEgo 26-35m/s)、出现在 **246↔0 目标丢失边缘**(8/10 跳 0,2/10 续 246),且钉死帧内 b6 恒 0x7F(无效态,见④c)。
- **连续最大值 246**(feet 口径 75.0m);247=边缘过渡/无效伴随码。建模与 T8 窗过滤按 `b7>=246 → 视为量程边缘不可信` 处理,原「疑 247=超量程」表述作废。
- 1-8 空洞 100% 复现:539 个「零段后首个非零」帧**无一**落 1-8(首值最小 10,众数 15/29)——量程下限结构性存在。

### ④b τ 阶梯:feet 获间接佐证(非定夺)
- 档位边界 r=b7/vEgo 池化中位 {c2:2.92, c3:4.79, c4:6.61} c/(m/s)(取自 309 对有效翻转的新旧平台)→ **τ@feet={0.89,1.46,2.01}s ≈ 工业标准 ACC 时距阶梯 ~{1.0,1.5,2.0}s**;τ@0.25={0.73,1.20,1.65}s 不成套。
- 类内稳态 hold 拟合(Theil-Sen,|a|<0.4m/s²、v∈(3,25]、b7≤200):c2 斜率中位 2.89(与翻转回归 2.85 一致)、c3 4.03(与 4.06 一致)、c4 被高速/前车拉开污染(p75=17.4,弃用)。
- **纪律不变**:阶梯形状本身 u-不变,绝对对齐只是旁证;风险#1 仍开口,路测实测一次跟停驻停距定夺(4.5m≈feet / 3.7m≈0.25)。

### ④c b6 定分:**纯二值状态字节,粗细分解假说证伪**(风险#8 销)
- 10-05_07 全库 676035 帧中 b6 **只有两个值 {0x7F, 0xFF}**(仅 bit7 之差):lead 窗 50.2%/49.8%,class-0 窗 98.1%/1.9%。b6 不含任何距离信息。
- 「b6/b7 平行递减(段偏置 -6/-12)」是 hour_logs 特定观察或误读,**在 415 段上不复现**——「粗细分解 or 双目标」建模路终止;ObjValid 假说(b6[7])保留,升为"纯标志位"。

### ④d 0x101 b7 更具体:表状量化第二槽(风险#6 维持开口)
- 非连续距离:lead 窗 169 distinct 但集中(216/0x80/0xD8 系);**远距离钉 0xD8=216**(b7_109∈121-200 → 57.7%,201-247 → 63.7%),近距带簇 29-34,与 b7_109 非钉值 corr≈0.60、比值中位 1.67 漂移。
- **时距假说(b101∝b7/vEgo)证伪**(corr=-0.318);class-0 中 b101 与 vEgo 无稳定相关(med -0.13)。结论:0x101/0x105 b7 为第二槽独立状态/距离混合信号,**不可作 0x109 回退或校验源**(与风险#6 既有结论一致、形态已具体化)。

### ④e T8 设计统计(审2 的 10 次翻转让位给全量)
- **b7 越界领先翻转 309/309(100%)**:309 对有效新旧平台的档位翻转(2↔3/3↔4)中,b7 在 8s 回看窗内全部先越过平台中点;翻转时刻 b7 平均只走完平台距离 37-59%(即提前量常超窗、需自适应回看)。
- **丢失路无提前量**:HasLead→0 后 b7==0 的滞后中位 0.02s(p90 0.06s)——0x32D 与 b7 是上下游同拍,丢失方向 b7 不领先。
- **捕获路提前量巨大**:HasLead↑前 b7>0/存在位=111 已存在 ≥10s(窗截断)——即 --142 机制的普遍化,雷达远早于 HUD 见目标。
- **静态阈值不可用**:以池化 r 中点的单一阈判档,类内 hold 误穿率 29-60% → T8 实现须用**新旧平台自适应穿越 + 迟滞/时间窗去抖**,且只用单调序与相对量(单位未钉纪律不变)。

### ④f 存在位「卡死假警」翻案:全是真实目标(风险#2 连带)
- fp>5% 的 12 段逐一解剖:**11/11 的 ghost 窗(pres==7 & class==0)b7>0(零率 0.0%)**;传输层无恙(40ms 帧率、CRC 100%)。
- 原「4 段卡死 100%」之首 --234:窗口内 **vEgo 恒 0 + b7 冻结在 15 = 静止前车驻停平台**(与 13 起跟停事件的 15c 平台同值,feet 4.6m/0.25 3.7m)——卡住的是 HUD HasLead,不是雷达。--142(b7=41 平 62s)、7a78691537--6x 系(b7 方差 4-20)皆为真实雷达可见目标。
- 过滤规则收敛为 **presence==7 AND b7>0**(纯「有在位无距离」ghost 在本库 0 例);「自车在动 + b7 冻结」的危险形态全库未出现,留作路测观察项。

## ⑤ T8 落码(2026-10-18 深夜,门控默认关=启用走路测)

- **链路**:BYD CarState 自带独立 CANParser 解 265/266(bus1,`update_radar_gap` 由 CarInterface.update 以 100Hz 驱动)→ **刻意不进 get_can_parsers**,canValid/canTimeout 聚合不受 bus1 影响 → `RadarGapMonitor`(opendbc/car/byd/radar_gap.py):u-free r=b7/vEgo 对分档下界 {2:1.99,3:3.86,4:5.70} c/(m/s) 做**向下穿越+斜率(≥0.2/s)+0.5s 去抖+1.0s 迟滞**判定,过滤=④f(presence==7 AND b7>0)且 b7≤245(④a 边缘带不可信);b7 由 dbc feet 因子还原为原始计数(单位不进此路)→ `CarState.radarGapClosing @61 / radarGapNew @62`(运行时 capnp,设备可直接 OTA)→ desire_helper 三级 `BydRadarGapGate` raw param(0=关,默认;**1**=closing 等同盲区命中重置 clear 窗+警中禁发起;**2**=追加"变道中冒出新目标且正在逼近"starting abort)。
- **abort 语义保守化**:仅新目标退——0x109 无 yRel(b3 UNKNOWN),本车道逼近(该继续躲)与目标车道逼近(该退)不可分,退回本车道撞向逼近车是更坏猜测;丢失方向同拍(④e)永不作为放行依据;finishing 不 abort(10-11 纪律)。
- **常数定标**(tools/byd/radar_t8_replay.py,415 段缓存回放,/tmp/radar_t8_replay.txt):稳态 hold 误阻塞 **8.3%**(④e 静态阈 29-60% 的一个数量级以下;分布偏态 med=0/p90=26.7%);closing 翻转预警覆盖 **4→3 33.3%(91/273)、3→2 22.9%(22/96)**,捕获事件提前翻转中位 **4.8s**(p10 1.4/p90 12.9);告警 22 次/h、新目标 41 次/h;opening 方向"误捕"9% 为 8s 窗内相邻 closing 重叠非独立误报。扫描过 margin 0.12/0.05、set_t 0.4/0.5/0.8、slope 0.15/0.2/0.3 取平衡组。
- **测试**:selfdrive/car/tests/test_byd_radar_gap.py **18 例**(穿越/去抖/迟滞/低噪摆动不误报/存在位/247 带/新目标时效/CarState 真解码/gate0 行为逐字节不变+gate1 阻止发起+gate2 新目标 abort/旧目标不退);host runner 全套 **105/105**、byd dbc/carstate 套件 **38/38**;radar_interface 主源、values.py、控制路径未动。
- **待办**:路测先开 gate=1 观察阻塞体验与告警时机(对照 b7/驻停距实测量一并钉单位)→ 视情 gate=2;后方数据源缺口与 T7 设备探针不在本门控覆盖内。

## ⑦ b3 yRel 探测收口(2026-10-19 凌晨,tools/byd/radar_b3_mine.py + radar_b3_lat.py;⑥=后向/bus3 定案见 docs/byd-radar-rear-bus3-2026-10-18.md)

- **一阶(415 段)**:b3 在 lead 态确实离开 0x7F(class2/3/4 非 127 占 75-90%,|off| med 1-2/p90 4-15/max 60+,class0 96.5% 恒 127=无效值模式);lag-1 几乎不动(med|db3|=0);出现事件 t+0 散布 ±6、1s 内收敛 ±3;13 起驻停平台 b3 med=128。**形态全部与「某慢变有符号量、中心 127」一致**。
- **二阶判决(横向回归)**:自 steeringAngleDeg(ratio 19.5/轴距 2.765)双积出横向位移,取目标连续可见窗口跑 b3~Δy 回归:123 事件 slope med=**0.0 LSB/m**、r2 med=**0.04**、r2>0.5 仅 3/121 且斜率 -1.4/+0.2/-0.1 符号互斥;其中 121 个漂移窗 |dy| 中位 **12.7m**——若是车道级 yRel,任何合理比例(0.1~0.5m/LSB)都必须产生 ±35~120 LSB 扫掠,**未发生**;仅有的 2 个真变道事件(blink=1、|dy|2.9/4.1m)db3=+4/+3 且符号不随位移方向翻转。
- **判定:b3 非横向距离,不可作分道输入**;候选语义收窄为相对航向角/慢偏置类(路测无关)。连带结论:**T8 level-2 的「新目标才退」语义即终态**——可及总线上不存在分道量(0x35x track 路 0 填充、0x101 表状量化、b3 非横向,⑥ 又排除后向总线),真分道两条路:MPC 私有 CAN 引线,或 **⑥ 新发现的 67ms J1850 多目标槽表**(0x380s/0x340s/0x3C8s,前向强相关 r=0.78-0.87,若含多目标则自带分道能力——登记为下一轮逆向,见 docs/byd-radar-rear-bus3-2026-10-18.md)。

## ⑧ hour_logs 控车段 vLead 盲区补测 + 距离族跨固件复验(2026-10-19,tools/byd/radar_hour_census.py + radar_hour_mine.py;专档 docs/byd-radar-hourlogs-vlead-2026-10-19.md)

- **动机**:本文件与 slot67 专档的全部 relV 判死都用「关闭速度 −d(b7·u)/dt」监督——稳态跟车段其方差退化 ~0,**绝对 vLead 字段对该监督系统不可见**(理论盲区);hour_logs 4 段恰是长稳态跟车(10.6/10.5/14/2.3min),为盲区假说最理想测试床。
- **距离族正结果**:0x109 b7/presence==7 与 HUD 一致复验;块 C 0x380 x12 对 b7 r=+0.744(全局窗、槽位切换稀释,同对象窗 slope≈8 判据维持);**67ms 槽表 9-28 固件已在播**(0x3E4-0x3FF 占位帧活跃,时间线前推);新发现头帧 **0x301/b67LE 伴生距离量 r=+0.724**(登记开口)。
- **vLead/vRel 三通道全灭**:vEgo 全量/去趋势高频/驻停窗(窗内去均值)× 110 地址 × 2420 字段变体,无稳定速度族;最强候选 0x381/b3 驻停窗 r=−0.87~−0.97 判**时间趋势混杂**(slope 每窗 −45/−21/−13/−8 不守恒、r(t)≈r(v)、r(aEgo) 弱、与 0x10A b3 不同源)——「无 vRel」判死穷尽化为四类速度语义监督全灭,盲区假说证伪。
- **f8_8/f9_12 = 自由巡航基线段**(presence 恒 0/b7 恒 0/HasLead 恒 0):无目标静默复验成立,登记为 free-space 伪影审查对照段。
- **9-28 新地址登记**:0x100(10Hz)/0x103/104/107/108(0x10x 伴生)/0x302/0x350-354(tangdm 槽位非恒空但 track 全 0 填充不变)/0x569——五通道扫描均无监督信号,登记不动;0x35x 与 0x10x 内容不同源(帧差 64/96 LSB)。
- 控制路径与 dbc 零变更;差分 b7 作 vRel 代理的风险#4 定性维持(hour_logs closing 仅 125 帧,不足以再标定)。

## ⑨ u 参数化(BydRadarDistScale)+b7 计数路径解耦(2026-10-19)

- **u=0.25 钉死同日,owner 要求留运行时可调口**:`BydRadarDistScale` raw param(默认 0.25,UI Developer 面板按钮组 0.20/0.25/0.3048/0.30,写路径走 `put_raw_param` raw 文件——设备 params_pyx.so 未注册该 key 也可用,先例 LaneChangeAssistSpeed/BydRadarGapGate);`raw_params.py` 新增 `get_float_param`/`put_raw_param` 通用 helper。
- **连带发现并修复 carstate 一致性 bug**:`update_radar_gap` 原先 `LongDist_109 / 0.3048` 还原计数——factor 改 0.25 后会得 0.82× 错位计数(gap 测试在 mac 因 params_pyx 无法运行而漏网);修复=b7 改走 dbc 新增的 **factor-1 `MSG_109_B7` raw 孪生信号**(与 B5/B6 占位同风格、与 LongDist_109 同位并存),计数路径与任何 scale 永久解耦——u 只影响米制侧(LongDist_109 解码值/未来 track 合成/距离显示),**RadarGapMonitor 的 r=b7/vEgo counts 空间对 u 的任何调整零响应**。
- 测试:010x dbc 套件 11/11(mac,新增 raw twin scale-free 断言);gap 套件新增 `test_b7_counts_scale_free`(mac 环境跑不了 params_pyx 链,设备/host-runner 侧执行);mac 冒烟:MSG_109_B7=15 精确、LongDist_109=3.75、端到端 closing 序列触发 ✓。

## ⑩ alpha 纵向验证回合:雷达合成 lead 上线(2026-10-19,radar_interface LIVE)

- **`CP.radarUnavailable = False`**(interface.py,首次):radarState 首次有 BYD 合成 lead。stock ACC 模式下 radarState 不进任何纵向路径(厂商 MPC 独占),flag 翻转在该模式行为中性;**AlphaLongitudinalEnabled 时合成 lead 直接进纵向 MPC**——本回合路测目标。
- **合成优先级链**(radar_interface.py):slot 路(0x351-354,私有 CAN 引线后自动点亮)→ **b7 连续 lead(0x109 presence==7 门控 + dRel=b7×u + med 差分 vRel)** → HUD 档位(纯位置 fallback,vRel=NaN,radard vel_sane 永不选中——有意设计)。b7 主源语义:④a/④f 门控(0<b7≤245、presence==7)、无效帧清空差分历史(vRel 永不跨目标丢失边缘)、0.4s med 窗(radar_dvrel_quality:稳态≈0/延迟 0.24s/跟停 r=0.56-0.90)。
- **u 注入**:card.py 构造 RI 后读 `BydRadarDistScale`(clamp 0.1-0.5)注入 `RadarInterface.radar_dist_scale`——opendbc 保持 params-free;`RADAR_HUD_BARS_TO_M` 注释改判:**档位=时距带**(τ {0.71,1.02,1.4}s@u=0.25),静态米映射仅末位 fallback。
- **测试**:test_byd_radar_interface 扩至 **18/18**(mac 实跑):b7 压 HUD 档位/稳态 vRel≈0/closing vRel 负且有限(NaN 会让 radard vel_sane 失配——HUD fallback 永不可选的原因)/presence+247 门控五连/gap 历史重置/inert 断言;旧 HUD/slot 套件全保持。
- **路测验证清单**:①stock ACC 模式先跑——swaglog/connect 看 radarState.leadOne.dRel 与真实车距对照(顺带钉 u);②再开 AlphaLongitudinalEnabled——纵向 MPC 吃合成 lead 的跟车/停止表现,重点观察 lead 瞬变(加减速)时 vRel 0.24s 延迟的影响;③BydRadarGapGate 可同时 1/2 观察变道门控。

## 证据链(416 段报告 + 三方审查)
- **档位单调分离**:113/415 段含全三档(≥50 帧),0x109 b7 跨段 mean(c4)>mean(c3)>mean(c2) 严格占 90.3%(102/113);类均值 c0/c2/c3/c4 = 1.35/31.2/49.7/112.9。同项 0x101 b7 仅 24.8%、c0 均值 30.6 非零——0x101 b7 不是该距离字段。
- **无目标归零**:0x109 b7 class-0 零率 96.3%(279985/290824),f8_8/f9_12 整段恒 0(uniq=1)——时间滚动计数器不可能;残留 3.7%(10839 帧,max 247)为捕获/丢失瞬态。
- **计数器/时间共线证伪(审1)**:类内 lag-1 自相关 0.897-0.968、Δ 双向、mod-256 回绕 0 次;档位跳变窗 b7 随车距**反向**(c3→c4 爬 49→110,1.2s 后 c4→c3 落 83→38),排除时间共线。
- **HUD 回声证伪(审2,CONFIRMED)**:H(b7|档位)=2.244 bits(回声自 3-4 档应≈0);10 次档位翻转 8/10 次 b7 提前离开旧平台并穿越中间值(如 4→3 走 [27..37]);恒档 hold 内 b7 逐帧变化率 8.7-14.6%,而 class-0 hold 0.02%——b7 是档位的上游驱动量。
- **单位≠1m(审3+建模)**:1m 假设隐含时距 3.4-10.9s、落 1.0-2.5s 跟车带比例 0.0%;减速窗 57% 内 b7 收敛速率>自车速,物理上界 u≤~1m;feet(0.3048) 下 fa_2 77% 落带(f4_16 29%);监督拟合 fa_2 slope=0.269m/计数、r²=0.691 为全 6 帧唯一跨段稳定距离型。
- **无 vRel**:全部字节/字节对对 -db7/dt 监督无一 |r|>0.4;b6/b3 与间距变化率零相关——Doppler 只在已证 0 填充的 0x35x 私有槽。
- 脚本 exit 0,仅 3 条 rlog Corrupted-events 警告非致命;多样性证据当时只来自 f4_16/fa_2 两个跟车段——**已由 415 段全量复跑替代并强化**(见「定案补录①」:distinct 240、H 中位 3.815 bits、跟车 hold 变化率中位 9.99% vs class-0 0.0000%)。

## 字段假说表
| 字段 | 假说 | 置信 | 关键证据/反例 |
|---|---|---|---|
| 0x109 b7 | LongDist_109:连续前向距离,**×0.25m(count,2026-10-19 owner 拍板钉死)**,0=无目标;**247=丢失边缘瞬态码非持续钉死,连续上限 246**(④a) | 高 | 单调 90.3%、零率 96.3%、领先档位翻转 309/309(④e)、r²=0.69 |
| 0x109 b6 | **纯二值 {0x7F,0xFF},仅 bit7=Valid/捕获标志,不含距离**(④c,粗细分解证伪) | 中高 | 10-05_07 全库 676035 帧无第三值;lead 窗 0xFF 占 49.8% vs class-0 1.9% |
| 0x109 b3 | 慢变有符号量(中心 127=无效值),**非横向距离**(⑦:123 个横移窗 slope med 0.0 LSB/m、r2 med 0.04;真变道 db3 符号不随方向翻转);候选=相对航向角/慢偏置 | 判定高/语义低 | lead 态 75-90% 离 127、lag-1 恒 0、驻停 med 128;|r(b3,b7)| med 0.218(推翻早先 <0.11)但与 Δy 零相关 |
| 0x101/0x105 b7 | 表状量化第二槽:远距钉 0xD8=216、近带簇 29-34、非钉值 corr≈0.60、时距假说证伪(④d),不可作 0x109 回退/校验 | 低 | 24.8% 单调、class-0 均值 30.6 非零、corr(b7/vEgo)=-0.318、比值中位 1.67 漂移 |
| 0x10A b3[7:5] | Presence:111=前向可见目标 | 高 | lead 96.5-97.5% vs no-lead 0.0-2%(全 384 位唯一) |
| 0x101≡0x105 | 全 8 字节含 b0 校验 99.7-100% 相等 = 传输重发 | 高 | 不得当 0x109 字段冗余用(审1②/审3c) |
| 0x102/0x106 b4 | 通道 A/B 慢物理量(~152/~103)= 传感器健康对 | 角色高/义 UNKNOWN | 与 lead/no-lead 无关 |
| 公共层 | b0=私有 CheckSum、b1=Counter、b2[7:4]=4-bit 滚动计数、109 b4/b5=SET_ME pad | 中高 | 同载荷 0 例 b0 冲突;低 nibble 每帧+1 |

## 与既有定案的冲突(需修订处)
1. **10-05_07"bus1 新面 0x101-0x10A 皆计数器伪影证伪为状态广播"——须改写**:该判定对 0x10A b5/b7、0x102/106 仍近似成立(状态/健康),但对 0x109 b7 被三方审查明确推翻;当年用于证伪 0x38x 的跨段单调检验在本帧上反而**支持**物理量(90.3%)。
2. 0x38x 证伪、0x35x 恒空、0x374 零帧——**维持不变**,且"0x10x 无 vRel"与"Doppler 在 0x35x 空槽"互证。
3. "双路 radar_interface(track 优先/档位回退)":track 优先仍无源;但"档位回退"上可新增 0x109+0x10A 连续距离路;此前隐含的"0x101/0x105 可作 0x109 校验副本"叙事作废。
4. `RADAR_HUD_BARS_TO_M{2:13,3:22,4:35}`(values.py:212):建模证 HUD 档位是**时距带**非固定米带(类内 corr(b7,vEgo)=0.70-0.95),该表不可作距离标定真值。
5. 0x32D 档位合成仍成立——b7 领先 0x32D 是上下游关系,非冲突;审2 定位翻转时刻精确吻合(±0.01s)。

## 落地建议
- **T8 距离门控**:用 0x109 b7 作 0x32D 档位上游的连续量,给 BSD 0.5s 发起前 clear 窗与 starting 途中 abort 提供"逼近速率/穿档提前量"增量——但单位未钉,先只做**单调性与相对序**检测(闭合判定、提前于档位翻越),标定后再升级米制阈值;c0 瞬态与 247 须按 lead 配对窗过滤。**④e 已给设计数字**:越界领先翻转 309/309(100%)、翻转时刻 b7 只走完平台 37-59%(提前量常超 8s 回看窗,须自适应);丢失路 0.02s 同拍无提前量(clear 窗的「消失」方向不能靠 b7);捕获路 b7 领先 ≥10s;单一静态阈值误穿 29-60% → 必须新旧平台自适应穿越+迟滞/去抖,禁固定计数阈。窗过滤按④f=presence==7 AND b7>0。**⑤ 已按此实现落码(门控默认关,路测启用)**。
- **track 路**:不可整路改接(无 vRel);可选建"0x109 dist + dt 差分 vRel(25Hz,噪声大)"中间路,track 槽继续默认惰性。
- **DBC**:**已入库**(2026-10-18,任务③):byd_general_pt.dbc BO_ 257/258/261/262/265/266,头部保留 HYPOTHESIS 注释与单位未钉声明,命名对齐 `_tang` 角色_来源风格(后缀取帧号 _109/_10A 等,避开与 0x35x 混淆的 _mrr);详见「定案补录③」,消费端=无(回归测试 test_byd_radar_010x_dbc.py)。
- **门禁**:维持"路测验证前不动控制行为"——设备 T7-T9 探针 + 415 全量重跑审2(a)(c)检 + 一次已知车距标定,三件齐前 radar_interface 主源不变。

## 风险清单(2026-10-18 复跑/入库后销账状态)
1. **部分销账(②-补收窄至二选一,④b 增 feet 旁证)**——1.0 与 0.45/0.5 **均已排除**(13 起跟停事件逐窗硬上界 u≤0.42-0.44);④b τ 阶梯:档位边界 b7/vEgo×u={0.89,1.46,2.01}s@feet ≈ 工业标准 ~{1.0,1.5,2.0}s,0.25 下 {0.73,1.20,1.65}s 不成套(**形状 u-不变,仅旁证非定夺**);路测实测跟停驻停距定夺(15c 平台→4.6m≈feet / 3.7m≈0.25);dbc 以 0.3048+HYPOTHESIS CM_ 入库,路测标定前禁止控制读取。
2. **已销账(④a+④f)**——0=无目标已证;**247 翻案=丢失边缘瞬态码非持续钉死**(48/742451 帧、10 episode 全 ≤0.5s、集中在 246↔0 边缘,连续上限按 246 处理);c0 残差与「存在位卡死 4 段」一并翻案:**11/11 ghost 窗 b7>0**(CRC/帧率无恙),--234=vEgo 恒 0 + b7=15 驻停平台(静止前车真实可见),卡的是 HUD HasLead 非雷达;过滤规则=**presence==7 AND b7>0**,「自车在动+b7 冻结」危险形态全库 0 例、留路测观察。
3. **已销账**——多样性/回声证据从 2 段升至 415 段全量(①:distinct 240、单调 90.3% 于 113 合格段、hold 变化率跟车 9.99% vs 无跟车 0.0000%),解析 0 致命错误。
4. 无 vRel;b6 纯二值(④c)、b3 非横向(⑦)——track 原语义与分道输入在可及总线均无;分道改押 ⑥ 的 67ms 槽表或多目标。
5. **销账(2026-10-18 重跑)**——b0 CRC **已破**:SAE J1850 CRC-8(poly 0x1D/init 0xFF/无反射/xorout 0xFF,载荷 b1..b7),hour_logs 2933 唯一载荷 100% + 独立日 hour_logs_2 80100/80100;byd_checksum 族确不可复用(不同算法),dbc CM_ 已命名。首轮 agent 试打未中系参数集不全(缺 xorout 0xFF 变体),作废。
6. **开口(④d 已具体化)**——0x101 跟踪策略与 0x109 不同,不可作回退/校验源(dbc 注释同口径);形态已明:表状量化、远距钉 0xD8=216、近带簇 29-34、时距假说证伪(corr -0.318)、非钉值 corr≈0.60——第二槽语义(预测目标/旁车?)仍待建模。
7. AccState 门控未细化(ACC_ACTIVE 子集或再移标定)。
8. **已销账(④c)**——b6/b7「平行递减/粗细分解」假说在 10-05_07 全库证伪:b6 纯二值 {0x7F,0xFF}(676035 帧无第三值、仅 bit7 之差),不含距离;-6/-12 偏置属 hour_logs 特定观察或误读,不再建模。
9. 既有依赖 HUD_BARS_TO_M 换算的代码路径整体存疑。
10. **已销账**——415 段全量解析 0 致命错误(3 条 Corrupted-events 警告未阻断)。
11. **更新(⑤)**——265/266 现由 BYD CarState 的独立 gap parser 解析(遥测+门控输入),不再"零消费端";但控制读取三级门控 `BydRadarGapGate` **默认 0=行为不变**,路测钉 u+开 gate1 前禁止升级;257/258/261/262 仍无消费者。原纪律(先路测钉 u、presence 新鲜度过滤已按④f 落实=presence AND b7>0)不变。
## carrot 差异(2026-10-20 移植到 carrotpilot/openpilot carrot-byd 记录)
本档与 tools/byd/ 由 sunnypilot dev-tici `817db989d3`(雷达合成 lead 上线)直抄移植,控制语义等价,以下六处**有意差异**(变更档案:openspec/changes/byd-radar-synth-lead-port/):
1. **HUD 档层 measured=False(唯一行为面偏离)**。源工程靠 vRel=NaN + sunnypilot radard 的 vel_sane 恒 False 保证档位 fallback 永不被选。本仓库 radard 为独立实现(carrot/radar_motion/):`snapshot_live_radar_points` 用 `_finite()` 把 NaN **归零**而非拒绝——档位点会以 vLead=vEgo 伪同速参与 vision 关联,若与真静止视觉前车关联上,被采纳 lead 的速度是错的(MPC 不刹),这是源工程保证覆盖不到的「拉坏纵向」路径。改以本仓库快照的第一道闸 `measured` 实现同一设计意图:档位点照发 liveTracks(显示/遥测不丢),radard 整点丢弃。测试 `test_byd_radar_interface.py::test_lead_present_makes_one_point` 断言随之反转。
2. **b7/slot 层无需打 radarSource 标签**:本仓库 car.capnp `RadarSource` 枚举默认值恰为 `frontRadar @0`,BYD 合成点天然进入 mode-1 的 front 选择集(源工程 radard 不读该字段)。
3. **capnp 字段换号**:`radarGapClosing @104`/`radarGapNew @105`(源工程 @61/@62 在本仓库已被 activateCruise/latEnabled 占用;本仓 CarState 已用至 @103)。
4. **u 注入点=radarcan.py 而非 card.py**:本仓库雷达接口在独立守护进程 `carrot/radar/radarcan.py` 构造(启动+输入故障复位两处都汇于 `_rebuild_radar_interface`,注入在彼处,hasattr 门 + clamp [0.1,0.5],opendbc 保持 params-free)。参数改动**重启 radarcan 进程生效**,不触发 CarParams 重生成(BYD 红线:断 0x316=锁 EPS);Developer 面板提示文本已注明。
5. **T8 门控落点=desire_helper_byd 三 hook**(本仓库分层规范:标品 desire_helper.py 零 BYD 行为);level-1「警中禁发起」经 clear 窗每帧归零隐式成立(本仓 `_clear_window_ok()` 无 carstate 形参)。
6. **`CarInterface.update()` 的 gap 写出带 try/except 守卫**(本仓库 card 不崩纪律强于源工程;最坏 gap 字段恒 False)。

**V2/V3 路测观察项(翻转后本仓库特有,源工程无)**:
- (a) `effective_radar_track_mode(byd, radarUnavailable=False)` → mode 1,而本仓库 `vision_only_lead_allowed(mode>0)=False`:vision 回落须过「强可见」门(`_central_vision_fallback_allowed`:概率+dPath 阈)。极端场景(b7 过 246↔0 丢失边缘 + vision 低置信)leadOne 可短暂 None 而现状(-2=纯 vision 无门)不会——统计翻转前后 leadOne=None 占比。
- (b) BYD 点从不设 aLead/jLead(源工程注释「radard KF 自推」):本仓库快照把缺失读成 0.0 喂 LeadDynamics,V2 起观察 aLeadK 表现。
- (c) HUD 档层 measured=False 丢弃生效确认(liveTracks 有点、radarState 不消费)。

**验证金字塔(路测门禁,§⑩ 路测清单的本仓库版)**:V0=Mac 三套件 49 例 + 归档段离线回放(已确认可行:hour_logs 4 段/415 段 rlog.zst 可喂本仓库纯 Python CANParser);V1=gap 链车上遥测(不过 radarUnavailable 闸);V2=stock ACC 下 dRel/vRel 同屏标定(AlphaLongitudinalEnabled=0,行为中性,观察项 a/b/c);V3=alpha 纵向跟停+瞬态(源工程 ⑩ 清单剩余项)。

## ⑪ HUD 档位对撞回归（2026-10-21，byd-offline-prep A，未标定）

用 415 段实车 rlog 把 HUD 档位（0x32D LeadingDistance）与实测跟车距离（0x109
b7·u，u=0.25 钉定）对撞，验证 `RADAR_HUD_BARS_TO_M` 占位表。脚本
`local_docs/byd/offline_prep_2026-10-21/hud_bar_fit.py`，产物 `hud_bar_fit.json`。

窗口判据（design D1）：跟车 engaged（AccState∈{2,3,5} ∧ HasLead=1）+ presence==7
+ 2≤b7≤245 + vEgo>1。按 (路线, 档位, vEgo 分层) 分桶统计 b7·0.25 的中位数/IQR/样本秒数。

**结论：四档全部不满足更新判据（spec 场景「样本不足」），表值保持不变。**

| 档 | 样本秒 | 中位 | 跨路线中位数偏差 | 判定 |
|---|---|---|---|---|
| bar1 | 58.3s | 11.75m | 单路线（不足60s） | FAIL 样本 |
| bar2 | 991.5s | 9.75m | 105.1%（{2:16.0, 3:5.75, 5:6.75}） | FAIL 偏差 |
| bar3 | 5557.3s | 9.75m | 71.8%（{2:14.5, 3:7.5, 5:12.5}） | FAIL 偏差 |
| bar4 | 7397.7s | 30.75m | 79.7%（{2:36.5, 3:12.0, 5:30.0}） | FAIL 偏差 |

**vEgo 漂移形态坐实**（design D1 升级条款触发）——同一档位中位数随车速单调上升：

| 档 | 1-5km/h | 5-10 | 10-20 | 20-40 |
|---|---|---|---|---|
| bar2 | 5.5 | 6.5 | 10.5 | 17.75 |
| bar3 | 7.25 | 9.75 | 15.25 | 20.25 |
| bar4 | 14.0 | 15.5 | 31.0 | 45.0 |

即 HUD 档位是**时距带（τ s × vEgo）而非固定距离**，与 ②/⑧「档位是厂商融合后的
量化」一致。单值米表在结构上就是错的形状。

**dry-run 独立佐证**（V0，见任务 3 产出）：route2--40 段 HUD 兜底点 dRel 与同期
b7·0.25 残差中位 **−14.75m**（表值系统性偏大），route5--30 段 +18.75m 聚于 22m
单值——表在低速/高速两端的错标方向不同，不可用一个标量修。

**处置**：`RADAR_HUD_BARS_TO_M` 注释登记本节证据 + 「UNCALIBRATED」声明（不改数值）。
「档×速二维表」升级提案登记于 `openspec/changes/byd-radar-synth-lead-port/observations.md`。
V2 上车复核仍用真实尺子（原门禁不变）。measured=False 闸使表在 radard 侧永不被消费，
故错标不直接进纵向控制——但 liveTracks 显示会用，UI 距离读数在跟车时会偏大。
