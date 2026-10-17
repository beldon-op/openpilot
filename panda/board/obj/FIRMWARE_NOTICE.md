# panda 固件说明（本目录二进制来源）

当前状态：**本目录固件 = 自建 BYD 固件（含 byd 安全模式 @35）**，构建源码基线
与 sunnypilot 主仓库同源（0.9.x 时代上游 panda `7287ff0c` + byd 移植，
HEAD `fee17db6`，UNO 板型强制 + USB VID 0x3801，DEV 签名）。

| 文件 | 说明 |
|---|---|
| `panda.bin.signed` | 主固件 F4（DEV 签名，byd@35 + 纵向 param bit，sha256 `dcb30487…739cdd25`）|
| `bootstub.panda.bin` | 配套 dev bootstub（同 VID，接受 DEV 签名 app，sha256 `b467926d…11e0dc48`）|
| `panda_h7.bin.signed` / `bootstub.panda_h7.bin` | H7 构建（本兼容板未用，随构建产物入库）|
| `version`（本地构建时）| `DEV-fee17db6-DEBUG` |

与主机侧的对应关系：本目录固件的 BYD safety 行为（TX 白名单、LKAS 限幅
18/帧 / driver allowance 120、0x32D bus-2 主状态 + 60 帧 fall hold、
`BYD_PARAM_LONGITUDINAL` bit、选择性中继）与本仓库
`opendbc/safety/safety/safety_byd.h`（libsafety 测试已覆盖）逐项一致。
注意两者框架代际不同（固件为 0.9.x 平铺结构、fwd 为 bus/addr 签名；
主机为 packet-fwd 结构且 carrot 的 `aol_allowed` 旁路了 engaged 门，故
safety_byd.h 自带 gate），逻辑等价，源码不能互换编译。

**USB VID 0x3801**：0.10.1 的 C++ pandad 数据路径只匹配 comma 注册 VID
（`panda.cc` `idVendor == 0x3801`），旧 VID 0xbbaa 固件会被无视并落入 SPI
兜底死循环（`SPI: timed out waiting for ACK`）→ UI「无 PANDA」。python 库
两个 VID 都收，python 层测试全通不代表 C++ 通路可用。

**0x32D 总线**：Song Plus DM-i 的 DiPilot 相机在相机侧总线（bus 2）发
0x32D；固件的 `pcm_cruise_check` 与 RX 检查必须读 bus 2，bus 0 永远收不到
会让 pcmCruise 下 controls_allowed 永不置位。

**板型强制**：c3l 兼容板的 GPIO 检测不匹配任何官方型号（官方代码
`assert_fatal(hw_type != UNKNOWN)` 挂起），固件在 `detect_board_type()`
末尾强制 UNO（0.9.x 基线，USB 传输）。

## 刷机与更新

- 常规更新：本目录固件随 OTA 分发；pandad 签名校验自洽（期望签名读自同
  目录文件）→ **无需手动刷写**。panda 芯片固件不受系统刷机影响；panda 意外
  进 DFU/bootstub 时 pandad 自动刷回本目录固件。
- 首次安装 / 排障恢复流程见 `docs/byd-panda-fw-build.md`（第五节）与
  `docs/byd-panda-recovery.md`。
- 重新构建：持久化构建树 `~/Documents/op/panda_fw_base`（工具链
  `~/Documents/op/arm-tc`），`scons -j8 board/obj/panda.bin.signed`；
  产物与入库文件逐字节一致（可复现构建已验证）。

**主机侧 health 兼容（勿移除）**：本固件 health packet 为 **v16**（无
`wake_on_can` 字段），而库在 a3fbd86f 起期望 v17。`panda/python/__init__.py`
按固件上报版本双解析（`LEGACY_HEALTH_PACKET_VERSION`/`LEGACY_HEALTH_STRUCT`，
`ensure_health_packet_version` 允许 16/17，v16 补 `wake_on_can=0`），单测
`test_panda_health_v16.py` 守护。**合并上游/升级库时必须保留 v16 分支**，
否则 pandad 在 `health()` 上版本不匹配崩溃循环 → UI「panda 否/离线」
（2026-10-17 实车复现过一次）。签名校验对此无感（新旧固件签名可相同），
不会触发自动重刷——不要试图靠重刷解决版本不匹配。

**C++ 侧 health 兼容（勿移除，2026-10-18）**：C++ `pandad` 直接 include
`panda/board/health.h`，v17 结构体比 v16 包多 1 字节——按 v17 读 v16 包时
`wake_on_can` 之后所有字段错位一字节：`harnessStatus` 读到固件
`safety_mode`（冻结固件 silent=19）→ 枚举越界，hardwared 在
STATUS_PACKET `to_dict()` 上 "Member was null" 崩溃循环 → deviceState
断流 → 设备不进工作状态（2026-10-18 台架复现）。修复：
`health.h` 增加 `legacy_health_t`（v16 布局），`pandad/panda.cc`
`Panda::get_state()` 按 control_read 实际返回字节数归一化
（58=v16 映射进 v17 结构，59=v17 直读，其它长度拒绝发布）。
合并上游时保留该分支；改 `health_t` 结构时同步更新 `legacy_health_t`。

## 不要做的事

- 不要用 sunnypilot 或本仓库 `panda/` 现编的新代固件刷这块 c3l 兼容板：
  0.10.x 基线 app 刷入后不枚举 USB（未深查，弃用），新代框架结构也不同。
  **已证实例外**：sunnypilot 树构建的 `DEV-80a7a508-RELEASEd`（2026-10-14，
  产物在 `~/Documents/work/op/carrotpilot/byd/openpilot/panda/board/obj/`）
  实车可正常启动（2026-10-17 确认）——"不枚举 USB"结论只对当时试过的构建
  成立，不是新代固件的通例。但它与 carrot 签名不同，carrot 的 pandad 会刷回
  本仓库产物；本仓库入库固件仍以 0.9.x BYD 基线为准，不更换。
