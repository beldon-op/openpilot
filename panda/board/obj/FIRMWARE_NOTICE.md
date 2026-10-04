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

## 不要做的事

- 不要用 sunnypilot 或本仓库 `panda/` 现编的新代固件刷这块 c3l 兼容板：
  0.10.x 基线 app 刷入后不枚举 USB（未深查，弃用），新代框架结构也不同。
