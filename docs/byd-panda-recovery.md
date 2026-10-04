# BYD 宋 PLUS DM-i 移植：panda 固件排障与恢复记录

> 时间：2026-09-26 · 设备：tici 兼容车机（非官方硬件）· 分支 `main-c3l-tici`
> 本文记录 BYD 移植过程中 panda 固件层的完整排障过程，供后续维护参考。

## 一、背景

BYD 宋 PLUS DM-i 车型 port（`opendbc/car/byd/`、`opendbc/safety/modes/byd.h`、`opendbc/dbc/byd_general_pt.dbc`）完成后上车验证，车机 UI 报 **"无 PANDA"**。

## 二、排障过程（按发现顺序）

### 1. 排除硬件与枚举层

- `lsusb`：`3801:ddcc comma.ai panda` ✓（运行态固件，非 bootstub `ddee` / DFU `0483:df11`）
- `Panda.list()` 能看到设备 ✓
- 结论：USB 枚举和硬件正常，问题在 openpilot 进程层

### 2. pandad 死循环（第一次根因）

swaglog（`/data/log/swaglog.*`，注意 openpilot 不写 journald）显示 pandad 无限循环：

```
ValueError: unknown HW type: bytearray(b'\x05')
```

**原因**：设备内置 panda 上报硬件类型 `0x05`（UNO）。sunnpilot 0.10.1 的 panda 库
中 UNO 属于 `DEPRECATED_DEVICES`，不在 `F4_DEVICES`/`INTERNAL_DEVICES`，
`get_mcu_type()` 直接抛异常 → pandad 每次循环死在连接阶段 → 固件从未被重刷 →
pandaState 永不发布 → UI"无 PANDA"。

之前厂商系统能跑，是因为厂商的 panda 库认识这个类型号。

### 3. 修复：panda 库类型兼容（commit 7968f1bd18、9b79ceec1e、66b78648fe）

```python
# panda/python/__init__.py
F4_DEVICES += HW_TYPE_UNO          # UNO 是 STM32F4，与 DOS 同族
INTERNAL_DEVICES += HW_TYPE_UNO    # pandad 的内置 panda 检查
MAX_FAN_RPMs[HW_TYPE_UNO] = 6500
DEPRECATED_DEVICES -= HW_TYPE_UNO  # pandad 和 flash() 都会拒绝 deprecated 设备
```

修好后暴露下一层：DFU 恢复刷入的**官方 bootstub** 在这块板上报 `HW_TYPE_UNKNOWN
(0x00)`（板子布局不匹配任何官方 panda 型号——兼容板），再次循环。
最终方案：`get_type()` 把 UNO/UNKNOWN 都归一化为 DOS（同为 F4，tici 栈按 DOS 处理）。

**教训**：方法体内引用类常量必须 `self.HW_TYPE_*`（第一次写成了裸名字 → NameError）。

### 4. 刷写卡死（第二次根因）

pandad 走通后：连接 → 签名不匹配 → `reset(enter_bootstub=True)` → **panda 从 USB
彻底消失**，pandad 卡在内核 D 状态（81% CPU）。

**原因**：厂商 bootstub 兼容性 + `BOOT0` 引脚残留。`recover_internal_panda()` 强制
DFU 时把 BOOT0(GPIO 134) 拉高后**没有恢复**，STM32 每次复位都采样 BOOT0 → 永远
进 ROM DFU（`0483:df11`）。

**恢复流程**（有效，已验证）：

```bash
sudo systemctl stop comma
# BOOT0 复位回 0，再脉冲 RST（GPIO 124），bootstub/app 即可正常启动
sudo sh -c 'echo 0 > /sys/class/gpio/gpio134/value;
            echo 1 > /sys/class/gpio/gpio124/value; sleep 0.5;
            echo 0 > /sys/class/gpio/gpio124/value'
```

`PandaDFU(serial).recover()` 可靠刷入干净 bootstub；app 固件经 bootstub 刷写。
注意 `/tmp` 在重启后清空，刷写文件要用绝对路径持久存放。

### 5. 签名与重刷循环

自建/第三方固件没有官方签名 → `pandad` 每次启动都因签名不匹配重刷 → 必须有跳过
机制。注意 openpilot 的参数键白名单**编译在 params_pyx C++ 里**，新键会抛
`UnknownKeyName`，所以用**文件标志**而非参数：

```
/data/panda_skip_flash   （存在即跳过刷写，commit a9acdb8eca）
```

另外发现：C++ pandad 对 deprecated 硬件类型也会自主跳过固件检查（hw_type=5 命中），
与 Python 层开关形成双保险。

### 6. 车型识别与 CAN 链路验证

- 清除 `CarParamsCache`/`CarParamsPersistent` + 禁用指纹自动匹配后：
  **CAN RX 完全正常**（50 秒 3218 帧，EPS 287@100Hz 等全部吻合 dbc）
- 指纹匹配 `BYD_SONG_PLUS_DMI_22`（社区指纹实车验证通过 ✓），但匹配会触发
  `SafetyModel.byd(35)` 下发 → 厂商固件不认识 → 回落 SILENT → **CAN RX 被杀**
  （厂商固件的未知模式回落实现有缺陷）。因此指纹匹配**临时禁用**（commit
  5b1a06241c，自建固件就绪后恢复），设备以"未识别 + 静默监听"状态采集数据。

## 三、固件复用结论

| 方案 | 可行性 | 说明 |
|---|---|---|
| 厂商固件作为**保底固件** | ✅ | 能启动、CAN RX 正常、已被本仓库收录（见
  `panda/board/obj/FIRMWARE_NOTICE.md`），刷包后设备立即可用（无 byd）|
| 厂商固件承载 **byd 模式** | ❌ | 其 byd 模式编号 `@29`（我们的 cereal 是
  `@35`），参数语义、报文白名单全部私有且无源码；无法安全对接 |
| 官方 v0.10.1 固件 | ❌ | 在此兼容板上主固件无法启动（bootstub 正常）|
| **自建固件（含 byd）** | ✅ 必经之路 | 以 0.9.x 时代上游 panda 源码为基线
  （厂商固件同期、已验证可启动），移植 `byd.h` 交叉编译 |

## 四、当前状态与下一步

- 设备运行厂商固件（cp_byd 的 `DEV-3a515542` 构建），panda 连接正常、CAN RX 正常、
  route 录制正常；`/data/panda_skip_flash` 标志在位
- 指纹匹配临时禁用；`CarPlatformBundle` 已清除
- **待办**：交叉编译工具链 → 上游 panda 源码（0.9.x 基线）→ 移植 byd.h → 构建 →
  `flash.py` 刷写测试 → 恢复指纹匹配 → 实车四级验证
- 待办：实车 CAN 日志采集（场景清单见对话记录），用于 dbc 信号核验与标定
  （`LKAS_Output` 缩放、驾驶员扭矩阈值、0x122 轮速布局、TX checksum 算法）

## 五、关键命令速查

```bash
# 看 panda 真实状态（三层：ddcc=app / ddee=bootstub / 0483:df11=DFU）
lsusb | grep -v "root hub"

# 强制内置 panda 进 DFU
python3 -c "from openpilot.system.hardware import HARDWARE; \
  HARDWARE.recover_internal_panda()"   # 需 capnp 环境（/usr/local/venv/bin/python）

# BOOT0 复位（DFU 刷完后恢复 app 启动，必须做！）
sudo sh -c 'echo 0 > /sys/class/gpio/gpio134/value;
            echo 1 > /sys/class/gpio/gpio124/value; sleep 0.5;
            echo 0 > /sys/class/gpio/gpio124/value'

# 读 pandad 日志（不在 journald 里！）
grep -a pandad /data/log/swaglog.* | tail -40

# 本机验证 CAN 流量（读 route 的 rlog）
python3 -c "from openpilot.tools.lib.logreader import LogReader; ..."
```
