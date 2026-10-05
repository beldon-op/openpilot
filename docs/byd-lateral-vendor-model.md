# BYD 横向：vendor（cp_byd）控制器层拆解与移植（A/B 档）

日期：2026-10-05。素材：`/Users/wujiafu/Documents/op/cp_byd/docs/pyarmor_decrypted/opendbc_repo/opendbc/car/byd/interface.py.disasm.txt`（权威）、`values.py.values.txt`、`carstate.py.disasm.txt`。
解码规律：Python 3.12 字节码，`LOAD_GLOBAL/LOAD_ATTR` 的名字索引 = 操作数>>1；`STORE_ATTR` 用原始索引。pyarmor 只在模块体/函数头插 armor，指令语义完整可解。

## 结论总览

vendor 的"横向稳"不在执行层（0x316 报文、session 架构——我们根因 15-18 已对齐其运行时值），
而在**控制器层**：Song Plus 走自定义的 siglin 非线性扭矩模型 + 期望值整形 + 有状态摩擦补偿。
本仓库现状是通用线性模型（`configure_torque_tune` 默认参数）。三层分档：

- **A 档（本次落地）**：`steeringAngleDeadzoneDeg=0.1` + 扭矩 PID `ki=0`。
- **B 档（本次落地）**：`torque_from_lateral_accel_siglin`（含 `non_linear_torque`、`get_adjusted_lateral_accel`）。
- **C 档（待办）**：`get_friction_enhanced` 有状态子系统（error_enhancer/BSUC/gf_last）。B 档先用通用 `get_friction` 占位。

## vendor 的调用链（interface.py）

`latcontrol_torque` 每帧调 `CI.torque_from_lateral_accel()` 取回调；回调签名与我们 carrot 完全兼容
（`LatControlInputs, LateralTorqueTuning, lateral_accel_error, lateral_accel_deadzone, friction_compensation, gravity_adjusted`）。

选择器（disasm L1848）：
```python
def torque_from_lateral_accel(self):
    if CP.carFingerprint in (BYD_QIN_PLUS_DMI, BYD_QZJ_05_DMI): return linear_speed_interp   # 不移植
    elif CP.carFingerprint in NON_LINEAR_TORQUE_PARAMS:        return siglin                  # ← Song Plus
    return linear
```

siglin 主体（disasm L1556），已按栈逐条核对：
```python
def torque_from_lateral_accel_siglin(self, inputs, torque_params, error, deadzone, fc, gravity_adjusted):
    lat = inputs.lateral_acceleration
    roll_comp = inputs.roll           # carrot 字段名（vendor 叫 roll_compensation）
    v_ego = max(8.0, inputs.vego)     # 模型速度下限 8 m/s（≈29km/h）
    (a,b,c,d,e,f,g) = NON_LINEAR_TORQUE_PARAMS[CP.carFingerprint]   # assert 缺失即 raise
    adj = self.get_adjusted_lateral_accel(lat, torque_params, self.CS.steer_mode, roll_comp, gravity_adjusted)
    steer_torque = non_linear_torque(adj, v_ego, a,b,c,d,e,f,g)
    # friction: gravity_adjusted=True → get_friction_enhanced（C 档）; 否则 get_friction(error, dz, 0.3, ...)
    return float(steer_torque) + friction
```

`non_linear_torque`（disasm L380；注意 L380 的 9 参版才是 Song Plus 用的，L269 的 `_old` 已弃用）：
```python
sig(x) = 1/(1+exp(-clamp(x,±700))) - 0.5
uf  = f5            if lat < 0 else f5 * f6      # 左右饱和幅度不对称（f6=乘子）
uf2 = f3            if lat < 0 else f7            # 左右线性增益不对称
hsf = 1 + min(0.5, max(0, (v_ego*3.6 - f4) / f4)) # 1→1.5，kph 在 [f4, 2·f4] 间爬升
torque = f2 * sig(lat * uf2 * hsf * f1 * hsf * (1 - uf)) + lat * uf2 * v_ego / 10
```
Song Plus 参数 `[14.99976405, -0.55974149, 0.09633187, 12.47500536, 2.99999827, 1.49999146, 0.06850084]`，
数值验证（v=20m/s，|lat|=1）：左 −0.473 / 右 +0.417 —— **sigmoid 项与线性项都是左强右弱**，
即对路拱右偏的结构性预补偿。hsf 因 v≥8m/s 下限恒为 1.5。
更正旧注释：本仓 `values.py` 曾把这行写成 "torque **limit** fit"，实为 **torque-from-lateral-accel 模型**系数（limit 表是另一套：STEER_MAX 等，已对齐）。

`get_adjusted_lateral_accel`（disasm L1218）：
```python
adj = lat;  if gravity_adjusted: adj += roll_comp
w = cs.lateralControlState.which()
curve_offset = 0
if w == 'torqueState':
    # 注意：是 .i（PID 积分项日志）——capnp LateralTorqueState.i 字段，carrot schema 存在
    curve_offset = interp(abs(adj), [0, 0.2], [0.2, 1.0]) * torqueState.i
lat_conv_factor = 1.0 - (torque_params.latAccelFactor - 2.5)   # Song Plus latAccelFactor=2.5 → 1.0
if not steer_mode:  lat_conv_factor += 0.3                     # EPS 未进入转向模式时期望值 ×1.3
lat_conv_factor *= (1 - curve_offset) if adj > 0 else (1 + curve_offset)
return adj * lat_conv_factor
```
vendor 自身 ki=0 ⇒ 它的 `i`≈0 ⇒ curve_offset≡0（保留实现，用户开积分/NNFF 后自动生效）。
`steer_mode` = 其 carstate 对独立 1Hz 报文 `STEER_MODE` 的判据 `not in (1,2)`（disasm L1528-1554）。
**本仓没有该报文**（vendor 包内无 BYD dbc）——B 档近似映射：`steer_mode := CS.cruise_activated`
（0x318 ACC_EPS_STATE，1=EPS 已接受横向会话）。这是行为最接近的可用信号；真报文待路测从总线逆向后再校正。

## 扭矩 PID tune（disasm L1953/L2347）

- `configure_torque_tune` 重写版：kp=1.0, kf=1.0, **ki=0**（基类是 0.1），friction/latAccelFactor 从 torque_data。
- `lateral_dz` 默认 **0.1** 传入 `steeringAngleDeadzoneDeg`（仅 TANG_DMI_21 组 0.5、QIN 组 0.5）；
  carrot 管线现成：`latcontrol_torque.py:76/179/195` 会把它换算成 `lateral_accel_deadzone`（按车速曲率等效）。
- 参数表我们 override.toml 的 `[2.5, 2.5, 0.10]` 与 vendor 明文 TOML **逐字相同**（legend 也一致：
  `LAT_ACCEL_FACTOR=2.5, MAX_LAT_ACCEL_MEASURED=2.5, FRICTION=0.10`，cp_byd torque_data 已核对）。
  注意 `get_friction` 的摩擦项幅值用的是 `torque_params.friction`（=0.10）；vendor 的
  `get_friction_enhanced` 内部则用 `friction_threshold`（=0.3）当饱和幅值——C 档移植时的差异点。

## C 档备忘：get_friction_enhanced（disasm L655，本次不实现）

有状态子系统，`gravity_adjusted=True` 的 ff 调用激活：
- `sm.update(0)` 读上一帧 `controlsState.lateralControlState.torqueState.desiredLateralAccel`；
- BSUC：每 500 帧 `BYD_STEER_UNBALANCE_COEF = int(Params['BYDAngleControlOffset'])*0.1`（设备端实时可调）；
- 重力项 `gf = roll*(-0.3 + 0.2*BSUC)`；与 desired 反向时乘 `interp(|desired|,[0.03,0.1],[1,0.1])` 衰减；
  再 `gf = clip(gf, gf_last±0.005)` 写回 `gf_last`（每帧斜率限幅）；
- `ENABLE_EE` 为真时走 `error_enhancer`（`PIDController`，pos/neg_limit 疑为 ±0.02，`k_f`、update 带
  `override/freeze_integrator` 关键字——carrot `opendbc/car/common/pid.py` API 兼容），输入来自
  `errorFilter/actualJerkFilter`（FirstOrderFilter）+ `lateral_error_last/_increament` 的迟滞链；
  积分器反号时 `out -= i; lateral_error_last=0`。`ENABLE_EE` 假时退化为 `error + gf_last`；
- `adjusted_friction = friction_threshold`（注意：**不是** speed-interp 版，那是 Qin 的 linear_speed_interp 用
  `interp(v,[0,25,40],[1.5,1,0.5])*friction`）；`friction = interp(error, ±threshold, ±adjusted_friction)`。
- 模块级常量 `ENABLE_EE`/`DEF_EE_E_FACTOR=0.5?`/`ALLOW_DP=False` 在 armor 掩盖的模块体里，无法确证数值；
  移植 C 档前需从设备活体（`/data/openpilot` pyarmor 包）或二次解码确认。

## 不移植清单

`linear_speed_interp`（Qin 专属，摩擦速度插值另成体系）、`BYD_LOW_TORQUE`（Song Plus 平台 flag 没有它）、
angle 控制路（Atto3 系 13 车型组）、`BYD_TORQUE_WITH_FACTOR`（Song Plus 唯一平台 flag，python 无消费、
仅镜像进 safetyParam；alt 报文里的 TORQUE_FACTOR interp([80,150]→[0,200]) 属于我们未走的 alt 路）。

## 路测验证点（A+B 后）

1. 直道：0x316 请求幅值应比旧版小（0.1°死区吞掉微修正）+ 无左右摆；
2. 缓弯进入：左/右曲线不对称感（预补偿生效方向 = 左强右弱）；
3. 低速（<29km/h）：模型冻结在 8m/s 行为，不应出现增益突跳；
4. `cruise_activated` 翻转瞬间（EPS 接受会话）：期望值 ×1.3→×1.0 是否有可感台阶——若有，考虑 steer_mode 映射换成带滞回的版本；
5. 对照采集：`byd_long_capture.py` 已在录 CS/CC——加测 lateral 字段（torqueState desired/actual 已有）。

## 本仓库改动索引

- `opendbc/car/byd/interface.py`：A 档两参 + B 档全栈（sig/non_linear_torque/siglin/get_adjusted_lateral_accel/apply 缓存）
- `opendbc/car/byd/values.py`：`NON_LINEAR_TORQUE_PARAMS` 注释更正
- 测试：`test_byd_carcontroller.py` 或新增 lateral 模型用例（见测试文件）
