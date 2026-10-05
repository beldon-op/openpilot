import math

import numpy as np

from opendbc.car import get_safety_config, get_friction, structs
from opendbc.car.interfaces import FRICTION_THRESHOLD, CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.values import HUD_MULTIPLIER, USE_ANGLE_STEERING, BydSafetyFlags, CarControllerParams


def sig(val: float) -> float:
  # vendor byd/interface.py (cp_byd pyarmor_decrypted, interface.py.disasm.txt L183):
  # clamp to +-700 to keep exp() finite, sigmoid re-centered so sig(0)==0
  val = max(min(val, 700.0), -700.0)
  return 1.0 / (1.0 + math.exp(-val)) - 0.5


def non_linear_torque(lateral_accel_value: float, v_ego: float,
                      f1: float, f2: float, f3: float, f4: float, f5: float, f6: float, f7: float) -> float:
  # vendor L380 (the 9-arg form - non_linear_torque_old at L269 is dead code). Stack-traced
  # against docs/byd-lateral-vendor-model.md:
  #   torque = f2 * sig(lat * uf2 * hsf * f1 * hsf * (1 - uf)) + lat * uf2 * v_ego / 10
  # uf/uf2 are left/right asymmetric: Song Plus computes ~-0.47 left vs +0.42 right at
  # |lat|=1 m/s2 @ 20 m/s, i.e. a structural pre-compensation for the road-crown pull.
  unbalanced_factor = f5 if lateral_accel_value < 0 else f5 * f6
  unbalanced_factor_2 = f3 if lateral_accel_value < 0 else f7
  # kph ramp 1.0 -> 1.5 across [f4, 2*f4]; with the max(8 m/s) floor the siglin entry passes
  # in, this is pinned to 1.5 in normal driving (model intentionally frozen below ~29 km/h)
  high_speed_factor = 1.0 + min(0.5, max(0.0, (v_ego * 3.6 - f4) / f4))
  sigmoid_term = f2 * sig(lateral_accel_value * unbalanced_factor_2 * high_speed_factor * f1 *
                          high_speed_factor * (1.0 - unbalanced_factor))
  linear_term = lateral_accel_value * unbalanced_factor_2 * v_ego / 10.0
  return sigmoid_term + linear_term


_lats_sm = None


def _read_lat_state() -> tuple[bool, float]:
  # vendor equivalent: module-level sm = SubMaster(...) on 'controlsState', read via
  # sm['controlsState'].lateralControlState (disasm L1287+). Returns (is_torqueState, i).
  # Any failure mode (no msgq, first frame, schema drift) must degrade to (False, 0.0) -
  # the curve term goes inert, the model keeps steering.
  global _lats_sm
  if _lats_sm is None:
    try:
      from cereal.messaging import SubMaster
      _lats_sm = SubMaster(['controlsState'])
    except Exception:
      _lats_sm = False
  if not _lats_sm:
    return False, 0.0
  try:
    _lats_sm.update(0)
    lat_state = _lats_sm['controlsState'].lateralControlState
    if lat_state.which() == 'torqueState':
      return True, float(lat_state.torqueState.i)
  except Exception:
    pass
  return False, 0.0


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController

  def __init__(self, CP: structs.CarParams):
    super().__init__(CP)
    # curve term inputs, refreshed once per loop in apply() (see _read_lat_state):
    # the carrot CarControl schema has NO actuators.lateralControlState - the vendor
    # reaches the lateral torque state through a SubMaster on controlsState (disasm
    # L1287+), we do the same. Cached here so the siglin callback stays allocation-free
    # (it runs several times per frame) and so tests can drive it directly.
    self._lat_state_is_torque = False
    self._lat_torque_i = 0.0

  def apply(self, c: structs.CarControl, now_nanos: int | None = None, model_v2=None, radar_state=None) -> tuple[structs.CarControl.Actuators, list]:
    # ABSOLUTE rule on BYD: apply()/card must never raise - a gap in the 0x316 stream
    # latches EPS TorqueFailed until the next ignition cycle. Everything below is soft.
    try:
      self._lat_state_is_torque, self._lat_torque_i = _read_lat_state()
    except Exception:
      self._lat_state_is_torque = False
      self._lat_torque_i = 0.0
    return super().apply(c, now_nanos, model_v2, radar_state)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"
    if USE_ANGLE_STEERING:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd, BydSafetyFlags.ANGLE_STEERING)]
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.steerActuatorDelay = 0.01  # DiPilot steering request is applied almost instantly
    else:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]
      ret.steerControlType = structs.CarParams.SteerControlType.torque
      # vendor live carParams (route 00000037): steerActuatorDelay=0.3
      ret.steerActuatorDelay = 0.3
      # A档 (vendor configure_torque_tune, interface.py.disasm L1953 + L2347):
      # - 0.1 deg steering-angle deadzone (Song Plus group default; latcontrol_torque converts
      #   it to a lateral-accel deadzone via the vehicle model every frame)
      # - ki=0: the vendor lateral PID has NO integrator (kp=kf=1 are already the base defaults).
      #   Their steady-state trimming is done by the model shaping (B档) and, on top of that,
      #   the C档 error-enhancer subsystem - not by the loop integrator.
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.1)
      ret.lateralTuning.torque.ki = 0.0
    ret.steerLimitTimer = 0.5

    ret.radarUnavailable = True

    # longitudinal control is done by the stock ACC; openpilot does lateral
    # control only (unless AlphaLongitudinalEnabled). The spoofed RES-button
    # auto-resume was REMOVED 2026-10-05 (user rule: 纵向要激活 ACC 才能控制,
    # 设置不应该控制) - OP must never activate the stock session by itself,
    # resuming from a standstill is the driver's action on the stalk.
    # pcmCruise ties openpilot's engagement to the ACC main posture (like the
    # Geely port): cruiseState.enabled is the arm latch (debounced main-on +
    # one genuine session this drive, see carstate) - OP enables on its rising
    # edge and only drops it when the driver turns ACC off. Brake and session
    # standby deliberately do not disengage; the longitudinal output is gated
    # on the live radar session in the controller instead.
    ret.openpilotLongitudinalControl = False
    ret.pcmCruise = True
    # true: a standstill must not fire STOP_REQUIRED-style disengagements
    # (the main-on latch keeps OP alive through the stop; the driver resumes)
    ret.autoResumeSng = True

    ret.wheelSpeedFactor = HUD_MULTIPLIER

    ret.minEnableSpeed = -1
    ret.stoppingDecelRate = 0.05  # reach stopping target smoothly

    # OP longitudinal (AlphaLongitudinalEnabled param): transparent replacement
    # of the stock radar's ACC_CMD/ACC_HUD/ACC_AEB on bus 0. The stock radar
    # keeps owning the session - its frames on bus 2 stay alive and keep feeding
    # cruiseState/pcm_cruise_check - so the engage chain above is unchanged and
    # no reflash is needed to toggle: the firmware keys its extra TX whitelist,
    # accel checks and 0x32D/E/F forward block off the LONGITUDINAL safety flag.
    # Declare availability: without this flag selfdrived deletes
    # AlphaLongitudinalEnabled at every startup and exp_button locks the onroad
    # toggle ("stock ACC is used" message). Running the param on firmware
    # predating the longitudinal port (fw_base fee17db6) is instead caught by
    # the safetyTxBlocked watchdog (bydLongFirmwareMissing, M1).
    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= int(BydSafetyFlags.LONGITUDINAL)
      # vendor interface params (decrypted op_byd interface.py, 2026-10-05
      # full-pipeline decompile): the vendor sets these once for ALL BYD with
      # op-long. longitudinalTuning kp=ki=0 there is a placeholder - carrot's
      # longcontrol overrides single-BP tuning with params LongTuningKpV/KiV/
      # Kf, and the vendor manager's defaults (100/0/100 -> kp=1.0, ki=0,
      # kf=1.0) match carrot's defaults, so effective gains already agree.
      ret.longitudinalActuatorDelay = 0.4
      ret.vEgoStarting = 0.2
      ret.vEgoStopping = 0.03
      ret.startAccel = 0.5
      ret.stoppingDecelRate = 0.25
      # the standstill -> go transition must go through LongCtrlState.starting
      # so the controller can pulse ACC_CMD ResumeFromStandstill
      ret.startingState = True

    return ret

  @staticmethod
  def get_pid_accel_limits(CP, current_speed, cruise_speed):
    # vendor ACCEL_MIN/MAX (base class returns -3.5/2.0; BYD commands down to
    # -4.0 m/s2, see route 37 TX)
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  # --- B档: vendor siglin lateral model (interface.py.disasm L1556/L1848, see
  # docs/byd-lateral-vendor-model.md). latcontrol_torque binds CI.torque_from_lateral_accel()
  # once at startup, so the selector below is the only hook needed. ---

  def torque_from_lateral_accel(self):
    if self.CP.carFingerprint in CarControllerParams.NON_LINEAR_TORQUE_PARAMS:
      return self.torque_from_lateral_accel_siglin
    return super().torque_from_lateral_accel()

  def get_adjusted_lateral_accel(self, lateral_accel_value: float,
                                 torque_params: structs.CarParams.LateralTorqueTuning,
                                 steer_mode: bool, roll_compensation: float,
                                 gravity_adjusted: bool) -> float:
    # vendor L1218. Two shapers on the commanded lateral accel, applied to every callback
    # input (setpoint, measurement, and feedforward):
    # 1) a gain around the latAccelFactor nominal (Song Plus: 2.5 -> exactly 1.0) plus
    #    +0.3 while the EPS is NOT in an active steering session (their STEER_MODE in (1,2);
    #    we approximate with the 0x318 CruiseActivated bit - see docs);
    # 2) a curve taper keyed on the PID integrator log (torqueState.i). With the vendor's
    #    ki=0 it is always 0 there; keeping it wired means it engages if the user raises ki
    #    or turns on the NNFF integrator on carrot.
    adjusted = lateral_accel_value
    if gravity_adjusted:
      adjusted += roll_compensation
    curve_offset = 0.0
    if self._lat_state_is_torque:
      curve_offset = float(np.interp(abs(adjusted), [0.0, 0.2], [0.2, 1.0])) * self._lat_torque_i
    lat_conv_factor = 1.0 - (torque_params.latAccelFactor - 2.5)
    if not steer_mode:
      lat_conv_factor += 0.3
    lat_conv_factor *= (1.0 - curve_offset) if adjusted > 0 else (1.0 + curve_offset)
    return adjusted * lat_conv_factor

  def torque_from_lateral_accel_siglin(self, latcontrol_inputs, torque_params: structs.CarParams.LateralTorqueTuning,
                                       lateral_accel_error: float, lateral_accel_deadzone: float,
                                       friction_compensation: bool, gravity_adjusted: bool) -> float:
    lateral_accel_value = latcontrol_inputs.lateral_acceleration
    roll_compensation = latcontrol_inputs.roll_compensation
    # vendor floors the model speed at 8 m/s (the siglin entry, disasm L110 region): below
    # ~29 km/h the nonlinear terms stop changing with speed
    v_ego = max(8.0, latcontrol_inputs.vego)
    params = CarControllerParams.NON_LINEAR_TORQUE_PARAMS.get(self.CP.carFingerprint)
    assert params is not None, 'The params are not defined'
    f1, f2, f3, f4, f5, f6, f7 = params
    # steer_mode: vendor's STEER_MODE msg (1 Hz) is absent from our dbc - CruiseActivated is
    # the closest live bit (True == EPS steering == their in(1,2)); default True so a missing
    # carstate attribute falls back to NO +0.3 boost, matching their in-session behavior
    steer_mode = bool(getattr(self.CS, "cruise_activated", True))
    adjusted = self.get_adjusted_lateral_accel(lateral_accel_value, torque_params,
                                               steer_mode, roll_compensation, gravity_adjusted)
    steer_torque = non_linear_torque(adjusted, v_ego, f1, f2, f3, f4, f5, f6, f7)
    # C档 (get_friction_enhanced, L655: BSUC live param, gf slew limiter, error-enhancer PID)
    # is NOT ported yet - both gravity branches fall back to the vanilla friction curve, which
    # is exactly what the vendor's get_friction does on non-gravity calls
    friction = get_friction(lateral_accel_error, lateral_accel_deadzone, FRICTION_THRESHOLD,
                            torque_params, friction_compensation)
    return float(steer_torque) + friction
