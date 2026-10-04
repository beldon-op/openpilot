import numpy as np

from opendbc.car.byd.values import CarControllerParams

_ACCEL_MIN = CarControllerParams.ACCEL_MIN
_ACCEL_MAX = CarControllerParams.ACCEL_MAX
_COMFORT_BAND_UPPER = CarControllerParams.COMFORT_BAND_UPPER
_COMFORT_BAND_LOWER = CarControllerParams.COMFORT_BAND_LOWER
_JERK_UPPER = CarControllerParams.JERK_UPPER_LIMIT
_JERK_LOWER = CarControllerParams.JERK_LOWER_LIMIT
_MIN_START_ACCEL = CarControllerParams.MIN_START_ACCEL


def byd_checksum(byte_key, dat):
  # not an actual known crc function, reverse engineered on Atto 3 (bukapilot);
  # TODO(Song Plus DM-i): verify against real vehicle CAN logs
  def byte_crc4_linear_inverse(byte_list):
    return (-1 * sum(byte_list) + 0x9) & 0xF

  second_bytes = [byte & 0xF for byte in dat]
  remainder = sum(second_bytes) >> 4
  second_bytes.append(byte_key >> 4)

  first_bytes = [byte >> 4 for byte in dat]
  first_bytes.append(byte_key & 0xF)

  return (((byte_crc4_linear_inverse(first_bytes) + (-1 * remainder + 5)) << 4) + byte_crc4_linear_inverse(second_bytes)) & 0xFF


def byd_checksum_xor(dat):
  # XOR checksum used on ACC_EPS_STATE (from the open BYD port)
  ret = 0
  for byte in dat:
    ret ^= byte
  return ret


# Fields echoed from the stock DiPilot camera's ACC_MPC_STATE so OP's spoofed
# frame keeps the same SETME_* / MPC_State / AutoFullBeam* values the ADAS
# domain expects. Building the message from scratch (leaving these as 0)
# faults the camera ('check multifunction video controller' + 'ACC restricted').
_ACC_MPC_STATE_ECHO_FIELDS = [
  "AutoFullBeamState", "LeftLaneState", "LKAS_Config", "SETME2_0x1",
  "MPC_State", "AutoFullBeam_OnOff", "LKAS_Output", "LKAS_Active",
  "SETME3_0x0", "TrafficSignRecognition_OnOff", "SETME4_0x0",
  "SETME5_0x1", "RightLaneState", "LKAS_State",
  "TrafficSignRecognition_Result", "LKAS_AlarmType", "SETME7_0x3",
]


def create_lkas_request(packer, cam_msg, apply_torque, lkas_active, lkas_req_prepare, lkas_config, raw_cnt):
  """50 Hz, spoofed ACC_MPC_STATE (790) carrying the LKAS torque request.

  Echoes the stock camera's ACC_MPC_STATE fields (SETME_*, MPC_State,
  AutoFullBeam*, LKAS_State) and overrides the torque request and lane
  states. Verified against real-vehicle CAN logs:
  - LeftLaneState/RightLaneState MUST be 2 or the EPS will not actuate LKA.
  - LKAS_Config MUST be 2 (LKA) whenever LeftLaneState=2 or LKAS_ReqPrepare=1:
    the stock camera idles at LKAS_Config=1 (ALARM), and EPS treats the
    combination Config=1 + LaneState=2 as illegal and latches SteerWarning
    (refusing to arm LKAS_Prepared) - which also drives the MPC's 'check
    multifunction video controller' fault via ACC_EPS_STATE.
  - LKAS_ReqPrepare=1 must be sent while engaged until the EPS reports
    LKAS_Prepared (ACC_EPS_STATE bit0).
  """
  values = {s: cam_msg[s] for s in _ACC_MPC_STATE_ECHO_FIELDS if s in cam_msg}
  values["ReqHandsOnSteeringWheel"] = 0
  values["LKAS_ReqPrepare"] = lkas_req_prepare
  values["LKAS_Config"] = lkas_config   # the vendor streams 3 in every state (idle included)
  values["Counter"] = raw_cnt

  if lkas_active:
    values.update({
      "LKAS_Output": apply_torque,   # steer torque request
      "LKAS_Active": 1,
      # The actuation gate is LKAS_Config=3 (ALARM_AND_LKA session), NOT the
      # state field: the vendor build (op_byd, route 00000037) steers with
      # exactly (State=1, MPC=0, Config=3, Active=1, lanes 2/2).
      "LKAS_State": 1,
      "LeftLaneState": 2,
      "RightLaneState": 2,
    })
  elif lkas_req_prepare:
    # engage/retry burst: the vendor's 3-frame prepare carries lanes 2/2 with
    # Active=0 and request 0 (route 00000037 t=16.78), then activates 50 ms
    # later without waiting for the EPS ack
    values.update({
      "LKAS_Output": 0,
      "LKAS_Active": 0,
      "LKAS_State": 1,
      "LeftLaneState": 2,
      "RightLaneState": 2,
    })
  else:
    values.update({
      "LKAS_Output": 0,
      "LKAS_Active": 0,
      # vendor standby: Config=3 with lanes 0/0 (no LKA target) - streamed
      # for minutes at a time on the vendor's own drive without EPS complaint
      "LKAS_State": 1,
      "LeftLaneState": 0,
      "RightLaneState": 0,
    })

  dat = packer.make_can_msg("ACC_MPC_STATE", 0, values)[1]
  crc = byd_checksum(0xAF, dat[:-1])
  values["CheckSum"] = crc
  return packer.make_can_msg("ACC_MPC_STATE", 0, values)


def send_buttons(packer, count):
  """Spoof ACC UP_RESETSPEED button press (BTN_AccUpDown_Cmd=3) for auto-resume from standstill."""
  values = {
    "SETME_1": 1,
    "BTN_AccUpDown_Cmd": 3,
    "SETME2_1": 1,
    "Counter": count,
  }

  dat = packer.make_can_msg("PCM_BUTTONS", 0, values)[1]
  crc = byd_checksum(0xAF, dat[:-1])
  values["CheckSum"] = crc
  return packer.make_can_msg("PCM_BUTTONS", 0, values)


def create_fake_eps_state(packer, eps_state_msg, fake_torque, counter):
  """Spoof ACC_EPS_STATE (792) EPS feedback so the ADAS domain does not fault (no DTC).

  Not sent by default; only needed if the real EPS feedback is not visible to the
  ADAS domain with the harness in place."""
  values = {s: eps_state_msg[s] for s in [
    "LKAS_Prepared",
    "CruiseActivated",
    "TorqueFailed",
    "SETME1_0x1",
    "SteerWarning",
    "SteerErrorCode",
    "SETME3_0x1",
    "SETME4_0x3",
    "SETME5_0xFF",
    "SETME6_0xFFF",
  ]}
  values["MainTorque"] = fake_torque
  values["SteerDriverTorque"] = 0
  values["ReportHandsNotOnSteeringWheel"] = 0
  values["Counter"] = counter

  dat = packer.make_can_msg("ACC_EPS_STATE", 0, values)[1]
  values["CheckSum"] = byd_checksum_xor(dat[:-1])
  return packer.make_can_msg("ACC_EPS_STATE", 0, values)


def create_can_steer_command(packer, steer_angle, steer_req, is_standstill, raw_cnt):
  """50 Hz, spoofed STEERING_MODULE_ADAS (482) angle command. Experimental angle path only."""
  set_me_xe = 0xB
  if is_standstill:
    set_me_xe = 0xE

  values = {
    "STEER_REQ": steer_req,
    "STEER_REQ_ACTIVE_LOW": not steer_req,
    "STEER_ANGLE": steer_angle * 1.02,     # desired steer angle
    "SET_ME_X01": 0x1 if steer_req else 0,  # must be 0x1 to steer
    "SET_ME_XE": set_me_xe if steer_req else 0,  # 0xB faults lesser, maybe higher value faults lesser,
                                            # 0xB also seems to have the highest angle limit at high speed
    "COUNTER": raw_cnt,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
  }

  dat = packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)[1]
  crc = byd_checksum(0xAF, dat[:-1])
  values["CHECKSUM"] = crc
  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)


def create_accel_command(packer, accel, enabled, active, resume, radar_acc_msg, raw_cnt):
  """50 Hz, transparent ACC_CMD (814) replacement for OP longitudinal control.

  The stock radar keeps owning the ACC session (its frames on bus 2 stay
  alive and feed cruiseState), OP re-broadcasts its frame onto bus 0 with the
  acceleration fields overridden while engaged. Byte template from the vendor
  build's real TX (route 00000037):

  - inactive echo: AccelCmd=0.0, ComfortBand=0/0, JerkUpper=0, JerkLower=0,
    AccControlActive=0, AccReqNotStandstill=1, EspBehaviour=1 (radar standby)
  - active: AccelCmd=OP accel ([-4, 2] m/s2), ComfortBand 0.1/0.05 only while
    accel >= 0, JerkUpper 1.0 / JerkLower -0.8 (fixed; vendor derives them
    from the plan jerk which we do not have), AccControlActive=1
  - ResumeFromStandstill pulses while long control is starting (never seen in
    the vendor log - no standstill in route 37 - so it follows the vendor
    decompile's starting-state handling)
  - counter free-runs at 50 Hz independent of the radar (verified: the vendor
    counter walks 0-f continuously across echo/active transitions)
  """
  if radar_acc_msg:
    # echo the radar's frame as the base (StandstillState / BrakeBehaviour /
    # EspBehaviour etc. are the radar's own values)
    values = {s: radar_acc_msg[s] for s in (
      "AccelCmd", "ComfortBandUpper", "ComfortBandLower", "SETME1_0x1",
      "JerkUpperLimit", "ResumeFromStandstill", "JerkLowerLimit",
      "StandstillState", "BrakeBehaviour", "AccReqNotStandstill",
      "AccControlActive", "AccOverrideOrStandstill", "EspBehaviour")}
  else:
    # no radar frame seen yet (ACC never on): synthesized idle frame, the
    # all-zero control-bits shape from route 37's byte5=0x00 population
    values = {
      "AccelCmd": 0.0, "ComfortBandUpper": 0.0, "ComfortBandLower": 0.0,
      "JerkUpperLimit": 0.0, "JerkLowerLimit": 0.0, "SETME1_0x1": 1,
      "ResumeFromStandstill": 0, "StandstillState": 0, "BrakeBehaviour": 0,
      "AccReqNotStandstill": 0, "AccControlActive": 0,
      "AccOverrideOrStandstill": 0, "EspBehaviour": 0,
    }

  if enabled and active:
    standstill = bool(radar_acc_msg.get("StandstillState", 0)) if radar_acc_msg else False
    accel = float(np.clip(accel, _ACCEL_MIN, _ACCEL_MAX))
    if resume and standstill:
      accel = max(accel, _MIN_START_ACCEL)
    values.update({
      "AccelCmd": accel,
      "ComfortBandUpper": _COMFORT_BAND_UPPER if accel >= 0 else 0.0,
      "ComfortBandLower": _COMFORT_BAND_LOWER if accel >= 0 else 0.0,
      "JerkUpperLimit": _JERK_UPPER,
      "JerkLowerLimit": _JERK_LOWER,
      "AccControlActive": 1,
      "AccReqNotStandstill": 0 if standstill else 1,
      "AccOverrideOrStandstill": 1 if standstill else 0,
      "StandstillState": 1 if standstill else 0,
      "EspBehaviour": 1,
      "ResumeFromStandstill": 1 if resume else 0,
    })

  values["Counter"] = raw_cnt
  values["SETME2_0xF"] = 0xF

  dat = packer.make_can_msg("ACC_CMD", 0, values)[1]
  values["CheckSum"] = byd_checksum(0xAF, dat[:-1])
  return packer.make_can_msg("ACC_CMD", 0, values)


def create_acc_hud_command(packer, adas_msg, raw_cnt):
  """50 Hz, ACC_HUD_ADAS (813) relay: pure echo of the stock camera's HUD frame.

  The camera keeps streaming its HUD on bus 2 (it stays the session owner);
  OP re-broadcasts it onto bus 0 with only the counter/checksum re-stamped.
  Route 37 shows the vendor's HUD content matching the camera's state 1:1
  (AccState/AccOn1/Notify all follow, SETME3_0xFFF=0xFFF, Status=4)."""
  values = {s: adas_msg[s] for s in (
    "SetSpeed", "HasLead", "SetDistance", "LeadingDistance", "AEB", "FCW",
    "SETME1_0x1", "AccState", "AccOn1", "CloseWarning", "SETME2_0x1",
    "Notify", "Status", "SETME3_0xFFF")}
  values["Counter"] = raw_cnt
  values["SETME4_0xF"] = 0xF

  dat = packer.make_can_msg("ACC_HUD_ADAS", 0, values)[1]
  values["CheckSum"] = byd_checksum(0xAF, dat[:-1])
  return packer.make_can_msg("ACC_HUD_ADAS", 0, values)


def create_acc_aeb_command(packer, aeb_msg, raw_cnt):
  """50 Hz, ACC_AEB (815) heartbeat relay: static payload + our counter.

  Byte template from route 37: payload `05 80 02 0f ff ff` constant, counter
  in byte 6 low nibble, SETME4_0xF, byd_checksum. AEB itself stays with the
  stock radar."""
  values = {"PAYLOAD": aeb_msg["PAYLOAD"]}
  values["Counter"] = raw_cnt
  values["SETME4_0xF"] = 0xF

  dat = packer.make_can_msg("ACC_AEB", 0, values)[1]
  values["CheckSum"] = byd_checksum(0xAF, dat[:-1])
  return packer.make_can_msg("ACC_AEB", 0, values)
