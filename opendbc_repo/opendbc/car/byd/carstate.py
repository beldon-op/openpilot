from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.values import DBC, CarControllerParams

# Boot-mid-cruise engage hold (see the latch block in update): when the device
# boots while the stock ACC is already engaged, cruiseState.enabled is held
# False for this long after CAN start, then let through - so the engagement
# edge happens exactly once, late, when every NO_ENTRY gate is ready. Matches
# the selfdrived ready window (~15 s) and the vendor's own boot-mid-cruise
# drive (route 7--12e: radar ACC active from t=0, their latch held 0 through
# t=17.5 s, then one edge - OP enabled the same frame).
BOOT_LATCH_HOLD_TIME = 15.0  # seconds

# ACC-main fall debounce: the camera itself drops the ACC main posture for
# ~2 s with no button event (route 11 t=156.4, route 12 t=200.3: AccState and
# AccOn1 fall together, then recover). Since main-on is the arm for BOTH axes,
# a raw fall flaps every state machine OP has. Hold main through drops shorter
# than this. The panda holds its own copy slightly LONGER (BYD_ACC_MAIN_FALL_HOLD
# in safety_byd.h) so OP always stops transmitting first - an OP/panda window
# mismatch in the other direction would punch holes into the 0x316/0x32E streams.
AVAIL_FALL_DEBOUNCE_TIME = 0.5  # seconds of card loop (100 Hz)


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["DRIVE_STATE"]["Gear"]

    self.lkas_prepared = False
    # EPS's own warning flag (0x318 SteerWarning). Observed in the two
    # post-v2 "LKAS Fault" latches (routes 28/2b 2026-10): 0.4-0.7 s BEFORE
    # err=4+TorqueFailed the EPS drops MainTorque to 0 and raises Warn while
    # keeping CruiseActivated=1 - a torque abort we could not see in
    # SteerErrorCode alone (it stayed 0 through the whole abort). The
    # controller stands down on it (root cause 18).
    self.steer_warning = False
    # EPS session-phase bit ("executing now") - read by the controller's c0
    # guard (root cause 17 B): Active=1 that never gets accepted is a dead
    # stream, not a session to keep storming at
    self.cruise_activated = False
    self.torque_failed = False
    self.steer_error = 0
    self.res_btn_pressed = False
    self.counter_pcm_buttons = 0
    # Stalk-blinker side mapping override for the Song (see update). Raw
    # /data/params file, readable before the prebuilt params_pyx.so knows the
    # key. 1 = swap L/R. Read once at init - restart to pick up changes.
    self.blinker_stalk_swap = False
    try:
      with open('/data/params/BydBlinkerStalkSwap') as f:
        self.blinker_stalk_swap = int(f.read().strip()) != 0
    except Exception:
      try:
        from openpilot.common.params import Params
        val = Params().get('BydBlinkerStalkSwap', block=False)
        self.blinker_stalk_swap = val is not None and int(val) != 0
      except Exception:
        pass
    self.eps_state_msg = {}
    self.is_cruise_latch = False
    self.boot_frames = 0  # update() calls since process start (100 Hz card loop)
    self.avail_fall_frames = 0  # consecutive updates without the ACC main posture
    # one genuine radar session (AccControlActive / AccState 2/3/5) seen this
    # drive cycle - same lifecycle as boot_frames (process start == ignition).
    # Gates the engage latch: the ignition-leftover posture (AccState=1,
    # SetSpeed=30, AccOn1=1, still in Park - route 0000000f) reports main
    # without a session and must never arm OP by itself.
    self.ever_engaged = False
    # stock camera's ACC_MPC_STATE (bus 2); echoed back on bus 0 by the
    # controller so the spoofed LKAS request keeps the camera's SETME_* fields
    self.cam_lkas = {}
    # ACC domain frames (bus 2) as the OP-longitudinal echo base
    self.radar_acc_msg = {}
    self.adas_msg = {}
    self.aeb_msg = {}

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_adas = can_parsers[Bus.adas]

    ret = structs.CarState()

    # EPS feedback (also read back by the ADAS domain)
    self.lkas_prepared = bool(cp.vl["ACC_EPS_STATE"]["LKAS_Prepared"])
    self.cruise_activated = bool(cp.vl["ACC_EPS_STATE"]["CruiseActivated"])
    self.torque_failed = bool(cp.vl["ACC_EPS_STATE"]["TorqueFailed"])
    self.steer_warning = bool(cp.vl["ACC_EPS_STATE"]["SteerWarning"])
    # EPS's own warning level: observed 0 while idle and in clean sessions,
    # 2 as a ~0.5 s stand-down warning before it escalates to 4 + SteerWarning
    # + TorqueFailed (route c9f1698c82 seg 0). The controller obeys it like
    # the vendor obeys the EPS state bits.
    self.steer_error = int(cp.vl["ACC_EPS_STATE"]["SteerErrorCode"])
    self.eps_state_msg = cp.vl["ACC_EPS_STATE"]

    # stock camera's ACC_MPC_STATE (bus 2); the controller echoes this frame
    # onto bus 0 with the LKAS torque overridden. Echoing the camera's SETME_*
    # / MPC_State fields is required - building the frame from scratch faults
    # the DiPilot ADAS domain ('check multifunction video controller').
    self.cam_lkas = cp_adas.vl["ACC_MPC_STATE"]

    # ACC domain frames (bus 2) cached as the echo base for OP longitudinal
    # (transparent ACC_CMD/ACC_HUD/ACC_AEB replacement onto bus 0). Empty dicts
    # before the first frame - the controller synthesizes an idle ACC_CMD then.
    self.radar_acc_msg = dict(cp_adas.vl["ACC_CMD"])
    self.adas_msg = dict(cp_adas.vl["ACC_HUD_ADAS"])
    self.aeb_msg = dict(cp_adas.vl["ACC_AEB"])

    # speed
    # Song Plus DM-i: the 0x122 wheel-speed message reads all zeros and the
    # 0x418 BSD_RADAR VEHICLE_SPEED field is constant (~8.5), so neither is a
    # usable speed. The real vehicle speed comes from 0x1f0 ESP_SPEED
    # (20 Hz, bus 0) byte 4 - note the signal is 0.9 km/h per bit, NOT 1.0
    # (GPS fit on the 2026-10-02/03 road tests: true = 1.113 x raw, flat over
    # 20-120 km/h; the earlier "verified against camera odometry" claim was
    # circular - the odometry scale was itself anchored to this signal).
    # CarStateBase folds that in via CP.wheelSpeedFactor = HUD_MULTIPLIER.
    vehicle_speed_kph = cp.vl["ESP_SPEED"]["VehicleSpeed"]
    ret.vEgoRaw = vehicle_speed_kph * CV.KPH_TO_MS * self.CP.wheelSpeedFactor
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    # The BYD cluster over-reads the calibrated (GPS-anchored, 10/9) speed by
    # ~9% (drive route 4: cluster 38/48 vs device 35/44 km/h, both episodes).
    # Put the UI number on the cluster's scale so driver and screen agree;
    # control stays anchored to calibrated truth (this field is display-only).
    ret.vEgoCluster = ret.vEgo * CarControllerParams.CLUSTER_OVERREAD
    ret.standstill = ret.vEgoRaw < 0.05

    ret.brakeHoldActive = False

    # gear
    can_gear = int(cp.vl["DRIVE_STATE"]["Gear"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.doorOpen = any([cp.vl["BCM"]["RearLeftDoor"],
                        cp.vl["BCM"]["FrontLeftDoor"],
                        cp.vl["BCM"]["RearRightDoor"],
                        cp.vl["BCM"]["FrontRightDoor"]])
    # the car already enforces the seatbelt warning itself; OP does not need to
    # block engagement on it. The DriverSeatBeltFasten bit encoding is also
    # unverified on Song Plus DM-i (would otherwise report unlatched while
    # buckled). Matches the community BYD_Files port.
    ret.seatbeltUnlatched = False

    # pedals - the analog 0.01/unit signals idle at LSB noise, and >0.01 means
    # a single count of noise fires the press. The 2026-10 road tests show the
    # cost: openpilot state flapped enabled<->overriding 8x in 2 s (route 2d
    # t=1485, 31 t=1486) - each false gas fire releases the longitudinal
    # actuator for seconds while the stock radar FORCE_ACCEL-bounces (st 5,
    # AccControlActive 0) - the "车控退出几秒才回来" the driver feels.
    # 0.05 (5 counts) clears the noise band and still trips on a real touch.
    ret.gasPressed = cp.vl["PEDAL"]["AcceleratorPedal"] > CarControllerParams.PEDAL_PRESS_THRESHOLD
    ret.brake = cp.vl["PEDAL"]["BrakePedal"]
    ret.brakePressed = bool(cp.vl["DRIVE_STATE"]["BrakePressed"]) or \
      ret.brake > CarControllerParams.PEDAL_PRESS_THRESHOLD

    # steering
    ret.steeringAngleDeg = cp.vl["EPS"]["SteeringAngle"]
    # EPS-reported wheel rate in deg/s (4 deg/s/bit, 0-1020); used by the LKAS
    # envelope gate to tell a settled wheel from a full-lock-speed swing
    ret.steeringRateDeg = cp.vl["EPS"]["SteeringAngleRate"]
    ret.steeringTorque = cp.vl["ACC_EPS_STATE"]["SteerDriverTorque"]
    ret.steeringTorqueEps = cp.vl["ACC_EPS_STATE"]["MainTorque"]
    # 5-frame debounce: single-frame torque noise spikes must not flap
    # steeringPressed (they would keep selfdrived in overriding)
    ret.steeringPressed = self.update_steering_pressed(
      bool(abs(ret.steeringTorque) > CarControllerParams.STEER_THRESHOLD), 5)

    # stock ACC status; LKA is coupled to the stock ACC on this platform
    # Song Plus DM-i encodes AccState differently from Han: 1 = main on /
    # standby (also the ignition-leftover state, SetSpeed=30, AccOn1=1, still
    # in Park - route 0000000f), 2/3/5 = engaged-only states. Two independent
    # things live in here:
    #
    #   main    = AccOn1 or AccState in (1,2,3,5)  - the stalk's main switch.
    #             Survives brake-cancel / CANCEL / the radar's bounce blips
    #             (AccOn1 stays 1 through all of them, route 11/12); drops
    #             only when the driver turns ACC off (or the camera glitches,
    #             hence the debounce).
    #   session = AccControlActive or AccState in (2, 3, 5) - the radar
    #             actually commanding (0x32e). Bounces (60-110 ms AccState
    #             2<->1 flaps), drops on brake (stock auto-resumes ~40 ms
    #             after release, route 11/12: 5/5 windows) and on CANCEL.
    #
    # cruiseState.available = debounced main - it must track the stalk, not
    # the radar's session churn.
    acc_state = int(cp_adas.vl["ACC_HUD_ADAS"]["AccState"])
    acc_on1 = bool(cp_adas.vl["ACC_HUD_ADAS"]["AccOn1"])
    raw_main = acc_on1 or acc_state in (1, 2, 3, 5)
    self.avail_fall_frames = 0 if raw_main else self.avail_fall_frames + 1
    acc_main = raw_main or self.avail_fall_frames < int(AVAIL_FALL_DEBOUNCE_TIME * 100)
    ret.cruiseState.available = acc_main
    set_speed = cp_adas.vl["ACC_HUD_ADAS"]["SetSpeed"]
    # follow the stock ACC set speed directly (community port does the same);
    # no artificial 30 km/h floor
    ret.cruiseState.speedCluster = set_speed * CV.KPH_TO_MS if acc_main else 0.
    ret.cruiseState.speed = ret.cruiseState.speedCluster

    acc_control_active = bool(cp_adas.vl["ACC_CMD"]["AccControlActive"])
    standstill_state = bool(cp_adas.vl["ACC_CMD"]["StandstillState"])
    ret.cruiseState.standstill = standstill_state
    ret.cruiseState.nonAdaptive = False

    self.res_btn_pressed = cp.vl["PCM_BUTTONS"]["BTN_AccUpDown_Cmd"] != 0
    self.counter_pcm_buttons = cp.vl["PCM_BUTTONS"]["Counter"]

    stock_acc_on = acc_control_active or acc_state in (2, 3, 5)
    # vendor parity (op_byd decompile 2026-10-05): the arm is re-judged EVERY
    # main-on cycle - main off clears ever_engaged, so an ACC off/on round
    # trip must show a fresh genuine session before OP arms again. Our old
    # process-monotonic flag let a stale session arm a later pure-set cycle.
    if acc_main:
      self.ever_engaged = self.ever_engaged or stock_acc_on
    else:
      self.ever_engaged = False

    # cruiseState.enabled = the arm latch: main-on (debounced) + ever engaged.
    # Brake and session drops deliberately do NOT clear it - that is the whole
    # point: the longitudinal state machine stops churning with the radar's
    # session (brake -> stock standby -> old latch fell -> pcmDisable killed
    # both axes; CANCEL/bounce did the same with no auto-recovery), lateral
    # survives everything short of main-off, and the longitudinal output is
    # gated on the live session downstream (carcontroller session gate +
    # panda acc checks). AccControlActive is the reliable session signal: 0
    # at ignition-leftover/standby, 1 whenever the MPC actually commands ACC,
    # including the SNG standstill hold (StandstillState=1).
    self.is_cruise_latch = acc_main and self.ever_engaged

    # boot-mid-cruise engage hold: reporting the raw latch immediately after
    # boot fires the pcmEnable edge while OP cannot act on it (canValid still
    # false / NO_ENTRY ready window), eating it for good - OP stays disabled
    # until the driver cycles ACC (route 0000000f: 20.6 s ACC-on/OP-off).
    # Holding the latch low here makes the edge land late and genuine instead:
    # OP engages the frame the hold expires. A SET pressed during the hold is
    # absorbed into the hold-end edge. cruiseState.available stays live.
    self.boot_frames += 1
    if self.boot_frames < int(BOOT_LATCH_HOLD_TIME * 100):  # card loop = 100 Hz
      self.is_cruise_latch = False

    ret.cruiseState.enabled = self.is_cruise_latch
    # stock LKA is coupled to the stock ACC on this platform; used by the
    # experimental angle path for engagement gating
    self.lka_on = ret.cruiseState.enabled

    # stalks and blind spots
    # Song Plus: the Han-layout LeftIndicator/RightIndicator bits (STALKS
    # byte0) NEVER move on this car - drive route 4 (2026-10-04): byte0
    # constant 0x01 through two 5-7 s stalk holds. The stalk position lives in
    # TURN_SIGNAL_SWITCH (byte4, 36|3@1+): 1 = neutral, 4/5 = the two held
    # detents (the actual lamps blink on 0x322 at 1.35 Hz during a hold).
    # The vendor agrees: BYD_SONG_PLUS is in its ALT_BLINKER_CARS list
    # (stalk-based blinkers, not lamp bits) - cp_byd carstate values dump.
    # Default 5=left is a guess until a real flick confirms the side; write
    # BydBlinkerStalkSwap=1 (raw file + restart) if the UI shows the opposite.
    switch = int(cp.vl["STALKS"]["TURN_SIGNAL_SWITCH"])
    ret.leftBlinker = switch == (4 if self.blinker_stalk_swap else 5)
    ret.rightBlinker = switch == (5 if self.blinker_stalk_swap else 4)
    ret.espDisabled = False
    ret.stockAeb = bool(cp_adas.vl["ACC_HUD_ADAS"]["AEB"])
    ret.stockFcw = bool(cp_adas.vl["ACC_HUD_ADAS"]["FCW"])

    ret.leftBlindspot = bool(cp.vl["BSD_RADAR"]["LEFT_APPROACH"])
    ret.rightBlindspot = bool(cp.vl["BSD_RADAR"]["RIGHT_APPROACH"])

    # The EPS latches TorqueFailed and gives up all steering input until it is
    # power-cycled (real vehicle, route 0000001c/20/22; the independent
    # yysnet/opendbc port documents the same behavior). Surface it so selfdrived
    # alerts the driver and blocks engagement instead of silently not steering;
    # the controller additionally stands down on the same bit.
    ret.steerFaultPermanent = bool(cp.vl["ACC_EPS_STATE"]["TorqueFailed"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    # Song Plus DM-i bus layout (verified from real vehicle CAN logs):
    #   bus0 = powertrain/chassis + ADAS feedback (EPS, BSD_RADAR, ACC_EPS_STATE, ...)
    #   bus2 = ACC/ADAS command domain (ACC_HUD_ADAS, ACC_CMD)
    dbc = DBC[CP.carFingerprint][Bus.pt]
    pt_messages = [
      ("EPS", 100),              # 0x11f
      ("WHEEL_SPEED", 50),       # 0x122
      ("ESP_SPEED", 20),         # 0x1f0, real vehicle speed in km/h
      ("BCM", 10),               # 0x12d
      ("STALKS", 10),            # 0x133
      ("DRIVE_STATE", 20),       # 0x242
      ("ACC_EPS_STATE", 50),     # 0x318
      ("PEDAL", 20),             # 0x342
      ("PCM_BUTTONS", 10),       # 0x3b0
      ("BSD_RADAR", 10),         # 0x418
    ]
    adas_messages = [
      ("ACC_HUD_ADAS", 50),      # 0x32d
      ("ACC_CMD", 50),           # 0x32e
      ("ACC_AEB", 50),           # 0x32f, stock radar AEB heartbeat (echoed on bus 0 by OP long)
      ("ACC_MPC_STATE", 50),     # 0x316, stock camera LKAS frame (echoed on bus 0)
    ]
    return {
      Bus.pt: CANParser(dbc, pt_messages, 0),
      Bus.adas: CANParser(dbc, adas_messages, 2),
    }
