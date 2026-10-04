from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, DbcDict, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarParts, SupportType
from opendbc.car.lateral import AngleSteeringLimits


class BydSafetyFlags(IntFlag):
  # experimental 482 camera angle path (Atto 3 style); Song Plus DM-i does not
  # transmit 0x1E2, the default (0) is the 790 LKAS_Output torque path
  ANGLE_STEERING = 1
  # OP longitudinal control (transparent ACC_CMD/ACC_HUD/ACC_AEB replacement).
  # Firmware gates the extra TX whitelist, the 0x32E accel checks and the
  # 0x32D/E/F forward block on this flag, so toggling the param needs no
  # reflash - just an OP restart (boardd re-applies the safety param).
  LONGITUDINAL = 2


# flip to True to use the experimental angle path instead of the default torque path
USE_ANGLE_STEERING = False


class CarControllerParams:
  # UI-only: the BYD cluster over-reads the GPS-anchored (10/9 calibrated)
  # speed by ~8-9% (drive route 4 2026-10-04: cluster 38/48 vs calibrated
  # 35/44 km/h in the two episodes). vEgoCluster is scaled to the cluster so
  # the screen matches the driver's frame of reference; control stays on
  # calibrated truth and never reads this field.
  CLUSTER_OVERREAD = 1.086

  # --- torque path (default) ---
  # EVERY scalar below is taken verbatim from the vendor build's runtime
  # values dump (docs_site/op_byd_data/values.json, the decrypted op_byd
  # opendbc.car.byd.values namespace) - no more hand-reconstruction. The
  # vendor's own real traffic matches them: |torque| p50=67/p90=128/max=193,
  # per-frame steps p99~16 within the Config=3 session (route 00000037).
  STEER_MAX = 300
  STEER_MAX_LOW = 150            # BYD_LOW_TORQUE flag variant (get_byd_torque_limits picks by platform flags)
  STEER_STEP = 2                 # 50 Hz command rate (100 Hz control loop)
  STEER_DELTA_UP = 16            # vendor 16/frame (its firmware caps at 17)
  STEER_DELTA_DOWN = 16
  STEER_DELTA_UP_LOW = 10        # LOW_TORQUE variant
  STEER_DELTA_DOWN_LOW = 10
  STEER_DRIVER_ALLOWANCE = 120
  STEER_DRIVER_MULTIPLIER = 3
  STEER_DRIVER_FACTOR = 1
  STEER_ERROR_MAX = 46           # vs EPS MainTorque echo; python-side reference only
  STEER_ERROR_MAX_UP = 46
  STEER_ERROR_MAX_DOWN = 46
  STEER_ERROR_MAX_HIGH = 150
  STEER_ERROR_MAX_UP_HIGH = 46
  STEER_ERROR_MAX_DOWN_HIGH = 46
  # 24 s of continuous steering triggers the vendor's derate + SDA hands-off
  # nudge (STEER_DEACTIVE_INTERVAL_MS / last_activate_nanos in its controller)
  STEER_DEACTIVE_INTERVAL_MS = 24000
  STEERING_RATE_DEG_LIMIT = 25       # deg/s hard cap (angle path)
  STEERING_ANGLE_DEG_LIMIT = 90      # deg hard cap (angle path)
  STEERING_TORQUE_LIMIT_SPEED = 20   # speed knee in the vendor's torque limit curve
  STEERING_INNER_EPS_SCALE_POINT = 30  # inner EPS scale point in the same curve
  # Armed-silence backstop (ours, not the vendor's - the vendor never sits
  # armed at zero because its demand loop is always steering): our measured
  # 0.48-0.72 s armed-zero latch band, exit to the retry burst before it.
  STEER_ZERO_EXIT_FRAMES = 21  # 50 Hz commands: ~0.42 s armed at |request| < 2 -> exit + re-burst
  # Request envelope (root cause 16, corrected): the vendor's own drive never
  # exceeds 193 absolute - INCLUDING its waiting-for-activation phase (up to
  # 2.9 s at Active=1 while the EPS still reports CruiseActivated=0, route
  # 7--12e: 415 waiting frames). The uncapped loop railed at +-300 in exactly
  # that phase (open-loop windup) and the residue on the bus at err=2 is what
  # escalated to err=4 + TorqueFailed; the vendor ELF also carries
  # HIGH error=200. STEER_MAX=300 stays the EPS absolute limit for the
  # firmware; the controller never emits past this envelope.
  STEER_MAX_REQUEST = 200
  # Low-speed fight guards A+B (root cause 17, route 1f 2026-09-30, doc
  # byd-lateral-lifecycle §七B): the only post-gate-removal latch was a
  # sustained Active=1 ±200 flip storm while the EPS had NOT accepted the
  # session (c=0, mt=0) against a driver holding 120-229 in the opposing
  # direction. The vendor's opposing full authority (-153 vs +166) lived in
  # a c=1 session; a c=0 armed stream fighting a heavy hand is a state it
  # never produced.
  STEER_YIELD_OPPOSING_TORQUE = 140  # yield output when |drv| past this AND demand opposes drv
  STEER_YIELD_DRV_RELEASE = 90       # hysteresis exit: below this, or demand swings same-direction
  STEER_C0_WAIT_FRAMES = 450         # 50 Hz: 9 s at Active=1 with c=0 -> stand down. Vendor wait max
  # 2.86 s (7--12e), our OBSERVED legit waits run to 4.1 s clean / 7.7 s next to an ACC bounce
  # (v2 road test) - 9 s stays clear of every healthy wait while capping the 1f-class stream
  # (45 s) into bounded duty cycles.
  STEER_C0_RETRY_HOLD_FRAMES = 300   # 50 Hz: hold the retry burst 6 s after a c0 stand-down (or until c rises)
  # The vendor streams Config=3 in EVERY session state - idle included
  # (route 00000037: 16 s of Cfg=3/Act=0/lanes 0/0 standby at boot, and
  # between every session). Lanes 0/0 make it standby, not an armed-silent
  # session; the "Config=3 idle latches" reading came from our early
  # from-scratch frames that missed the SETME_* fields.
  STEER_SESSION_CONFIG = 3
  # driver torque for steeringPressed (raw EPS scale: hands-off noise <50,
  # light grip 60-150; the vendor has no equivalent - openpilot-ism)
  STEER_THRESHOLD = 80  # TODO(Song Plus DM-i): calibrate
  # analog pedal press threshold (signals carry 0.01/unit; >0.01 = one count
  # of idle noise - see the routes 2d/31 override-flap log). 0.05 = 5 counts.
  PEDAL_PRESS_THRESHOLD = 0.05
  # torque-echo divergence guard (root cause 18, routes 28/2b 2026-10): the
  # two "LKAS Fault" latches were both preceded by the EPS silently aborting
  # torque (MainTorque -> 0, SteerWarning -> 1, SteerErrorCode still 0,
  # CruiseActivated still 1) while our request ramped 121->180 open-loop for
  # 0.4-0.7 s until err=4 + TorqueFailed latched. The vendor's own values
  # carry exactly this check (STEER_ERROR_MAX=46 vs the EPS MainTorque echo);
  # we never had it python-side. Stand down when our last applied request
  # and the EPS echo diverge past it for STEER_ECHO_GUARD_FRAMES command
  # frames (0.2 s) - real steering lags the request by <=20 counts (win28
  # trace), a refusing/aborted EPS sits at 0 while demand grows - then hold
  # the retry STEER_ECHO_RETRY_HOLD_FRAMES (3 s: long enough for the vendor
  # 0.5 s warning cycle to clear, short enough that a real abort recovers
  # "within seconds" once the EPS returns; guard B's 6 s hold stays the
  # outer backstop for the sustained-c=0 case).
  STEER_ECHO_GUARD_FRAMES = 10
  STEER_ECHO_RETRY_HOLD_FRAMES = 150

  # --- angle path (experimental 482), vendor tables verbatim ---
  ANGLE_RATE_LIMIT_UP = ([5., 10., 15.], [0.5, 2.0, 3.0])
  ANGLE_RATE_LIMIT_DOWN = ([5., 10., 15.], [0.5, 2.0, 3.0])
  ANGLE_RATE_LIMIT_UP_LOW = ([5., 10., 15.], [0.25, 1.0, 1.5])
  ANGLE_RATE_LIMIT_DOWN_LOW = ([5., 10., 15.], [0.25, 1.0, 1.5])
  ANGLE_LIMIT_UP = 220           # raw actuator units of the vendor's angle path
  ANGLE_LIMIT_DOWN = 220
  ANGLE_LIMIT_UP_LOW = 50
  ANGLE_LIMIT_DOWN_LOW = 50
  ANGLE_ERROR_ALLOWED = 5
  ANGLE_ERROR_ALLOWED_LOW = 2.5
  ANGLE_ERROR_MAX = 5
  # openpilot angle-path struct, built from the vendor tables above
  ANGLE_LIMITS: AngleSteeringLimits = AngleSteeringLimits(
    STEERING_ANGLE_DEG_LIMIT,
    ANGLE_RATE_LIMIT_UP,
    ANGLE_RATE_LIMIT_DOWN,
  )

  # per-platform nonlinear torque limit fits (the vendor's
  # NON_LINEAR_TORQUE_PARAMS); ours is the Song Plus DM-i 22 row
  NON_LINEAR_TORQUE_PARAMS = {
    'BYD_SONG_PLUS_DMI_22': [14.99976405, -0.55974149, 0.09633187, 12.47500536, 2.99999827, 1.49999146, 0.06850084],
  }

  # --- longitudinal (OP ACC_CMD), from the decrypted op_byd build and its
  # real-vehicle TX frames (route 00000037, docs_site/op_byd_logs) ---
  ACCEL_MIN = -4.0           # vendor ACCEL_MIN (opendbc base is -3.5)
  ACCEL_MAX = 2.0            # vendor ACCEL_MAX; raw 140 = +2.0 was the TX cap
  COMFORT_BAND_UPPER = 0.1   # sent only while accel >= 0 (route 37: band 0.1/0.05)
  COMFORT_BAND_LOWER = 0.05
  JERK_UPPER_LIMIT = 1.0     # raw 5; 692/697 active frames in route 37
  # raw 46; vendor clip(jerk, -4, -0.8) with no plan jerk available to us, so the calm-floor value
  JERK_LOWER_LIMIT = -0.8
  MIN_START_ACCEL = 0.2      # vendor: can_accel = max(0.2, can_accel) on resume
  STOP_ACCEL = -4.0          # vendor: adas TOO_CLOSE -> ACCEL_MIN

  def __init__(self, CP):
    pass


@dataclass
class BYDCarDocs(CarDocs):
  package: str = "DiPilot"
  car_parts: CarParts = field(default_factory=CarParts)
  # community port, not validated by comma
  support_type: SupportType = SupportType.COMMUNITY


@dataclass(frozen=True)
class BYDCarSpecs(CarSpecs):
  # specs cross-checked against the working vendor build's live carParams
  # (route 00000037): mass=1926, steerRatio=19.5
  centerToFrontRatio: float = 0.44
  steerRatio: float = 19.5
  mass: float = 1926.


@dataclass
class BYDPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'byd_general_pt'})


class CAR(Platforms):
  BYD_SONG_PLUS_DMI_22 = BYDPlatformConfig(
    [BYDCarDocs("BYD Song Plus DM-i 2022")],
    BYDCarSpecs(mass=1926., wheelbase=2.765),
  )


DBC = CAR.create_dbc_map()

# ratio between the wheel speed sensor and the real speed.
# Measured on the real car, 2026-10-02/03 road tests: GPS vs ESP_SPEED (0x1f0
# byte 4, "1 km/h/bit") over ~19k one-second samples across 6 boot sessions
# (0000002a/2b/2c/2d/31/32): true/GPS ratio p50 = 1.112-1.118, flat from
# 20 to 120 km/h (0-20 and the cluster side aside). The ESP encodes
# VehicleSpeed at 0.9 km/h per bit, not 1.0 - a raw 65 is 72 km/h true,
# which is what made "set 65 -> actually ~76 on the cluster" (cluster adds
# its usual ~+4%). 10/9 corrects vEgo to true speed.
HUD_MULTIPLIER = 10 / 9
