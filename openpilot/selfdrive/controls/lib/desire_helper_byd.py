"""BYD desire/lane-change adaptation - the vendor port, kept out of the stock
DesireHelper. Selected in modeld via get_desire_helper(CP.brand).

Three stock hooks are overridden here:
  _update_clear_window / _clear_window_ok - the BSD rear-traffic clear window
    (sunnypilot 5d45f8f9c9). BSD_RADAR 0x418 (10 Hz) is the only rear-traffic
    input on the 宋 - the rear radar is unwired (0x374 has zero frames on every
    logged bus) - so a single-frame blindspot sample at the start edge is
    lag-prone and the stalk decodes a 0.2-0.3 s transient as a blinker. A
    change may only start after the same-side blindspot stayed continuously
    clear for LANE_CHANGE_CLEAR_TIME_MIN. Raw carstate matches selfdrived's
    laneChangeBlocked predicate; the 2 s side.bsd_hold stays the existing
    (separate, stronger) start-blocker.
  _mid_maneuver_abort - a fresh same-side BSD hit mid-starting falls back to
    preLaneChange KEEPING the direction: DESIRES[dir][pre] is Desire.none so
    the car centers, and selfdrived's preLaneChange+blindspot predicate raises
    laneChangeBlocked on the next frame. laneChangeFinishing is deliberately
    never aborted (swinging back across the line that late is worse than
    completing the maneuver).

Plus the vendor lane-change assist speed floor (sunnypilot 1b0713ac11 T2),
LaneChangeAssistSpeed MPH; default "20" = the vendor's
dp_lat_lane_change_assist_speed, 0 = assist-less. Read through get_int_param
(stale compiled .so raises UnknownKeyName, and the C++ get_int would report an
unset key as 0 = assist-less); the raw file under /data/params is the manual
provisioning path until params_pyx.so is rebuilt with the new key. =0 -> inf:
the off->pre gate can't fire, an in-progress preLaneChange bails to off, and
the bluetooth request gate is denied - the assist machine is fully off at any
speed, which is also the switch that arms the vendor's assist-less yield mode
(AssistLessLaneChange, controlsd/selfdrived side).
"""
from openpilot.common.constants import CV
from openpilot.common.raw_params import get_int_param
from openpilot.common.realtime import DT_MDL

from openpilot.selfdrive.controls.lib.desire_helper import DesireHelper
from openpilot.selfdrive.controls.lib.desire_lib.constants import LaneChangeDirection

# BYD rear-traffic hardening (sunnypilot 5d45f8f9c9, docs/byd-lane-change.md
# §八): BSD_RADAR 0x418 (10 Hz) is the only rear-traffic input on this platform
# - the rear radar is unwired (0x374 has zero frames on every logged bus).
# Single-frame blindspot sampling cannot cover the BSD lag or the 0.2-0.3 s
# stalk transient, so starting requires this much continuous same-side
# blindspot clearance and a fresh hit mid-change aborts back to preLaneChange.
LANE_CHANGE_CLEAR_TIME_MIN = 0.5


class BydDesireHelper(DesireHelper):
  def __init__(self):
    super().__init__()
    # stock floor is 30 KPH; the vendor param replaces it. Init read closes the
    # frame%100 boot window (modeld's first periodic pass lands 5 s in)
    self._read_assist_speed()

  def _read_params(self):
    super()._read_params()
    self._read_assist_speed()

  def _read_assist_speed(self):
    mph = get_int_param(self.params, "LaneChangeAssistSpeed", 20)
    self.lane_change_speed_floor_ms = mph * CV.MPH_TO_MS if mph > 0 else float("inf")

  def _same_side_blindspot(self, carstate) -> bool:
    return ((carstate.leftBlindspot and self.lane_change_direction == LaneChangeDirection.left) or
            (carstate.rightBlindspot and self.lane_change_direction == LaneChangeDirection.right))

  def _update_clear_window(self, carstate):
    # accumulate continuous same-side clearance, reset on any hit
    if self._same_side_blindspot(carstate):
      self.lane_change_clear_time = 0.0
    else:
      self.lane_change_clear_time += DT_MDL

  def _clear_window_ok(self) -> bool:
    return self.lane_change_clear_time >= LANE_CHANGE_CLEAR_TIME_MIN - 1e-6

  def _mid_maneuver_abort(self, carstate) -> bool:
    return self._same_side_blindspot(carstate)
