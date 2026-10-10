"""Turn-signal lane change assist threshold + BSD rear-traffic gate
(BYD port of sunnypilot 1b0713ac11 T2 / 5d45f8f9c9, docs/byd-lane-change.md §三/§八).

LaneChangeAssistSpeed (MPH): >0 replaces the old hardcoded 30 KPH floor, 0
disables the assist state machine at any speed (the assist-less yield mode
then owns the maneuver).

BYD rear traffic: BSD_RADAR 0x418 (10 Hz) is the only rear-traffic source on
the platform (rear radar unwired, zero 0x374 frames in all road-test segments),
so a change may only start after the same-side blindspot stayed continuously
clear for LANE_CHANGE_CLEAR_TIME_MIN in preLaneChange, and a same-side hit
mid-maneuver aborts back to preLaneChange.
"""
from types import SimpleNamespace

from openpilot.cereal import log
from openpilot.common.constants import CV
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.controls.lib.desire_helper import get_desire_helper, DesireHelper
from openpilot.selfdrive.controls.lib.desire_helper_byd import LANE_CHANGE_CLEAR_TIME_MIN
from openpilot.selfdrive.controls.lib.desire_lib.constants import LANE_CHANGE_SPEED_MIN

LaneChangeState = log.LaneChangeState
LaneChangeDirection = log.LaneChangeDirection

V_LC = 30 * CV.MPH_TO_MS  # above every tested threshold
FRAMES_CLEAR = int(round(LANE_CHANGE_CLEAR_TIME_MIN / DT_MDL))  # 10 frames @ 20 Hz


def make_dh(threshold_mph: float, brand: str = "byd") -> DesireHelper:
  # BYD brand: the vendor port (param threshold, clear window, mid abort) is
  # gated on brand - non-BYD keeps the stock behavior (guarded below)
  dh = get_desire_helper(brand)
  # freeze param reads (tests must not depend on the machine's param store)
  dh._update_params_periodic = lambda: None
  dh._process_sides = lambda car, model, radar: None
  dh._check_desire_state = lambda model, car, maneuver: None
  dh.bluetooth_commands = SimpleNamespace(read=lambda allowed: None)
  dh.laneChangeNeedTorque = 0
  dh.laneChangeBsd = 0
  dh.laneLineCheck = 0
  # driver-blinker path with a clean, available left side (fixture mirrors
  # test_desire_helper.py: carrot starts a driver-requested change without a
  # torque nudge when laneChangeNeedTorque=0)
  side = dh.left
  side.lane_available = True
  side.edge_available = False
  side.dist_to_edge_far = 2.0
  side.lane_change_available_geom = True
  side.lane_change_available = True
  side.side_object_detected = False
  side.bsd_hold_counter = 0
  side.lane_line_info_mod = 0
  side.lane_line_info_edge_detect = False
  side.lane_change_available_released = False
  side.lane_available_trigger = False
  side.lane_appeared = False
  side.lane_exist_count.counter = 10
  if brand == "byd":
    dh.lane_change_speed_floor_ms = float("inf") if threshold_mph == 0 else threshold_mph * CV.MPH_TO_MS
  return dh


def feed(dh: DesireHelper, v_ego: float, left_blinker: bool = False, right_blinker: bool = False,
         torque: int = 0, pressed: bool = False, left_bs: bool = False, right_bs: bool = False,
         lane_change_prob: float = 0.1) -> DesireHelper:
  cs = SimpleNamespace(
    canValid=True, vEgo=v_ego, aEgo=0.0, trailerConnected=False,
    leftBlinker=left_blinker, rightBlinker=right_blinker,
    leftBlindspot=left_bs, rightBlindspot=right_bs,
    steeringTorque=torque, steeringPressed=pressed,
  )
  carrot_man = SimpleNamespace(atcType="", carrotCmdIndex=0, carrotCmd="", carrotArg="")
  dh.update(cs, SimpleNamespace(), True, lane_change_prob, carrot_man, SimpleNamespace())
  return dh


def start_left_change(dh: DesireHelper) -> DesireHelper:
  """blinker edge + the full clear window, returns dh in laneChangeStarting"""
  dh = feed(dh, V_LC, left_blinker=True)
  assert dh.lane_change_state == LaneChangeState.preLaneChange
  for _ in range(FRAMES_CLEAR):
    dh = feed(dh, V_LC, left_blinker=True)
  assert dh.lane_change_state == LaneChangeState.laneChangeStarting
  return dh


class TestLaneChangeAssistSpeed:

  def test_stock_threshold_engages_after_clear_window(self):
    dh = make_dh(20)
    dh = feed(dh, V_LC, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    # the clear window has to elapse before the blinker can start the change
    for _ in range(FRAMES_CLEAR - 1):
      dh = feed(dh, V_LC, left_blinker=True)
      assert dh.lane_change_state == LaneChangeState.preLaneChange
    dh = feed(dh, V_LC, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    assert dh.desire == log.Desire.laneChangeLeft

  def test_below_threshold_stays_off(self):
    dh = make_dh(20)
    dh = feed(dh, 10 * CV.MPH_TO_MS, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.off

  def test_raised_threshold_blocks_old_pass_speed(self):
    dh = make_dh(30)
    dh = feed(dh, 25 * CV.MPH_TO_MS, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.off
    dh = make_dh(30)
    dh = feed(dh, 35 * CV.MPH_TO_MS, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange

  def test_zero_disables_assist_at_any_speed(self):
    dh = make_dh(0)
    for v in (5 * CV.MPH_TO_MS, 60 * CV.MPH_TO_MS):
      dh = feed(dh, v, left_blinker=True)
      assert dh.lane_change_state == LaneChangeState.off
      assert dh.desire == log.Desire.none

  def test_zero_aborts_an_in_progress_change(self):
    dh = make_dh(20)
    dh = feed(dh, V_LC, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    # driver flips the param to 0 mid-maneuver: preLaneChange bails to off
    dh.lane_change_speed_floor_ms = float("inf")
    dh = feed(dh, V_LC, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.off


class TestBlindspotRearTrafficGate:

  def test_blindspot_at_edge_never_starts(self):
    # a blindspot hit keeps the change blocked even after the window would
    # otherwise have elapsed (it is reset on every same-side hit)
    dh = make_dh(20)
    dh = feed(dh, V_LC, left_blinker=True)
    for _ in range(FRAMES_CLEAR + 5):
      dh = feed(dh, V_LC, left_blinker=True, left_bs=True)
      assert dh.lane_change_state == LaneChangeState.preLaneChange

  def test_blindspot_during_window_reopens_it(self):
    dh = make_dh(20)
    dh = feed(dh, V_LC, left_blinker=True)
    for _ in range(FRAMES_CLEAR - 4):  # partial window
      dh = feed(dh, V_LC, left_blinker=True)
    dh = feed(dh, V_LC, left_blinker=True, left_bs=True)  # resets to 0
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    for _ in range(FRAMES_CLEAR - 1):
      dh = feed(dh, V_LC, left_blinker=True)
      assert dh.lane_change_state == LaneChangeState.preLaneChange
    dh = feed(dh, V_LC, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting

  def test_same_side_blindspot_mid_maneuver_aborts(self):
    dh = start_left_change(make_dh(20))
    dh = feed(dh, V_LC, left_blinker=True, left_bs=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    assert dh.desire == log.Desire.none  # lat request released, car centers
    assert dh.lane_change_direction == LaneChangeDirection.left  # blocked alert keeps the side

  def test_opposite_blindspot_does_not_abort(self):
    dh = start_left_change(make_dh(20))
    dh = feed(dh, V_LC, left_blinker=True, right_bs=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    assert dh.desire == log.Desire.laneChangeLeft

  def test_abort_requires_full_window_to_restart(self):
    dh = start_left_change(make_dh(20))
    dh = feed(dh, V_LC, left_blinker=True, left_bs=True)  # abort
    dh = feed(dh, V_LC, left_blinker=True)  # bs gone, window from 0
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    for _ in range(FRAMES_CLEAR - 2):
      dh = feed(dh, V_LC, left_blinker=True)
      assert dh.lane_change_state == LaneChangeState.preLaneChange
    dh = feed(dh, V_LC, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting

  def test_consecutive_change_requires_new_window(self):

    dh = start_left_change(make_dh(20))
    # run starting -> finishing -> preLaneChange (blinker stays on): ll_prob
    # fades out over .5s at 2*DT_MDL, back in over 1s at DT_MDL; lane_change_prob
    # low so the starting->finishing edge passes
    for _ in range(FRAMES_CLEAR):
      dh = feed(dh, V_LC, left_blinker=True, lane_change_prob=0.0)
      if dh.lane_change_state == LaneChangeState.laneChangeFinishing:
        break
    assert dh.lane_change_state == LaneChangeState.laneChangeFinishing
    for _ in range(FRAMES_CLEAR * 2 + 2):
      dh = feed(dh, V_LC, left_blinker=True, lane_change_prob=0.0)
      if dh.lane_change_state == LaneChangeState.preLaneChange:
        break
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    # the next change must NOT start immediately - window reopened
    dh = feed(dh, V_LC, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange


class TestStockBrandUntouched:
  """The BYD port is brand-gated - stock behavior must be identical elsewhere."""

  def test_non_byd_keeps_stock_threshold_and_no_clear_window(self):
    # 30.6 KPH: above the stock 30 KPH floor but below the BYD 20 MPH default;
    # non-BYD enters preLaneChange and starts on the very next frame (no window)
    dh = make_dh(20, brand="toyota")
    assert dh.lane_change_speed_floor_ms == LANE_CHANGE_SPEED_MIN
    v = 30.6 * CV.KPH_TO_MS
    dh = feed(dh, v, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.preLaneChange
    dh = feed(dh, v, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting

  def test_param_ignored_on_stock_brand(self):
    # even with the device param set to 0, non-BYD keeps the stock constant
    dh = make_dh(0, brand="toyota")
    assert dh.lane_change_speed_floor_ms == LANE_CHANGE_SPEED_MIN

  def test_no_mid_maneuver_abort_on_stock_brand(self):
    dh = make_dh(20, brand="toyota")
    dh = feed(dh, V_LC, left_blinker=True)
    dh = feed(dh, V_LC, left_blinker=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
    # same-side blindspot is the BYD abort trigger only - stock just had the
    # (pre-gated) bsd_hold path, mid-starting stayed untouched
    dh = feed(dh, V_LC, left_blinker=True, left_bs=True)
    assert dh.lane_change_state == LaneChangeState.laneChangeStarting
