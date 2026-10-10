"""Assist-less lane change latch (vendor port, sunnypilot 1b0713ac11 T3) -
latch matrix ported from sunnypilot test_assist_less_lane_change.py."""
from openpilot.cereal import car

from openpilot.selfdrive.controls.lib.assist_less_lane_change import (
  AssistLessLaneChange, get_assist_less_lane_change,
)


class TestAssistLessLaneChange:

  def setup_method(self):
    self.assist_less = AssistLessLaneChange()
    self._reset_states()

  def _reset_states(self):
    self.assist_less.assist_disabled = True
    self.assist_less.active = False

    self.CS = car.CarState.new_message()
    self.CS.leftBlinker = False
    self.CS.rightBlinker = False
    self.CS.steeringPressed = False
    self.CS.steeringTorque = 0

  def _feed(self, blinker: str, torque: int, pressed: bool, frames: int = 2):
    self.CS.leftBlinker = blinker == 'left'
    self.CS.rightBlinker = blinker == 'right'
    self.CS.steeringTorque = torque
    self.CS.steeringPressed = pressed
    results = [self.assist_less.update(self.CS) for _ in range(frames)]
    return results

  def test_assisted_mode_never_yields(self):
    # LaneChangeAssistSpeed > 0: the desire_helper state machine owns the maneuver
    self.assist_less.assist_disabled = False
    assert self._feed('left', 50, True) == [False, False]

  def test_left_activation_blinker_plus_left_torque(self):
    # sign convention verified on real routes: SteerDriverTorque left = positive
    # (activation is same-frame, like the vendor: both conditions held -> latch)
    assert self._feed('left', 50, True) == [True, True]

  def test_right_activation_blinker_plus_right_torque(self):
    assert self._feed('right', -50, True) == [True, True]

  def test_opposite_torque_does_not_activate(self):
    # left blinker but the wheel is held right (driver resisting) - no latch,
    # and no false "Changing Lanes" either
    assert self._feed('left', -50, True) == [False, False]

  def test_torque_without_pressed_flag_does_not_activate(self):
    assert self._feed('left', 50, False) == [False, False]

  def test_latch_holds_after_driver_releases_wheel(self):
    self._feed('left', 50, True)
    self.CS.steeringPressed = False
    self.CS.steeringTorque = 0
    assert self.assist_less.update(self.CS) is True

  def test_latch_holds_while_other_blinker_takes_over(self):
    # vendor de-activation needs BOTH blinkers off; a left->right flip keeps the latch
    self._feed('left', 50, True)
    self.CS.leftBlinker = False
    self.CS.rightBlinker = True
    assert self.assist_less.update(self.CS) is True

  def test_release_on_blinkers_off(self):
    self._feed('left', 50, True)
    assert self._feed('none', 0, False) == [False, False]

  def test_reactivation_after_release(self):
    self._feed('left', 50, True)
    self._feed('none', 0, False)
    assert self._feed('left', 50, True) == [True, True]

  def test_non_byd_never_yields(self):
    # brand factory: other brands get the no-op latch and keep lateral engaged
    # even with blinker + same-direction torque - stock behavior untouched
    other = get_assist_less_lane_change("toyota")
    self.CS.leftBlinker = True
    self.CS.steeringPressed = True
    self.CS.steeringTorque = 50
    assert other.update(self.CS) is False
    assert get_assist_less_lane_change("byd").__class__ is AssistLessLaneChange
