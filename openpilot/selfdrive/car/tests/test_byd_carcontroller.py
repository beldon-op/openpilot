import pytest

from opendbc.car import Bus, structs
from opendbc.car.byd.carcontroller import ACCEL_SLEW_UP, CarController
from opendbc.car.byd.values import DBC, CAR, CarControllerParams

LongCtrlState = structs.CarControl.Actuators.LongControlState

ACC_CMD_ADDR = 0x32E
ACCEL_MIN_START = CarControllerParams.MIN_START_ACCEL
# command cadence: _update_longitudinal fires every other 100 Hz frame
DT_CMD = 0.02


class FakeCS:
  """CarState stand-in: the controller only reads .out plus the cached bus-2
  dicts (radar_acc_msg/adas_msg/aeb_msg) and the lateral EPS flags."""

  def __init__(self):
    self.out = structs.CarState()
    self.radar_acc_msg = {}
    self.adas_msg = {}
    self.aeb_msg = {}
    self.cam_lkas = {}
    self.cruise_activated = False
    self.torque_failed = False
    self.steer_error = 0
    self.steer_warning = False


def make_controller() -> CarController:
  CP = structs.CarParams.new_message()
  CP.carFingerprint = CAR.BYD_SONG_PLUS_DMI_22
  CP.openpilotLongitudinalControl = True
  dbc_names = {Bus.pt: DBC[CP.carFingerprint][Bus.pt]}
  return CarController(dbc_names, CP)


def make_cc(accel: float, long_active: bool = True, enabled: bool = True,
            state: LongCtrlState = LongCtrlState.pid) -> structs.CarControl:
  CC = structs.CarControl()
  CC.enabled = enabled
  CC.latActive = False
  CC.longActive = long_active
  CC.actuators.accel = accel
  CC.actuators.longControlState = state
  return CC.as_reader()   # the controller calls actuators.as_builder(), a reader method


def step(cc, cs: FakeCS, controller: CarController) -> float:
  """Run one 100 Hz update pair and return the AccelCmd physically sent in
  the 50 Hz 0x32E (decoded from byte 0)."""
  acc_raw = None
  for _ in range(2):
    _, can_sends = controller.update(cc, cs, 0)
    for addr, dat, _ in can_sends:
      if addr == ACC_CMD_ADDR:
        acc_raw = dat[0]
  assert acc_raw is not None, "no 0x32E sent"
  return acc_raw * 0.05 - 5.0


def radar_frame(active: bool, accel: float = 0.0, standstill: bool = False) -> dict:
  return {
    "AccelCmd": accel, "ComfortBandUpper": 0.0, "ComfortBandLower": 0.0,
    "SETME1_0x1": 1, "JerkUpperLimit": 0.0, "ResumeFromStandstill": 0,
    "JerkLowerLimit": 0.0, "StandstillState": int(standstill), "BrakeBehaviour": 0,
    "AccReqNotStandstill": 1, "AccControlActive": int(active),
    "AccOverrideOrStandstill": 0, "EspBehaviour": 1,
  }

def adas_frame(set_speed: float = 40.0) -> dict:
  return {"SetSpeed": set_speed, "HasLead": 0, "SetDistance": 3, "LeadingDistance": 0,
          "AEB": 0, "FCW": 0, "SETME1_0x1": 1, "AccState": 2, "AccOn1": 1,
          "CloseWarning": 0, "SETME2_0x1": 1, "Notify": 0, "Status": 4, "SETME3_0xFFF": 0xFFF}


def aeb_frame() -> dict:
  return {"PAYLOAD": 0}


class TestBydLongitudinalSessionGate:
  """No driver activation (session) -> no OP longitudinal, ever (the 设置 state
  must not control): the controller re-broadcasts the radar frame verbatim."""

  def test_set_without_session_does_not_command(self):
    c = make_controller()
    cs = FakeCS()
    cs.radar_acc_msg = radar_frame(active=False, accel=-2.5)
    cs.adas_msg = adas_frame(40.0)
    cs.aeb_msg = aeb_frame()
    step(make_cc(accel=1.0), cs, c)
    # echo of the radar's own -2.5, not our +1.0 request
    assert step(make_cc(accel=1.0), cs, c) == pytest.approx(-2.5)

  def test_brake_press_yields_to_echo(self):
    c = make_controller()
    cs = FakeCS()
    cs.radar_acc_msg = radar_frame(active=True, accel=0.5)
    cs.adas_msg = adas_frame(40.0)
    cs.aeb_msg = aeb_frame()
    step(make_cc(accel=0.0), cs, c)
    cs.out.brakePressed = True
    cs.radar_acc_msg = radar_frame(active=False, accel=0.0)
    assert step(make_cc(accel=-2.0), cs, c) == pytest.approx(0.0)

  def test_no_set_speed_never_accelerates(self):
    c = make_controller()
    cs = FakeCS()
    cs.radar_acc_msg = radar_frame(active=True, accel=0.0)
    cs.adas_msg = adas_frame(0.0)
    cs.aeb_msg = aeb_frame()   # engage frame: session up, speed not yet in
    for _ in range(20):
      assert step(make_cc(accel=1.5), cs, c) <= 0.0 + 1e-9


class TestBydLongitudinalSlew:
  """Command continuity (the 2026-10-05 "一直在加油刹车" complaint): every
  commanded accel ramps from what was actually last broadcast, including
  across OP->radar->OP handovers."""

  def test_engage_ramps_from_echoed_value(self):
    c = make_controller()
    cs = FakeCS()
    cs.radar_acc_msg = radar_frame(active=False, accel=-2.0)
    cs.adas_msg = adas_frame(40.0)
    cs.aeb_msg = aeb_frame()
    step(make_cc(accel=1.0), cs, c)   # one yield frame: tracker snaps to -2.0
    assert c.accel_cmd_sent == pytest.approx(-2.0)

    cs.radar_acc_msg = radar_frame(active=True, accel=-2.0)
    deltas = []
    prev = c.accel_cmd_sent
    for _ in range(100):
      step(make_cc(accel=1.0), cs, c)
      deltas.append(c.accel_cmd_sent - prev)
      prev = c.accel_cmd_sent
    slew_step = ACCEL_SLEW_UP * DT_CMD
    assert all(d <= slew_step + 1e-9 for d in deltas), "accel stepped beyond the slew"
    assert deltas[0] == pytest.approx(slew_step)     # moved from the echo value, not a jump
    assert c.accel_cmd_sent == pytest.approx(1.0)    # converged to the demand

  def test_resume_bump_steps_through_the_limit(self):
    c = make_controller()
    cs = FakeCS()
    cs.radar_acc_msg = radar_frame(active=True, accel=-1.0, standstill=True)
    cs.adas_msg = adas_frame(40.0)
    cs.aeb_msg = aeb_frame()
    for _ in range(50):   # ramp down: the slew reaches -1.0 in 0.03 steps, not a jump
      step(make_cc(accel=-1.0), cs, c)
    assert c.accel_cmd_sent == pytest.approx(-1.0)
    # standstill release is a deliberate step: full MIN_START_ACCEL at once
    assert step(make_cc(accel=-0.5, state=LongCtrlState.starting), cs, c) >= ACCEL_MIN_START - 1e-9
