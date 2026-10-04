from collections import defaultdict

from opendbc.car import Bus, structs
from opendbc.car.byd.carstate import AVAIL_FALL_DEBOUNCE_TIME, BOOT_LATCH_HOLD_TIME, CarState
from opendbc.car.byd.values import CAR


class FakeParser:
  """CANParser stand-in: all signals read 0 unless set. CarState only touches
  .vl[message][signal]."""

  def __init__(self):
    self.vl = defaultdict(lambda: defaultdict(float))


def make_carstate() -> CarState:
  CP = structs.CarParams.new_message()
  CP.carFingerprint = CAR.BYD_SONG_PLUS_DMI_22
  return CarState(CP)


def run_update(cs: CarState, main: bool = True, session: bool = True, brake: bool = False):
  """main = AccOn1 (stalk main switch posture); session = AccControlActive
  (radar commanding). Independent on the wire - brake cancel and CANCEL drop
  the session while main stays up."""
  pt, adas = FakeParser(), FakeParser()
  if main:
    adas.vl['ACC_HUD_ADAS']['AccOn1'] = 1
    adas.vl['ACC_HUD_ADAS']['AccState'] = 1
  if session:
    adas.vl['ACC_CMD']['AccControlActive'] = 1
    adas.vl['ACC_HUD_ADAS']['AccState'] = 2
    adas.vl['ACC_HUD_ADAS']['AccOn1'] = 1
  if brake:
    pt.vl['DRIVE_STATE']['BrakePressed'] = 1
  ret = cs.update({Bus.pt: pt, Bus.adas: adas})
  return ret


HOLD_FRAMES = int(BOOT_LATCH_HOLD_TIME * 100)
DEBOUNCE_FRAMES = int(AVAIL_FALL_DEBOUNCE_TIME * 100)


class TestBydBootLatchHold:
  """Boot-mid-cruise: the stock ACC is engaged when the device boots.

  cruiseState.enabled must stay False through the boot hold and then produce
  ONE genuine rising edge once the hold expires (vendor mechanism, route
  7--12e) - reporting the raw latch immediately eats the edge in the generic
  layer (canValid / ready window NO_ENTRY) and OP never engages.
  """

  def test_hold_delays_the_engage_edge(self):
    cs = make_carstate()
    for _ in range(10):
      ret = run_update(cs, main=True, session=True)
      assert not ret.cruiseState.enabled        # latch held low
      assert ret.cruiseState.available          # available stays live

    cs.boot_frames = HOLD_FRAMES - 2            # last held frame (+1 in update)
    ret = run_update(cs, main=True, session=True)
    assert not ret.cruiseState.enabled
    ret = run_update(cs, main=True, session=True)   # hold expires -> the one edge
    assert ret.cruiseState.enabled
    ret = run_update(cs, main=True, session=True)   # stays engaged, no flapping
    assert ret.cruiseState.enabled

  def test_set_during_hold_is_absorbed_into_hold_end_edge(self):
    cs = make_carstate()
    for _ in range(10):
      run_update(cs, main=True, session=False)  # main before SET, no session yet
    cs.boot_frames = HOLD_FRAMES - 3
    ret = run_update(cs, main=True, session=True)   # SET lands during the hold
    assert not ret.cruiseState.enabled
    ret = run_update(cs, main=True, session=True)   # still held
    assert not ret.cruiseState.enabled
    ret = run_update(cs, main=True, session=True)   # hold end -> the one edge
    assert ret.cruiseState.enabled

  def test_no_session_no_edge(self):
    # ignition leftover: main posture without a session this drive must never arm
    cs = make_carstate()
    cs.boot_frames = HOLD_FRAMES + 100
    for _ in range(5):
      ret = run_update(cs, main=True, session=False)
      assert not ret.cruiseState.enabled
      assert ret.cruiseState.available


class TestBydMainLatch:
  """cruiseState.enabled = debounced main-on + one genuine session this drive.
  Brake and session drops deliberately do not clear it; only main-off does."""

  def test_brake_does_not_exit(self):
    cs = make_carstate()
    for _ in range(HOLD_FRAMES + 2):
      run_update(cs, main=True, session=True)
    assert cs.is_cruise_latch
    ret = run_update(cs, main=True, session=True, brake=True)   # session drops on brake
    assert ret.cruiseState.enabled

  def test_session_standby_does_not_exit(self):
    # CANCEL / bounce: radar leaves the engaged states, main stays up
    cs = make_carstate()
    for _ in range(HOLD_FRAMES + 2):
      run_update(cs, main=True, session=True)
    assert cs.is_cruise_latch
    for _ in range(200):
      ret = run_update(cs, main=True, session=False)
      assert ret.cruiseState.enabled
      assert ret.cruiseState.available

  def test_main_off_exits(self):
    cs = make_carstate()
    for _ in range(HOLD_FRAMES + 2):
      run_update(cs, main=True, session=True)
    assert cs.is_cruise_latch
    for _ in range(DEBOUNCE_FRAMES + 2):
      ret = run_update(cs, main=False, session=False)
    assert not ret.cruiseState.enabled
    assert not ret.cruiseState.available

  def test_bounce_blip_does_not_flap_available(self):
    # the camera's ~2 s main glitches (route 11 t=156.4 / route 12 t=200.3)
    # must not flap cruiseState.available - but a real main-off (longer than
    # the debounce) must still come through
    cs = make_carstate()
    for _ in range(HOLD_FRAMES + 2):
      run_update(cs, main=True, session=True)
    for _ in range(DEBOUNCE_FRAMES - 1):
      ret = run_update(cs, main=False, session=False)
      assert ret.cruiseState.available          # held through the drop
      assert ret.cruiseState.enabled
    ret = run_update(cs, main=True, session=True)   # recovers: no edge was lost
    assert ret.cruiseState.enabled

    for _ in range(DEBOUNCE_FRAMES + 2):            # sustained main-off
      ret = run_update(cs, main=False, session=False)
    assert not ret.cruiseState.available
    assert not ret.cruiseState.enabled
