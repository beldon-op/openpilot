from types import SimpleNamespace

import pytest

from openpilot.common.constants import CV
from openpilot.cereal import car
from openpilot.selfdrive.car.cruise import VCruiseCarrot


def make_v_cruise_helper(brand, pcm_cruise, speed_from_pcm, prev_kph, available_last):
  helper = VCruiseCarrot.__new__(VCruiseCarrot)
  helper.CP = SimpleNamespace(brand=brand, pcmCruise=pcm_cruise)
  helper.params = SimpleNamespace(get_int=lambda k: speed_from_pcm, get_bool=lambda k: False, get_float=lambda k: 0.0)
  helper.speed_from_pcm = speed_from_pcm
  helper.frame = 0
  helper.is_metric = True
  helper.v_cruise_kph = prev_kph
  helper.v_cruise_cluster_kph = prev_kph
  helper.v_cruise_kph_last = prev_kph
  helper.v_ego_kph_set = 40
  helper._cruise_speed_min = 5
  helper._cruise_speed_max = 135
  helper._cruise_ready = False
  helper._paddle_decel_active = False
  helper._soft_hold_count = 0
  helper._soft_hold_active = 0
  helper._cancel_timer = 0
  helper._lat_enabled = False
  helper._activate_cruise = 0
  helper._cruise_available = True
  helper._v_cruise_kph_at_brake = 0
  helper.cruise_state_available_last = available_last
  helper.enabled_last = False
  helper.autoCruiseControl_cancel_timer = 0
  helper.bluetooth_commands = SimpleNamespace(read=lambda **kwargs: None)
  helper._add_log = lambda log: None
  helper.update_params = lambda is_metric: None
  helper._update_carrot_man = lambda sm: None
  helper._prepare_brake_gas = lambda CS, CC: None
  helper._update_cruise_buttons = lambda CS, CC, v_cruise_kph: v_cruise_kph
  return helper


class FakeSM(dict):
  pass


def run_update(helper, available, speed_kph, cluster_kph):
  CS = car.CarState(
    gearShifter=car.CarState.GearShifter.drive,
    vEgoCluster=40 * CV.KPH_TO_MS,
    cruiseState={"available": available, "speed": speed_kph * CV.KPH_TO_MS,
                 "speedCluster": cluster_kph * CV.KPH_TO_MS},
  )
  sm = FakeSM({"carControl": car.CarControl(enabled=False)})
  sm.alive = {"longitudinalPlan": False, "radarState": False, "drivingModelData": False}
  helper.update_v_cruise(CS, sm, True)


def test_byd_follows_stalk_set_speed_instead_of_virtual_clip():
  # BYD (Song Plus DM-i) is pcmCruise with the SpeedFromPCM=2 default, whose
  # virtual target nothing on the car can raise (no buttonEvents in
  # byd/carstate.py) - pre-fix the helper clipped to 30 and OP never
  # accelerated. The branch must take the stalk's radar SetSpeed instead.
  helper = make_v_cruise_helper("byd", pcm_cruise=True, speed_from_pcm=2, prev_kph=30, available_last=True)
  run_update(helper, available=True, speed_kph=72, cluster_kph=72)
  assert helper.v_cruise_kph == pytest.approx(72)
  assert helper.v_cruise_cluster_kph == pytest.approx(72)


def test_byd_holds_last_target_through_set_speed_zero():
  # The radar's own frames can carry SetSpeed==0 for the first 1-2 s of a
  # session (byd/carcontroller.py:339-346, routes 31/2d); sync must not chase
  # the blip to 0 (that would command a stop).
  helper = make_v_cruise_helper("byd", pcm_cruise=True, speed_from_pcm=2, prev_kph=50, available_last=True)
  run_update(helper, available=True, speed_kph=0, cluster_kph=0)
  assert helper.v_cruise_kph == 50
  assert helper.v_cruise_cluster_kph == 50


@pytest.mark.parametrize("speed_from_pcm", [1, 2])
def test_byd_sync_ignores_speed_from_pcm(speed_from_pcm):
  # The brand gate fires regardless of the hidden param, so a fresh device
  # (default 2) works without user configuration.
  helper = make_v_cruise_helper("byd", pcm_cruise=True, speed_from_pcm=speed_from_pcm, prev_kph=30, available_last=True)
  run_update(helper, available=True, speed_kph=88, cluster_kph=88)
  assert helper.v_cruise_kph == pytest.approx(88)


def test_other_brands_keep_speed_from_pcm_paths():
  # toyota, param==2: unchanged virtual clip (pre-fix BYD behavior).
  helper = make_v_cruise_helper("toyota", pcm_cruise=True, speed_from_pcm=2, prev_kph=20, available_last=True)
  run_update(helper, available=True, speed_kph=60, cluster_kph=60)
  assert helper.v_cruise_kph == 30  # clip(20, 30, 135)

  # toyota, param==1: unchanged pcm sync.
  helper = make_v_cruise_helper("toyota", pcm_cruise=True, speed_from_pcm=1, prev_kph=20, available_last=True)
  run_update(helper, available=True, speed_kph=60, cluster_kph=60)
  assert helper.v_cruise_kph == pytest.approx(60)


def test_byd_unavailable_keeps_carrot_default_branch():
  # ACC main off: available==False path untouched - clip(min, max) of the
  # virtual value, NOT the stalk sync (main off has no meaningful SetSpeed).
  helper = make_v_cruise_helper("byd", pcm_cruise=True, speed_from_pcm=2, prev_kph=90, available_last=True)
  run_update(helper, available=False, speed_kph=0, cluster_kph=0)
  assert helper.v_cruise_kph == 90
