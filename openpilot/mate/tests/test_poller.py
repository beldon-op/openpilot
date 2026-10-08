import json
import time

from openpilot.cereal import car, custom, log
from openpilot.mate.poller import (
  EVENT_FLAGS,
  SoundTracker,
  button_payloads,
  car_snapshot,
  diff_events,
  enum_name,
  event_to_dict,
  opstate_payload,
  sound_payload,
)
from openpilot.selfdrive.ui.soundd import AudibleAlert


class FakeSM:
  """最小 SubMaster 替身：updated / 下标 / recv_time 三个接口。"""

  def __init__(self, **messages):
    self._messages = dict(messages)
    self.updated = dict.fromkeys(self._messages, True)
    now = time.monotonic()
    self.recv_time = dict.fromkeys(self._messages, now)

  def __getitem__(self, key):
    return self._messages[key]


def selfdrive_state(**kwargs):
  defaults = {"state": "disabled", "enabled": False, "active": False, "engageable": False,
              "alertSound": "none", "alertText1": "", "alertText2": ""}
  return log.SelfdriveState.new_message(**{**defaults, **kwargs})


def make_sm(ss=None, carrot=None, cs=None, events=None):
  sm = FakeSM(
    selfdriveState=ss or selfdrive_state(),
    carrotMan=carrot or custom.CarrotMan.new_message(leftSec=100),
    carState=cs or car.CarState.new_message(),
    onroadEvents=events or log.OnroadEvent.new_message(),
  )
  return sm


class TestEvents:
  def test_event_to_dict_all_flags(self):
    e = log.OnroadEvent.new_message(name="doorOpen", warning=True, noEntry=True)
    d = event_to_dict(e)
    assert d["name"] == "doorOpen"
    assert d["warning"] and d["noEntry"]
    assert not d["softDisable"]
    assert set(d) == {"name", *EVENT_FLAGS}

  def test_diff_added_removed(self):
    prev: dict[str, dict] = {}
    e1 = event_to_dict(log.OnroadEvent.new_message(name="doorOpen", warning=True))
    prev, payload = diff_events(prev, [e1])
    assert payload is not None
    assert payload["added"] == ["doorOpen"] and payload["removed"] == []
    assert payload["active"] == [e1]

    e2 = event_to_dict(log.OnroadEvent.new_message(name="seatbeltNotLatched", warning=True))
    prev, payload = diff_events(prev, [e1, e2])
    assert payload["added"] == ["seatbeltNotLatched"] and payload["removed"] == []

    prev, payload = diff_events(prev, [e2])
    assert payload["added"] == [] and payload["removed"] == ["doorOpen"]

    prev, payload = diff_events(prev, [e2])
    assert payload is None  # 无变化不推

  def test_diff_replaces_state_on_noop(self):
    # diff_events 即使不产生 payload 也要返回可继续累积的 map（同一对象原样返回）
    e = event_to_dict(log.OnroadEvent.new_message(name="canError", permanent=True))
    prev, payload = diff_events({}, [e])
    prev2, payload2 = diff_events(prev, [e])
    assert payload2 is None
    assert prev2 is prev


class TestOpstateButton:
  def test_opstate_payload(self):
    ss = selfdrive_state(state="enabled", enabled=True, active=True, engageable=True)
    p = opstate_payload(ss)
    assert p == {"state": "enabled", "enabled": True, "active": True, "engageable": True}

  def test_button_payloads(self):
    cs = car.CarState.new_message(buttonEvents=[
      {"type": "setCruise", "pressed": True},
      {"type": "setCruise", "pressed": False},
    ])
    assert button_payloads(cs) == [
      {"button": "setCruise", "pressed": True},
      {"button": "setCruise", "pressed": False},
    ]

  def test_enum_name_unknown(self):
    assert enum_name(AudibleAlert, 99999) == "unknown"


class TestCarSnapshot:
  def test_car_snapshot_fields(self):
    cs = car.CarState.new_message(
      vEgo=10.0, aEgo=0.5, standstill=False, gearShifter="drive",
      cruiseState={"enabled": True, "available": True, "speed": 12.5, "speedCluster": 12.3,
                   "standstill": False, "nonAdaptive": False},
      vCruise=12.5, doorOpen=True, latEnabled=True, carrotCruise=2,
    )
    snap = car_snapshot(cs)
    assert snap["vEgo"] == 10.0 and snap["vEgoKph"] == 36.0
    assert snap["gearShifter"] == "drive"
    assert snap["cruiseState"]["speedKph"] == 45.0
    assert snap["doorOpen"] is True
    assert snap["latEnabled"] is True
    assert snap["carrotCruise"] == 2
    assert json.dumps(snap)  # 必须始终可序列化

  def test_car_snapshot_defaults_readable(self):
    # 从未 update 过的 CarState（offroad 首帧）也要能出快照，不能抛
    snap = car_snapshot(car.CarState.new_message())
    assert snap["standstill"] is False and snap["gearShifter"] == "unknown"


class TestSoundTracker:
  def test_alert_passthrough_and_dedupe(self):
    t = SoundTracker()
    sm = make_sm(ss=selfdrive_state(alertSound="stopStop", alertText1="BRAKE!", alertText2="Risk"))
    p = t.update(sm, main_toggle=False)
    assert p == {"sound": "stopStop", "file": "audio_stopstop.wav", "loop": True,
                 "alertText1": "BRAKE!", "alertText2": "Risk"}
    assert t.update(sm, main_toggle=False) is None  # 同一 alert 不重复推
    sm = make_sm(ss=selfdrive_state(alertSound="none"))
    p = t.update(sm, main_toggle=False)
    assert p["sound"] == "none" and p["file"] is None and p["loop"] is False

  def test_carrot_countdown_synthesis(self):
    t = SoundTracker()
    # 初值 0、leftSec 100：soundd 语义下 100 不合成任何音效（非 0..11），且会记录 count_down
    sm = make_sm(carrot=custom.CarrotMan.new_message(leftSec=100))
    assert t.update(sm, main_toggle=False) is None
    sm = make_sm(carrot=custom.CarrotMan.new_message(leftSec=3))
    p = t.update(sm, main_toggle=False)
    assert p["sound"] == "audio3" and p["file"] == "audio_3.wav" and p["loop"] is True
    sm = make_sm(carrot=custom.CarrotMan.new_message(leftSec=11))
    assert t.update(sm, main_toggle=False)["sound"] == "promptDistracted"
    sm = make_sm(carrot=custom.CarrotMan.new_message(leftSec=1))
    assert t.update(sm, main_toggle=False)["sound"] == "audio1"
    sm = make_sm(carrot=custom.CarrotMan.new_message(leftSec=0))
    assert t.update(sm, main_toggle=False)["sound"] == "longDisengaged"
    sm = make_sm(carrot=custom.CarrotMan.new_message(leftSec=100))
    assert t.update(sm, main_toggle=False)["sound"] == "none"

  def test_alert_sound_wins_over_countdown(self):
    t = SoundTracker()
    sm = make_sm(ss=selfdrive_state(alertSound="prompt"), carrot=custom.CarrotMan.new_message(leftSec=5))
    p = t.update(sm, main_toggle=False)
    assert p["sound"] == "prompt"

  def test_main_toggle_short_circuits_to_prompt(self):
    t = SoundTracker()
    sm = make_sm(ss=selfdrive_state(alertSound="stopStop"))
    p = t.update(sm, main_toggle=True)
    assert p["sound"] == "prompt"

  def test_sound_payload_unknown_alert(self):
    # 未收录进 sound_list 的值：file/loop 兜底，与 soundd 的 loaded_sounds 检查对应
    p = sound_payload(99999, selfdrive_state())
    assert p["sound"] == "unknown" and p["file"] is None and p["loop"] is False
