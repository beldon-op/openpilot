import json

from openpilot.mate.server import Hub


class TestHubSnapshot:
  def test_set_state_stamped(self):
    hub = Hub()
    hub.set_state("car", {"vEgo": 10.0})
    snap = hub.state_snapshot()
    assert snap["data"]["car"] == {"ts": snap["data"]["car"]["ts"], "data": {"vEgo": 10.0}}
    assert snap["data"]["car"]["ts"] > 1_700_000_000  # unix 墙钟秒
    json.dumps(snap)

  def test_emit_updates_state_for_state_kinds_only(self):
    hub = Hub()  # loop 未绑定：emit 只落快照/补发，不广播
    hub.emit("opstate", {"state": "enabled", "enabled": True, "active": True, "engageable": True})
    hub.emit("button", {"button": "setCruise", "pressed": True})
    snap = hub.state_snapshot()
    assert snap["data"]["opstate"]["data"]["state"] == "enabled"
    assert "button" not in snap["data"]  # 边沿瞬时事件不进快照

  def test_replay_follows_events_and_sound(self):
    hub = Hub()
    hub.emit("events", {"added": ["doorOpen"], "removed": [], "active": [{"name": "doorOpen"}]})
    assert len(hub.replay_snapshot()) == 1
    hub.emit("events", {"added": [], "removed": ["doorOpen"], "active": []})
    assert hub.replay_snapshot() == []  # 告警清空后不再补发
    assert hub.state_snapshot()["data"]["events"]["data"]["active"] == []  # 但快照保留空态

    hub.emit("sound", {"sound": "stopStop", "file": "audio_stopstop.wav", "loop": True,
                       "alertText1": "", "alertText2": ""})
    assert len(hub.replay_snapshot()) == 1
    hub.emit("sound", {"sound": "none", "file": None, "loop": False, "alertText1": "", "alertText2": ""})
    assert hub.replay_snapshot() == []
