import json

from openpilot.cereal import custom
from openpilot.mate.poller import SDI_MILESTONES_M, diff_navi, navi_state


def navi_msg(**kwargs):
  """carrotNavi 测试消息构造器：默认全 meta.present=false（APK 未激活）。"""
  return custom.CarrotNaviState.new_message(**kwargs)


def speed_group(limit=None, sdi=None, sdi2=None):
  """speed 子结构：limit=kph 或 None；sdi=(type, dist, limit)；sdi2 同。"""
  kw: dict = {"meta": {"present": True}, "roadLimitValid": limit is not None}
  if limit is not None:
    kw["roadLimitKph"] = limit
  if sdi is not None:
    t, d, lim = sdi
    kw.update({"sdiPresent": True, "sdiType": t, "sdiDistanceM": d})
    if lim:
      kw["sdiSpeedLimitKph"] = lim
  if sdi2 is not None:
    t, d, lim = sdi2
    kw.update({"secondarySdiPresent": True, "secondarySdiType": t,
               "secondarySdiDistanceM": d})
    if lim:
      kw["secondarySdiSpeedLimitKph"] = lim
  return {"speed": kw}


def signal_group(visible=True, distance=120, **lights):
  """trafficSignal 子结构：lights 传 (valid, on, remain)，如 green=(True, True, 8)。"""
  kw: dict = {"meta": {"present": True}, "visible": visible, "distanceM": distance}
  for key in ("red", "green", "left", "right", "uturn"):
    v = lights.get(key)
    if v is not None:
      valid, on, remain = v
      kw.update({f"{key}Valid": valid, f"{key}On": on, f"{key}RemainSec": remain})
  return {"trafficSignal": kw}


def guidance_group(text="前方 500m 右转", turn=3, distance=480):
  return {"guidanceCurrent": {"meta": {"present": True}, "mainText": text,
                              "turnType": turn, "distanceM": distance}}


class TestNaviState:
  def test_empty_when_not_present(self):
    # APK 未激活：所有子结构 meta.present=false → 快照为空 dict
    assert navi_state(navi_msg()) == {}

  def test_speed_limit_and_sdi(self):
    s = navi_state(navi_msg(**speed_group(limit=80, sdi=(0, 480, 80), sdi2=(22, 930, 0))))
    assert s["speedLimit"] == {"valid": True, "limitKph": 80}
    assert s["sdi"] == {"present": True, "type": 0, "distanceM": 480, "limitKph": 80,
                        "secondary": {"type": 22, "distanceM": 930}}
    json.dumps(s)

  def test_invalid_limit_omits_value(self):
    s = navi_state(navi_msg(**speed_group(limit=None)))
    assert s["speedLimit"] == {"valid": False}
    assert "limitKph" not in s["speedLimit"]

  def test_sdi_without_limit(self):
    # type=0 是有效测速值；限速 0/缺失时 limitKph 不出现
    s = navi_state(navi_msg(**speed_group(sdi=(22, 93, 0))))
    assert s["sdi"]["type"] == 22 and "limitKph" not in s["sdi"]

  def test_signal_lights_only_valid(self):
    s = navi_state(navi_msg(**signal_group(green=(True, True, 8), red=(False, False, 0))))
    assert s["signal"]["present"] is True and s["signal"]["distanceM"] == 120
    assert s["signal"]["green"] == {"on": True, "remainSec": 8}
    assert "red" not in s["signal"] and "left" not in s["signal"]

  def test_route_and_guidance_and_lane(self):
    m = navi_msg(**guidance_group(),
                 navigationStatus={"meta": {"present": True}, "mode": "guiding",
                                   "guidanceActive": True, "offRoute": False},
                 route={"meta": {"present": True}, "remainingDistanceM": 12300,
                        "remainingTimeSec": 900},
                 laneCurrent={"meta": {"present": True}, "count": 3, "currentLane": 2,
                              "distanceM": 60},
                 vehicle={"meta": {"present": True}, "roadName": "京港澳高速"})
    s = navi_state(m)
    assert s["route"] == {"offRoute": False, "guidanceActive": True, "mode": "guiding",
                          "remainDistanceM": 12300, "remainTimeSec": 900}
    assert s["guidance"] == {"mainText": "前方 500m 右转", "turnType": 3,
                             "distanceM": 480, "roadName": ""}
    assert s["lane"] == {"count": 3, "currentLane": 2, "distanceM": 60}
    assert s["roadName"] == "京港澳高速"
    json.dumps(s)


class TestDiffNavi:
  def test_first_frame_builds_state_without_push(self):
    state, payload = diff_navi(None, navi_state(navi_msg(**speed_group(limit=80))))
    assert payload is None  # 首轮只建状态，初始化走快照
    assert state["sent"]["speedLimit"] == (True, 80)

  def test_no_change_is_silent(self):
    cur = navi_state(navi_msg(**speed_group(limit=80, sdi=(0, 700, 80))))
    state, p1 = diff_navi(None, cur)
    _, p2 = diff_navi(state, cur)
    assert p2 is None

  def test_speed_limit_change(self):
    state, _ = diff_navi(None, navi_state(navi_msg(**speed_group(limit=80))))
    _, payload = diff_navi(state, navi_state(navi_msg(**speed_group(limit=100))))
    assert payload == {"speedLimit": {"valid": True, "limitKph": 100}}

  def test_speed_limit_invalidated(self):
    state, _ = diff_navi(None, navi_state(navi_msg(**speed_group(limit=80))))
    state, payload = diff_navi(state, navi_state(navi_msg(**speed_group(limit=None))))
    assert payload == {"speedLimit": {"valid": False}}
    # 消失（speed 组整体缺失）等价于失效，不重复推
    _, payload2 = diff_navi(state, navi_state(navi_msg()))
    assert payload2 is None

  def test_sdi_appear_and_disappear(self):
    state, _ = diff_navi(None, navi_state(navi_msg(**speed_group())))
    _, payload = diff_navi(state, navi_state(navi_msg(**speed_group(sdi=(0, 480, 80)))))
    assert payload["sdi"] == {"present": True, "type": 0, "distanceM": 480, "limitKph": 80}
    state, _ = diff_navi(state, navi_state(navi_msg(**speed_group(sdi=(0, 480, 80)))))
    _, payload = diff_navi(state, navi_state(navi_msg(**speed_group())))
    assert payload == {"sdi": {"present": False}}

  def test_sdi_milestones_once_each(self):
    state, _ = diff_navi(None, navi_state(navi_msg(**speed_group(sdi=(0, 800, 80)))))
    fired = []
    for dist in (700, 550, 499, 400, 199, 100):
      state, payload = diff_navi(state, navi_state(navi_msg(**speed_group(sdi=(0, dist, 80)))))
      if payload and "sdi" in payload:
        fired.append((dist, payload["sdi"]["distanceM"]))
    # 500 与 200 两个里程碑各一次（499 首次 <500，199 首次 <200）
    assert fired == [(499, 499), (199, 199)]

  def test_sdi_milestone_reset_on_new_sdi(self):
    state, _ = diff_navi(None, navi_state(navi_msg(**speed_group(sdi=(0, 400, 80)))))
    _, p = diff_navi(state, navi_state(navi_msg(**speed_group(sdi=(0, 400, 80)))))
    assert p is None  # 同一 SDI 稳态不重复
    # 换 SDI（type 变化）→ 里程碑重置，新点在 500 内再推一次
    _, p = diff_navi(state, navi_state(navi_msg(**speed_group(sdi=(2, 300, 60)))))
    assert p["sdi"]["type"] == 2 and p["sdi"]["distanceM"] == 300

  def test_signal_on_edge_and_green_crossing(self):
    state, _ = diff_navi(None, navi_state(navi_msg()))
    # 红灯亮起：on 边沿
    state, p = diff_navi(state, navi_state(navi_msg(**signal_group(red=(True, True, 30)))))
    assert p["signal"]["red"]["on"] is True
    # 剩余秒递减不推
    state, p = diff_navi(state, navi_state(navi_msg(**signal_group(red=(True, True, 20)))))
    assert p is None
    # 绿灯亮起：on 边沿（红灯同时熄灭）
    state, p = diff_navi(state, navi_state(navi_msg(**signal_group(green=(True, True, 9)))))
    assert p["signal"]["green"]["on"] is True
    # 绿灯剩余跨越 5：9 → 4 一次
    state, p = diff_navi(state, navi_state(navi_msg(**signal_group(green=(True, True, 4)))))
    assert p["signal"]["green"]["remainSec"] == 4
    # 已提醒，继续倒数不重复
    _, p = diff_navi(state, navi_state(navi_msg(**signal_group(green=(True, True, 2)))))
    assert p is None

  def test_signal_disappear_edge(self):
    state, _ = diff_navi(None, navi_state(navi_msg(**signal_group(green=(True, True, 9)))))
    state, _ = diff_navi(state, navi_state(navi_msg(**signal_group(green=(True, True, 9)))))
    # APK 断连：signal 组整体消失 → 消失边沿一次，不残留旧值
    _, payload = diff_navi(state, navi_state(navi_msg()))
    assert payload == {"signal": {"present": False}}

  def test_route_off_route_edge(self):
    kw = {"navigationStatus": {"meta": {"present": True}, "mode": "guiding",
                               "guidanceActive": True, "offRoute": False}}
    state, _ = diff_navi(None, navi_state(navi_msg(**kw)))
    _, payload = diff_navi(state, navi_state(navi_msg(
      **{**kw, "navigationStatus": {"meta": {"present": True}, "mode": "off_route",
                                    "guidanceActive": True, "offRoute": True}})))
    assert payload["route"]["offRoute"] is True
    # 导航结束（组消失）→ 回落为非偏航
    _, payload = diff_navi(state, navi_state(navi_msg()))
    assert payload["route"] == {"offRoute": False, "guidanceActive": False}

  def test_guidance_text_change_and_absence_reset(self):
    state, _ = diff_navi(None, navi_state(navi_msg(**guidance_group())))
    # 文本变化即推（含 distanceM）
    _, payload = diff_navi(state, navi_state(navi_msg(**guidance_group(text="前方 200m 左转", turn=2))))
    assert payload["guidance"] == {"mainText": "前方 200m 左转", "turnType": 2,
                                   "distanceM": 480, "roadName": ""}
    # 消失仅重置：不推空值
    _, payload = diff_navi(state, navi_state(navi_msg()))
    assert payload is None or "guidance" not in payload
    # 再出现 → 重新当边沿推
    state, _ = diff_navi(state, navi_state(navi_msg()))
    _, payload = diff_navi(state, navi_state(navi_msg(**guidance_group())))
    assert payload["guidance"]["mainText"] == "前方 500m 右转"

  def test_multi_group_same_frame(self):
    state, _ = diff_navi(None, navi_state(navi_msg(**speed_group(limit=60))))
    # 限速变化 + SDI 出现同帧：两组各推一次
    _, payload = diff_navi(state, navi_state(navi_msg(**speed_group(limit=80, sdi=(0, 490, 80)))))
    assert set(payload) == {"speedLimit", "sdi"}

  def test_milestone_thresholds_are_design_values(self):
    assert SDI_MILESTONES_M == (500, 200)
