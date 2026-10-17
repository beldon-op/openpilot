"""
mate 事件源：订阅 cereal 总线，推导五类边沿事件并交给广播器（server.py）。

  events  onroadEvents 集合变化   {"added", "removed", "active"}
  opstate selfdriveState 状态机跳转 {"state", "enabled", "active", "engageable"}
  button  方向盘按键边沿           {"button", "pressed"}
  sound   播报声音变化             {"sound", "file", "loop", "alertText1", "alertText2"}
  navi    导航/交通信息边沿        {"speedLimit", "sdi", "signal", "route", "guidance"}
          （carrotNavi 2Hz 帧间差分，payload 只含变化分组，限速/SDI/信号灯/偏航/转向）

参考实现是 cp_byd local_collector/collector.py；周期帧（frame）、topic 订阅过滤与
admin API（日志/参数管理）都不在 mate 范围内。

除边沿事件外，poller 还维护一份最新状态快照，供 server.py 的 GET /state 与
WS snapshot 请求做客户端初始化，避免客户端为了拿“当前值”而重引入周期推流：

  car       每帧 carState 刷新
  navi      每帧 carrotNavi 刷新（carrot_navi always_run，offroad 也更新）
  events/opstate/sound  随边沿更新
  meta      每轮心跳（frame 计数 + onroad 标志）——快照整体变旧 = poller 挂了
  services  每轮 SubMaster 健康度——topic seen=false = 数据源进程没跑（offroad 常态）
  panda     随 pandaStates 更新——pandad always_run，offroad 下唯一的活数据源
"""
import time
from functools import cache

import openpilot.cereal.messaging as messaging
from openpilot.cereal import car, log
from openpilot.common.params import Params
from openpilot.common.swaglog import cloudlog
from openpilot.selfdrive.car.openpilot_toggle import CruiseMainOpenpilotToggle
from openpilot.selfdrive.ui.soundd import (
  AudibleAlert,
  ButtonType,
  check_selfdrive_timeout_alert,
  sound_list,
)

SERVICES = ["carState", "selfdriveState", "onroadEvents", "carrotMan", "carrotNavi", "pandaStates"]

EVENT_FLAGS = ("enable", "noEntry", "warning", "userDisable", "softDisable",
               "immediateDisable", "preEnable", "permanent",
               "overrideLateral", "overrideLongitudinal")

NONE_ALERT = int(AudibleAlert.none)
MS_TO_KPH = 3.6

# navi（carrotNavi 2Hz）边沿差分参数：SDI 出现期间临近里程碑、绿灯剩余跨过该秒数
SDI_MILESTONES_M = (500, 200)
GREEN_REMAIN_ALERT_S = 5
SIGNAL_LIGHTS = ("red", "green", "left", "right", "uturn")


@cache
def _enum_map(enum_type) -> dict[int, str]:
  return {v: k for k, v in enum_type.schema.enumerants.items()}


def enum_name(enum_type, value) -> str:
  try:
    return _enum_map(enum_type).get(int(value), "unknown")
  except Exception:
    return "unknown"


# ---------------- 纯函数：事件推导 ----------------

def event_to_dict(e) -> dict:
  out = {"name": enum_name(log.OnroadEvent.EventName, e.name.raw)}
  for f in EVENT_FLAGS:
    out[f] = bool(getattr(e, f))
  return out


def diff_events(prev: dict[str, dict], current: list[dict]) -> tuple[dict[str, dict], dict | None]:
  """返回 (新状态, payload)；集合无变化时 payload 为 None（去抖，不每帧推）。"""
  cur_map = {e["name"]: e for e in current}
  added = sorted(cur_map.keys() - prev.keys())
  removed = sorted(prev.keys() - cur_map.keys())
  if not added and not removed:
    return prev, None
  return cur_map, {"added": added, "removed": removed, "active": list(cur_map.values())}


def opstate_payload(ss) -> dict:
  return {
    "state": enum_name(log.SelfdriveState.OpenpilotState, ss.state.raw),
    "enabled": ss.enabled,
    "active": ss.active,
    "engageable": ss.engageable,
  }


def button_payloads(cs) -> list[dict]:
  # CarState.buttonEvents 本身就是变化沿列表（按下/松开各一条），只在 updated 帧消费
  return [{"button": enum_name(car.CarState.ButtonEvent.Type, b.type.raw), "pressed": b.pressed}
          for b in cs.buttonEvents]


def car_snapshot(cs) -> dict:
  """GET /state 的 car 段：当前车辆状态快照（非周期推流），字段对照本仓 car.capnp。"""
  cruise = cs.cruiseState
  return {
    "vEgo": round(cs.vEgo, 3),
    "vEgoKph": round(cs.vEgo * MS_TO_KPH, 1),
    "vEgoCluster": round(cs.vEgoCluster, 3),
    "aEgo": round(cs.aEgo, 3),
    "standstill": cs.standstill,
    "gearShifter": enum_name(car.CarState.GearShifter, cs.gearShifter.raw),
    "gas": round(cs.gas, 3),
    "gasPressed": cs.gasPressed,
    "brake": round(cs.brake, 3),
    "brakePressed": cs.brakePressed,
    "steeringAngleDeg": round(cs.steeringAngleDeg, 2),
    "steeringPressed": cs.steeringPressed,
    "cruiseState": {
      "enabled": cruise.enabled,
      "available": cruise.available,
      "speed": round(cruise.speed, 3),
      "speedKph": round(cruise.speed * MS_TO_KPH, 1),
      "speedCluster": round(cruise.speedCluster, 3),
      "standstill": cruise.standstill,
      "nonAdaptive": cruise.nonAdaptive,
    },
    "vCruise": round(cs.vCruise, 3),
    "vCruiseKph": round(cs.vCruise * MS_TO_KPH, 1),
    "leftBlinker": cs.leftBlinker,
    "rightBlinker": cs.rightBlinker,
    "leftBlindspot": cs.leftBlindspot,
    "rightBlindspot": cs.rightBlindspot,
    "doorOpen": cs.doorOpen,
    "seatbeltUnlatched": cs.seatbeltUnlatched,
    "parkingBrake": cs.parkingBrake,
    "brakeHoldActive": cs.brakeHoldActive,
    "latEnabled": cs.latEnabled,
    "speedLimit": round(cs.speedLimit, 3),
    "speedLimitKph": round(cs.speedLimit * MS_TO_KPH, 1),
    "engineRpm": round(cs.engineRpm, 1),
    "accFaulted": cs.accFaulted,
    "espDisabled": cs.espDisabled,
    "canValid": cs.canValid,
    "canTimeout": cs.canTimeout,
    "carrotCruise": cs.carrotCruise,
  }


# ---------------- navi：carrotNavi 导航/交通信息 ----------------

def navi_state(navi) -> dict:
  """carrotNavi → navi 快照段当前值（JSON-able）。分组仅在对应子结构 meta.present
  为真时出现；无效字段（valid/present 为假）不出现或只留标志位；polyline 不进快照。"""
  out: dict = {}
  sp = navi.speed
  if sp.meta.present:
    out["speedLimit"] = ({"valid": True, "limitKph": int(sp.roadLimitKph)} if sp.roadLimitValid
                         else {"valid": False})
    sdi: dict = {"present": bool(sp.sdiPresent)}
    if sp.sdiPresent:
      sdi.update({"type": int(sp.sdiType), "distanceM": int(sp.sdiDistanceM)})
      if sp.sdiSpeedLimitKph > 0:
        sdi["limitKph"] = int(sp.sdiSpeedLimitKph)
      if sp.secondarySdiPresent:
        secondary: dict = {"type": int(sp.secondarySdiType), "distanceM": int(sp.secondarySdiDistanceM)}
        if sp.secondarySdiSpeedLimitKph > 0:
          secondary["limitKph"] = int(sp.secondarySdiSpeedLimitKph)
        sdi["secondary"] = secondary
    out["sdi"] = sdi
  ts = navi.trafficSignal
  if ts.meta.present:
    signal: dict = {"present": True, "visible": bool(ts.visible)}
    if ts.visible:
      signal["distanceM"] = int(ts.distanceM)
    for key, valid, on, remain in (
        ("red", ts.redValid, ts.redOn, ts.redRemainSec),
        ("green", ts.greenValid, ts.greenOn, ts.greenRemainSec),
        ("left", ts.leftValid, ts.leftOn, ts.leftRemainSec),
        ("right", ts.rightValid, ts.rightOn, ts.rightRemainSec),
        ("uturn", ts.uturnValid, ts.uturnOn, ts.uturnRemainSec)):
      if valid:
        signal[key] = {"on": bool(on), "remainSec": int(remain)}
    out["signal"] = signal
  ns = navi.navigationStatus
  if ns.meta.present:
    route: dict = {"offRoute": bool(ns.offRoute), "guidanceActive": bool(ns.guidanceActive),
                   "mode": str(ns.mode)}
    if navi.route.meta.present:
      route["remainDistanceM"] = int(navi.route.remainingDistanceM)
      route["remainTimeSec"] = int(navi.route.remainingTimeSec)
    out["route"] = route
  gc = navi.guidanceCurrent
  if gc.meta.present:
    out["guidance"] = {"mainText": str(gc.mainText), "turnType": int(gc.turnType),
                       "distanceM": int(gc.distanceM), "roadName": str(gc.roadName)}
  lc = navi.laneCurrent
  if lc.meta.present:
    out["lane"] = {"count": int(lc.count), "currentLane": int(lc.currentLane),
                   "distanceM": int(lc.distanceM)}
  veh = navi.vehicle
  if veh.meta.present:
    out["roadName"] = str(veh.roadName)
  return out


def _cmp_speed_limit(g: dict):
  return (g.get("valid"), g.get("limitKph"))


def _cmp_sdi(g: dict):
  sec = g.get("secondary") or {}
  return (g.get("present"), g.get("type"), g.get("limitKph"),
          bool(sec), sec.get("type"), sec.get("limitKph"))


def _cmp_signal(g: dict):
  # 只比 present/visible/各灯 on 边沿；remainSec 每秒递减、distanceM 连续变化，不参与比较
  return (g.get("present"), g.get("visible"),
          *(g.get(k, {}).get("on") for k in SIGNAL_LIGHTS))


def _cmp_route(g: dict):
  return (g.get("offRoute"), g.get("guidanceActive"))


def _cmp_guidance(g: dict):
  return (g.get("turnType"), g.get("mainText"))


NAVI_COMPARE = {
  "speedLimit": _cmp_speed_limit,
  "sdi": _cmp_sdi,
  "signal": _cmp_signal,
  "route": _cmp_route,
  "guidance": _cmp_guidance,
}

# 分组消失（meta.present 为假）时的等价形态；guidance 消失仅重置，不推空值
NAVI_GROUP_DEFAULTS = {
  "speedLimit": {"valid": False},
  "sdi": {"present": False},
  "signal": {"present": False},
  "route": {"offRoute": False, "guidanceActive": False},
}


def diff_navi(prev: dict | None, cur: dict) -> tuple[dict, dict | None]:
  """navi 分组边沿差分（纯函数）。prev 为上轮返回的 state（首轮 None），cur 为
  navi_state() 输出。返回 (新 state, payload)：payload 只含发生变化的分组，无变化
  为 None；首轮只建状态不推送（客户端初始化走 /state 快照）。"""
  first = prev is None
  state: dict = {"sent": {}, "milestones": [], "sdiKey": None,
                 "greenOn": False, "greenRemain": None, "greenFired": False}
  payload: dict = {}

  for group, compare in NAVI_COMPARE.items():
    g = cur.get(group)
    if g is None:
      g = NAVI_GROUP_DEFAULTS.get(group)
      if g is None:  # guidance 消失：仅重置，下次出现重新当边沿推
        state["sent"][group] = None
        continue
    form = compare(g)
    state["sent"][group] = form
    if not first and prev["sent"].get(group) != form:
      payload[group] = g

  # SDI 临近里程碑：同一 SDI（present/type/limit 身份）存续期内 500/200m 各推一次
  sdi = cur.get("sdi") or NAVI_GROUP_DEFAULTS["sdi"]
  state["sdiKey"] = (sdi.get("present"), sdi.get("type"), sdi.get("limitKph"))
  if sdi.get("present"):
    state["milestones"] = [] if first or state["sdiKey"] != prev.get("sdiKey") \
      else list(prev.get("milestones", []))
    for m in SDI_MILESTONES_M:
      if m not in state["milestones"] and sdi.get("distanceM", 1 << 30) < m:
        state["milestones"].append(m)
        if not first:
          payload.setdefault("sdi", sdi)

  # 绿灯剩余跨越 GREEN_REMAIN_ALERT_S（由大到小）推一次；灯灭即重置
  green = (cur.get("signal") or {}).get("green") or {}
  state["greenOn"] = bool(green.get("on"))
  state["greenRemain"] = green.get("remainSec") if state["greenOn"] else None
  if state["greenOn"] and not first and prev.get("greenOn"):
    fired = prev.get("greenFired", False)
    prev_remain = prev.get("greenRemain")
    if not fired and prev_remain is not None and \
        prev_remain > GREEN_REMAIN_ALERT_S >= state["greenRemain"]:
      fired = True
      payload.setdefault("signal", cur["signal"])
    state["greenFired"] = fired

  return state, (payload or None)


def services_status(sm) -> dict:
  """GET /state 的 services 段：SubMaster 视角的各 topic 健康度。
  seen=启动以来是否收到过（frequency=0 的 on-demand topic 恒判 alive，
  全靠 seen 区分“没跑”与“跑了但不新鲜”）；ageS 用本进程单调钟差值，
  logMonoTime 同为 CLOCK_MONOTONIC，仅同机同次开机内可比。"""
  now_ns = time.monotonic_ns()
  out = {}
  for s in sm.services:
    seen = bool(sm.seen[s])
    out[s] = {"seen": seen,
              "alive": bool(sm.alive[s]),
              "valid": bool(sm.valid[s]),
              "ageS": round(max(0.0, (now_ns - sm.logMonoTime[s]) / 1e9), 2) if seen else None}
  return out


def panda_state_to_dict(p) -> dict:
  can_states = [{"bus": i, "busOff": c.busOff, "busOffCnt": c.busOffCnt,
                 "errorWarning": c.errorWarning, "errorPassive": c.errorPassive,
                 "canSpeed": c.canSpeed, "totalRxCnt": c.totalRxCnt, "totalTxCnt": c.totalTxCnt}
                for i, c in enumerate((p.canState0, p.canState1, p.canState2))]
  return {
    "type": enum_name(log.PandaState.PandaType, p.pandaType.raw),
    "heartbeatLost": p.heartbeatLost,
    "ignitionLine": p.ignitionLine,
    "controlsAllowed": p.controlsAllowed,
    "safetyModel": enum_name(car.CarParams.SafetyModel, p.safetyModel.raw),
    "safetyParam": p.safetyParam,
    "faultStatus": enum_name(log.PandaState.FaultStatus, p.faultStatus.raw),
    "faults": [enum_name(log.PandaState.FaultType, f.raw) for f in p.faults],
    "harnessStatus": enum_name(log.PandaState.HarnessStatus, p.harnessStatus.raw),
    "uptime": p.uptime,
    "canStates": can_states,
  }


def panda_payload(panda_states) -> dict:
  """GET /state 的 panda 段：台架/车上 panda 是否在线、CAN 收发是否活着，
  offroad 也持续更新（pandad always_run）。"""
  return {"count": len(panda_states), "pandas": [panda_state_to_dict(p) for p in panda_states]}


def sound_payload(alert: int, ss) -> dict:
  entry = sound_list.get(alert)
  return {
    "sound": enum_name(AudibleAlert, alert),
    "file": entry[0] if entry else None,
    "loop": entry[1] is None if entry else False,
    "alertText1": ss.alertText1,
    "alertText2": ss.alertText2,
  }


class SoundTracker:
  """复刻 soundd 的 alert 推导链（soundd.py update_alert / update_carrot_alert /
  get_audible_alert），但没有音频时钟：转入 none 立即上报，不等有限长音效播完，
  客户端只会提前停止播放。"""

  def __init__(self):
    self.alert = NONE_ALERT
    self.carrot_count_down = 0  # 与 soundd 一致的初值，静止期不误发 longDisengaged
    self.selfdrive_timeout_alert = False

  def _synthesize_carrot(self, new_alert: int, left_sec: int) -> int:
    # soundd 只在 alertSound==none 时用 carrot 倒计时覆盖
    if new_alert != NONE_ALERT:
      return new_alert
    if self.carrot_count_down != left_sec:
      self.carrot_count_down = left_sec
      if left_sec == 0:
        return int(AudibleAlert.longDisengaged)
      if 0 < left_sec <= 10:
        return int(getattr(AudibleAlert, f"audio{left_sec}"))
      if left_sec == 11:
        return int(AudibleAlert.promptDistracted)
    return new_alert

  def update(self, sm, main_toggle: bool) -> dict | None:
    """每轮 SubMaster 更新后调用；alert 变化时返回 sound payload，否则 None。"""
    if main_toggle:
      new_alert = int(AudibleAlert.prompt)
    elif sm.updated["selfdriveState"] or sm.updated["carrotMan"]:
      new_alert = self._synthesize_carrot(int(sm["selfdriveState"].alertSound.raw),
                                          int(sm["carrotMan"].leftSec))
    elif check_selfdrive_timeout_alert(sm):
      self.selfdrive_timeout_alert = True
      new_alert = int(AudibleAlert.warningImmediate)
    elif self.selfdrive_timeout_alert:
      self.selfdrive_timeout_alert = False
      new_alert = NONE_ALERT
    else:
      return None

    if new_alert != NONE_ALERT and new_alert not in sound_list:
      cloudlog.error(f"mate received unsupported alert {new_alert}")
      new_alert = NONE_ALERT
    if new_alert == self.alert:
      return None
    self.alert = new_alert
    return sound_payload(new_alert, sm["selfdriveState"])


# ---------------- 采集线程 ----------------

class MatePoller:
  def __init__(self, emit, set_state=None):
    self.emit = emit  # emit(kind: str, data: dict)，server.Hub 提供，线程安全
    self.set_state = set_state  # set_state(kind, data)，维护 /state 快照（car 段），可选

  def run(self):
    sm = messaging.SubMaster(SERVICES)
    params = Params()
    events_map: dict[str, dict] = {}
    last_opstate: dict | None = None
    navi: dict | None = None  # diff_navi 状态（首轮 None）
    tracker = SoundTracker()
    cruise_main_toggle = CruiseMainOpenpilotToggle(ButtonType.mainCruise)

    cloudlog.info("mate poller started")
    while True:
      sm.update(100)  # 与参考 collector 一致：有消息即返回，空闲最多 100ms 一轮

      # 每轮心跳 + 总线健康度：/state 的 data 为空时，客户端靠这两段自查——
      # meta ts 变旧=poller 线程挂了；services 全 seen=false=offroad 没数据源。
      if self.set_state is not None:
        self.set_state("meta", {"frame": sm.frame, "onroad": params.get_bool("IsOnroad")})
        self.set_state("services", services_status(sm))
        if sm.updated["pandaStates"]:
          self.set_state("panda", panda_payload(sm["pandaStates"]))

      if sm.updated["onroadEvents"]:
        events_map, payload = diff_events(events_map, [event_to_dict(e) for e in sm["onroadEvents"]])
        if payload:
          self.emit("events", payload)

      if sm.updated["selfdriveState"]:
        payload = opstate_payload(sm["selfdriveState"])
        if payload != last_opstate:
          last_opstate = payload
          self.emit("opstate", payload)

      # navi：diff_navi 返回的 payload 是「变化分组」而非全量值，所以快照不能走
      # emit→set_state（那会拿 payload 覆盖状态段），由这里显式喂全量 cur。
      if sm.updated["carrotNavi"]:
        cur = navi_state(sm["carrotNavi"])
        navi, payload = diff_navi(navi, cur)
        if payload:
          self.emit("navi", payload)
        if self.set_state is not None:
          self.set_state("navi", cur)

      if sm.updated["carState"]:
        cs = sm["carState"]
        if self.set_state is not None:
          self.set_state("car", car_snapshot(cs))
        for payload in button_payloads(cs):
          self.emit("button", payload)

      # 声音：与 soundd 主循环一致——每轮先过 main toggle 短路，再做推导。
      # update() 必须在拿到新 carState 后立刻调用，长按计时依赖逐帧喂同一批事件。
      toggled = cruise_main_toggle.update(sm["carState"].buttonEvents, sm["selfdriveState"].enabled)
      payload = tracker.update(sm, toggled)
      if payload:
        self.emit("sound", payload)
