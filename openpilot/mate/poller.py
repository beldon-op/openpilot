"""
mate 事件源：订阅 cereal 总线，推导四类边沿事件并交给广播器（server.py）。

  events  onroadEvents 集合变化   {"added", "removed", "active"}
  opstate selfdriveState 状态机跳转 {"state", "enabled", "active", "engageable"}
  button  方向盘按键边沿           {"button", "pressed"}
  sound   播报声音变化             {"sound", "file", "loop", "alertText1", "alertText2"}

参考实现是 cp_byd local_collector/collector.py；周期帧（frame）、topic 订阅过滤与
admin API（日志/参数管理）都不在 mate 范围内。

除边沿事件外，poller 还维护一份最新状态快照，供 server.py 的 GET /state 与
WS snapshot 请求做客户端初始化，避免客户端为了拿“当前值”而重引入周期推流：

  car       每帧 carState 刷新
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

SERVICES = ["carState", "selfdriveState", "onroadEvents", "carrotMan", "pandaStates"]

EVENT_FLAGS = ("enable", "noEntry", "warning", "userDisable", "softDisable",
               "immediateDisable", "preEnable", "permanent",
               "overrideLateral", "overrideLongitudinal")

NONE_ALERT = int(AudibleAlert.none)
MS_TO_KPH = 3.6


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
