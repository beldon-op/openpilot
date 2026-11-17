"""
mate 事件源：订阅 cereal 总线，推导边沿事件与周期状态帧并交给广播器（server.py）。

  events  onroadEvents 集合变化   {"added", "removed", "active"}
  opstate selfdriveState 状态机跳转 {"state", "enabled", "active", "engageable"}
  button  方向盘按键边沿           {"button", "pressed"}
  sound   播报声音变化             {"sound", "file", "loop", "alertText1", "alertText2"}
  frame   2Hz 周期状态帧           {"car"?, "radar"?, "carrot"?}：驾驶屏连续值，
          段与快照同源；窗口内 carState/radarState/carrotMan 均无更新（offroad）则不推；
          carrot 段为协议 v4 新增（carrotMan 遥测），仅进 frame 不进快照

v3 起不再有 navi kind：其输入 carrotNavi 即 carrotmate（客户端）经 7706/7714 上行
推送的导航数据，回推属回声——服务定位原则：mate 只推 op 独有数据，客户端自产的
数据不回推。

除边沿事件与 frame 外，poller 还维护一份最新状态快照，供 server.py 的 GET /state 与
WS snapshot 请求做客户端初始化，避免客户端为了拿"当前值"而重引入周期推流：

  car/radar 每帧刷新（与 frame 的同名段共用同一构建函数）
  events/opstate/sound  随边沿更新
  meta      每轮心跳（frame 计数 + onroad 标志）——快照整体变旧 = poller 挂了
  services  每轮 SubMaster 健康度——topic seen=false = 数据源进程没跑（offroad 常态）
  panda     随 pandaStates 更新——pandad always_run，offroad 下唯一的活数据源

v5 起 /state 与 WS snapshot 扩展 14 个纯快照段（只进快照，不广播、不补发，
机制与 meta/services/panda 相同，段未收到过数据不出现）。按本 fork topic 实际
存在性收敛（见 design D2），SP 独有段（carStateSP/carControlSP/carParamsSP/
longitudinalPlanSP/modelDataV2SP/backupManagerSP/liveMapDataSP/extrinsicsCalibration/
lateralTorqueParameters/vehicleParameters/liveLocationKalman）不订阅，对应段结构
保留 `sp: null` 字段位以稳定客户端解析。

参考实现是 cp_byd local_collector/collector.py；其 10Hz 周期帧不移植（frame 为
2Hz 状态语义），admin API 也不在 mate 范围内。
"""
import time
from functools import cache
from typing import Optional

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

SERVICES = [
  # 事件/状态推导骨干（v1 起即为 frame/边沿事件数据源）
  "carState", "selfdriveState", "onroadEvents", "carrotMan", "radarState", "pandaStates",
  # v5 快照段：设备/系统状态
  "deviceState", "peripheralState", "managerState", "procLog", "temperatureSensor",
  # v5 快照段：车辆/控制
  "carParams", "controlsState", "carControl",
  "longitudinalPlan", "lateralManeuverPlan",
  # v5 快照段：定位/感知/模型
  "gpsLocationExternal", "modelV2", "drivingModelData", "cameraOdometry",
]

EVENT_FLAGS = ("enable", "noEntry", "warning", "userDisable", "softDisable",
               "immediateDisable", "preEnable", "permanent",
               "overrideLateral", "overrideLongitudinal")

NONE_ALERT = int(AudibleAlert.none)
MS_TO_KPH = 3.6

# frame 周期状态帧：驾驶屏连续值刷新率；窗口内无新数据即整帧跳过（offroad 自然停推）
FRAME_HZ = 2
FRAME_PERIOD_S = 1.0 / FRAME_HZ


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
  """GET /state 的 car 段：当前车辆状态快照（非周期推流），字段对照本仓 car.capnp。
  与 frame 的 car 段同源同函数（一个拉一个推）。"""
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
    "fuelGauge": round(cs.fuelGauge, 3),
    "charging": cs.charging,
  }


def radar_snapshot(rs) -> dict:
  """radarState → radar 快照段 / frame radar 段（JSON-able）。字段名与 carrotmate
  Lead 模型一致（其 trackId 对应 cereal radarTrackId）；radarErrors 为非空错误名列表
  （Car.RadarData.Error 是四个 Bool 的 struct）。"""
  def lead(l) -> dict:
    return {
      "status": bool(l.status),
      "dRel": round(l.dRel, 2),
      "yRel": round(l.yRel, 2),
      "vRel": round(l.vRel, 2),
      "aRel": round(l.aRel, 2),
      "vLead": round(l.vLead, 2),
      "dPath": round(l.dPath, 2),
      "fcw": bool(l.fcw),
      "trackId": int(l.radarTrackId),
    }

  err = rs.radarErrors
  return {
    "leadOne": lead(rs.leadOne),
    "leadTwo": lead(rs.leadTwo),
    "leadLeft": lead(rs.leadLeft),
    "leadRight": lead(rs.leadRight),
    "leadsCenterCount": len(rs.leadsCenter),
    "radarErrors": [name for name, on in (
      ("canError", err.canError), ("radarFault", err.radarFault),
      ("wrongConfig", err.wrongConfig),
      ("radarUnavailableTemporary", err.radarUnavailableTemporary)) if on],
  }


# ---------------- v5 纯快照段：设备/系统状态 ----------------


def _round_list(values, nd: int = 3) -> list[float]:
  return [round(float(v), nd) for v in values]


def device_snapshot(ds) -> dict:
  """deviceState → device 段（2Hz）：网络/存储/内存/温度/热状态/风扇/功耗。"""
  return {
    "networkType": enum_name(log.DeviceState.NetworkType, ds.networkType.raw),
    "networkStrength": enum_name(log.DeviceState.NetworkStrength, ds.networkStrength.raw),
    "networkMetered": ds.networkMetered,
    "started": ds.started,
    "freeSpacePercent": round(float(ds.freeSpacePercent), 1),
    "memoryUsagePercent": int(ds.memoryUsagePercent),
    "gpuUsagePercent": int(ds.gpuUsagePercent),
    "cpuUsagePercent": _round_list(ds.cpuUsagePercent, 1),
    "powerDrawW": round(float(ds.powerDrawW), 2),
    "somPowerDrawW": round(float(ds.somPowerDrawW), 2),
    "cpuTempC": _round_list(ds.cpuTempC, 1),
    "gpuTempC": _round_list(ds.gpuTempC, 1),
    "memoryTempC": round(float(ds.memoryTempC), 1),
    "maxTempC": round(float(ds.maxTempC), 1),
    "thermalZones": [{"name": z.name, "tempC": round(float(z.temp), 1)} for z in ds.thermalZones],
    "thermalStatus": enum_name(log.DeviceState.ThermalStatus, ds.thermalStatus.raw),
    "fanSpeedPercentDesired": int(ds.fanSpeedPercentDesired),
  }


def peripheral_snapshot(ps) -> dict:
  """peripheralState → peripheral 段（2Hz）：panda 电压/电流/风扇转速。"""
  return {
    "pandaType": enum_name(log.PandaState.PandaType, ps.pandaType.raw),
    "voltage": int(ps.voltage),
    "current": int(ps.current),
    "fanSpeedRpm": int(ps.fanSpeedRpm),
  }


def manager_snapshot(ms) -> dict:
  """managerState → manager 段（2Hz）：各进程 running/shouldBeRunning/exitCode。"""
  return {
    "processes": [
      {"name": p.name, "pid": int(p.pid), "running": p.running,
       "shouldBeRunning": p.shouldBeRunning, "exitCode": int(p.exitCode)}
      for p in ms.processes
    ],
  }


def proc_snapshot(pl) -> dict:
  """procLog → proc 段（0.5Hz）：内存总量 + 逐进程 CPU/内存（运维看板）。"""
  mem = pl.mem
  return {
    "mem": {
      "totalMb": round(mem.total / 1048576, 1),
      "freeMb": round(mem.free / 1048576, 1),
      "availableMb": round(mem.available / 1048576, 1),
    },
    "procs": [
      {"name": p.name, "pid": int(p.pid),
       "cpuUser": round(float(p.cpuUser), 2), "cpuSystem": round(float(p.cpuSystem), 2),
       "threads": int(p.numThreads),
       "rssMb": round(p.memRss / 1048576, 1), "pssMb": round(p.memPss / 1048576, 1)}
      for p in pl.procs
    ],
  }


def temp_snapshot(e) -> dict:
  """temperatureSensor → temp 段（2Hz）。匿名 union 未选中温度分支时 pycapnp 抛异常，
  兜底为 None（客户端判读为无效）。"""
  try:
    temperature = round(float(e.temperature), 2)
  except Exception:
    temperature = None
  return {
    "timestamp": int(e.timestamp),
    "source": enum_name(log.SensorEventData.SensorSource, e.source.raw),
    "temperatureC": temperature,
  }


# ---------------- v5 纯快照段：车辆/控制 ----------------


def car_params_snapshot(cp) -> dict:
  """carParams → carParams 段（0.02Hz 静态车辆身份/动力学参数）。本 fork 无
  carParamsSP topic，sp 子对象填 null（design D2 字段位保留策略）。"""
  return {
    "brand": str(cp.brand or ""),
    "carFingerprint": str(cp.carFingerprint or ""),
    "fuzzyFingerprint": cp.fuzzyFingerprint,
    "carVin": str(cp.carVin or ""),
    "passive": cp.passive,
    "pcmCruise": cp.pcmCruise,
    "flags": int(cp.flags),
    "minEnableSpeed": round(float(cp.minEnableSpeed), 2),
    "minSteerSpeed": round(float(cp.minSteerSpeed), 2),
    "maxLateralAccel": round(float(cp.maxLateralAccel), 2),
    "wheelbase": round(float(cp.wheelbase), 3),
    "centerToFront": round(float(cp.centerToFront), 3),
    "steerRatio": round(float(cp.steerRatio), 3),
    "steerControlType": enum_name(car.CarParams.SteerControlType, cp.steerControlType.raw),
    "radarUnavailable": cp.radarUnavailable,
    "steerActuatorDelay": round(float(cp.steerActuatorDelay), 3),
    "longitudinalActuatorDelay": round(float(cp.longitudinalActuatorDelay), 3),
    "openpilotLongitudinalControl": cp.openpilotLongitudinalControl,
    "transmissionType": enum_name(car.CarParams.TransmissionType, cp.transmissionType.raw),
    "networkLocation": enum_name(car.CarParams.NetworkLocation, cp.networkLocation.raw),
    "sp": None,
  }


def controls_snapshot(cs) -> dict:
  """controlsState → controls 段（100Hz 快照取最新）。本仓 ControlsState 的 state/alert
  字段在 deprecated group 下（carrot 树遗留布局），此处按既有路径读取。"""
  dep = cs.deprecated
  return {
    "state": enum_name(log.SelfdriveState.OpenpilotState, dep.state.raw),
    "enabled": dep.enabled,
    "active": dep.active,
    "alertText1": str(dep.alertText1 or ""),
    "alertText2": str(dep.alertText2 or ""),
    "alertType": str(dep.alertType or ""),
    "alertStatus": enum_name(log.SelfdriveState.AlertStatus, dep.alertStatus.raw),
    "alertSize": enum_name(log.SelfdriveState.AlertSize, dep.alertSize.raw),
    "alertSound": enum_name(car.CarControl.HUDControl.AudibleAlert, dep.alertSound2.raw),
    "longControlState": enum_name(car.CarControl.Actuators.LongControlState, cs.longControlState.raw),
    "curvature": round(float(cs.curvature), 5),
    "desiredCurvature": round(float(cs.desiredCurvature), 5),
    "forceDecel": cs.forceDecel,
    "decelForTurn": dep.decelForTurn,
    "decelForModel": dep.decelForModel,
  }


def control_snapshot(cc) -> dict:
  """carControl → control 段（100Hz 快照取最新）：op 实际下发执行器指令与 HUD 提示。
  本 fork 无 carControlSP topic，mads 子对象填 null。"""
  a = cc.actuators
  hud = cc.hudControl
  cruise = cc.cruiseControl
  return {
    "enabled": cc.enabled,
    "latActive": cc.latActive,
    "longActive": cc.longActive,
    "leftBlinker": cc.leftBlinker,
    "rightBlinker": cc.rightBlinker,
    "currentCurvature": round(float(cc.currentCurvature), 5),
    "actuators": {
      "torque": round(float(a.torque), 4),
      "steeringAngleDeg": round(float(a.steeringAngleDeg), 2),
      "curvature": round(float(a.curvature), 5),
      "accel": round(float(a.accel), 3),
      "jerk": round(float(a.jerk), 3),
      "gas": round(float(a.gas), 3),
      "brake": round(float(a.brake), 3),
      "torqueOutputCan": round(float(a.torqueOutputCan), 2),
      "speed": round(float(a.speed), 2),
      "longControlState": enum_name(car.CarControl.Actuators.LongControlState, a.longControlState.raw),
    },
    "hud": {
      "setSpeed": round(float(hud.setSpeed), 2),
      "speedVisible": hud.speedVisible,
      "lanesVisible": hud.lanesVisible,
      "leadVisible": hud.leadVisible,
      "leftLaneVisible": hud.leftLaneVisible,
      "rightLaneVisible": hud.rightLaneVisible,
      "leftLaneDepart": hud.leftLaneDepart,
      "rightLaneDepart": hud.rightLaneDepart,
      "visualAlert": enum_name(car.CarControl.HUDControl.VisualAlert, hud.visualAlert.raw),
      "leadDistanceBars": int(hud.leadDistanceBars),
    },
    "cruise": {"cancel": cruise.cancel, "resume": cruise.resume, "override": cruise.override},
    "mads": None,
  }


def plan_snapshot(lp, lmp) -> dict:
  """longitudinalPlan + lateralManeuverPlan → plan 段（20Hz）。accels/speeds/jerks
  为未来 2.5s 规划序列；desiredCurvature 取自 lateralManeuverPlan。本 fork 无
  longitudinalPlanSP，sp 子对象填 null。"""
  return {
    "hasLead": lp.hasLead,
    "fcw": lp.fcw,
    "source": enum_name(log.LongitudinalPlan.LongitudinalPlanSource, lp.longitudinalPlanSource.raw),
    "aTarget": round(float(lp.aTarget), 3),
    "shouldStop": lp.shouldStop,
    "allowThrottle": lp.allowThrottle,
    "allowBrake": lp.allowBrake,
    "accels": _round_list(lp.accels),
    "speeds": _round_list(lp.speeds),
    "jerks": _round_list(lp.jerks),
    "processingDelay": round(float(lp.processingDelay), 3),
    "solverExecutionTime": round(float(lp.solverExecutionTime), 3),
    "desiredCurvature": round(float(lmp.desiredCurvature), 5),
    "sp": None,
  }


# ---------------- v5 纯快照段：定位/感知/模型 ----------------


def gps_snapshot(g) -> dict:
  """gpsLocationExternal → gps 段（10Hz 原始 GNSS 兜底）。speed m/s、bearingDeg 度。"""
  return {
    "latitude": round(float(g.latitude), 7),
    "longitude": round(float(g.longitude), 7),
    "altitude": round(float(g.altitude), 2),
    "speed": round(float(g.speed), 3),
    "bearingDeg": round(float(g.bearingDeg), 2),
    "horizontalAccuracy": round(float(g.horizontalAccuracy), 2),
    "verticalAccuracy": round(float(g.verticalAccuracy), 2),
    "bearingAccuracyDeg": round(float(g.bearingAccuracyDeg), 2),
    "speedAccuracy": round(float(g.speedAccuracy), 2),
    "unixTimestampMillis": int(g.unixTimestampMillis),
    "source": enum_name(log.GpsLocationData.SensorSource, g.source.raw),
    "vNED": _round_list(g.vNED),
    "hasFix": g.hasFix,
    "satelliteCount": int(g.satelliteCount),
  }


def _lead_summary(l) -> dict:
  """LeadDataV3 取 t=0 首采样摘要（完整逐点序列体积大，不进快照）。"""
  def _first(values, nd: int = 3) -> float:
    vals = list(values)
    return round(float(vals[0]), nd) if vals else 0.0
  return {
    "prob": round(float(l.prob), 3),
    "probTime": round(float(l.probTime), 2),
    "t": _first(l.t, 2),
    "x": _first(l.x, 2),
    "y": _first(l.y, 2),
    "v": _first(l.v, 2),
    "a": _first(l.a, 2),
  }


def model_snapshot(m) -> dict:
  """modelV2 → model 段（20Hz）。leads 只留 t=0 首采样；本 fork 无
  modelDataV2SP，sp 子对象填 null。"""
  return {
    "frameId": int(m.frameId),
    "frameDropPerc": round(float(m.frameDropPerc), 1),
    "modelExecutionTime": round(float(m.modelExecutionTime), 3),
    "big": m.big,
    "confidence": enum_name(log.ModelDataV2.ConfidenceClass, m.confidence.raw),
    "engagedProb": round(float(m.meta.engagedProb), 3),
    "hardBrakePredicted": m.meta.hardBrakePredicted,
    "laneChangeState": enum_name(log.LaneChangeState, m.meta.laneChangeState.raw),
    "laneChangeDirection": enum_name(log.LaneChangeDirection, m.meta.laneChangeDirection.raw),
    "action": {
      "desiredCurvature": round(float(m.action.desiredCurvature), 5),
      "desiredAcceleration": round(float(m.action.desiredAcceleration), 3),
      "shouldStop": m.action.shouldStop,
    },
    "laneLineProbs": _round_list(m.laneLineProbs),
    "laneLineStds": _round_list(m.laneLineStds),
    "leads": [_lead_summary(l) for l in m.leadsV3],
    "sp": None,
  }


def dmodel_snapshot(dm) -> dict:
  """drivingModelData → dmodel 段（20Hz UI 驾驶模型）。path 为多项式系数（非逐点）。"""
  return {
    "frameId": int(dm.frameId),
    "frameDropPerc": round(float(dm.frameDropPerc), 1),
    "modelExecutionTime": round(float(dm.modelExecutionTime), 3),
    "big": dm.big,
    "action": {
      "desiredCurvature": round(float(dm.action.desiredCurvature), 5),
      "desiredAcceleration": round(float(dm.action.desiredAcceleration), 3),
      "shouldStop": dm.action.shouldStop,
    },
    "laneLineMeta": {
      "leftY": round(float(dm.laneLineMeta.leftY), 2),
      "rightY": round(float(dm.laneLineMeta.rightY), 2),
      "leftProb": round(float(dm.laneLineMeta.leftProb), 3),
      "rightProb": round(float(dm.laneLineMeta.rightProb), 3),
    },
    "laneChangeState": enum_name(log.LaneChangeState, dm.meta.laneChangeState.raw),
    "laneChangeDirection": enum_name(log.LaneChangeDirection, dm.meta.laneChangeDirection.raw),
    "path": {
      "x": _round_list(dm.path.xCoefficients, 4),
      "y": _round_list(dm.path.yCoefficients, 4),
      "z": _round_list(dm.path.zCoefficients, 4),
    },
  }


def odom_snapshot(co) -> dict:
  """cameraOdometry → odom 段（20Hz 视觉里程计）。"""
  return {
    "frameId": int(co.frameId),
    "trans": _round_list(co.trans),
    "rot": _round_list(co.rot),
    "transStd": _round_list(co.transStd),
    "rotStd": _round_list(co.rotStd),
    "wideFromDeviceEuler": _round_list(co.wideFromDeviceEuler),
    "wideFromDeviceEulerStd": _round_list(co.wideFromDeviceEulerStd),
  }


# ---------------- frame：周期状态帧 ----------------

def carrot_snapshot(cm) -> dict:
  """carrotMan → frame carrot 段（协议 v4 新增，JSON-able）。字段名与 cereal
  carrotMan 一致，源数据即 carrot_man 20Hz 聚合输出；Text 字段 None 归一空串。
  仅进 frame 不进快照（快照段集合不变，客户端初始化由首帧 frame 补齐）。"""
  return {
    "nRoadLimitSpeed": int(cm.nRoadLimitSpeed),
    "xSpdLimit": int(cm.xSpdLimit),
    "xSpdDist": int(cm.xSpdDist),
    "xSpdCountDown": int(cm.xSpdCountDown),
    "xDistToTurn": int(cm.xDistToTurn),
    "xTurnCountDown": int(cm.xTurnCountDown),
    "desiredSpeed": int(cm.desiredSpeed),
    "desiredSource": str(cm.desiredSource or ""),
    "szPosRoadName": str(cm.szPosRoadName or ""),
    "leftSec": int(cm.leftSec),
    "trafficState": int(cm.trafficState),
  }


def frame_payload(car: dict | None, radar: dict | None, carrot: dict | None = None) -> dict | None:
  """frame 帧组装（纯函数）：仅出现有数据的段；各段全空（窗口无更新）返回 None。"""
  if car is None and radar is None and carrot is None:
    return None
  out: dict = {}
  if car is not None:
    out["car"] = car
  if radar is not None:
    out["radar"] = radar
  if carrot is not None:
    out["carrot"] = carrot
  return out


class FrameTicker:
  """frame 周期调度（纯逻辑，时钟注入便于测试）：到点检查窗口内是否有段更新，
  有则出帧并清空窗口（next tick 重新积累），无更新返回 None——offroad 数据源
  停止后自然停推。节拍落后（轮询阻塞）时按 period 推进而非追赶积压。"""

  def __init__(self, period_s: float, now: float) -> None:
    self.period_s = period_s
    self.next_t = now + period_s
    self.car: Optional[dict] = None
    self.radar: Optional[dict] = None
    self.carrot: Optional[dict] = None

  def tick(self, now: float) -> Optional[dict]:
    if now < self.next_t:
      return None
    self.next_t = max(self.next_t + self.period_s, now + self.period_s)
    payload = frame_payload(self.car, self.radar, self.carrot)
    self.car = self.radar = self.carrot = None
    return payload


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
    self.set_state = set_state  # set_state(kind, data)，维护 /state 快照，可选

  def run(self):
    sm = messaging.SubMaster(SERVICES)
    params = Params()
    events_map: dict[str, dict] = {}
    last_opstate: dict | None = None
    tracker = SoundTracker()
    cruise_main_toggle = CruiseMainOpenpilotToggle(ButtonType.mainCruise)
    ticker = FrameTicker(FRAME_PERIOD_S, time.monotonic())

    cloudlog.info("mate poller started")
    while True:
      sm.update(100)  # 与参考 collector 一致：有消息即返回，空闲最多 100ms 一轮
      now = time.monotonic()

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

      # car/radar：快照段与 frame 窗口同源同帧——快照取全量，frame 只取窗口内更新。
      # radarState 100Hz，onroad 下 poller 每轮（~10ms）都 updated，随轮刷新即可
      # （快照写入与 frame 窗口赋值都是 O(1)，与既有 meta/services 每轮刷新同模式）。
      if sm.updated["carState"]:
        cs = sm["carState"]
        car = car_snapshot(cs)
        if self.set_state is not None:
          self.set_state("car", car)
        ticker.car = car
        for payload in button_payloads(cs):
          self.emit("button", payload)

      if sm.updated["radarState"]:
        radar = radar_snapshot(sm["radarState"])
        if self.set_state is not None:
          self.set_state("radar", radar)
        ticker.radar = radar

      # carrot 遥测：20Hz 输入随轮喂 frame 窗口（协议 v4 carrot 段）；不进快照
      if sm.updated["carrotMan"]:
        ticker.carrot = carrot_snapshot(sm["carrotMan"])

      # v5 纯快照段：只进 /state 与 WS snapshot（set_state），不进事件流。
      # 高频 topic（100Hz）快照取最新值，与 car/radar 段同模式（O(1) 字典写入）；
      # 多 topic 合成段（carParams/control/plan/model）任一来源更新即重建。
      if sm.updated["deviceState"]:
        self.set_state("device", device_snapshot(sm["deviceState"]))
      if sm.updated["peripheralState"]:
        self.set_state("peripheral", peripheral_snapshot(sm["peripheralState"]))
      if sm.updated["managerState"]:
        self.set_state("manager", manager_snapshot(sm["managerState"]))
      if sm.updated["procLog"]:
        self.set_state("proc", proc_snapshot(sm["procLog"]))
      if sm.updated["temperatureSensor"]:
        self.set_state("temp", temp_snapshot(sm["temperatureSensor"]))
      if sm.updated["carParams"]:
        self.set_state("carParams", car_params_snapshot(sm["carParams"]))
      if sm.updated["controlsState"]:
        self.set_state("controls", controls_snapshot(sm["controlsState"]))
      if sm.updated["carControl"]:
        self.set_state("control", control_snapshot(sm["carControl"]))
      if sm.updated["longitudinalPlan"] or sm.updated["lateralManeuverPlan"]:
        self.set_state("plan", plan_snapshot(sm["longitudinalPlan"], sm["lateralManeuverPlan"]))
      if sm.updated["gpsLocationExternal"]:
        self.set_state("gps", gps_snapshot(sm["gpsLocationExternal"]))
      if sm.updated["modelV2"]:
        self.set_state("model", model_snapshot(sm["modelV2"]))
      if sm.updated["drivingModelData"]:
        self.set_state("dmodel", dmodel_snapshot(sm["drivingModelData"]))
      if sm.updated["cameraOdometry"]:
        self.set_state("odom", odom_snapshot(sm["cameraOdometry"]))

      payload = ticker.tick(now)
      if payload:
        self.emit("frame", payload)

      # 声音：与 soundd 主循环一致——每轮先过 main toggle 短路，再做推导。
      # update() 必须在拿到新 carState 后立刻调用，长按计时依赖逐帧喂同一批事件。
      toggled = cruise_main_toggle.update(sm["carState"].buttonEvents, sm["selfdriveState"].enabled)
      payload = tracker.update(sm, toggled)
      if payload:
        self.emit("sound", payload)
