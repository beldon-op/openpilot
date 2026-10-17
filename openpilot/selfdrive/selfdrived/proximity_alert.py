"""接近性提醒 tracker：前车急刹 / TTC 过近 / 前车起步（纯提醒，不参与控制）。

全部为「条件上升沿 + 冷却」：条件由假转真且冷却已过才触发，条件持续为真不重复。
now 由调用方注入（selfdrived 用 `car_state_time_s if REPLAY else time.monotonic()`），
tracker 自身不取时钟，回放与单测可注入任意时间序列。
"""

LEAD_HARD_BRAKE_AREL = -2.5    # m/s²，低于此值视为前车急刹
LEAD_HARD_BRAKE_DREL = 40.     # m
LEAD_HARD_BRAKE_VEGO = 5.      # m/s
LEAD_HARD_BRAKE_COOLDOWN_S = 3.

LEAD_TTC_CLOSE_TTC = 1.5       # s
LEAD_TTC_CLOSE_DREL = 60.      # m
LEAD_TTC_CLOSE_VEGO = 8.       # m/s
LEAD_TTC_CLOSE_COOLDOWN_S = 5.
LEAD_TTC_VREL_EPS = 0.1        # TTC 分母下限，前车远离时 TTC 视为无穷大

LEAD_START_DREL = 15.          # m
LEAD_START_STILL_VREL = 0.3    # m/s，|vRel| 低于此视为前车静止
LEAD_START_STILL_TIME_S = 2.   # 前车静止须持续
LEAD_START_GO_VREL = 0.5       # m/s，前车起步判定
LEAD_START_GO_TIME_S = 0.5     # 起步速度须持续


class _EdgeCooldownTracker:
  """条件上升沿 + 冷却：条件持续为真只触发一次，冷却期内抑制再次触发。"""

  def __init__(self, cooldown_s: float) -> None:
    self.cooldown_s = cooldown_s
    self.prev_condition = False
    self.last_trigger: float | None = None

  def _update(self, condition: bool, now: float) -> bool:
    trigger = condition and not self.prev_condition and \
      (self.last_trigger is None or now - self.last_trigger >= self.cooldown_s)
    if trigger:
      self.last_trigger = now
    self.prev_condition = condition
    return trigger

  def reset(self) -> None:
    self.prev_condition = False
    self.last_trigger = None


class LeadHardBrakeTracker(_EdgeCooldownTracker):
  """前车急刹：aRel 低于阈值且距离近、有车速。持续急刹只报一次。"""

  def __init__(self) -> None:
    super().__init__(LEAD_HARD_BRAKE_COOLDOWN_S)

  def update(self, lead_status: bool, lead_d_rel: float, lead_a_rel: float,
             v_ego: float, now: float) -> bool:
    condition = (lead_status and lead_a_rel < LEAD_HARD_BRAKE_AREL and
                 0 < lead_d_rel < LEAD_HARD_BRAKE_DREL and v_ego > LEAD_HARD_BRAKE_VEGO)
    return self._update(condition, now)


class LeadTtcCloseTracker(_EdgeCooldownTracker):
  """TTC 过近：dRel / 接近速度 低于阈值。前车远离（vRel≥0）时 TTC 视为无穷大。"""

  def __init__(self) -> None:
    super().__init__(LEAD_TTC_CLOSE_COOLDOWN_S)

  def update(self, lead_status: bool, lead_d_rel: float, lead_v_rel: float,
             v_ego: float, now: float) -> bool:
    closing = max(-lead_v_rel, LEAD_TTC_VREL_EPS)
    condition = (lead_status and 0 < lead_d_rel < LEAD_TTC_CLOSE_DREL and
                 lead_d_rel / closing < LEAD_TTC_CLOSE_TTC and v_ego > LEAD_TTC_CLOSE_VEGO)
    return self._update(condition, now)


class LeadStartTracker:
  """前车从静止起步而本车未动。

  状态机：前车静止（|vRel| 低于阈值且距离近）持续达标 → 进入起步观察；
  起步速度持续达标且本车 standstill 时触发一次。本车起步（离开 standstill）
  后重新武装；前车丢失/远离/速度不明即复位。"""

  def __init__(self) -> None:
    self.reset()

  def reset(self) -> None:
    self.still_since: float | None = None
    self.go_since: float | None = None
    self.armed = True
    self.prev_standstill: bool | None = None

  def update(self, lead_status: bool, lead_d_rel: float, lead_v_rel: float,
             standstill: bool, now: float) -> bool:
    trigger = False
    if self.prev_standstill and not standstill:
      self.armed = True  # 本车起步消费本次提醒，下个停车周期重新武装
    self.prev_standstill = standstill

    near = lead_status and 0 < lead_d_rel < LEAD_START_DREL
    lead_still = near and abs(lead_v_rel) < LEAD_START_STILL_VREL
    lead_go = near and lead_v_rel > LEAD_START_GO_VREL

    if lead_still:
      if self.go_since is not None:  # 起步中断，重新积累静止时长
        self.go_since = None
        self.still_since = None
      if self.still_since is None:
        self.still_since = now
    elif lead_go and self.still_since is not None and \
        now - self.still_since >= LEAD_START_STILL_TIME_S:
      if self.go_since is None:
        self.go_since = now
      if self.armed and standstill and now - self.go_since >= LEAD_START_GO_TIME_S:
        trigger = True
        self.armed = False
    else:  # 前车丢失/远离/速度不明：状态机复位
      self.still_since = None
      self.go_since = None

    return trigger
