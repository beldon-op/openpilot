from openpilot.selfdrive.selfdrived.proximity_alert import (
  LEAD_HARD_BRAKE_COOLDOWN_S,
  LEAD_TTC_CLOSE_COOLDOWN_S,
  LeadHardBrakeTracker,
  LeadStartTracker,
  LeadTtcCloseTracker,
)


class TestLeadHardBrake:
  def test_triggers_and_holds_once(self):
    t = LeadHardBrakeTracker()
    # 高速跟车 30m，前车 aRel -3.0
    assert t.update(True, 30.0, -3.0, 20.0, now=0.0)
    # 持续急刹（2s < 3s 冷却）不重复
    assert not t.update(True, 25.0, -3.0, 20.0, now=2.0)
    assert not t.update(True, 20.0, -3.0, 20.0, now=3.0)

  def test_repeats_after_cooldown_and_condition_cycle(self):
    t = LeadHardBrakeTracker()
    assert t.update(True, 30.0, -3.0, 20.0, now=0.0)
    assert not t.update(True, 25.0, -3.0, 20.0, now=2.0)
    # 条件解除后再急刹且冷却已过 → 可再次触发
    assert not t.update(True, 25.0, -1.0, 20.0, now=4.0)
    assert t.update(True, 25.0, -3.0, 20.0, now=LEAD_HARD_BRAKE_COOLDOWN_S + 4.0)

  def test_low_speed_no_trigger(self):
    t = LeadHardBrakeTracker()
    assert not t.update(True, 30.0, -3.0, 4.9, now=0.0)  # vEgo <= 5

  def test_no_lead_no_trigger(self):
    t = LeadHardBrakeTracker()
    assert not t.update(False, 30.0, -3.0, 20.0, now=0.0)


class TestLeadTtcClose:
  def test_ttc_above_threshold_no_trigger(self):
    t = LeadTtcCloseTracker()
    # 25 m/s、相对速度 -10、距离 35m → TTC 3.5s
    assert not t.update(True, 35.0, -10.0, 25.0, now=0.0)

  def test_ttc_below_threshold_triggers_with_cooldown(self):
    t = LeadTtcCloseTracker()
    # 距离 30m、接近速度 25 m/s → TTC 1.2s
    assert t.update(True, 30.0, -25.0, 25.0, now=0.0)
    assert not t.update(True, 25.0, -20.0, 25.0, now=LEAD_TTC_CLOSE_COOLDOWN_S - 1.0)

  def test_lead_opening_no_trigger(self):
    # 前车远离：vRel > 0 → TTC 视为无穷大
    t = LeadTtcCloseTracker()
    assert not t.update(True, 30.0, 5.0, 25.0, now=0.0)

  def test_low_speed_no_trigger(self):
    t = LeadTtcCloseTracker()
    assert not t.update(True, 30.0, -25.0, 7.9, now=0.0)  # vEgo <= 8


class TestLeadStart:
  def test_green_light_lead_starts(self):
    t = LeadStartTracker()
    # 红灯排队：前车静止 8m，本车 standstill
    for now in (0.0, 1.0, 2.0):
      assert not t.update(True, 8.0, 0.0, True, now)
    # 前车起步 0.5s，本车未动 → 触发
    assert not t.update(True, 8.0, 0.8, True, now=2.2)   # 起步未满 0.5s
    assert t.update(True, 8.0, 0.8, True, now=2.8)

  def test_ego_moved_no_trigger(self):
    t = LeadStartTracker()
    for now in (0.0, 1.0, 2.0):
      t.update(True, 8.0, 0.0, True, now)
    # 前车起步时本车已动（非 standstill）
    assert not t.update(True, 8.0, 0.8, False, now=2.8)

  def test_once_per_stop_cycle(self):
    t = LeadStartTracker()
    for now in (0.0, 1.0, 2.0):
      t.update(True, 8.0, 0.0, True, now)
    assert not t.update(True, 8.0, 0.8, True, now=2.2)  # 起步未满 0.5s
    assert t.update(True, 8.0, 0.8, True, now=2.8)
    # 本车未动：前车再停再走不再提醒（armed 未恢复）
    t.update(True, 10.0, 0.0, True, now=5.0)
    assert not t.update(True, 10.0, 0.8, True, now=5.8)
    assert not t.update(True, 10.0, 0.8, True, now=6.0)
    # 本车起步又停下（新周期）→ 重新武装
    t.update(True, 10.0, 0.5, False, now=6.5)
    for now in (6.5, 7.5, 8.5):
      t.update(True, 10.0, 0.0, True, now)
    assert not t.update(True, 10.0, 0.8, True, now=8.7)
    assert t.update(True, 10.0, 0.8, True, now=9.2)

  def test_transient_go_is_filtered(self):
    t = LeadStartTracker()
    for now in (0.0, 1.0, 2.0):
      t.update(True, 8.0, 0.0, True, now)
    # 前车 vRel 超阈值不足 0.5s（瞬时扰动）后回到静止
    t.update(True, 8.0, 0.8, True, now=2.2)
    t.update(True, 8.0, 0.0, True, now=2.5)   # 静止重新积累
    t.update(True, 8.0, 0.0, True, now=4.5)   # 满 2s
    assert not t.update(True, 8.0, 0.8, True, now=4.7)  # 起步未满 0.5s
    assert t.update(True, 8.0, 0.8, True, now=5.2)

  def test_lead_lost_resets(self):
    t = LeadStartTracker()
    for now in (0.0, 1.0, 2.0):
      t.update(True, 8.0, 0.0, True, now)
    # 前车丢失（status=False）：状态机复位，静止时长清零
    t.update(False, 8.0, 0.0, True, now=2.2)
    # 前车重现并直接起步（静止时长未重新积累）→ 不触发
    assert not t.update(True, 8.0, 0.8, True, now=2.4)

  def test_distant_lead_no_trigger(self):
    t = LeadStartTracker()
    for now in (0.0, 1.0, 2.0):
      t.update(True, 20.0, 0.0, True, now)  # 距离超 15m
    assert not t.update(True, 20.0, 0.8, True, now=2.8)
