"""BYD T8 forward-radar gap monitor: 0x109 b7 as the upstream continuous signal
for lane-change gating (docs/byd-radar-010x.md §落地建议 T8 + §④e/§④f design
stats; docs/byd-lane-change.md §8.4).

What the archive says (2026-10-18 ④):
  - b7 crosses the old/new plateau midpoint BEFORE the HUD gear flips in
    309/309 informative transitions, and at flip time the distance has only
    travelled 37-59% of the plateau gap -> a boundary-cross detector on b7
    leads the gear (and any HUD-side reaction) by ~0.2 s to >8 s.
  - the loss path is SYNCHRONOUS (b7==0 within 0.02 s of HasLead dropping),
    so "target gone" carries no lead time and this monitor never uses
    disappearance as a clear signal.
  - a single static threshold misclassifies 29-60% of frames inside constant
    holds -> detection must be boundary-cross with hysteresis + time debounce.
  - presence/visibility filter (④f): presence==7 AND b7>0; b7>=246 is the
    range-edge band (④a: 246 is the continuous max, 247 a loss-edge transient
    code) and is treated as NOT visible.

u-free by construction: everything runs in r = b7 / vEgo [counts per m/s]
against plateau medians pooled from the 415-segment archive - no meters, no
assumed unit (feet vs 0.25 stays a road-test open item, §④b).

Boundaries: B[3]/B[4] are the midpoints of the pooled plateau medians
{2: 2.92, 3: 4.79, 4: 6.61} (④e pre-table); B[2] has no lower neighbour in
the archive (bar 1 unobserved as a follow class) so it is extrapolated with
the same half-step. cls < 2 or an untracked-but-visible target (the --142
mechanism) uses the tightest band B[2].
"""
from collections import deque

PLATEAU_R = {2: 2.92, 3: 4.79, 4: 6.61}          # counts/(m/s), archive-pooled
_HALF_34 = (PLATEAU_R[4] - PLATEAU_R[3]) / 2.0    # 0.91
_HALF_23 = (PLATEAU_R[3] - PLATEAU_R[2]) / 2.0    # 0.935
BAND_BOUNDARY_DOWN = {2: PLATEAU_R[2] - _HALF_23,   # 1.985
                      3: PLATEAU_R[2] + _HALF_23,   # 3.855
                      4: PLATEAU_R[3] + _HALF_34}   # 5.70

# tuning fixed by tools/byd/radar_t8_replay.py on the 415-segment archive
# (2026-10-18 night): steady-hold would-block 8.3% of visible hold seconds
# (vs ④e's static-threshold 29-60% false-cross), closing-flip pre-warning
# 33% (4->3) / 23% (3->2) with median lead 4.8 s, 22 alert episodes/h.
SET_MARGIN = 0.05    # set while r < boundary*(1-SET_MARGIN) ...
SET_TIME = 0.5       # ... for this many seconds (debounce; ④e false-cross guard)
REL_MARGIN = -0.02   # release once r > boundary*(1-REL_MARGIN) i.e. slightly above
REL_TIME = 1.0       # ... held this long (hysteresis against band-edge chatter)
SLOPE_DROP = 0.2     # r must also be descending: r(now) < r(now-1s) - this
                     # (== relative closing of slope*vEgo m/s: 2 m/s @10, 5.4 @27)
SLOPE_WINDOW = 1.0
V_EGO_MIN = 3.0      # m/s - below this r explodes (stop platforms are not alerts)
B7_LO = 1            # ④f: b7>0 required (b7==0 = no-target sentinel)
B7_HI = 245          # ④a: >=246 is the range-edge band, not a trustworthy gap
PRESENCE_VISIBLE = 7
NEW_HOLD = 10.0      # radar capture leads the HUD gear by >=10 s (④e) - keep the
                     # "new target" flag this long after the visible rising edge
STALE_TIME = 0.5     # 0x109 silent this long -> not fresh, treat as invisible


class RadarGapMonitor:
  def __init__(self):
    self.t = 0.0
    self.closing = False
    self.new_target = False
    self.visible = False
    self._below_t = 0.0
    self._above_t = 0.0
    self._invisible_t = 0.0
    self._stale_t = 0.0
    self._edge_t = None       # last visible rising edge
    self._hist = deque()      # (t, r) for the descending-slope check

  @staticmethod
  def _boundary(cls: int) -> float:
    if cls >= 4:
      return BAND_BOUNDARY_DOWN[4]
    if cls == 3:
      return BAND_BOUNDARY_DOWN[3]
    return BAND_BOUNDARY_DOWN[2]

  def update(self, dt: float, b7: int, presence: int, hud_cls: int,
             v_ego: float, frame_fresh: bool) -> None:
    self.t += dt

    if not frame_fresh:
      self._stale_t += dt
    else:
      self._stale_t = 0.0
    fresh = self._stale_t < STALE_TIME

    visible = bool(fresh and presence == PRESENCE_VISIBLE and
                   B7_LO <= b7 <= B7_HI)
    if visible and not self.visible:
      # rising edge only after a confirmed invisible period (④f: a flickering
      # presence nibble must not stamp every frame as "new")
      if self._invisible_t >= 0.5:
        self._edge_t = self.t
      self.new_target = False
    elif not visible:
      self._invisible_t += dt
    else:
      self._invisible_t = 0.0
    if visible and self.new_target and self._edge_t is not None and \
       (self.t - self._edge_t) > NEW_HOLD:
      self.new_target = False
    self.visible = visible
    if visible and self._edge_t is not None and (self.t - self._edge_t) <= NEW_HOLD:
      self.new_target = True

    r = None
    if visible and v_ego > V_EGO_MIN:
      r = b7 / v_ego
      self._hist.append((self.t, r))
      while self._hist and self.t - self._hist[0][0] > SLOPE_WINDOW + dt:
        self._hist.popleft()

    b = self._boundary(hud_cls)
    set_lim = b * (1.0 - SET_MARGIN)
    rel_lim = b * (1.0 - REL_MARGIN)

    below = r is not None and r < set_lim
    above = r is None or r > rel_lim   # no usable gap -> drift to release
    self._below_t = self._below_t + dt if below else 0.0
    self._above_t = self._above_t + dt if above else 0.0

    if not self.closing:
      old = self._hist[0][1] if self._hist else None
      descending = r is not None and old is not None and r < old - SLOPE_DROP
      if below and descending and self._below_t >= SET_TIME:
        self.closing = True
    else:
      if self._above_t >= REL_TIME or not visible:
        self.closing = False
