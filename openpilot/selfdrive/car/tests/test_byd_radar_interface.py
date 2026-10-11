"""Host tests for the BYD radar interface: MRR track slots + HUD-bar fallback (§8.5)."""
import time

import pytest

from opendbc.car import CanData, structs
from opendbc.car.byd.radar_interface import (ACC_HUD_ADAS_MSG, HUD_TRACK_ID, RadarInterface,
                                             TRACK_ID_BASE, TRACK_MSGS)
from opendbc.car.byd.values import CAR, RADAR_HUD_BARS_TO_M

MSG_T_NS = int(1e9 / 50)  # HUD rate; track frames ride the same tick, fine for tests


def hud_data(has_lead: bool, bar: int) -> bytes:
  # dbc layout (byd_general_pt): HasLead = bit 9 (byte1 b1), LeadingDistance =
  # bits 13-15 (byte1 b5-7). A real CANParser decoding these bytes is itself
  # the bit-position regression check.
  dat = bytearray(8)
  dat[1] = ((1 if has_lead else 0) << 1) | (bar << 5)
  return bytes(dat)


def track_data(long_m: float, lat_m: float, rel_kmh: float, tid: int = 1, valid: bool = True) -> bytes:
  # tangdm slot layout: LongDist 16|12@1- (0.06,-4), LatDist 28|10@1+ (0.05,-25.6),
  # TrackID 38|6, RelSpeed 44|10 (1,-512), IsValid 56|1
  dat = bytearray(8)

  def put(start: int, length: int, raw: int) -> None:
    for i in range(length):
      if (raw >> i) & 1:
        dat[(start + i) // 8] |= 1 << ((start + i) % 8)

  put(16, 12, int(round((long_m + 4) / 0.06)) & 0xFFF)
  put(28, 10, int(round((lat_m + 25.6) / 0.05)) & 0x3FF)
  put(38, 6, tid & 0x3F)
  put(44, 10, int(round(rel_kmh + 512)) & 0x3FF)
  if valid:
    dat[7] |= 1  # bit 56
  return bytes(dat)


def make_cp(radar_unavailable: bool) -> structs.CarParams:
  CP = structs.CarParams.new_message()
  CP.carFingerprint = CAR.BYD_SONG_PLUS_DMI_22
  CP.radarUnavailable = radar_unavailable
  return CP


def make_ri(radar_unavailable: bool) -> RadarInterface:
  return RadarInterface(make_cp(radar_unavailable))


def feed(ri: RadarInterface, has_lead: bool, bar: int, n: int = 60, tracks=None) -> structs.RadarDataT | None:
  out = None
  for i in range(n):
    frames = [CanData(ACC_HUD_ADAS_MSG, hud_data(has_lead, bar), 2)]
    if tracks is not None:
      for idx, payload in enumerate(tracks):
        if payload is not None:
          frames.append(CanData(TRACK_MSGS[idx], payload, 1))
    ret = ri.update([(i * MSG_T_NS, frames)])
    if ret is not None:
      out = ret
  return out


class TestInert:
  def test_no_parsers_when_radar_unavailable(self):
    ri = make_ri(True)
    assert ri.rcp_hud is None and ri.rcp_tracks is None

  def test_never_produces_tracks_when_unavailable(self):
    out = feed(make_ri(True), True, 4)
    # base RadarInterfaceBase returns an empty RadarData every 5 frames
    if out is not None:
      assert len(out.points) == 0


class TestHudFallback:
  def test_lead_present_makes_one_point(self):
    out = feed(make_ri(False), True, 4)
    assert out is not None and len(out.points) == 1
    p = out.points[0]
    assert p.trackId == HUD_TRACK_ID
    assert p.dRel == pytest.approx(RADAR_HUD_BARS_TO_M[4])
    assert p.measured
    assert p.vRel != p.vRel  # NaN

  def test_bar_ordering_and_switch(self):
    ri = make_ri(False)
    feed(ri, True, 2)
    assert ri.pts[HUD_TRACK_ID].dRel == pytest.approx(RADAR_HUD_BARS_TO_M[2])
    feed(ri, True, 4)
    assert ri.pts[HUD_TRACK_ID].dRel == pytest.approx(RADAR_HUD_BARS_TO_M[4])

  def test_no_lead_clears_point(self):
    ri = make_ri(False)
    feed(ri, True, 4)
    assert len(ri.pts) == 1
    out = feed(ri, False, 0)
    assert len(ri.pts) == 0 and len(out.points) == 0

  def test_all_table_bars_parse(self):
    for bar, dist in RADAR_HUD_BARS_TO_M.items():
      assert bar <= 7
      ri = make_ri(False)
      feed(ri, True, bar)
      assert ri.pts[HUD_TRACK_ID].dRel == pytest.approx(dist)

  def test_invalid_slots_keep_fallback(self):
    # the Song's real condition: slot frames present, all IsValid=0 -> HUD path
    idle = track_data(0.0, 0.0, 0.0, valid=False)
    out = feed(make_ri(False), True, 3, tracks=[idle] * 4)
    assert len(out.points) == 1
    assert out.points[0].trackId == HUD_TRACK_ID


class TestTrackPriority:
  def test_valid_slot_decodes_and_beats_hud(self):
    t0 = track_data(12.2, 3.1, -10.0, tid=7)  # slot 0x351 valid
    out = feed(make_ri(False), True, 4, tracks=[t0, None, None, None])
    pts = {p.trackId: p for p in out.points}
    assert set(pts) == {TRACK_ID_BASE}  # only the valid slot, HUD suppressed
    p = pts[TRACK_ID_BASE]
    assert p.dRel == pytest.approx(12.2, abs=0.06)
    assert p.yRel == pytest.approx(3.1, abs=0.05)
    assert p.vRel == pytest.approx(-10.0 / 3.6, abs=0.01)
    assert p.measured

  def test_multi_slot(self):
    tracks = [track_data(20.0, 1.0, -5.0), track_data(45.0, -2.5, 2.0, valid=False),
              track_data(30.0, 0.5, -1.0), None]
    out = feed(make_ri(False), True, 2, tracks=tracks)
    ids = {p.trackId for p in out.points}
    assert ids == {TRACK_ID_BASE, TRACK_ID_BASE + 2}

  def test_slot_goes_stale_then_fallback_returns(self):
    ri = make_ri(False)
    feed(ri, True, 4, tracks=[track_data(10.0, 0.0, 0.0), None, None, None])
    assert set(ri.pts) == {TRACK_ID_BASE}
    # next cycle: slot invalid, HUD lead present -> fallback point, track point gone
    feed(ri, True, 3, tracks=[track_data(0.0, 0.0, 0.0, valid=False), None, None, None])
    assert set(ri.pts) == {HUD_TRACK_ID}

  def test_zero_track_id_is_empty_sentinel(self):
    # vendor _update: track_id == 0 means the slot is empty even if IsValid set
    out = feed(make_ri(False), True, 4, tracks=[track_data(15.0, 0.0, 0.0, tid=0, valid=True), None, None, None])
    assert {p.trackId for p in out.points} == {HUD_TRACK_ID}

  def test_no_hud_frames_yet_reports_can_error(self):
    # HUD parser is the guaranteed bus; its silence (not track silence) gates errors.
    # Adapted for this fork's Python CANParser: it timestamps first_seen at parser
    # construction with time.monotonic_ns() and grants a 2 s never-seen grace
    # (opendbc/can/parser.py), so the feed must start at the real clock and run
    # past grace + CAN_INVALID_CNT before can_valid flips - same end behavior as
    # sunnypilot's C++ parser, just after ~2 s of silence instead of immediately.
    ri = make_ri(False)
    out = None
    t0 = time.monotonic_ns()
    for i in range(300):  # 300 * 20 ms = 6 s > 2 s grace + 5 invalid counters
      ret = ri.update([(t0 + i * MSG_T_NS, [CanData(TRACK_MSGS[0], track_data(10.0, 0.0, 0.0), 1)])])
      if ret is not None and ret.errors.canError:
        out = ret
    assert out is not None and out.errors.canError
