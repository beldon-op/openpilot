"""BYD Song Plus DM-i radar interface: MRR track slots with HUD-bar fallback.

Two sources, in priority order (docs/byd-lane-change.md §8.5):

  1. 0x351-0x354 RADAR_TRACK_35x on bus 1 - MRR target slots, ~25 Hz, layout
     taken verbatim from the mypilot community byd_tangdm_radar.dbc (same IDs
     the vendor's decompiled radar_interface parses for some BYD models).
     On the Song these slots are logged across all 415 archive segments as
     IsValid=0 placeholder frames (793k ticks/slot, 0 with IsValid=1 or
     TrackID!=0 - even during 0.9M HUD lead frames, docs §8.5.7), i.e. the
     MRR does not export targets onto bus 1 - but the frames ARE this
     protocol family, and the slot path auto-lights the moment a real target
     stream becomes reachable (radar private CAN tap; no duty state on the
     car's wired buses fills the slots - 0x374 is also 0 frames on every bus
     across the full archive rescan).

  2. 0x32D ACC_HUD_ADAS on bus 2 - the stock radar's fused single-lead
     summary (HasLead + LeadingDistance 0-7 gap bars). Always available;
     used whenever no track slot is valid.

Vendor branch sets, extracted from the decompiled _create_radar_can_parser
bytecode (co_names, 2026-10-11): {BYD_TANG_GAS, BYD_TANG_DM} parse (849,852)
@25Hz on CanBus.BYD_AUX=bus 1 - this IS the tangdm slot protocol; the Song is
in NEITHER vendor branch (no parser), because the MRR does not export targets
on the Song's bus 1. We therefore parse all four slots ourselves with the
community layout; the vendor's Tang-branch consumption is confirmed consistent
(vRel = REL_SPEED * KPH_TO_MS, yRel clip +-5, track_id==0 sentinel).

RelSpeed unit caveat: the community dbc labels it km/s (obviously wrong) and
its code passes the raw value (factor 1, offset -512) to vRel as m/s - an
implausible +-512 m/s range, so we apply KPH_TO_MS as the physically sane
reading. TODO(belabor): verify against the first real IsValid=1 frame.

Status: inert by default. interface.py keeps CP.radarUnavailable=True until
RADAR_HUD_BARS_TO_M is calibrated on-road and a track source is live;
flipping that flag is then the only change needed. Behavior today is
unchanged (no parser, no liveTracks).
"""
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.byd.values import DBC, RADAR_HUD_BARS_TO_M
from opendbc.car.interfaces import RadarInterfaceBase

ACC_HUD_ADAS_MSG = 813  # 0x32D, bus 2
TRACK_MSGS = [849, 850, 851, 852]  # 0x351-0x354, bus 1 (MRR slot block)
TRACK_FREQ = 25
TRACK_BUS = 1
HUD_BUS = 2
# track-slot point ids are namespaced away from the HUD fallback id so
# switching sources mid-drive never aliases a KF-tracked point
TRACK_ID_BASE = 101
HUD_TRACK_ID = 1


def _create_track_can_parser(CP):
  return CANParser(DBC[CP.carFingerprint][Bus.radar], [(addr, TRACK_FREQ) for addr in TRACK_MSGS], TRACK_BUS)


def _create_hud_can_parser(CP):
  return CANParser(DBC[CP.carFingerprint][Bus.radar], [(ACC_HUD_ADAS_MSG, 50)], HUD_BUS)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP):
    # NOTE: single-arg CP - this fork's RadarInterfaceBase takes no CP_SP
    # (sunnypilot source had (CP, CP_SP); adapted on port, 2026-10-11)
    super().__init__(CP)
    self.rcp_tracks = None if CP.radarUnavailable else _create_track_can_parser(CP)
    self.rcp_hud = None if CP.radarUnavailable else _create_hud_can_parser(CP)
    self.updated_messages: set[int] = set()

  def _update_tracks(self) -> bool:
    """Refresh points from valid MRR slots. True if any slot is valid."""
    any_valid = False
    live_ids = set()
    for i, addr in enumerate(TRACK_MSGS):
      t = self.rcp_tracks.vl[addr]
      # vendor _update: track_id == 0 is the empty-slot sentinel
      if not t["IsValid_tang"] or t["TrackID_tang"] == 0:
        continue
      any_valid = True
      track_id = TRACK_ID_BASE + i
      live_ids.add(track_id)
      if track_id not in self.pts:
        self.pts[track_id] = structs.RadarData.RadarPoint()
        self.pts[track_id].trackId = track_id
      pt = self.pts[track_id]
      pt.dRel = t["LongDist_tang"]
      # vendor clips LAT_DIST to +-5 m; TODO(sign): confirm left/right convention
      # against the first real frame (passed as-is, same choice as the community)
      pt.yRel = max(-5.0, min(5.0, t["LatDist_tang"]))
      # vendor Tang branch: vRel = REL_SPEED * KPH_TO_MS (bytecode-confirmed)
      pt.vRel = t["RelSpeed_tang"] * CV.KPH_TO_MS
      pt.aRel = float('nan')
      pt.yvRel = float('nan')
      pt.measured = True
    # slots that dropped or went invalid this cycle are stale (community _update
    # keeps zero-filled invalid points; we drop them - radarState only wants
    # targets). live_ids is empty when nothing is valid, so this also wipes the
    # slot block the moment the last target leaves.
    for track_id in list(self.pts.keys()):
      if track_id >= TRACK_ID_BASE and track_id not in live_ids:
        del self.pts[track_id]
    return any_valid

  def _update_hud(self) -> None:
    vl = self.rcp_hud.vl[ACC_HUD_ADAS_MSG]
    if vl["HasLead"] and vl["LeadingDistance"] in RADAR_HUD_BARS_TO_M:
      if HUD_TRACK_ID not in self.pts:
        self.pts[HUD_TRACK_ID] = structs.RadarData.RadarPoint()
        self.pts[HUD_TRACK_ID].trackId = HUD_TRACK_ID
      pt = self.pts[HUD_TRACK_ID]
      pt.dRel = RADAR_HUD_BARS_TO_M[int(vl["LeadingDistance"])]
      pt.yRel = 0.0
      pt.vRel = float('nan')
      pt.aRel = float('nan')
      pt.yvRel = float('nan')
      pt.measured = True
    else:
      self.pts.pop(HUD_TRACK_ID, None)

  def update(self, can_packets):
    if self.rcp_hud is None:
      return super().update(can_packets)

    self.updated_messages.update(self.rcp_tracks.update(can_packets))
    self.updated_messages.update(self.rcp_hud.update(can_packets))
    if not (self.updated_messages & set(TRACK_MSGS) or ACC_HUD_ADAS_MSG in self.updated_messages):
      return None
    self.updated_messages.clear()

    ret = structs.RadarData()
    # only the HUD parser is guaranteed present on the car's wired buses;
    # the track block can be legitimately silent (e.g. radar powered down on
    # some trims) so its timeout must not raise canError on its own
    if not self.rcp_hud.can_valid:
      ret.errors.canError = True
      return ret

    if not self._update_tracks():
      self._update_hud()

    ret.points = list(self.pts.values())
    return ret
