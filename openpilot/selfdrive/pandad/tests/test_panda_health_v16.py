"""Host-side health packet v16 compat for the BYD c3l board.

The c3l board runs the frozen 0.9.x-baseline firmware (health packet v16,
see panda/board/obj/FIRMWARE_NOTICE.md); it cannot be reflashed with this
repo's new-generation firmware. The python panda library must therefore keep
accepting v16 — dropping it (as a3fbd86f did) makes pandad crash-loop on
health() and the UI shows panda 否 / offline.
"""
from types import SimpleNamespace

import pytest

from panda.python.__init__ import ensure_health_packet_version
from panda import Panda


@ensure_health_packet_version
def _read_health(self):
  return "ok"


def _fake_panda(health_version):
  return SimpleNamespace(health_version=health_version, HEALTH_PACKET_VERSION=Panda.HEALTH_PACKET_VERSION)


def test_ensure_health_accepts_v16_and_v17():
  assert _read_health(_fake_panda(Panda.LEGACY_HEALTH_PACKET_VERSION)) == "ok"
  assert _read_health(_fake_panda(Panda.HEALTH_PACKET_VERSION)) == "ok"


def test_ensure_health_rejects_unknown_version():
  with pytest.raises(RuntimeError, match="health packet version mismatch"):
    _read_health(_fake_panda(18))


def test_v16_layout_maps_onto_v17_dict():
  assert Panda.LEGACY_HEALTH_STRUCT.size == Panda.HEALTH_STRUCT.size - 1

  # sentinel payload per the 0.9.x firmware health.h field order
  vals = [111, 112, 113, 114, 115, 116, 117, 118,   # 8×u32
          121, 122, 123, 124, 125, 126,             # ignition…safety_mode + safety_param
          127, 128, 129, 130,                       # …heartbeat_lost + alternative_experience
          131.5,                                    # interrupt_load
          132, 133, 134, 135, 136, 137, 138]        # …som_reset_triggered
  a = list(Panda.LEGACY_HEALTH_STRUCT.unpack(Panda.LEGACY_HEALTH_STRUCT.pack(*vals)))
  a.insert(10, 0)  # wake_on_can, added in health v17

  assert a[9] == 122    # ignition_can
  assert a[10] == 0     # wake_on_can (not in v16 firmware)
  assert a[11] == 123   # controls_allowed
  assert a[17] == 129   # heartbeat_lost
  assert a[18] == 130   # alternative_experience
  assert abs(a[19] - 131.5) < 0.01  # interrupt_load
  assert a[26] == 138   # som_reset_triggered
