#!/usr/bin/env python3
"""BYD rear/radar traffic bus probe - DEVICE-SIDE, READ-ONLY.

Context (docs/byd-lane-change.md section 8): the turn-signal lane change can
only see cars that already reached the BSD warning ring - 0x418 two live bits
at 10 Hz - because RADAR_MRR (0x374) has zero frames on every logged bus
(verified over all 415 road-test segments). To get a real "car approaching from
behind" input (distance + closing speed -> proper gate/TTC like stock radar
can_change_lane), we first need to find where the radar objects actually are:
a different bus, or behind the gateway.

Stop the openpilot stack first - it owns the panda otherwise (canerd):
    sudo systemctl stop 'sunnypilot*'   # or however the stack is supervised
    sudo /usr/local/venv/bin/python tools/byd/radar_bus_probe.py [seconds]

The Song Plus harness currently wires bus 0 and bus 2; buses OP is not wired
to will simply show 0 frames - that result is itself the answer for the next
step (wiring). The probe only receives. It never sends any CAN frame and does
not touch the safety mode beyond enabling RX on the four panda buses.
"""
import sys
import time
import collections

from panda import Panda

SECONDS = float(sys.argv[1]) if len(sys.argv) > 1 else 10.0
BUSES = (0, 1, 2, 3)

# already-known BYD frames (bus 0/2); everything else is "unknown - candidate"
KNOWN = {
  0x11f: "EPS", 0x122: "WHEEL_SPEED", 0x12d: "BCM", 0x133: "STALKS", 0x1f0: "ESP_SPEED",
  0x242: "DRIVE_STATE", 0x318: "ACC_EPS_STATE", 0x32d: "ACC_HUD_ADAS", 0x32e: "ACC_CMD",
  0x342: "PEDAL", 0x374: "RADAR_MRR", 0x3b0: "PCM_BUTTONS", 0x418: "BSD_RADAR",
}


def main() -> None:
  p = Panda()
  for bus in BUSES:
    p.set_can_enabled(bus, True)
  # drain anything queued before timing the window
  while p.can_recv():
    pass

  counts = collections.Counter()
  first_bytes = {}
  t0 = time.monotonic()
  while time.monotonic() - t0 < SECONDS:
    for _addr, _dat, _src in p.can_recv():
      counts[(_addr, _src)] += 1
      first_bytes.setdefault((_addr, _src), bytes(_dat[:8]))
    time.sleep(0.001)
  dur = time.monotonic() - t0

  print(f"listened {dur:.1f}s on buses {BUSES}; distinct (addr,bus) pairs: {len(counts)}\n")
  for bus in BUSES:
    rows = sorted(((a, n) for (a, s), n in counts.items() if s == bus), key=lambda r: -r[1])
    print(f"== bus {bus}: {len(rows)} addresses ==")
    for addr, n in rows:
      hz = n / dur
      name = KNOWN.get(addr, "??")
      data = first_bytes.get((addr, bus), b"")
      # radar candidates: >1 Hz senders we don't know yet
      tag = "  <-- unknown sender, inspect (radar candidate)" if name == "??" and hz > 1.0 else ""
      print(f"  {addr:#06x} {hz:7.1f} Hz  {name:16s} {data.hex()} {tag}")
    print()

  mrr = sum(n for (a, _s), n in counts.items() if a == 0x374)
  print(f"RADAR_MRR 0x374 frames seen this window: {mrr}")
  if mrr == 0:
    print("=> 0x374 absent on all four buses (stack stopped, RX only): the objects are not on "
          "the harness-wired buses or not sent in this key position - check other key states / "
          "gateway, or re-sniff with the vendor BSD feature actively triggered.")


if __name__ == "__main__":
  main()
