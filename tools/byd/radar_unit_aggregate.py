"""Aggregate radar_unit_pin.py per-segment JSONs into a u verdict.

Usage: .venv/bin/python tools/byd/radar_unit_aggregate.py /tmp/up_hl2.json /tmp/up_hl.json /tmp/up_1005.json
"""
import sys, json, collections
import numpy as np

DOCS = sys.argv[1:]
rows = []
flips = []
cls_stats = []
pres_lead = [0, 0]
pres_no = [0, 0]
crc_rows = []
for path in DOCS:
  for r in json.load(open(path)):
    for w in r["windows"]:
      w["src"] = path
      rows.append(w)
    flips += [dict(f, seg=r["seg"]) for f in r["flips"]]
    for c, s in r["cls_stats"].items():
      cls_stats.append((int(c), *s))
    p = r["pres"]
    pres_lead[0] += p[0]; pres_lead[1] += p[1]
    pres_no[0] += p[2]; pres_no[1] += p[3]
    crc_rows += r.get("crc", [])

def q(vals, ps=(5, 10, 25, 50, 75, 95)):
  return np.percentile(vals, ps) if len(vals) else []

def line(name, vals, fmt="%.4f"):
  if len(vals) < 3:
    print(f"{name}: n={len(vals)} (too few)"); return
  p = q(vals)
  print(f"{name}: n={len(vals)}  p05={fmt%p[0]} p10={fmt%p[1]} p25={fmt%p[2]} med={fmt%p[3]} p75={fmt%p[4]} p95={fmt%p[5]}")

# ---- A. per-window hard upper bound u_max (min over frames; lead stationary -> equality)
line("u_max(all windows)", [w["u_max"] for w in rows if np.isfinite(w["u_max"])])
tierA = [w for w in rows if w["tier"] == "A"]
line("u_max(tierA)", [w["u_max"] for w in tierA if np.isfinite(w["u_max"])])

# ---- stop-to-lead: u_int equality estimator
stop = [w for w in rows if w["stop_hold"]]
line("u_int(stop-to-lead, equality)", [w["u_int"] for w in stop])
line("u_int(stop, drop>=20)", [w["u_int"] for w in stop if w["drop"] >= 20])
# regression slope a_v = 1/u on stationary-lead windows
good = [w for w in rows if w["r_cve"] > 0.7 and w["a_v"] > 0]
line("u=1/a_v (r>0.7 windows)", [1.0 / w["a_v"] for w in good])
# b_v intercept: closing = (v - v_lead)/u -> b_v>0 means lead moving forward
bvs = np.array([w["b_v"] for w in good])
print(f"  b_v sign: med={np.median(bvs):.2f} (positive shift = creeping lead inflates u)")

# hard-bound intersection estimate: u <= min over all u_max
ums = [w["u_max"] for w in rows if np.isfinite(w["u_max"])]
if ums:
  print(f"  GLOBAL upper bound u <= {min(ums):.4f} m/count   (vs feet 0.3048 / meter 1.0)")

# ---- B. flips: b7 = slope*v at crossings
for pair in ((2, 3), (3, 4), (1, 2), (2, 4), (3, 5), (4, 5)):
  pts = [(f["ve"], f["b7"]) for f in flips if tuple(f["pair"]) == pair and f["ve"] > 2]
  if len(pts) < 5:
    continue
  v = np.array([p[0] for p in pts]); b = np.array([p[1] for p in pts])
  slope, inter = np.polyfit(v, b, 1)
  for u, name in ((0.3048, "feet"), (1.0, "meter")):
    print(f"  flip {pair[0]}-{pair[1]}: n={len(pts)} slope={slope:.3f}c/(m/s) -> tau@{name} = {u*slope:.2f}s (+offset {u*inter:.1f}m)")

# ---- validation: hour_logs_2 independent reproduction (per-seg monotonicity)
by_seg = collections.defaultdict(dict)
for c, n, s, z, mx in cls_stats:
  pass
mono_pass = mono_tot = 0
seg_cls = collections.defaultdict(dict)
for path in DOCS:
  for r in json.load(open(path)):
    means = {}
    for c, s in r["cls_stats"].items():
      if s[0] >= 50:
        means[int(c)] = s[1] / s[0]
    if all(k in means for k in (2, 3, 4)):
      mono_tot += 1
      if means[4] > means[3] > means[2]:
        mono_pass += 1
print(f"\nvalidation: per-seg monotonicity {mono_pass}/{mono_tot} = {mono_pass/max(mono_tot,1):.1%}")
print(f"presence vs lead: lead {pres_lead[0]/max(pres_lead[1],1):.1%}  no-lead {pres_no[0]/max(pres_no[1],1):.1%}")

# ---- C. CRC-8 brute over collected payloads (dedupe payload->b0)
if crc_rows:
  pairs = {}
  for t, h in crc_rows:
    d = bytes.fromhex(h)
    pairs.setdefault(d[1:8], d[0])
  # consistency: same payload different b0?
  samples = list(pairs.items())
  print(f"\nCRC: unique payloads={len(samples)}")
  def crc8(data, poly, init, refin, refout, xorout):
    rev = lambda b: int('{:08b}'.format(b)[::-1], 2)
    crc = init
    for b in data:
      b = rev(b) if refin else b
      crc ^= b
      for _ in range(8):
        crc = ((crc << 1) ^ poly) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return (rev(crc) if refout else crc) ^ xorout
  best = (0, None)
  grid = [(0x1D, "SAE J1850/ARS"), (0x07, "CRC-8"), (0x39, "MAXIM"), (0x9B, "WCDMA"), (0x2F, "ENG1939-1")]
  for poly, name in grid:
    for init in (0x00, 0xFF):
      for refin in (False, True):
        for refout in (False, True):
          for xorout in (0x00, 0xFF):
            hit = sum(1 for d, c in samples if crc8(d, poly, init, refin, refout, xorout) == c)
            frac = hit / max(len(samples), 1)
            if frac > best[0]:
              best = (frac, (name, hex(poly), init, refin, refout, xorout))
            if len(samples) >= 300 and hit == len(samples):
              best = (1.0, (name, hex(poly), init, refin, refout, xorout)); break
  print(f"  best match: {best[0]:.1%} {best[1]}")
  # simple xor/sum families
  x = sum(1 for d, c in samples if (d[0]^d[1]^d[2]^d[3]^d[4]^d[5]^d[6]) == c)
  s1 = sum(1 for d, c in samples if ((d[0]+d[1]+d[2]+d[3]+d[4]+d[5]+d[6]) & 0xFF) == c)
  s2 = sum(1 for d, c in samples if ((-sum(d)-d[0]) & 0xFF) == c)
  print(f"  xor-only {x/len(samples):.1%}  sum-only {s1/len(samples):.1%}  -(sum) incl b0? {s2/len(samples):.1%}")
