"""Unit-anchor decode: find object-distance fields in the 67ms slot table using
STOPPED-LEAD approach events (physics identity: dx/dt = -vEgo exactly, lead immovable).

Rebuilds the 13 stop2lead-style events from /tmp/seg1005.pkl (same criteria as
radar_stop2lead.py: standstill>=2s + plateau b7 in 5..80 + presence 7 + approach drop>=10),
then for each event window, for each slot address, every field variant:
   u16 windows: (b2,b3),(b3,b4),(b4,b5),(b5,b6),(b6,b7) x LE/BE
   plus the same as i16 (sign at 0x8000)
   require: window length >=3s, mean vEgo >=0.4, field monotone-ish:
     r = corr(d field/dt, -vEgo) > 0.9  (distance field, ANY unit)
   unit = median(d field/dt / vEgo) counts/(m/s), sign-corrected
Aggregate across events per (addr, variant): how many events fired, unit spread
(std/mean must be < 0.15 to count as a real physical field with consistent scale).

Usage: .venv/bin/python tools/byd/radar_slot67_units.py
"""
import sys, pickle, json, collections
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
import numpy as np

S67 = pickle.load(open("/tmp/slot67.pkl", "rb"))
S10 = pickle.load(open("/tmp/seg1005.pkl", "rb"))
ADDRS = sorted({int(k[:3], 16) for k in next(iter(S67.values())) if k.endswith("_t")})

VARIANTS = {}
for p in (2, 3, 4, 5, 6):
  VARIANTS[f"{p}{p+1}LE"] = (p, "LE")
  VARIANTS[f"{p}{p+1}BE"] = (p, "BE")
  if p in (2, 4):   # 12-bit nibble-packed variants: high byte's low nibble holds bits 8..11
    VARIANTS[f"{p}{p+1}nL"] = (p, "NLO")    # ((b_{p+1}&0xF)<<8)|b_p
    VARIANTS[f"{p}{p+1}nH"] = (p, "NHI")    # ((b_{p+1}>>4)<<8)|b_p
    VARIANTS[f"{p}8"] = (p, "U8")           # single byte b_p


def field(d, p, endian):
  lo = d[:, p].astype(np.float64)
  hi = d[:, p + 1].astype(np.float64) if p < 7 else lo
  if endian == "LE":
    return lo + hi * 256.0
  if endian == "BE":
    return hi + lo * 256.0
  if endian == "NLO":
    return lo + (hi % 16.0) * 256.0
  if endian == "NHI":
    return lo + np.floor(hi / 16.0) * 256.0
  if endian == "U8":
    return lo
  raise ValueError(endian)


def build_events():
  """stopped-lead approach events: list of (seg, t0_abs, t1_abs) where t1 = plateau start."""
  ev = []
  for name, k in S10.items():
    if name not in S67 or not len(k["109_t"]):
      continue
    t = k["109_t"].astype(np.float64)
    b7 = k["109_d"][:, 7].astype(np.float64)
    vp = (k["10a_d"][:, 3] >> 5) & 7 if len(k["10a_t"]) else np.zeros(len(t), int)
    v = np.interp(t, k["cs_t"].astype(np.float64), k["cs_v"].astype(np.float64))
    i = 0
    while i < len(t) - 90:
      # plateau: >=90 frames, b7 in [5,80], stable +-2, ego basically stopped
      if (vp[i:i+90] == 7).all() and 5 <= b7[i] <= 80 and np.ptp(b7[i:i+90]) <= 2 \
         and np.nanmean(v[i:i+90]) < 0.25:
        # walk back through the approach (allow v dips to ~0.15, presence must stay 7)
        j = i
        drop_ok = False
        while j > 0 and vp[j] == 7 and b7[j] >= b7[i]:
          j -= 1
        if i - j >= 50 and b7[j:i].mean() > 0 and (b7[j] - b7[i]) >= 10 \
           and np.nanmean(v[j:i]) > 0.4:
          ev.append((name, t[j], t[i]))
          i += 150
          continue
        i += 90
      else:
        i += 10
  return ev


def main():
  events = build_events()
  print(f"# {len(events)} stopped-lead events")
  agg = collections.defaultdict(lambda: {"unit": [], "r": []})
  for name, t0, t1 in events:
    s = S67[name]
    k = S10[name]
    tv = k["cs_t"].astype(np.float64); vv = k["cs_v"].astype(np.float64)
    for a in ADDRS:
      ta = s[f"{a:03x}_t"].astype(np.float64); da = s[f"{a:03x}_d"]
      m = (ta >= t0) & (ta <= t1)
      if m.sum() < 30:
        continue
      tw, dw = ta[m], da[m]
      if not (0.4 < np.nanmean(np.interp(tw, tv, vv))):
        continue
      dt = np.diff(tw) / 1e9
      if dt.min() <= 0:
        continue
      # stopped-lead physics: object x shrinks exactly by ego displacement.
      # Regress field vs cumulative meters traveled - robust to dm quantization.
      vm = np.interp(tw[:-1], tv, vv)
      disp = np.concatenate([[0.0], np.cumsum(vm * dt)])
      if disp[-1] < 3.0 or len(disp) < 40:
        continue
      for vn, (p, e) in VARIANTS.items():
        fv = field(dw, p, e)[:-1]
        if np.std(fv) < 3:
          continue
        sl, ic = np.polyfit(disp[:-1], fv, 1)
        pred = sl * disp[:-1] + ic
        ss = 1.0 - np.sum((fv - pred) ** 2) / max(1e-9, np.sum((fv - fv.mean()) ** 2))
        if ss > 0.9:                                               # linear in displacement => distance-like
          agg[f"{a:03x}/{vn}"]["unit"].append(sl)                  # counts per meter (10=dm, 100=cm)
          agg[f"{a:03x}/{vn}"]["r"].append(ss)
          if a == 0x380 and vn == "23nL":
            print(f"   [380/23nL] {name[-16:]} dur={(tw[-1]-tw[0])/1e9:5.1f}s disp={disp[-1]:6.1f}m "
                  f"b7start={np.interp(t0, S10[name]['109_t'].astype(float), S10[name]['109_d'][:,7]):4.0f} unit={sl:6.1f} R2={ss:.4f}")
  rows = []
  for kk, g in agg.items():
    u = np.array(g["unit"])
    if len(u) < 3:
      continue
    rows.append((kk, len(u), np.median(u), np.std(u), np.mean(np.abs(g["r"])),
                 min(u), max(u)))
  rows.sort(key=lambda x: x[3] / max(1e-9, abs(x[2])))
  print(f" {'addr/variant':16s} {'n_ev':>4} {'unit_med':>9} {'unit_std':>9} {'r':>6} {'min':>9} {'max':>9}")
  for kk, n, um, us, rm, mn, mx in rows[:25]:
    print(f" {kk:16s} {n:4d} {um:9.2f} {us:9.2f} {rm:6.3f} {mn:9.2f} {mx:9.2f}")


if __name__ == "__main__":
  main()
