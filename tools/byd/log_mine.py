"""Second-pass mine of the BYD rlog archive (docs_site/byd_logs_2026-10-05_07, build b33616e596).

Metrics: 1 engage->EPS c=1 latency; 2 c=0 windup/rail; 3 straight-window torque
sign-flips (governor evidence); 4 cruise-enabled fall attribution; 5 SteerWarning/
ErrorCode/TorqueFailed; 6 vEgo & cluster vs GPS calibration; 7 gas blips;
8 BSD 0x418 semantics vs speed; 9 TX 0x316 gaps; 10 onroadEvents histogram.
"""
import sys, os, glob, collections, statistics as st
from bisect import bisect_left
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from openpilot.tools.lib.logreader import LogReader
from opendbc.can.parser import CANParser
from opendbc.car.byd.values import DBC, CAR
from opendbc.car import Bus

cp = CANParser(DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.pt], [(0x318, 50), (0x418, 10)], 0)

def seg_analyze(f):
  cs_t, cc_t, cc_lat, cc_tq = [], [], [], []
  cs = collections.defaultdict(list)
  eps_rows, bsd_rows, gps_t, gps_v = [], [], [], []
  events = collections.Counter()
  tx790, lat_off_t = [], []
  lat_on = False
  for m in LogReader(f):
    w = m.which(); t = m.logMonoTime / 1e9
    if w == "can":
      pairs = [(c.address, c.dat, c.src) for c in m.can]
      cp.update([(m.logMonoTime, pairs)])
      vl = cp.vl
      if any(p[0] == 0x318 for p in pairs):
        e = vl["ACC_EPS_STATE"]
        eps_rows.append((t, int(e["CruiseActivated"]), int(e["SteerWarning"]),
                         int(e["SteerErrorCode"]), int(e["TorqueFailed"])))
      if any(p[0] == 0x418 for p in pairs):
        b = vl["BSD_RADAR"]
        bsd_rows.append((t, int(b["LEFT_APPROACH"]), int(b["RIGHT_APPROACH"])))
    elif w == "carState":
      s = m.carState
      cs_t.append(t)
      cs["vEgo"].append(s.vEgo); cs["angle"].append(s.steeringAngleDeg)
      cs["drv"].append(s.steeringTorque); cs["brake"].append(s.brakePressed)
      cs["gas"].append(s.gasPressed); cs["pressed"].append(s.steeringPressed)
      cs["enabled"].append(s.cruiseState.enabled)
      cs["fault"].append(s.steerFaultPermanent or s.steerFaultTemporary)
      cs["cluster"].append(s.vEgoCluster)
    elif w == "carControl":
      c = m.carControl
      if lat_on and not c.latActive:
        lat_off_t.append(t)
      cc_t.append(t); cc_lat.append(c.latActive); cc_tq.append(c.actuators.torque)
      lat_on = c.latActive
    elif w == "sendcan":
      if lat_on:
        for p in m.sendcan:
          if p.address == 790: tx790.append(t)
    elif w == "onroadEvents":
      for e in m.onroadEvents: events[str(e.name)] += 1
    elif w == "gpsLocationExternal":
      g = m.gpsLocationExternal
      sp = g.vEgo if hasattr(g, "vEgo") else g.speed
      if 0.2 < sp < 90:
        gps_t.append(t); gps_v.append(sp)
  return dict(cs_t=cs_t, cs=cs, cc_t=cc_t, cc_lat=cc_lat, cc_tq=cc_tq,
              eps=eps_rows, bsd=bsd_rows, gps_t=gps_t, gps_v=gps_v, events=events,
              tx=tx790, lat_off=lat_off_t)

def at(col_ts, col, t):
  i = max(0, min(bisect_left(col_ts, t), len(col_ts)-1))
  return col[i]

agg_events = collections.Counter()
engage_lat = []; c0_frames = 0; c0_rail = 0; c1_rail = 0
straight_windows = 0; flip_counts = []
falls = collections.Counter(); warn_eps = []; eps_vals = collections.Counter(); tf_rises = 0
esp_ratios = []; cl_ratios = []; gas_blip = 0; gas_all = 0
bsd_frames = bsd_true = bsd_parked = 0; bsd_ge2 = []; bsd_parked_ge2 = 0
tx_gaps = []; segs = 0; hours = 0.0; latactive_s = 0.0

files = sorted(glob.glob(sys.argv[1] + "/**/rlog.zst", recursive=True))
if len(sys.argv) > 2: files = files[:int(sys.argv[2])]

for f in files:
  d = seg_analyze(f)
  segs += 1
  cs_t, cs = d["cs_t"], d["cs"]
  if cs_t: hours += (cs_t[-1] - cs_t[0]) / 3600.0
  cc_t, cc_lat, cc_tq, eps = d["cc_t"], d["cc_lat"], d["cc_tq"], d["eps"]
  eps_t = [r[0] for r in eps]
  lat_on_idx = [i for i, v in enumerate(cc_lat) if v]
  latactive_s += len(lat_on_idx) * 0.05
  for i in lat_on_idx:
    if i and not cc_lat[i-1]:
      j = bisect_left(eps_t, cc_t[i])
      while j < len(eps) and eps[j][0] < cc_t[i] + 20:
        if eps[j][1] == 1:
          engage_lat.append(eps[j][0] - cc_t[i]); break
        j += 1
    jj = max(0, bisect_left(eps_t, cc_t[i]) - 1)
    c_now = eps[jj][1] if eps else 0
    if c_now == 0:
      c0_frames += 1
      if abs(cc_tq[i]) > 0.8: c0_rail += 1
    elif abs(cc_tq[i]) > 0.8:
      c1_rail += 1
  for i in range(10, len(cc_t)):
    if not cc_lat[i]: continue
    lo = bisect_left(cc_t, cc_t[i] - 0.5)
    if lo >= i or not all(cc_lat[k] for k in range(lo, i+1)): continue
    angs = [at(cs_t, cs["angle"], cc_t[k]) for k in range(lo, i+1)]
    angs = [a for a in angs if a is not None]
    if not angs or max(angs)-min(angs) > 0.6 or abs(sum(angs)/len(angs)) > 3: continue
    straight_windows += 1
    tqs = [cc_tq[k] for k in range(lo, i+1) if abs(cc_tq[k]) > 0.05]
    flip_counts.append(sum(1 for a, b in zip(tqs, tqs[1:]) if a*b < 0))
  for i in range(1, len(cs_t)):
    if cs["enabled"][i-1] and not cs["enabled"][i]:
      k = ("brake" if cs["brake"][i] else "steer" if cs["pressed"][i] else
           "fault" if cs["fault"][i] else "gas" if cs["gas"][i] else "other")
      falls[k] += 1
  for r in eps: eps_vals[r[3]] += 1
  for i in range(1, len(eps)):
    if eps[i][2] and not eps[i-1][2]:
      warn_eps.append((round(at(cs_t, cs["vEgo"], eps[i][0]), 1), int(at(cs_t, cs["drv"], eps[i][0] or 0))))
    if eps[i][4] and not eps[i-1][4]: tf_rises += 1
  gt, gv = d["gps_t"], d["gps_v"]
  if gt:
    for i in range(0, len(cs_t), 10):
      v = cs["vEgo"][i]
      if v > 5:
        j = min(bisect_left(gt, cs_t[i]), len(gt)-1)
        if abs(gt[j]-cs_t[i]) < 0.25 and gv[j] > 5:
          esp_ratios.append(v/gv[j]); cl_ratios.append(cs["cluster"][i]/gv[j])
  run = 0
  for i in range(1, len(cs_t)):
    if cs["gas"][i] and not cs["gas"][i-1]: run = cs_t[i]
    elif not cs["gas"][i] and run:
      gas_all += 1
      if cs_t[i]-run < 0.15: gas_blip += 1
      run = 0
  for t, L, R in d["bsd"]:
    bsd_frames += 1
    if L or R:
      bsd_true += 1
      v = at(cs_t, cs["vEgo"], t)
      if v < 0.5: bsd_parked += 1
      if L >= 2 or R >= 2:
        bsd_ge2.append(round(v, 1))
        if v < 0.5: bsd_parked_ge2 += 1
  tx = d["tx"]
  lo = d["lat_off"]
  for a, b in zip(tx, tx[1:]):
    if b-a > 0.1 and not any(a < x < b for x in lo):  # hole strictly inside a continuous latActive window
      tx_gaps.append(b-a)
  agg_events.update(d["events"])

def q(xs, ps=(50, 90, 99)):
  if not xs: return []
  s = sorted(xs)
  out = []
  for p in ps:
    k = min(len(s)-1, max(0, int(round((p/100)*(len(s)-1)))))
    out.append(round(s[k], 3))
  return out
print(f"segments {segs} | onroad {hours:.1f} h | latActive {latactive_s/3600:.2f} h")
print(f"1 engage->c=1 s: n={len(engage_lat)} p50/90/99={q(engage_lat)} max={round(max(engage_lat),2) if engage_lat else '-'}")
print(f"2 latActive frames with c=0: {c0_frames} ({100*c0_frames/max(latactive_s*20,1):.1f}% of latActive), rail@ c=0: {c0_rail}, rail@ c=1: {c1_rail}")
print(f"3 straight 0.5s windows: {straight_windows}; flips/window p50/90/99={q(flip_counts)}")
print(f"4 enabled-fall causes: {dict(falls)}")
print(f"5 SteerWarning episodes n={len(warn_eps)} (v_km/s? raw m/s, drv) first20={warn_eps[:20]}")
print(f"  SteerErrorCode hist={dict(eps_vals)}; TorqueFailed rises={tf_rises}")
print(f"6 vEgo/GPS n={len(esp_ratios)} p50/90={q(esp_ratios,(50,90))} mean={round(st.mean(esp_ratios),3) if esp_ratios else '-'} | cluster/GPS p50={q(cl_ratios,(50,))}")
print(f"7 gas presses n={gas_all}, <0.15s blips={gas_blip}")
print(f"8 BSD frames={bsd_frames} true={bsd_true} ({100*bsd_true/max(bsd_frames,1):.1f}%) parked(v<0.5m/s)={bsd_parked} | value>=2 n={len(bsd_ge2)} parked among them={bsd_parked_ge2} v samples={bsd_ge2[:15]}")
print(f"9 TX 0x316 gaps>0.1s: n={len(tx_gaps)} p50/90/99={q(sorted(tx_gaps))}")
print("10 events:", agg_events.most_common(18))
