"""Layout-free radar RE mining on bus1 of docs_site/hour_logs rlogs.

Forget IsValid / forget guessed DBC offsets: the only ground truth we trust is
the 0x32D HUD (bus2) following-distance bar, which is a discrete proxy for
lead range (2=near .. 4=far; 0=no lead displayed). For every bus1 message and
every payload byte we compute:

  A. MI(byte ; bar-class)   - per segment, averaged. Wrapping-safe (histogram
     based, not mean based). Rolling counters -> ~0; real distance fields -> >0.
  B. change-point hits      - byte value changes within +/-0.5 s of a HUD
     bar transition, vs the byte's baseline change rate.
  C. class histograms       - top values per class, printed for the winners so
     a human can read off scale/offset.

Usage:
  .venv/bin/python tools/byd/radar_re_mine.py docs_site/hour_logs/radar_rlogs
"""
import sys, os, glob, math, bisect, collections
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from openpilot.tools.lib.logreader import LogReader
from opendbc.can import CANParser
from opendbc.car import Bus, CanData
from opendbc.car.byd.values import DBC, CAR

HUD_MSG = 0x32D
SEC = 1_000_000


def _hexp(d):
  return bytes(d).hex()


def entropy(counter, total):
  h = 0.0
  for v in counter.values():
    p = v / total
    h -= p * math.log2(p)
  return h


def mi_bytes(x, y):
  """MI in bits between two discrete series (same length)."""
  joint = collections.Counter(zip(x, y))
  n = len(x)
  if n == 0:
    return 0.0
  cx, cy = collections.Counter(x), collections.Counter(y)
  hx, hy = entropy(cx, n), entropy(cy, n)
  hxy = entropy(joint, n)
  return max(hx + hy - hxy, 0.0)


def analyze(f):
  dbc = DBC[CAR.BYD_SONG_PLUS_DMI_22][Bus.radar]
  hp = CANParser(dbc, [(HUD_MSG, 50)], 2)
  hud_t, hud_cls = [], []           # class timeline from HUD
  cls_prev = None
  transitions = []                  # (t_us, old, new) bar transitions while lead
  frames = []                       # (t_us, addr, bytes) all bus1
  for m in LogReader(f):
    if m.which() != "can":
      continue
    hf = []
    for c in m.can:
      if c.src == 1:
        frames.append((m.logMonoTime, c.address, bytes(c.dat)))
      elif c.src == 2 and c.address == HUD_MSG:
        hf.append(CanData(c.address, bytes(c.dat), c.src))
    if hf:
      hp.update([(m.logMonoTime, hf)])
      v = hp.vl[HUD_MSG]
      h, b = int(v["HasLead"]), int(v["LeadingDistance"])
      cls = b if h else 0
      if cls_prev is not None and cls != cls_prev and cls_prev != 0 and cls != 0:
        transitions.append((m.logMonoTime, cls_prev, cls))
      hud_t.append(m.logMonoTime)
      hud_cls.append(cls)
      cls_prev = cls

  # pair every bus1 frame with its class (nearest HUD sample, <=0.5 s window)
  per_cls = collections.defaultdict(lambda: collections.defaultdict(list))  # addr -> byte_idx -> [(cls,val)]
  n_tot = collections.Counter()
  trans_set = [t for t, _, _ in transitions]
  for t, a, d in frames:
    i = bisect.bisect_left(hud_t, t)
    cls = None
    for j in (i - 1, i):
      if 0 <= j < len(hud_t) and abs(hud_t[j] - t) <= 0.5 * SEC:
        cls = hud_cls[j]
        break
    n_tot[a] += 1
    for b_idx, val in enumerate(d):
      if cls is not None:
        per_cls[a][b_idx].append((cls, val))
  # change stats per (addr,byte): iterate frames in time order
  byte_prev = {}
  chg_tot = collections.Counter()
  chg_hit = collections.Counter()
  frames.sort(key=lambda x: x[0])
  for t, a, d in frames:
    for b_idx, val in enumerate(d):
      key = (a, b_idx)
      bp = byte_prev.get(key)
      if bp is not None and bp != val:
        chg_tot[key] += 1
        lo = bisect.bisect_left(trans_set, t - 0.5 * SEC)
        if lo < len(trans_set) and trans_set[lo] <= t + 0.5 * SEC:
          chg_hit[key] += 1
      byte_prev[key] = val

  mi = {}
  for a, rows in per_cls.items():
    for b_idx, pairs in rows.items():
        if len(pairs) < 200:
          continue
        x = [v for _, v in pairs]
        y = [c for c, _ in pairs]
        mi[(a, b_idx)] = mi_bytes(x, y)
  return {"seg": os.path.basename(os.path.dirname(f)), "mi": mi,
          "chg": chg_tot, "hit": chg_hit, "n": n_tot,
          "per_cls": {a: {b: v for b, v in rows.items()} for a, rows in per_cls.items()},
          "transitions": transitions}


def main():
  segs = sorted(glob.glob(sys.argv[1] + "/*/rlog.zst"))
  res = [analyze(f) for f in segs]
  all_keys = set()
  for r in res:
    all_keys |= set(r["mi"])
  # average MI across segments where the addr appears
  rows = []
  for k in all_keys:
    mis = [r["mi"][k] for r in res if k in r["mi"]]
    hits = sum(r["hit"].get(k, 0) for r in res)
    chg = sum(r["chg"].get(k, 0) for r in res)
    tot = sum(r["n"].get(k[0], 0) for r in res)
    trans_n = sum(len(r["transitions"]) for r in res)
    rows.append((k, sum(mis) / len(mis), hits, chg, tot, len(mis)))
  rows.sort(key=lambda x: -x[1])
  print(f"# segments={len(res)} total_bar_transitions={sum(len(r['transitions']) for r in res)}")
  print("addr:byte   MI(avg)  hits/chg  change_rate   segs   note")
  for (a, b), m, hits, chg, tot, ns in rows[:25]:
    rate = chg / max(tot, 1)
    note = "counter-ish" if rate > 0.5 else ""
    print(f"{a:08x}:{b}    {m:6.4f}   {hits:>4}/{chg:<5}  {rate:6.2%}        {ns}   {note}")
  print("\n# top-6 by MI: class histograms (val:count per class 0/2/3/4)")
  for (a, b), m, *_ in rows[:6]:
    if m < 0.003:
      break
    for r in res:
      pairs = r["per_cls"].get(a, {}).get(b)
      if not pairs:
        continue
      cls_vals = collections.defaultdict(collections.Counter)
      for c, v in pairs:
        cls_vals[c][v] += 1
      hs = " | ".join(f"c{c}: {[f'{v:02x}x{n}' for v, n in cv.most_common(3)]}"
                      for c, cv in sorted(cls_vals.items()))
      print(f"  {r['seg']} {a:08x}:{b}  {hs}")
  print("\n# transitions")
  for r in res:
    print(f"  {r['seg']}: {r['transitions']}")


if __name__ == "__main__":
  main()
