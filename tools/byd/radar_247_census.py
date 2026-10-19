"""247 sentinel census on /tmp/seg1005.pkl (risk #2 close-out attempt).

Question: is 0x109 b7==247 a full-scale/saturation code (distance >= max),
an error code, or a real 247th step? Evidence from the archive:
  a) high-tail profile: frame counts per value 238..247 (a real quantity tapers;
     a saturation peg piles up at the cap)
  b) episodes of consecutive b7==247 (>=2 frames): context (vEgo, class,
     presence, b6) and EXIT paths - does b7 descend CONTIGUOUSLY 247->246->...
     (saturation un-pegging) or JUMP (247->1xx, code boundary)?
  c) while pinned at 247, does b7 stay for long stretches at high vEgo far
     from lead (approaching a distant queue = physical saturation), and do
     b6/other bytes still move (sensor alive, distance clipped)?
  d) entry-from-zero: first nonzero b7 after a zero run (the 1-8 hole shape).

Usage: .venv/bin/python tools/byd/radar_247_census.py [--pkl /tmp/seg1005.pkl]
       --out /tmp/radar_247.txt
"""
import sys, argparse, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import radar_seglib as L


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument("--pkl", default="/tmp/seg1005.pkl")
  ap.add_argument("--out", default="/tmp/radar_247.txt")
  a = ap.parse_args()
  cache = L.load(a.pkl)
  say_lines = []
  def say(*x):
    s = " ".join(str(v) for v in x)
    print(s, flush=True)
    say_lines.append(s)

  hi = collections.Counter()          # frames per b7 value 235..247
  ep_n = 0
  ep_pin_secs, ep_exit_contig, ep_exit_jump, ep_vlead_far = [], 0, 0, 0
  ep_details = []
  entry_first = collections.Counter() # first nonzero b7 after a zero run
  b7_at_247_pin_other = {"b6_const": 0, "b6_moves": 0}

  for seg_name, s in cache.items():
    if len(s["109_t"]) < 50:
      continue
    t = s["109_t"]; d = s["109_d"]; b7 = d[:, 7].astype(np.int32); b6 = d[:, 6]
    # (d) entry-from-zero across whole segment
    zrun = False
    for v in b7:
      if v == 0:
        zrun = True
      elif zrun:
        entry_first[int(v)] += 1
        zrun = False
    if b7[0] > 0:
      entry_first[int(b7[0])] += 1
    # (a) high tail
    for v, n in zip(*np.unique(b7, return_counts=True)):
      if v >= 235:
        hi[int(v)] += int(n)
    # (b) 247 episodes
    idx247 = np.where(b7 == 247)[0]
    if len(idx247) == 0:
      continue
    groups = np.split(idx247, np.where(np.diff(idx247) > 1)[0] + 1)
    ve = L.vego_at(t, s); cl = L.cls_at(t, s); pr = L.pres_at(t, s)
    for g in groups:
      if len(g) < 2:
        continue
      ep_n += 1
      i0, i1 = int(g[0]), int(g[-1])
      dur = (t[i1] - t[i0]) / L.SEC
      ep_pin_secs.append(dur)
      vemed = float(np.nanmedian(ve[i0:i1 + 1]))
      clmode = int(collections.Counter(cl[i0:i1 + 1].tolist()).most_common(1)[0][0])
      prmode = int(collections.Counter(pr[i0:i1 + 1].tolist()).most_common(1)[0][0])
      b6seg = b6[i0:i1 + 1]
      if len(b6seg) > 3 and (b6seg.max() - b6seg.min()) >= 2:
        b7_at_247_pin_other["b6_moves"] += 1
      else:
        b7_at_247_pin_other["b6_const"] += 1
      # exit path
      after = b7[i1 + 1:i1 + 4].tolist() if i1 + 1 < len(b7) else []
      before = b7[max(0, i0 - 3):i0].tolist()
      if after and after[0] == 246:
        ep_exit_contig += 1
      elif after and after[0] < 240:
        ep_exit_jump += 1
      # physical-saturation context: pin while moving fast, target classified lead
      if vemed > 8 and clmode >= 2:
        ep_vlead_far += 1
      if len(ep_details) < 40:
        ep_details.append((seg_name, round(dur, 1), round(vemed, 1), clmode, prmode,
                           before, after, int(b6[i0]), int(b6seg.max()), int(b6seg.min())))

  say("== (a) high-tail frame counts b7=235..247 (real quantity tapers; saturation piles at cap)")
  for v in range(235, 248):
    say(f"  b7={v}: {hi.get(v, 0)}")
  tot = sum(hi.values())
  say(f"  total high-tail frames(>=235) = {tot};  b7==247 share of tail = "
      f"{hi.get(247, 0) / max(tot, 1):.1%}")

  say("\n== (b) episodes of consecutive b7==247 (>=2 frames)")
  say(f"  n_episodes = {ep_n}")
  if ep_pin_secs:
    ps = sorted(ep_pin_secs)
    say(f"  pinned duration s: med={ps[len(ps)//2]:.2f} p75={ps[3*len(ps)//4]:.2f} "
        f"p90={ps[int(len(ps)*.9)]:.2f} max={ps[-1]:.2f}")
    say(f"  exit contiguous (next frame==246): {ep_exit_contig}  exit jump (<=239): {ep_exit_jump}")
    say(f"  pin context vEgo>8 & class>=2 (approach far lead): {ep_vlead_far}/{ep_n}")
    say(f"  b6 during pin: moves(>=2 range) {b7_at_247_pin_other['b6_moves']} / "
        f"still {b7_at_247_pin_other['b6_const']}")
  say("\n  first 40 episodes [seg dur_s vEgo cls pres b7_before b7_after b6 b6max b6min]:")
  for e in ep_details:
    say("   ", e)

  say("\n== (d) first nonzero b7 after a zero run (entry shape)")
  top = sorted(entry_first.items())[:20]
  say("  value:count ->", top)
  nz8 = sum(n for v, n in entry_first.items() if v <= 8)
  say(f"  entries landing in 1..8 hole: {nz8} of {sum(entry_first.values())} "
      f"({nz8 / max(sum(entry_first.values()), 1):.2%})")

  say("\n== verdict hints")
  tail247 = hi.get(247, 0)
  tail246 = hi.get(246, 0)
  if tail247 and tail246:
    say(f"  247 vs 246 count ratio = {tail247 / tail246:.2f} "
        f"(>3 with long pins => peg/saturation; ~1 => ordinary top step)")
  with open(a.out, "w") as fp:
    fp.write("\n".join(say_lines) + "\n")


if __name__ == "__main__":
  main()
