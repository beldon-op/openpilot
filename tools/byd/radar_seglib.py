"""Shared loader for the radar_seg_extract.py cache (see that file's docstring).

All times are logMonoTime ns. HUD class coding matches radar_0109_validate:
cls = HasLead * LeadingDistance (0 when no lead); classes {0,2,3,4}.
"""
import pickle
import numpy as np

SEC = 1_000_000_000
ADDRS = (0x101, 0x102, 0x105, 0x106, 0x109, 0x10A)


def load(path="/tmp/seg1005.pkl"):
  with open(path, "rb") as fp:
    return pickle.load(fp)


def vego_at(t, seg):
  """vEgo interpolated onto the (sorted) t array."""
  cs_t, cs_v = seg["cs_t"], seg["cs_v"]
  if len(cs_t) < 2:
    return np.full(len(t), np.nan, np.float64)
  return np.interp(t, cs_t, cs_v, left=np.nan, right=np.nan)


def cls_at(t, seg):
  """HUD class at nearest 0x32D frame within +-0.5 s, else -1 (unclassified)."""
  ht, hc = seg["hud_t"], seg["hud_cls"]
  out = np.full(len(t), -1, np.int16)
  if len(ht) == 0:
    return out
  i = np.clip(np.searchsorted(ht, t), 1, len(ht) - 1)
  lo, hi = ht[i - 1], ht[i]
  pick = np.where(t - lo <= hi - t, i - 1, i)
  d = np.abs(ht[pick] - t)
  ok = d <= SEC // 2
  out[ok] = hc[pick[ok]]
  return out


def pres_at(t, seg):
  """0x10A presence nibble b3[7:5] at nearest 0x10A frame within +-0.5 s, else -1."""
  pt, pd = seg["10a_t"], seg["10a_d"]   # extract keys are lowercase hex (f"{0x10A:03x}"=="10a")
  out = np.full(len(t), -1, np.int16)
  if len(pt) == 0:
    return out
  i = np.clip(np.searchsorted(pt, t), 1, len(pt) - 1)
  pick = np.where(t - pt[i - 1] <= pt[i] - t, i - 1, i)
  ok = np.abs(pt[pick] - t) <= SEC // 2
  out[ok] = (pd[pick[ok], 3] >> 5) & 0x07
  return out
