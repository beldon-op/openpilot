#!/usr/bin/env python3
"""Layout solver + rd_spec_table diff (byd-radar-multitarget-port task 1.3).

Reads the device probe table (radar_probe_run.py output) and the static
.sso spec dump (local_docs/rd_spec_table_2026-10-31.json), reverse-solves
(start, len, scale[, sign_extend]) per (probe group, output field, slot), and
cross-references every discovered layout against the vendor table's 91 valid
records. Output: a markdown-ish diff report with three verdicts per entry -
MATCH (both sources agree), CONFLICT (both present, disagree), ONE-SOURCE
(single side only). The vendor table has no field names (the group->message
mapping is itself an open question), so matching is by (bit_len, scale) with
tolerance and the report lists the candidates.

  .venv/bin/python tools/byd/radar_probe_solve.py /tmp/probe_table.json \
      --spec local_docs/rd_spec_table_2026-10-31.json --out /tmp/probe_diff.md
"""
import argparse
import json
import sys
from collections import defaultdict

EPS = 1e-4
SCALE_TOL = 0.02          # relative tolerance for scale matches
LEN_TOL = 1               # bit-length tolerance


def solve_field(deltas: dict[int, float]):
  """Group per-bit deltas into power-of-2 runs -> candidate (start, len,
  scale, sign_extend) layouts. A field of length L contributes
  delta(bit) = scale * 2^(bit-start) for bit < start+L (unsigned), with the
  top bit NEGATIVE when sign_extend (two's complement). Runs stop at the
  first ratio break or sign flip."""
  bits = sorted(b for b, d in deltas.items() if abs(d) > EPS)
  runs = []
  i = 0
  while i < len(bits):
    start = bits[i]
    scale = deltas[start]
    sign = scale < 0
    if sign:
      i += 1
      continue  # lone negative delta: sign-extend MSB candidate, keep for report
    j = i
    while j + 1 < len(bits) and deltas[bits[j + 1]] > EPS and \
          abs(deltas[bits[j + 1]] / deltas[bits[j]] - 2.0) < 0.05:
      j += 1
    runs.append({'start': start, 'len': bits[j] - start + 1,
                 'scale': scale, 'sign_extend': False})
    i = j + 1
  # negative deltas: sign-extend MSB candidates (pair them with a run whose
  # top bit is one above)
  for b in bits:
    d = deltas[b]
    if d < -EPS:
      runs.append({'start': b, 'len': 1, 'scale': -d, 'sign_extend': True})
  return runs


def load_spec(path):
  d = json.load(open(path))
  recs = []
  for gi, group in enumerate(d['groups']):
    for si, r in enumerate(group):
      if r['length'] > 0:
        recs.append({'group': gi, 'sig': si, 'start': r['start'], 'len': r['length'],
                     'scale': r['scale'], 'offset': r['offset'],
                     'signext': bool(r['signext']), 'flagC': r['flagC']})
  return recs


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('probe_table')
  ap.add_argument('--spec', default='local_docs/rd_spec_table_2026-10-31.json')
  ap.add_argument('--out', default='/tmp/probe_diff.md')
  a = ap.parse_args()

  tab = json.load(open(a.probe_table))
  spec = load_spec(a.spec)

  # (group, field) -> {slot: {bit: delta}} using first_delta as the canonical
  # magnitude (max_abs can smear across slots)
  obs = defaultdict(lambda: defaultdict(dict))
  for p in tab['probes']:
    if p.get('bit') is None:
      continue
    for key, c in p.get('changes', {}).items():
      if '@slot' not in key or 'RECORD_COUNT' in key:
        continue
      field, slot = key.split('@slot', 1)
      obs[(p['group'], field)][slot][p['bit']] = c['first_delta']

  lines = ['# probe layout solve + rd_spec_table diff',
           f"- probe table: {a.probe_table} (n_probes={len(tab['probes'])}, "
           f"flip_k={tab['meta'].get('flip_k')}, flip_at={tab['meta'].get('flip_at')}s)",
           f"- spec records (len>0): {len(spec)}", '']

  for (group, field), slots in sorted(obs.items()):
    lines.append(f'## {group} -> {field}')
    for slot, deltas in sorted(slots.items()):
      runs = solve_field(deltas)
      if not runs:
        continue
      for r in runs:
        # vendor candidates: same length +-tol and scale within tolerance
        cands = [s for s in spec
                 if abs(s['len'] - r['len']) <= LEN_TOL and s['scale'] > 0 and
                 abs(s['scale'] - r['scale']) <= SCALE_TOL * r['scale']]
        if r['sign_extend']:
          verdict = 'SIGN-MSB (sign_extend candidate)'
        elif not cands:
          verdict = 'ONE-SOURCE (probe only)'
        else:
          verdict = 'MATCH: ' + ', '.join(
            f"spec g{s['group']}/sig{s['sig']} @{s['start']}|{s['len']} "
            f"scale={s['scale']:g}" + (' [start differs]' if abs(s['start'] - r['start']) > 2 else '')
            for s in cands[:4])
        lines.append(f"- slot {slot}: @{r['start']}|{r['len']} "
                     f"scale={r['scale']:.6g} sign={r['sign_extend']} -> {verdict}")
    lines.append('')

  # reverse direction: spec records with no probe counterpart (bits probed but
  # silent) - only meaningful for the groups the probe covers
  probed_groups = {g for (g, _f) in obs}
  lines.append('## spec records with no probe counterpart (probed groups only)')
  covered = {(g, s['start'], s['len'], round(s['scale'], 4))
             for (g, _f), slots in obs.items() for _s, deltas in slots.items()
             for r in solve_field(deltas) if not r['sign_extend']
             for g in [g] for s in spec
             if abs(s['len'] - r['len']) <= LEN_TOL and s['scale'] > 0 and
             abs(s['scale'] - r['scale']) <= SCALE_TOL * r['scale']}
  n = 0
  for s in spec:
    if s['group'] not in probed_groups:
      continue
    if (s['group'], s['start'], s['len'], round(s['scale'], 4)) in covered:
      continue
    lines.append(f"- spec g{s['group']}/sig{s['sig']}: @{s['start']}|{s['len']} "
                 f"scale={s['scale']:g} offset={s['offset']:g} "
                 f"signext={s['signext']} -> ONE-SOURCE (spec only)")
    n += 1
  if not n:
    lines.append('- (none - every spec record in the probed groups has a probe match)')

  report = '\n'.join(lines) + '\n'
  open(a.out, 'w').write(report)
  print(report)


if __name__ == '__main__':
  main()
