"""
Read helper for params that a stale compiled registry does not know yet.

common/params_pyx.so carries a key registry compiled from common/params_keys.h
at build time. Device deployments can run a .so older than the checked-out
params_keys.h (exactly the sunnypilot LaneChangeAssistSpeed boot crash:
Params.get() raises UnknownKeyName - not TypeError/ValueError - and the
consumer constructor dies), and the C++ getInt additionally returns 0 (not the
registry default) for a missing file, so plain get_int cannot be trusted for
new keys.

The storage layer itself is just plain files under /data/params/<prefix>/<key>,
reachable through Params.get_param_path() (which does NOT validate the key), so
the param can be provisioned on-device by writing the file directly, e.g.
`echo 0 > /data/params/d/LaneChangeAssistSpeed`. get_int_param() prefers the
registered path (applying the registry default via return_default=True) and
falls back to that raw file, so new keys are configurable before the .so
catches up; once a rebuilt .so registers the key, the Params.get() path wins
and the fallback goes unused.

Caveats while a key is unregistered: Params.put()/remove()/clearAll() and
backup all skip it, so the raw file is never managed automatically.
Ported from sunnypilot 31284410c8 (sunnypilot/common/raw_params.py).
"""
from openpilot.common.params import Params, UnknownKeyName


def get_int_param(params: Params, key: str, default: int) -> int:
  try:
    return int(params.get(key, return_default=True))
  except UnknownKeyName:
    pass
  except (TypeError, ValueError):
    return default
  try:
    with open(params.get_param_path(key)) as f:
      val = f.read().strip()
    return int(val) if val else default
  except (OSError, ValueError):
    return default
