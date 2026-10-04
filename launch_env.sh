#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.8-carrot-bt1"
fi

export STAGING_ROOT="/data/safe_staging"

# BYD Song Plus DM-i: the DiPilot camera (MPC) faults with 'check multifunction
# video controller' when it sees the UDS/isotp firmware-query frames that card
# broadcasts on the powertrain bus during startup (relayed to bus 2 by the
# panda while still in elm327 mode). The platform is fixed via the vehicle
# selection UI (CarSelected3) and CAN auto-match, so the query is unnecessary
# - skip it entirely. card.py also sets this as a default (the env route alone
# proved unreliable on this device).
export SKIP_FW_QUERY=1
