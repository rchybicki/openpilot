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
  export AGNOS_VERSION="12.8"
fi

export STAGING_ROOT="/data/safe_staging"

eval "$(/data/openpilot/frogpilot/system/environment_variables)"

# Load user-provided environment overrides from persistent storage.
# This allows secrets like WEATHER_TOKEN to survive repo updates, since /data is not replaced.
if [ -f "/data/launch_env.sh" ]; then
  # shellcheck source=/dev/null
  source "/data/launch_env.sh"
  if [ -n "${WEATHER_TOKEN}" ]; then
    echo "launch_env: WEATHER_TOKEN detected from /data/launch_env.sh"
  else
    echo "launch_env: WEATHER_TOKEN not set after sourcing /data/launch_env.sh"
  fi
fi
