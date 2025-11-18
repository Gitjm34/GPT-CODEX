#!/usr/bin/env bash
set -euo pipefail

ROS_SETUP=${ROS_SETUP:-/opt/ros/noetic/setup.bash}
NS3_PATH=${NS3_PATH:-"$HOME/ns-allinone-3.35/ns-3.35"}
MW_HOME=${MW_HOME:-"$HOME/mw_ns3"}
PYTHON_BIN=${PYTHON_BIN:-python3}
ALT_TOPIC=${ALT_TOPIC:-/mavros/global_position/rel_alt}
ALT_HZ=${ALT_HZ:-1.0}
ALT_PRECISION=${ALT_PRECISION:-2}
ALT_SCRIPT=${ALT_SCRIPT:-"$MW_HOME/alt2positions.py"}

if [[ -f "$ROS_SETUP" ]]; then
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
fi

export NS3_PATH

exec "$PYTHON_BIN" "$ALT_SCRIPT" \
  --ns3-path "$NS3_PATH" \
  --topic "$ALT_TOPIC" \
  --hz "$ALT_HZ" \
  --precision "$ALT_PRECISION"
