#!/usr/bin/env bash
set -euo pipefail

NS3_PATH=${NS3_PATH:-"$HOME/ns-allinone-3.35/ns-3.35"}
MW_HOME=${MW_HOME:-"$HOME/mw_ns3"}
LOG_DIR=${LOG_DIR:-"$HOME/.uav_ids"}
PYTHON_BIN=${PYTHON_BIN:-python3}
POSITIONS_FILE=${POSITIONS_FILE:-"$NS3_PATH/positions.txt"}
QGC_UPLINK_PORT=${QGC_UPLINK_PORT:-14640}
QGC_DOWNLINK_PORT=${QGC_DOWNLINK_PORT:-14550}
PX4_HOST=${PX4_HOST:-127.0.0.1}
PX4_CMD_PORT=${PX4_CMD_PORT:-14540}
PX4_TELEMETRY_PORT=${PX4_TELEMETRY_PORT:-14550}
METRICS_INTERVAL=${METRICS_INTERVAL:-1.0}
METRICS_TIMEOUT=${METRICS_TIMEOUT:-4.0}

mkdir -p "$LOG_DIR"
export NS3_PATH

exec "$PYTHON_BIN" "$MW_HOME/udp_mw_ns3.py" \
  --ns3-path "$NS3_PATH" \
  --positions-file "$POSITIONS_FILE" \
  --log-dir "$LOG_DIR" \
  --qgc-uplink-port "$QGC_UPLINK_PORT" \
  --qgc-downlink-port "$QGC_DOWNLINK_PORT" \
  --px4-host "$PX4_HOST" \
  --px4-cmd-port "$PX4_CMD_PORT" \
  --px4-telemetry-port "$PX4_TELEMETRY_PORT" \
  --metrics-interval "$METRICS_INTERVAL" \
  --metrics-timeout "$METRICS_TIMEOUT"
