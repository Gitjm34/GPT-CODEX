#!/usr/bin/env bash
set -euo pipefail

RUN_ID=${1:-}

if [[ -z "$RUN_ID" ]]; then
  latest=$(ls -1t "$HOME/pcaps"/*.tcpdump.pid 2>/dev/null | head -n1 || true)
  if [[ -z "$latest" ]]; then
    echo "Usage: $0 RUN_ID (no pidfile found and no RUN_ID given)" >&2
    exit 1
  fi
  RUN_ID=$(basename "$latest" .tcpdump.pid)
  echo "[info] RUN_ID omitted; using latest: $RUN_ID"
fi

PID_FILE="$HOME/pcaps/${RUN_ID}.tcpdump.pid"
if [[ ! -f "$PID_FILE" ]]; then
  echo "PID file not found: $PID_FILE" >&2
  exit 1
fi

TCPDUMP_PID=$(cat "$PID_FILE")
echo "[stop] killing tcpdump pid=$TCPDUMP_PID"
sudo kill "$TCPDUMP_PID" 2>/dev/null || true
sleep 1
sudo sync
echo "[pcap saved] $HOME/pcaps/${RUN_ID}.all.pcapng"
