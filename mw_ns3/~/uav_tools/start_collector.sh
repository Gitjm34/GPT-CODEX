#!/usr/bin/env bash
set -euo pipefail

mkdir -p "$HOME/uav_logs"
pkill -f "collector.py|uvicorn" 2>/dev/null || true

nohup python3 "$HOME/collector.py" >"$HOME/uav_logs/collector.log" 2>&1 &
sleep 1
curl -s "http://127.0.0.1:8080/obs/latest?k=1" >/dev/null || true
echo "[collector up] http://0.0.0.0:8080"
