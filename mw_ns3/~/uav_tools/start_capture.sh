#!/usr/bin/env bash
set -euo pipefail

RUN_ID=${1:-"run_$(date +%y%m%d_%H%M%S)"}
PCAP_DIR="$HOME/pcaps"
mkdir -p "$PCAP_DIR"

IFACE=${IFACE:-lo}
PORT_FILTER=${PORT_FILTER:-'udp port 14540 or udp port 14550 or udp port 14556 or udp port 14558 or udp port 14640'}

PCAP_FILE="${PCAP_DIR}/${RUN_ID}.all.pcapng"
LOG_FILE="${PCAP_DIR}/${RUN_ID}.tcpdump.log"
PID_FILE="${PCAP_DIR}/${RUN_ID}.tcpdump.pid"

echo "[start capture] run_id=${RUN_ID} iface=${IFACE}"
echo "[pcap] ${PCAP_FILE}"

sudo tcpdump -i "$IFACE" -w "$PCAP_FILE" $PORT_FILTER >"$LOG_FILE" 2>&1 &
echo $! >"$PID_FILE"
echo "[pid] $(cat "$PID_FILE")"
