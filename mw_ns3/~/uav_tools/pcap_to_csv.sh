#!/usr/bin/env bash
set -euo pipefail

RUN_ID=${1:-}

if [[ -z "$RUN_ID" ]]; then
  latest=$(ls -1t "$HOME/pcaps"/*.all.pcapng 2>/dev/null | head -n1 || true)
  if [[ -z "$latest" ]]; then
    echo "Usage: $0 RUN_ID (no pcap found and no RUN_ID given)" >&2
    exit 1
  fi
  RUN_ID=$(basename "$latest" .all.pcapng)
  echo "[info] RUN_ID omitted; using latest: $RUN_ID"
fi

PCAP="$HOME/pcaps/${RUN_ID}.all.pcapng"
OUT="$HOME/pcap_csv/${RUN_ID}.basic.csv"
mkdir -p "$HOME/pcap_csv"

if [[ ! -f "$PCAP" ]]; then
  echo "no pcap: $PCAP" >&2
  exit 1
fi

tshark -r "$PCAP" -Y udp -T fields \
  -e frame.time_epoch -e ip.src -e udp.srcport -e ip.dst -e udp.dstport \
  -e frame.len -e udp.length -e data \
  -E header=y -E separator=, >"$OUT"

echo "[csv] $OUT"
