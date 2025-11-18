#!/usr/bin/env python3
"""
UDP middleware that relays MAVLink packets between QGC and PX4 while applying
delay/loss/rate metrics calculated by an ns-3 helper program.
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import random
import shlex
import socket
import subprocess
import threading
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import requests

DEFAULT_DELAY_MS = 20.0
DEFAULT_LOSS_PCT = 0.0
DEFAULT_RATE_KBPS = 5000.0
DEFAULT_IDS_TIMEOUT = 0.8
SOCKET_TIMEOUT = 0.5
MAX_DATAGRAM = 65535


@dataclass(frozen=True)
class LinkMetrics:
    delay_ms: float
    loss_pct: float
    rate_kbps: float


class ThreadSafeMetrics:
    def __init__(self, initial: LinkMetrics) -> None:
        self._value = initial
        self._lock = threading.Lock()

    def snapshot(self) -> LinkMetrics:
        with self._lock:
            return self._value

    def update(self, value: LinkMetrics) -> None:
        with self._lock:
            self._value = value


class FlowStats:
    def __init__(self) -> None:
        self._up = 0
        self._down = 0
        self._lock = threading.Lock()

    def add(self, direction: str, length: int) -> None:
        with self._lock:
            if direction == "up":
                self._up += length
            else:
                self._down += length

    def snapshot(self) -> Tuple[int, int]:
        with self._lock:
            up, down = self._up, self._down
            self._up = 0
            self._down = 0
            return up, down


class MetricsPoller(threading.Thread):
    def __init__(
        self,
        ns3_path: pathlib.Path,
        positions_file: pathlib.Path,
        interval: float,
        timeout: float,
        metrics_state: ThreadSafeMetrics,
        stop_event: threading.Event,
    ) -> None:
        super().__init__(daemon=True)
        self._ns3_path = ns3_path
        self._positions = positions_file
        self._interval = max(0.2, interval)
        self._timeout = max(1.0, timeout)
        self._state = metrics_state
        self._stop = stop_event

    def run(self) -> None:
        while not self._stop.is_set():
            metrics = self._invoke_ns3()
            if metrics:
                self._state.update(metrics)
            self._stop.wait(self._interval)

    def _invoke_ns3(self) -> Optional[LinkMetrics]:
        if not self._ns3_path.exists():
            print(f"[NS3] path {self._ns3_path} not found; keeping previous metrics")
            return None

        run_target = "scratch/mw-link-metrics"
        if self._positions:
            quoted = shlex.quote(str(self._positions))
            run_target += f" --positions={quoted}"

        cmd = ["./waf", "--run", run_target]
        try:
            result = subprocess.run(
                cmd,
                cwd=str(self._ns3_path),
                capture_output=True,
                text=True,
                timeout=self._timeout,
                check=True,
            )
        except subprocess.CalledProcessError as exc:
            print(f"[NS3] command failed ({exc.returncode}): {exc.stderr.strip()}")
            return None
        except subprocess.TimeoutExpired:
            print(f"[NS3] command timed out after {self._timeout}s")
            return None

        delay = loss = rate = None
        for line in result.stdout.splitlines():
            line = line.strip()
            if not line:
                continue
            if line.startswith("DELAY_MS:"):
                delay = float(line.split(":", 1)[1])
            elif line.startswith("LOSS_PCT:"):
                loss = float(line.split(":", 1)[1])
            elif line.startswith("RATE_KBPS:"):
                rate = float(line.split(":", 1)[1])

        if delay is None or loss is None or rate is None:
            print("[NS3] missing metrics in output; ignoring frame")
            return None

        return LinkMetrics(delay_ms=delay, loss_pct=loss, rate_kbps=rate)


class FlowLogger(threading.Thread):
    def __init__(
        self,
        log_path: pathlib.Path,
        stats: FlowStats,
        metrics_state: ThreadSafeMetrics,
        stop_event: threading.Event,
        run_id: str,
        node_id: str,
        ids_endpoint: Optional[str],
        ids_timeout: float,
    ) -> None:
        super().__init__(daemon=True)
        self._log_path = log_path
        self._stats = stats
        self._metrics = metrics_state
        self._stop = stop_event
        self._run_id = run_id
        self._node_id = node_id
        self._ids_endpoint = ids_endpoint
        self._ids_timeout = max(0.1, ids_timeout)
        self._seq = 0
        self._session = requests.Session() if ids_endpoint else None

    def run(self) -> None:
        while not self._stop.is_set():
            self._stop.wait(1.0)
            up, down = self._stats.snapshot()
            metrics = self._metrics.snapshot()
            self._seq += 1
            entry = {
                "run_id": self._run_id,
                "seq": self._seq,
                "node_id": self._node_id,
                "ts": time.time(),
                "up_bytes": up,
                "down_bytes": down,
                "delay_ms": metrics.delay_ms,
                "loss_pct": metrics.loss_pct,
                "rate_kbps": metrics.rate_kbps,
            }
            try:
                with self._log_path.open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps(entry) + "\n")
            except OSError as exc:
                print(f"[MW] unable to write log {self._log_path}: {exc}")

            if self._ids_endpoint:
                try:
                    session = self._session or requests
                    session.post(
                        self._ids_endpoint,
                        json=entry,
                        timeout=self._ids_timeout,
                    )
                except requests.RequestException as exc:
                    print(f"[IDS] POST to {self._ids_endpoint} failed: {exc}")

            print(
                f"[MW] up:{up}B down:{down}B | delay={metrics.delay_ms:.1f}ms "
                f"loss={metrics.loss_pct:.1f}% rate={metrics.rate_kbps:.0f}kbps",
                flush=True,
            )
          help="PX4 command port (default: %(default)s)",
    )
    parser.add_argument(
        "--px4-telemetry-port",
        type=int,
        default=14550,
        help="PX4 telemetry port (default: %(default)s)",
    )
    parser.add_argument(
        "--px4-bind-host",
        default="0.0.0.0",
        help="Host to bind for PX4 telemetry interception (default: %(default)s)",
    )
    parser.add_argument(
        "--log-dir",
        default="~/.uav_ids",
        help="Directory for flow.jsonl (default: %(default)s)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Optional RNG seed for loss simulation",
    )
    parser.add_argument(
        "--static-qgc-host",
        action="store_false",
        dest="follow_qgc_host",
        help="Disable auto-updating the downlink host based on incoming QGC packets",
    )
    parser.add_argument(
        "--run-id",
        default=None,
        help="Run identifier reported in logs/HTTP push (default: $RUN_ID or generated)",
    )
    parser.add_argument(
        "--node-id",
        default=None,
        help="Override node identifier (default: hostname)",
    )
    parser.add_argument(
        "--ids-endpoint",
        default=os.environ.get("IDS_ENDPOINT"),
        help="Optional HTTP endpoint for pushing metrics (default: %(default)s)",
    )
    parser.add_argument(
        "--ids-timeout",
        type=float,
        default=float(os.environ.get("IDS_TIMEOUT", str(DEFAULT_IDS_TIMEOUT))),
        help="Timeout (seconds) for HTTP pushes (default: %(default)s)",
    )
    parser.set_defaults(follow_qgc_host=True)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.seed is not None:
        random.seed(args.seed)

    ns3_path = pathlib.Path(args.ns3_path).expanduser()
    positions_file = (
        pathlib.Path(args.positions_file).expanduser()
        if args.positions_file
        else ns3_path / "positions.txt"
    )
    log_dir = pathlib.Path(args.log_dir).expanduser()
    log_dir.mkdir(parents=True, exist_ok=True)
    log_path = log_dir / "flow.jsonl"

    metrics_state = ThreadSafeMetrics(
        LinkMetrics(DEFAULT_DELAY_MS, DEFAULT_LOSS_PCT, DEFAULT_RATE_KBPS)
    )
    stats = FlowStats()
    stop_event = threading.Event()
    qgc_endpoint = QgcEndpoint(
        args.qgc_downlink_host, args.qgc_downlink_port, args.follow_qgc_host
    )

    run_id = args.run_id or os.environ.get("RUN_ID") or time.strftime("run_%Y%m%d_%H%M%S")
    node_id = args.node_id or os.environ.get("NODE_ID") or socket.gethostname()
    ids_endpoint = (args.ids_endpoint or os.environ.get("IDS_ENDPOINT"))
    if ids_endpoint:
        ids_endpoint = ids_endpoint.strip()
        if not ids_endpoint:
            ids_endpoint = None
    ids_timeout = max(0.1, args.ids_timeout or DEFAULT_IDS_TIMEOUT)

    print(f"[MW] run_id={run_id} node_id={node_id}")
    if ids_endpoint:
        print(f"[MW] pushing metrics to {ids_endpoint} (timeout={ids_timeout:.1f}s)")

    poller = MetricsPoller(
        ns3_path, positions_file, args.metrics_interval, args.metrics_timeout, metrics_state, stop_event
    )
    flow_logger = FlowLogger(
        log_path,
        stats,
        metrics_state,
        stop_event,
        run_id,
        node_id,
        ids_endpoint,
        ids_timeout,
    )
