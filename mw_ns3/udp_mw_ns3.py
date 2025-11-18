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

DEFAULT_DELAY_MS = 20.0
DEFAULT_LOSS_PCT = 0.0
DEFAULT_RATE_KBPS = 5000.0
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
    ) -> None:
        super().__init__(daemon=True)
        self._log_path = log_path
        self._stats = stats
        self._metrics = metrics_state
        self._stop = stop_event

    def run(self) -> None:
        while not self._stop.is_set():
            self._stop.wait(1.0)
            up, down = self._stats.snapshot()
            metrics = self._metrics.snapshot()
            entry = {
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

            print(
                f"[MW] up:{up}B down:{down}B | delay={metrics.delay_ms:.1f}ms "
                f"loss={metrics.loss_pct:.1f}% rate={metrics.rate_kbps:.0f}kbps",
                flush=True,
            )


class QgcEndpoint:
    def __init__(self, host: Optional[str], port: int, follow_host: bool) -> None:
        self._host = host
        self._port = port
        self._follow = follow_host
        self._lock = threading.Lock()

    def learn(self, host: str) -> None:
        if not self._follow or not host:
            return
        with self._lock:
            if self._host != host:
                self._host = host

    def target(self) -> Optional[Tuple[str, int]]:
        with self._lock:
            if not self._host:
                return None
            return (self._host, self._port)


def apply_impairments(
    sock: socket.socket,
    data: bytes,
    dest: Tuple[str, int],
    direction: str,
    stats: FlowStats,
    metrics_state: ThreadSafeMetrics,
) -> None:
    metrics = metrics_state.snapshot()
    if metrics.loss_pct > 0 and random.random() < (metrics.loss_pct / 100.0):
        return

    if metrics.delay_ms > 0:
        time.sleep(metrics.delay_ms / 1000.0)

    bytes_per_second = max(metrics.rate_kbps, 1.0) * 125.0  # kbps -> bytes/sec
    transmit_time = len(data) / bytes_per_second
    if transmit_time > 0:
        time.sleep(transmit_time)

    sock.sendto(data, dest)
    stats.add(direction, len(data))


def uplink_loop(
    sock: socket.socket,
    px4_addr: Tuple[str, int],
    qgc_endpoint: QgcEndpoint,
    stats: FlowStats,
    metrics_state: ThreadSafeMetrics,
    stop_event: threading.Event,
) -> None:
    print(f"[UPLINK] listening on {sock.getsockname()} -> {px4_addr}")
    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(MAX_DATAGRAM)
        except socket.timeout:
            continue
        except OSError:
            break
        qgc_endpoint.learn(addr[0])
        apply_impairments(sock, data, px4_addr, "up", stats, metrics_state)


def downlink_loop(
    sock: socket.socket,
    qgc_endpoint: QgcEndpoint,
    stats: FlowStats,
    metrics_state: ThreadSafeMetrics,
    stop_event: threading.Event,
) -> None:
    print(f"[DOWNLINK] listening on {sock.getsockname()}")
    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(MAX_DATAGRAM)
        except socket.timeout:
            continue
        except OSError:
            break

        dest = qgc_endpoint.target()
        if not dest:
            continue
        apply_impairments(sock, data, dest, "down", stats, metrics_state)


def build_socket(bind_host: str, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((bind_host, port))
    sock.settimeout(SOCKET_TIMEOUT)
    return sock


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="ns-3 aware PX4/QGC middleware")
    parser.add_argument(
        "--ns3-path",
        default=os.environ.get("NS3_PATH", "~/ns-allinone-3.35/ns-3.35"),
        help="Path to the ns-3 source tree (default: %(default)s)",
    )
    parser.add_argument(
        "--positions-file",
        default=None,
        help="Override path to positions.txt (default: <ns3-path>/positions.txt)",
    )
    parser.add_argument(
        "--metrics-interval",
        type=float,
        default=1.0,
        help="Seconds between ns-3 polls (default: %(default)s)",
    )
    parser.add_argument(
        "--metrics-timeout",
        type=float,
        default=4.0,
        help="Seconds to wait for ns-3 output (default: %(default)s)",
    )
    parser.add_argument(
        "--qgc-bind-host",
        default="0.0.0.0",
        help="Host for the QGC uplink socket (default: %(default)s)",
    )
    parser.add_argument(
        "--qgc-uplink-port",
        type=int,
        default=14640,
        help="Port that QGC connects to (default: %(default)s)",
    )
    parser.add_argument(
        "--qgc-downlink-host",
        default="127.0.0.1",
        help="Destination host for telemetry toward QGC (default: %(default)s)",
    )
    parser.add_argument(
        "--qgc-downlink-port",
        type=int,
        default=14550,
        help="Destination port for telemetry toward QGC (default: %(default)s)",
    )
    parser.add_argument(
        "--px4-host",
        default="127.0.0.1",
        help="PX4 SITL host (default: %(default)s)",
    )
    parser.add_argument(
        "--px4-cmd-port",
        type=int,
        default=14540,
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

    poller = MetricsPoller(
        ns3_path, positions_file, args.metrics_interval, args.metrics_timeout, metrics_state, stop_event
    )
    flow_logger = FlowLogger(log_path, stats, metrics_state, stop_event)

    uplink_sock = build_socket(args.qgc_bind_host, args.qgc_uplink_port)
    downlink_sock = build_socket(args.px4_bind_host, args.px4_telemetry_port)
    px4_addr = (args.px4_host, args.px4_cmd_port)

    uplink_thread = threading.Thread(
        target=uplink_loop,
        args=(uplink_sock, px4_addr, qgc_endpoint, stats, metrics_state, stop_event),
        daemon=True,
    )
    downlink_thread = threading.Thread(
        target=downlink_loop,
        args=(downlink_sock, qgc_endpoint, stats, metrics_state, stop_event),
        daemon=True,
    )

    poller.start()
    flow_logger.start()
    uplink_thread.start()
    downlink_thread.start()

    try:
        while uplink_thread.is_alive() and downlink_thread.is_alive():
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n[MW] shutting down...")
    finally:
        stop_event.set()
        uplink_sock.close()
        downlink_sock.close()
        uplink_thread.join()
        downlink_thread.join()
        poller.join()
        flow_logger.join()


if __name__ == "__main__":
    main()
