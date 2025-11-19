#!/usr/bin/env python3
"""
Client utility for controlling attackd via its UNIX socket.
"""

import argparse
import json
import os
import socket
import sys
from typing import Any, Dict


def send_command(sock_path: str, payload: Dict[str, Any]) -> Dict[str, Any]:
    data = (json.dumps(payload) + "\n").encode()
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
        sock.connect(sock_path)
        sock.sendall(data)
        resp = b""
        while not resp.endswith(b"\n"):
            chunk = sock.recv(4096)
            if not chunk:
                break
            resp += chunk
    if not resp:
        raise RuntimeError("empty response from attackd")
    return json.loads(resp.decode())


def cmd_dos(args: argparse.Namespace) -> Dict[str, Any]:
    payload = {
        "action": "dos",
        "duration": args.duration,
        "payload_bytes": args.payload_bytes,
        "pps": args.pps,
        "host": args.target_host,
        "port": args.target_port,
    }
    return send_command(args.socket, payload)


def cmd_hb(args: argparse.Namespace) -> Dict[str, Any]:
    payload = {
        "action": "hb",
        "duration": args.duration,
        "loss": args.loss,
        "iface": args.iface,
    }
    return send_command(args.socket, payload)


def cmd_state(args: argparse.Namespace) -> Dict[str, Any]:
    return send_command(args.socket, {"action": "state"})


def cmd_stop(args: argparse.Namespace) -> Dict[str, Any]:
    return send_command(args.socket, {"action": "stop", "attack_id": args.attack_id})


def cmd_stop_all(args: argparse.Namespace) -> Dict[str, Any]:
    return send_command(args.socket, {"action": "stop_all"})


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Control UAV attack daemon")
    parser.add_argument("--socket", default=os.path.expanduser("~/.uav_ids/attackd.sock"), help="attackd UNIX socket")

    sub = parser.add_subparsers(dest="cmd", required=True)

    p_dos = sub.add_parser("dos", help="Start a UDP flood")
    p_dos.add_argument("duration", type=float, help="Duration in seconds")
    p_dos.add_argument("--payload-bytes", type=int, default=800, help="Bytes per packet (default: %(default)s)")
    p_dos.add_argument("--pps", type=float, default=30.0, help="Packets per second (default: %(default)s)")
    p_dos.add_argument("--target-host", default="127.0.0.1", help="Destination host (default: %(default)s)")
    p_dos.add_argument("--target-port", type=int, default=14640, help="Destination port (default: %(default)s)")
    p_dos.set_defaults(func=cmd_dos)

    p_hb = sub.add_parser("hb", help="Apply heartbeat-drop loss via tc netem")
    p_hb.add_argument("duration", type=float, help="Duration in seconds")
    p_hb.add_argument("--loss", type=float, default=0.6, help="Loss ratio 0.0-1.0 (default: %(default)s)")
    p_hb.add_argument("--iface", default="lo", help="Interface to shape (default: %(default)s)")
    p_hb.set_defaults(func=cmd_hb)

    p_state = sub.add_parser("state", help="Show active attacks")
    p_state.set_defaults(func=cmd_state)

    p_stop = sub.add_parser("stop", help="Stop a specific attack")
    p_stop.add_argument("attack_id", help="Attack identifier from 'state'")
    p_stop.set_defaults(func=cmd_stop)

    p_stop_all = sub.add_parser("stop-all", help="Stop every attack")
    p_stop_all.set_defaults(func=cmd_stop_all)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    result = args.func(args)
    json.dump(result, sys.stdout, indent=2)
    sys.stdout.write("\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
