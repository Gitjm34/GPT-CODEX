#!/usr/bin/env python3
"""
ROS node that mirrors MAVROS relative altitude into the ns-3 positions.txt file.
"""

import argparse
import os
import pathlib
import threading
from typing import Optional

import rospy
from std_msgs.msg import Float64


class AltitudeRecorder:
    def __init__(self, positions_path: pathlib.Path, topic: str, hz: float, precision: int) -> None:
        self._path = positions_path
        self._precision = max(0, precision)
        self._lock = threading.Lock()
        self._altitude = 0.0

        self._path.parent.mkdir(parents=True, exist_ok=True)
        rospy.Subscriber(topic, Float64, self._callback, queue_size=1)
        period = max(0.1, 1.0 / hz)
        rospy.Timer(rospy.Duration(period), self._write)

        rospy.loginfo("Writing %s from %s at %.2f Hz", self._path, topic, 1.0 / period)

    def _callback(self, msg: Float64) -> None:
        with self._lock:
            self._altitude = max(0.0, float(msg.data))

    def _write(self, _event: rospy.timer.TimerEvent) -> None:
        with self._lock:
            altitude = round(self._altitude, self._precision)

        tmp_path = self._path.with_suffix(".tmp")
        try:
            with tmp_path.open("w", encoding="utf-8") as handle:
                handle.write("0 0 0\n0 0 %.{prec}f\n".format(prec=self._precision) % altitude)
            tmp_path.replace(self._path)
        except OSError as exc:
            rospy.logwarn("Failed to update %s: %s", self._path, exc)


def parse_args() -> argparse.Namespace:
    default_ns3 = pathlib.Path(os.environ.get("NS3_PATH", "~/ns-allinone-3.35/ns-3.35")).expanduser()
    parser = argparse.ArgumentParser(description="Mirror MAVROS altitude into ns-3 positions.txt")
    parser.add_argument(
        "--ns3-path",
        default=str(default_ns3),
        help="Base ns-3 path (default: %(default)s)",
    )
    parser.add_argument(
        "--positions-file",
        default=None,
        help="Override path to positions.txt (default: <ns3-path>/positions.txt)",
    )
    parser.add_argument(
        "--topic",
        default="/mavros/global_position/rel_alt",
        help="Altitude topic to monitor (default: %(default)s)",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=1.0,
        help="Write frequency in Hz (default: %(default)s)",
    )
    parser.add_argument(
        "--precision",
        type=int,
        default=2,
        help="Decimal places to persist (default: %(default)s)",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    ns3_path = pathlib.Path(args.ns3_path).expanduser()
    positions_file = pathlib.Path(args.positions_file).expanduser() if args.positions_file else ns3_path / "positions.txt"

    rospy.init_node("px4_alt_to_positions")
    AltitudeRecorder(positions_file, args.topic, args.hz, args.precision)
    rospy.spin()


if __name__ == "__main__":
    main()
