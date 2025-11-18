#!/usr/bin/env python3
"""ROS node that summarizes MAVROS telemetry and pushes it to the collector."""

from __future__ import annotations

import math
import os
import socket
import threading
import time
from collections import Counter
from typing import Dict, Optional

import requests
import rospy
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import GPSRAW, Mavlink, State
from sensor_msgs.msg import BatteryState, NavSatFix, NavSatStatus
from std_msgs.msg import Float64

RUN_ID = os.environ.get("RUN_ID", time.strftime("run_%Y%m%d_%H%M%S"))
ENDPOINT = os.environ.get("IDS_EXTRA_ENDPOINT", "http://127.0.0.1:8080/ingest_extra")
NODE_ID = os.environ.get("NODE_ID", socket.gethostname())
PUSH_HZ = float(os.environ.get("EXTRA_PUSH_HZ", "1.0"))
REQUEST_TIMEOUT = float(os.environ.get("IDS_EXTRA_TIMEOUT", "0.8"))
HB_ID = 0
SYS_STATUS_ID = 1
PARAM_SET_ID = 23
COMMAND_LONG_ID = 76
GPS_INT_ID = 33

state_lock = threading.Lock()
STATE: Dict[str, Optional[float]] = {
    "altitude_m": None,
    "groundspeed_mps": None,
    "battery_pct": None,
    "heading_deg": None,
    "heartbeat_gap_ms": None,
}
GNSS: Dict[str, Optional[float]] = {
    "fix_type": None,
    "eph": None,
    "epv": None,
    "satellites_used": None,
    "jamming_indicator": None,
}
last_heartbeat_ts = None

counter_lock = threading.Lock()
msg_counter: Counter[int] = Counter()
prev_counter: Counter[int] = Counter()
prev_counter_ts = time.time()


def _set_state(key: str, value) -> None:
    with state_lock:
        STATE[key] = value


def cb_alt(msg: Float64) -> None:
    _set_state("altitude_m", float(msg.data))


def cb_vel(msg: TwistStamped) -> None:
    speed = math.hypot(msg.twist.linear.x, msg.twist.linear.y)
    _set_state("groundspeed_mps", float(speed))


def cb_heading(msg: Float64) -> None:
    _set_state("heading_deg", float(msg.data))


def cb_battery(msg: BatteryState) -> None:
    value = None
    if msg.percentage not in (None, float("nan")):
        value = float(msg.percentage) * 100.0
    elif msg.capacity and msg.capacity > 0 and msg.charge is not None:
        ratio = max(0.0, min(1.0, msg.charge / msg.capacity))
        value = ratio * 100.0
    _set_state("battery_pct", value)


def cb_state(msg: State) -> None:
    global last_heartbeat_ts
    if msg.header.stamp:
        now = msg.header.stamp.to_sec()
    else:
        now = time.time()
    if last_heartbeat_ts is not None:
        _set_state("heartbeat_gap_ms", (now - last_heartbeat_ts) * 1000.0)
    last_heartbeat_ts = now


def cb_gps_raw(msg: GPSRAW) -> None:
    with state_lock:
        GNSS["fix_type"] = int(msg.fix_type)
        GNSS["eph"] = float(msg.eph) if msg.eph > 0 else None
        GNSS["epv"] = float(msg.epv) if msg.epv > 0 else None
        GNSS["satellites_used"] = int(msg.satellites_visible)
        if hasattr(msg, "jamming_indicator"):
            GNSS["jamming_indicator"] = float(msg.jamming_indicator)


def cb_navsat(msg: NavSatFix) -> None:
    with state_lock:
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            try:
                GNSS["eph"] = math.sqrt(max(0.0, msg.position_covariance[0]))
                GNSS["epv"] = math.sqrt(max(0.0, msg.position_covariance[8]))
            except Exception:  # noqa: BLE001
                pass
        status = msg.status.status
        if status == NavSatStatus.STATUS_NO_FIX:
            GNSS["fix_type"] = 0
        elif status == NavSatStatus.STATUS_FIX:
            GNSS["fix_type"] = 3
        elif status == NavSatStatus.STATUS_SBAS_FIX:
            GNSS["fix_type"] = 4
        elif status == NavSatStatus.STATUS_GBAS_FIX:
            GNSS["fix_type"] = 5


def cb_mavlink(msg: Mavlink) -> None:
    with counter_lock:
        msg_counter[int(msg.msgid)] += 1


def _freq(delta: Counter[int], mid: int, dt: float) -> float:
    return max(0.0, delta.get(mid, 0)) / dt


def push_loop() -> None:
    global prev_counter, prev_counter_ts
    hz = max(0.1, PUSH_HZ)
    rate = rospy.Rate(hz)
    session = requests.Session() if ENDPOINT else None
    if not ENDPOINT:
        rospy.logwarn("IDS_EXTRA_ENDPOINT is empty; skipping HTTP pushes")
    while not rospy.is_shutdown():
        now = time.time()
        with counter_lock:
            snapshot = msg_counter.copy()
        delta = Counter()
        for mid, count in snapshot.items():
            delta[mid] = count - prev_counter.get(mid, 0)
        prev_counter = snapshot
        dt = max(1e-3, now - prev_counter_ts)
        prev_counter_ts = now

        with state_lock:
            payload = {
                "run_id": RUN_ID,
                "ts": now,
                "node_id": NODE_ID,
                "altitude_m": STATE["altitude_m"],
                "groundspeed_mps": STATE["groundspeed_mps"],
                "battery_pct": STATE["battery_pct"],
                "heading_deg": STATE["heading_deg"],
                "heartbeat_gap_ms": STATE["heartbeat_gap_ms"],
                "fix_type": GNSS["fix_type"],
                "eph": GNSS["eph"],
                "epv": GNSS["epv"],
                "satellites_used": GNSS["satellites_used"],
                "jamming_indicator": GNSS["jamming_indicator"],
                "hb_hz": _freq(delta, HB_ID, dt),
                "status_hz": _freq(delta, SYS_STATUS_ID, dt),
                "paramset_hz": _freq(delta, PARAM_SET_ID, dt),
                "cmdlong_hz": _freq(delta, COMMAND_LONG_ID, dt),
                "gpsint_hz": _freq(delta, GPS_INT_ID, dt),
            }

        if session:
            try:
                session.post(ENDPOINT, json=payload, timeout=REQUEST_TIMEOUT)
            except requests.RequestException as exc:
                rospy.logwarn("Failed to POST extras: %s", exc)

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ros_extra_pusher")
    rospy.Subscriber("/mavros/global_position/rel_alt", Float64, cb_alt, queue_size=1)
    rospy.Subscriber("/mavros/local_position/velocity_local", TwistStamped, cb_vel, queue_size=1)
    rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, cb_heading, queue_size=1)
    rospy.Subscriber("/mavros/state", State, cb_state, queue_size=1)
    rospy.Subscriber("/mavros/battery", BatteryState, cb_battery, queue_size=1)
    rospy.Subscriber("/mavros/gpsstatus/gps1/raw", GPSRAW, cb_gps_raw, queue_size=1)
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, cb_navsat, queue_size=1)
    rospy.Subscriber("/mavlink/from", Mavlink, cb_mavlink, queue_size=200)

    threading.Thread(target=push_loop, daemon=True).start()
    rospy.loginfo(
        "ros_extra_pusher sending extras for run_id=%s to %s", RUN_ID, ENDPOINT
    )
    rospy.spin()
