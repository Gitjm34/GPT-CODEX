#!/usr/bin/env python3
"""FastAPI-based collector that accepts middleware/ROS observations."""

from __future__ import annotations

import os
import socket
import time
from typing import Any, Dict, List, Optional

from fastapi import FastAPI
from pydantic import BaseModel, Field
import uvicorn

APP_HOST = os.environ.get("COLLECTOR_HOST", "0.0.0.0")
APP_PORT = int(os.environ.get("COLLECTOR_PORT", "8080"))
NODE_ID = os.environ.get("COLLECTOR_NODE_ID", socket.gethostname())
MAX_WINDOW = 1000

app = FastAPI(title="UAV IDS Collector", version="0.1.0")

STORE: List[Dict[str, Any]] = []
STORE_EXTRA: List[Dict[str, Any]] = []
SEQ_BY_RUN: Dict[str, int] = {}
LAST_EXTRA_BY_RUN: Dict[str, Dict[str, Any]] = {}
EXTRA_FIELDS = [
    "altitude_m",
    "groundspeed_mps",
    "battery_pct",
    "heading_deg",
    "heartbeat_gap_ms",
    "fix_type",
    "eph",
    "epv",
    "satellites_used",
    "jamming_indicator",
    "hb_hz",
    "status_hz",
    "paramset_hz",
    "cmdlong_hz",
    "gpsint_hz",
    "node_id",
]


class Obs(BaseModel):
    run_id: str
    ts: float = Field(default_factory=lambda: time.time())
    up_bytes: int
    down_bytes: int
    delay_ms: float
    loss_pct: float
    rate_kbps: float
    node_id: Optional[str] = None
    seq: Optional[int] = None


class ExtraObs(BaseModel):
    run_id: str
    ts: float = Field(default_factory=lambda: time.time())
    node_id: Optional[str] = None
    altitude_m: Optional[float] = None
    groundspeed_mps: Optional[float] = None
    battery_pct: Optional[float] = None
    heading_deg: Optional[float] = None
    heartbeat_gap_ms: Optional[float] = None
    fix_type: Optional[int] = None
    eph: Optional[float] = None
    epv: Optional[float] = None
    satellites_used: Optional[int] = None
    jamming_indicator: Optional[float] = None
    hb_hz: Optional[float] = None
    status_hz: Optional[float] = None
    paramset_hz: Optional[float] = None
    cmdlong_hz: Optional[float] = None
    gpsint_hz: Optional[float] = None


def _model_to_dict(model: BaseModel) -> Dict[str, Any]:
    return model.dict()


@app.post("/ingest")
async def ingest(obs: Obs) -> Dict[str, Any]:
    record = _model_to_dict(obs)
    run_id = record["run_id"]
    seq = SEQ_BY_RUN.get(run_id, 0) + 1
    SEQ_BY_RUN[run_id] = seq
    record["seq"] = record.get("seq") or seq
    record["node_id"] = record.get("node_id") or NODE_ID

    extra = LAST_EXTRA_BY_RUN.get(run_id)
    if extra:
        for field in EXTRA_FIELDS:
            record.setdefault(field, extra.get(field))

    STORE.append(record)
    return {"status": "ok", "seq": record["seq"], "count": len(STORE)}


@app.post("/ingest_extra")
async def ingest_extra(obs: ExtraObs) -> Dict[str, Any]:
    record = _model_to_dict(obs)
    record["node_id"] = record.get("node_id") or NODE_ID
    STORE_EXTRA.append(record)
    LAST_EXTRA_BY_RUN[record["run_id"]] = record
    return {"status": "ok", "count": len(STORE_EXTRA)}


@app.get("/obs/latest")
async def latest(k: int = 1) -> List[Dict[str, Any]]:
    size = max(1, min(k, MAX_WINDOW))
    return STORE[-size:]


@app.get("/obs/extra/latest")
async def extra_latest(k: int = 1) -> List[Dict[str, Any]]:
    size = max(1, min(k, MAX_WINDOW))
    return STORE_EXTRA[-size:]


@app.get("/obs/seq")
async def by_seq(
    since_seq: int = 0,
    limit: int = 10,
    run_id: Optional[str] = None,
) -> List[Dict[str, Any]]:
    rid = run_id or (STORE[-1]["run_id"] if STORE else None)
    if not rid:
        return []
    max_items = max(1, min(limit, MAX_WINDOW))
    filtered = [rec for rec in STORE if rec["run_id"] == rid and rec["seq"] >= since_seq]
    return filtered[:max_items]


if __name__ == "__main__":
    uvicorn.run(app, host=APP_HOST, port=APP_PORT)
