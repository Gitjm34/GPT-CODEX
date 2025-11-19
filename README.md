# UAV Network Simulator for Adaptive RL-based IDS
**Realistic PX4-SITL MAVLink Traffic Generator with ns-3-controlled Link, RAW Packet Capture, and Synchronous Telemetry Collection**

![System Architecture](https://via.placeholder.com/1200x800.png?text=UAV+Network+Simulator+Architecture)
*(Note: This diagram represents the data flow: PX4 SITL ↔ Middleware ↔ QGC ↔ ns-3 ↔ Collector ↔ MAVROS)*

## Project Overview

This repository contains the **UAV-domain network simulator** developed as part of the research project **"Adaptive AI-based IDS via Reinforcement Learning"** (Byeongchang Kim et al., GIST · Korea Univ. · Kyung Hee Univ.).

The ultimate goal of the entire project is to train **a single reinforcement learning agent** that can detect intrusions across **heterogeneous network domains** (UAV/IoT, vehicular, corporate, home) without being restricted to a specific topology, protocol, or pre-defined attack set.

Conventional IDS research often fails in the real world because:
- Trained on fixed topologies and pre-specified attack categories
- Cannot adapt to unseen/evolving attacks or topology changes
- Requires massive labeled data for every new environment
- Ignores real-world network dynamics (node mobility, link quality variation, etc.)

→ We chose **model-based RL (AMAGO-style) + Graph Neural Networks + contrastive reward prediction + generative augmentation** to create an IDS that learns from experience and generalizes across domains.

My contribution is the **UAV/IoT domain simulator and dataset generator**, which produces perfectly synchronized, realistic, multi-modal data:
- **Raw MAVLink/UDP packets (pcapng)**
- **Network metrics** (delay, loss, rate, up/down bytes) controlled by ns-3 according to drone altitude
- **Drone telemetry** (altitude, groundspeed, GPS fix, EPH/EPV, attitude, heartbeat Hz, etc.)
- All data tagged with the same `RUN_ID` for perfect alignment

This data will be used to train the RL agent that operates in the UAV environment and, thanks to domain-invariant design, also in other environments.

## System Architecture

```text
PX4 SITL (Gazebo)                   Middleware (udp_mw_ns3.py)                  QGroundControl
MAVLink #0: UDP 14540 (server) ←──────────────────────→ Receives: 14640 (from QGC)
MAVLink #1: UDP 14550 ─────────────────────────────→ Forwards to PX4 14540 (uplink)
          ↓                         Forwards to QGC dynamic port (~1550)
                                    Applies ns-3 delay/loss/rate
                                    Logs up_bytes, down_bytes, seq
                                    POST /ingest → Collector (1 Hz)

MAVROS ←────────────────────────────── UDP 14556 (extra downlink instance)
   ↓
alt2positions_light.py → Positions.txt (1 Hz) → ns-3 (mw-link-metrics)
   ↓
ros_extra_pusher.py → POST /ingest_extra → Collector

ns-3 (mw-link-metrics)
Reads Positions.txt → calculates delay/loss/rate (for shaping)
Current formula example: delay_ms = 10 + h (altitude_m)

Collector (FastAPI, port 8080)
POST /ingest          → network metrics
POST /ingest_extra    → drone telemetry + MAVLink Hz stats
GET /obs/latest, /obs/seq → for RL agent to pull observations
Development Timeline (Actual Progress Logs)DateMilestoneKey Achievement2024-10-07PX4 ↔ QGC basic connection (direct UDP)SITL stable baseline2024-10-08Middleware insertion (udp_mw_ns3.py)All traffic goes through middleware, ready for impairment2024-10-09ns-3 integration (mw-link-metrics)Real-time delay/loss/rate according to altitude (delay = 10 + h ms, etc.)2024-10-17HTTP Push/Pull collector (collector.py, FastAPI)Synchronous storage of network + telemetry data2024-10-22RAW packet capture function (tcpdump + scripts)Full pcapng + JSON synchronized by RUN_ID2024-11-02DoS/Flooding and Heartbeat Drop attack experimentsSuccessful capture of network metric changes under attack2024-11-10GUIDE-main dataset analysis + Feature separationClear definition of detectable attack types per modalityThe simulator is now fully functional and has been used to collect normal flight and attack datasets.Key Features ImplementedRealistic Link Impairmentns-3 calculates delay, loss, bandwidth in real time from drone altitude → middleware applies to every MAVLink packet → identical to real RF link behavior.Synchronous Multi-Modal Data CollectionRAW packets (pcapng) via tcpdump on loopbackNetwork metrics (1 Hz) via middleware → /ingestTelemetry + MAVLink statistics (1 Hz) via MAVROS → /ingest_extraAll records contain the same run_id → perfect alignment.Easy Experiment ScriptingBash~/uav_tools/start_capture.sh my_experiment_001    # starts tcpdump + collector + run_id
# perform normal flight or inject attack
~/uav_tools/stop_capture.sh                       # auto-detects latest run_id
~/uav_tools/pcap_to_csv.sh my_experiment_001.all.pcapng
Attack Injection Examples (Already Tested)DoS/Flooding: Massive increase in uplink packets, loss ↑, rate ↑, down_bytes ↓Heartbeat Drop: Heartbeat Hz ↓, QGC eventually shows "link lost"Collector API (for RL agent)Bashcurl http://localhost:8080/obs/latest?k=10      # latest 10 observations (network + telemetry merged)
curl http://localhost:8080/obs/seq?seq=100      # all observations after sequence 100
Data Modality & Detectable AttacksModalityData SourceDetectable Attacks (proven in literature & our experiments)MAVLink protocol layerMessage ID sequence, IAT, packet rate, byte ratiosDoS/Flooding, Heartbeat Drop, Message Drop/Injection, Link SpoofingSensor/telemetry layerAltitude, EPH/EPV, GPS coords, attitude, groundspeedGPS Spoofing, GPS Jamming, Sensor noise attacks→ The simulator collects both modalities simultaneously → the RL agent can learn multi-modal anomaly detection.Usage (Quick Start)Follow this sequence to bring up the full stack.Bash# 0. Clean up previous runs
~/uav_tools/reset_all.sh

# 1. PX4 SITL
cd ~/PX4-Autopilot && PX4_NO_FORK=1 make px4_sitl_default none_iris

# 2. MAVROS (Bridge & Extra Downlink)
# Note: Ensure setup.bash is sourced
source /opt/ros/noetic/setup.bash
roscore &
rosrun mavros mavros_node _fcu_url:=udp://0.0.0.0:14556@127.0.0.1:14540 &

# 3. ns-3 Position Feeder (Mirror altitude to positions.txt)
# Corrected filename: alt2positions_light.py
python3 ~/mw_ns3/alt2positions_light.py &

# 4. Collector (Data Aggregator)
~/uav_tools/start_collector.sh

# 5. Middleware (The Core: Relays traffic with ns-3 shaping)
export IDS_ENDPOINT="[http://127.0.0.1:8080/ingest](http://127.0.0.1:8080/ingest)"
python3 ~/mw_ns3/udp_mw_ns3.py &

# 6. QGroundControl
# IMPORTANT: Go to Application Settings -> Comm Links -> Disable "UDP Auto-Connect" on Port 14550.
# Then add a new Link: UDP, Port 14640, Server 127.0.0.1
flatpak run org.mavlink.qgroundcontrol &

# 7. Start Experiment (Capture & Attack)
export RUN_ID="experiment_dos_01"
~/uav_tools/start_capture.sh "$RUN_ID"

# (Optional) Inject Attack
python3 ~/mw_ns3/attackctl.py dos 20 --target-port 14640

# 8. Stop & Save
~/uav_tools/stop_capture.sh "$RUN_ID"
~/uav_tools/pcap_to_csv.sh "$RUN_ID"
Current Status (November 19, 2025)Simulator 100 % stable.Normal flight + DoS + Heartbeat Drop datasets collected (pcap + JSON + CSV).Ready for GPS Spoofing/Jamming experiments (just modify PX4 sensor injection or add spoofing middleware).Collector contains >200 k synchronized records from various altitudes and attacks.Next step: Export to parquet or HDF5 for RL training pipeline.AcknowledgmentsThis simulator is part of the paper "Adaptive AI-based IDS via Reinforcement Learning".Authors: Byeongchang Kim (GIST), Jae-min Jung (Kyung Hee Univ.), Yoo-hee Park, Ye-ji Lee, Sun-young Hwang, Shin-young Ryu (Korea Univ.).The UAV domain data generated here will be combined with vehicular, corporate, and home network data generated by other team members to train a single cross-domain RL IDS agent.Feel free to open issues or contact me if you want the collected datasets or want to extend the simulator to other attacks (GPS spoofing, message injection, etc.).License: MIT
