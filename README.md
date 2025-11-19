# UAV Network Simulator for Adaptive RL-based IDS  
**Realistic PX4-SITL MAVLink Traffic Generator with ns-3-controlled Link, RAW Packet Capture, and Synchronous Telemetry Collection**

![System Architecture](https://via.placeholder.com/1200x800.png?text=UAV+Network+Simulator+Architecture)  
*(Actual diagram: UAV Network Simulator - System Architecture – the one you attached with PX4 SITL ↔ Middleware ↔ QGC ↔ ns-3 ↔ Collector ↔ MAVROS)*

## Project Overview

This repository contains the **UAV-domain network simulator** developed as part of the research project  
**"Adaptive AI-based IDS via Reinforcement Learning"** (Byeongchang Kim et al., GIST · Korea Univ. · Kyung Hee Univ.).

The ultimate goal of the entire project is to train **a single reinforcement learning agent** that can detect intrusions across **heterogeneous network domains** (UAV/IoT, vehicular, corporate, home) without being restricted to a specific topology, protocol, or pre-defined attack set.

Conventional IDS research shows >99 % accuracy on static benchmarks (CIC-IDS, NSL-KDD, etc.), but fails miserably in the real world because:

- Trained on fixed topologies and pre-specified attack categories  
- Cannot adapt to unseen/evolving attacks or topology changes  
- Require massive labeled data for every new environment  
- Ignore real-world network dynamics (node mobility, link quality variation, etc.)

→ We chose **model-based RL (AMAGO-style) + Graph Neural Networks + contrastive reward prediction + generative augmentation** to create an IDS that learns from experience and generalizes across domains.

My contribution is the **UAV/IoT domain simulator and dataset generator**, which produces perfectly synchronized, realistic, multi-modal data:

- Raw MAVLink/UDP packets (pcapng)  
- Network metrics (delay, loss, rate, up/down bytes) controlled by ns-3 according to drone altitude  
- Drone telemetry (altitude, groundspeed, GPS fix, EPH/EPV, attitude, heartbeat Hz, etc.)  
- All data tagged with the same `RUN_ID` for perfect alignment

This data will be used to train the RL agent that operates in the UAV environment and, thanks to domain-invariant design, also in other environments.

## System Architecture
PX4 SITL (Gazebo)                  Middleware (udp_mw_ns3.py)                  QGroundControl
MAVLink #0: UDP 14540 (server) ←──────────────────────→ Receives: 14640 (from QGC)
MAVLink #1: UDP 14550 ─────────────────────────────→ Forwards to PX4 14540 (uplink)
↓          Forwards to QGC dynamic port (~1550)
Applies ns-3 delay/loss/rate
Logs up_bytes, down_bytes, seq
POST /ingest → Collector (1 Hz)
MAVROS ←────────────────────────────── UDP 14556 (extra downlink instance)
alti2positions.py → Positions.txt (1 Hz) → ns-3 (mw-link-metrics)
ros_extra_pusher.py → POST /ingest_extra → Collector
ns-3 (mw-link-metrics)
Reads Positions.txt → calculates delay/loss/rate (for shaping)
Current formula example: delay_ms = 10 + h (altitude_m)
Collector (FastAPI, port 8080)
POST /ingest          → network metrics
POST /ingest_extra    → drone telemetry + MAVLink Hz stats
GET /obs/latest, /obs/seq → for RL agent to pull observations
text## Development Timeline (Actual Progress Logs)

| Date       | Milestone                                                                 | Key Achievement |
|------------|---------------------------------------------------------------------------|-----------------|
| 2024-10-07 | PX4 ↔ QGC basic connection (direct UDP)                                   | SITL stable baseline |
| 2024-10-08 | Middleware insertion (udp_mw_ns3.py)                                      | All traffic goes through middleware, ready for impairment |
| 2024-10-09 | ns-3 integration (mw-link-metrics)                                        | Real-time delay/loss/rate according to altitude (delay = 10 + h ms, etc.) |
| 2024-10-17 | HTTP Push/Pull collector (collector.py, FastAPI)                          | Synchronous storage of network + telemetry data |
| 2024-10-22 | RAW packet capture function (tcpdump + start/stop/pcap_to_csv.sh)          | Full pcapng + JSON synchronized by RUN_ID |
| 2024-11-02 | DoS/Flooding and Heartbeat Drop attack experiments                        | Successful capture of network metric changes under attack |
| 2024-11-10 | GUIDE-main dataset analysis + MAVLink vs. sensor feature separation        | Clear definition of detectable attack types per modality |

The simulator is now fully functional and has been used to collect **normal flight** and **attack** datasets.

## Key Features Implemented

1. **Realistic Link Impairment**  
   ns-3 calculates delay, loss, bandwidth in real time from drone altitude → middleware applies to every MAVLink packet → identical to real RF link behavior.

2. **Synchronous Multi-Modal Data Collection**  
   - RAW packets (pcapng) via tcpdump on loopback  
   - Network metrics (1 Hz) via middleware → `/ingest`  
   - Telemetry + MAVLink statistics (1 Hz) via MAVROS → `/ingest_extra`  
   → All records contain the same `run_id` → perfect alignment.

3. **Easy Experiment Scripting**  
   ```bash
   ~/uav_tools/start_capture.sh my_experiment_001    # starts tcpdump + collector + run_id
   # perform normal flight or inject attack
   ~/uav_tools/stop_capture.sh                       # auto-detects latest run_id
   ~/uav_tools/pcap_to_csv.sh my_experiment_001.all.pcapng

Attack Injection Examples (Already Tested)
DoS/Flooding → massive increase in uplink packets, loss ↑, rate ↑ down_bytes
Heartbeat Drop → heartbeat Hz ↓, QGC eventually shows "link lost"

Collector API (for RL agent)Bashcurl http://localhost:8080/obs/latest?k=10      # latest 10 observations (network + telemetry merged)
curl http://localhost:8080/obs/seq?seq=100      # all observations after sequence 100

Data Modality & Detectable Attacks (from 11/10 analysis of GUIDE-main dataset)




















ModalityData SourceDetectable Attacks (proven in literature & our experiments)MAVLink protocol layerMessage ID sequence, IAT, packet rate, byte ratiosDoS/Flooding, Heartbeat Drop, Message Drop/Injection, Link SpoofingSensor/telemetry layeraltitude, EPH/EPV, GPS coords, attitude, groundspeedGPS Spoofing, GPS Jamming, Sensor noise attacks
→ The simulator collects both modalities simultaneously → the RL agent can learn multi-modal anomaly detection.
Current Status (November 19, 2025)

Simulator 100 % stable
Normal flight + DoS + Heartbeat Drop datasets collected (pcap + JSON + CSV)
Ready for GPS Spoofing/Jamming experiments (just modify PX4 sensor injection or add spoofing middleware)
Collector contains >200 k synchronized records from various altitudes and attacks
Next step: export to parquet or HDF5 for RL training pipeline

Usage (Quick Start)
Bash# 0 Clean
./clean.sh   # or the pkill/lsof command in 빌드 과정 정리본

# 1 PX4 SITL
cd ~/PX4-Autopilot && make px4_sitl gazebo

# 2 MAVROS (extra downlink)
roslaunch mavros px4.launch fcu_url:=udp://:14556@127.0.0.1:14540

# In PX4 console (pxh>)
mavlink start -u 14558 -r 40000 -o 14556 -t 127.0.0.1 -m onboard

# 3 ns-3 position feeder
python3 ~/mw_ns3/alti2positions.py &

# 4 Collector
uvicorn collector.py --port 8080 &

# 5 Middleware (this is the core)
python3 udp_mw_ns3.py &

# 6 QGroundControl → connect to UDP 14640

# 7 Start capture when ready
~/uav_tools/start_capture.sh uav_dos_run_001
Acknowledgments
This simulator is part of the paper
"Adaptive AI-based IDS via Reinforcement Learning"
Byeongchang Kim (GIST), Jae-min Jung (Kyung Hee Univ.), Yoo-hee Park, Ye-ji Lee, Sun-young Hwang, Shin-young Ryu (Korea Univ.)
The UAV domain data generated here will be combined with vehicular, corporate, and home network data generated by other team members to train a single cross-domain RL IDS agent.
Feel free to open issues or contact me if you want the collected datasets or want to extend the simulator to other attacks (GPS spoofing, message injection, etc.).
License: MIT
(Or whatever license you prefer)
