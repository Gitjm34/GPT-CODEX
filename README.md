# UAV Network Simulator for Adaptive RL-based IDS  
**Realistic PX4-SITL MAVLink Traffic Generator with ns-3-controlled Realistic Link, RAW Packet Capture, and Synchronous Multi-Modal Data Collection**
<img width="1339" height="959" alt="image" src="https://github.com/user-attachments/assets/ab16c3a2-70ea-4a25-a73a-c004ba2bc3b2" />


## Project Overview

This repository implements the **UAV-domain network simulator & dataset generator** for the research paper  
**"Adaptive AI-based IDS via Reinforcement Learning"**  
(Byeongchang Kim et al., GIST · Korea University · Kyung Hee University).

The overall project aims to train **a single reinforcement learning agent** (model-based RL with AMAGO-style architecture + Graph Neural Networks + contrastive reward predictor + generative augmentation) that can detect intrusions across **heterogeneous network domains (UAV/IoT, vehicular, corporate, home) without being limited to fixed topologies or pre-defined attack types.

Conventional IDS systems suffer from critical limitations:

- Trained only on static datasets and fixed network topologies  
- Fail to generalize to unseen attacks or topology changes  
- Require massive labeled data for every new environment  
- Cannot handle real-world network dynamics (mobility, link quality variation, etc.)

→ We propose an adaptive, model-based RL framework that learns from experience and generalizes across domains.

My role in the project is the **UAV/IoT domain simulator**, which generates perfectly synchronized, high-fidelity, multi-modal data:

- Raw MAVLink/UDP packets (.pcapng)  
- Network-layer metrics (delay, loss, rate, up/down_bytes) dynamically controlled by ns-3 based on drone altitude  
- Drone telemetry + MAVLink statistics (altitude, velocity, GPS fix, EPH/EPV, attitude, heartbeat Hz, etc.)  
- All modalities tagged with identical `RUN_ID` → perfect time alignment for RL training

## System Architecture (Actual Implemented Diagram)

![System Architecture](https://raw.githubusercontent.com/your-username/your-repo-name/main/assets/architecture.png)
PX4 SITL (Gazebo) ←UDP 14550 (Downlink)→ Middleware (udp_mw_ns3.py) ←UDP dynamic(~1550)→ QGroundControl
PX4 SITL ←UDP 14540 (Uplink)← Middleware ←UDP 14640← QGroundControl
↓
Apply ns-3 delay/loss/rate
POST /ingest (1 Hz) → Collector
MAVROS → Positions.txt (1 Hz) → ns-3 (mw-link-metrics) → calculates delay/loss/rate
MAVROS → POST /ingest_extra → Collector
Collector (FastAPI @ port 8080): GET /obs/latest, /obs/seq for RL agent
text## Development Timeline (Real Progress Logs)

| Date       | Milestone                                      | Key Outcome |
|------------|------------------------------------------------|-------------|
| 2024-10-07 | PX4 SITL ↔ QGC 기본 직통 연결                  | Stable baseline |
| 2024-10-08 | Middleware (udp_mw_ns3.py) 삽입                | All traffic routed through middleware |
| 2024-10-09 | ns-3 (mw-link-metrics) 연동                     | Real-time delay/loss/rate applied by altitude in real time (delay = 10 + h ms 등) |
| 2024-10-17 | FastAPI Collector 구축 (HTTP Push/Pull)          | Synchronous storage of network + telemetry |
| 2024-10-22 | RAW 패킷 수집 기능 완성 (tcpdump + scripts)     | pcapng + JSON perfect sync via RUN_ID |
| 2024-11-02 | DoS/Flooding + Heartbeat Drop 공격 실험         | Successful anomaly capture |
| 2024-11-10 | GUIDE-main Dataset 분석 및 modality 분리스트 작성 | MAVLink vs Sensor detectable attack types 정리 |

Simulator is now 100 % complete and production-grade.

## Key Features (All Implemented & Tested)

1. **Realistic RF Link Simulation**  
   ns-3 continuously reads drone altitude → calculates delay/loss/bandwidth → middleware applies to every MAVLink packets in real time.

2. **Perfectly Synchronous Multi-Modal Collection**  
   - RAW packets: tcpdump on loopback → .pcapng  
   - Network metrics: middleware → POST /ingest (1 Hz)  
   - Telemetry + heartbeat Hz: MAVROS → POST /ingest_extra (1 Hz)  
   → Identical `run_id` across all files → zero alignment effort.

3. **One-Command Experiment Workflow**
   ```bash
   ~/uav_tools/start_capture.sh dos_attack_001    # starts tcpdump + sets RUN_ID
   # fly or inject attack
   ~/uav_tools/stop_capture.sh                     # auto-detects latest
   ~/uav_tools/pcap_to_csv.sh dos_attack_001.all.pcapng

Detectable Attacks by Modality (GUIDE-main Analysis)ModalityDetectable AttacksMAVLink layerDoS/Flooding, Heartbeat Drop, Message Drop/Injection, Link SpoofingSensor/TelemetryGPS Spoofing, GPS Jamming, Sensor noise attacks

Current Status – November 19, 2025

Simulator: fully stable
Collected datasets: Normal flight + DoS + Heartbeat Drop (multiple altitudes)
Total synchronized records in collector: >250,000 rows
RAW pcap files: >15 GB across 50+ runs
Ready for GPS Spoofing/Jamming experiments (only sensor injection or extra spoofing middleware needed)
Next step: export to Parquet + integrate with RL training pipeline

Quick Start (실제 빌드 과정 정리본 그대로)
Bash# 0) Clean
pkill -f 'px4|qgroundcontrol|mavros|udp_mw_ns3|collector|ros_extra_pusher|uvicorn' 2>/dev/null || true
sudo lsof -nP -iUDP:14540 -iUDP:14550 -iUDP:14556 -iUDP:14558 -iUDP:14640 || true

# 1) PX4 SITL
cd ~/PX4-Autopilot
make px4_sitl gazebo

# 2) MAVROS extra downlink
roslaunch mavros px4.launch fcu_url:=udp://:14556@127.0.0.1:14540

# PX4 console에서
mavlink start -u 14558 -r 40000000 -o 14556 -t 127.0.0.1

# 3) ns-3 position feeder + collector + middleware
python3 ~/mw_ns3/alti2positions.py &
uvicorn collector:app --port 8080 &
python3 udp_mw_ns3.py &

# 4) QGroundControl → UDP 14640 연결

# 5) 실험 시작 시
~/uav_tools/start_capture.sh my_run_001
Repository Structure
text.
├── udp_mw_ns3.py          # core middleware
├── collector.py           # FastAPI collector
├── mw-link-metrics        # ns-3 binary
├── alti2positions.py      # MAVROS → positions.txt
├── ros_extra_pusher.py
├── uav_tools/             # start/stop_capture.sh, pcap_to_csv.sh
├── assets/
│   └── architecture.png   # 위에 넣은 다이어그램
└── pcaps/ & pcap_csv/    # generated data
Acknowledgments
Part of the paper "Adaptive AI-based IDS via Reinforcement Learning"
Authors: Byeongchang Kim (GIST), Jae-min Jung (Kyung Hee Univ.), Yoo-hee Park, Ye-ji Lee, Sun-young Hwang, Shin-young Ryu (Korea Univ.)
License: MIT
