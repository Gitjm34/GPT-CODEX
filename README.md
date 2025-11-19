# Adaptive AI-based IDS Simulation Framework for Heterogeneous UAV Networks

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS](https://img.shields.io/badge/ROS-Noetic-blue)](http://wiki.ros.org/noetic)
[![PX4](https://img.shields.io/badge/PX4-Autopilot-black)](https://px4.io/)
[![ns-3](https://img.shields.io/badge/ns--3-3.35-green)](https://www.nsnam.org/)

## 📖 프로젝트 개요 (Project Overview)

[cite_start]본 프로젝트는 **이기종 UAV 네트워크 환경**에서 동작하는 **적응형 AI 기반 침입 탐지 시스템(IDS)**을 연구 및 검증하기 위한 고충실도 시뮬레이션 프레임워크입니다[cite: 1].

[cite_start]기존 IDS 연구는 정적인 토폴로지와 고정된 데이터셋에 의존하여, 드론과 같이 토폴로지가 급변하는 동적 환경에서의 보안 위협을 효과적으로 방어하지 못했습니다[cite: 1]. [cite_start]본 연구는 **PX4 SITL**, **ns-3**, **ROS**, 그리고 **강화학습(AMAGO)**을 통합하여, 물리적 비행 상태가 네트워크 품질에 실시간으로 영향을 미치는 현실적인 환경을 구현했습니다[cite: 1, 2].

### 🎯 핵심 목표 (Key Objectives)
* [cite_start]**고충실도 시뮬레이션:** 실제 비행 제어(PX4)와 네트워크 시뮬레이터(ns-3) 연동[cite: 1].
* [cite_start]**동적 네트워크 환경:** 드론의 고도 및 기동에 따른 실시간 통신 품질(Delay, Loss, BW) 변화 구현[cite: 3].
* [cite_start]**이기종 데이터 수집:** 네트워크 패킷, 드론 텔레메트리, RAW 패킷의 동기화된 데이터 파이프라인 구축[cite: 1].
* [cite_start]**적대적 공격 재현:** DoS 및 Heartbeat Drop 등 실제 프로토콜 기반 공격 시나리오 시뮬레이션[cite: 1].

---

## 🏗️ 시스템 아키텍처 (System Architecture)

[cite_start]이 시뮬레이터는 **비행 제어**, **지상 관제**, **네트워크 중계**, **데이터 수집**의 4계층으로 구성됩니다.



### 🔌 포트 매핑 및 데이터 흐름 (Port Mapping)
[cite_start]모든 트래픽은 미들웨어(`udp_mw_ns3.py`)를 경유하며, 이 과정에서 ns-3 모델에 기반한 네트워크 지연 및 손실이 적용됩니다.

| Source Component | Source Port | Dest Component | Dest Port | Direction | Description |
|------------------|-------------|----------------|-----------|-----------|-------------|
| **QGroundControl** | Dynamic | **Middleware** | `14640` | Uplink | GCS → Drone 제어 명령 |
| **Middleware** | Dynamic | **PX4 SITL** | `14540` | Uplink | 지연/손실 적용 후 전달 |
| **PX4 SITL** | `14550` | **Middleware** | `14550` | Downlink | Drone → GCS 상태 정보 |
| **Middleware** | `14550` | **QGroundControl** | Dynamic | Downlink | 지연/손실 적용 후 전달 |
| **PX4 SITL** | Dynamic | **MAVROS** | `14556` | Offboard | ROS 연동 채널 |

---

## ⚙️ 설치 및 요구사항 (Installation & Prerequisites)

### 환경 요구사항 (Requirements)
* [cite_start]**OS:** Ubuntu 20.04 LTS [cite: 7]
* **Middleware:** Python 3.8+ (FastAPI, uvicorn, pymavlink)
* **Simulation:** PX4-Autopilot, Gazebo Classic
* [cite_start]**Network:** ns-3 (version 3.35 권장) [cite: 7]
* **Robotics:** ROS Noetic

### 설치 가이드 (Installation Steps)

1. **Repository Clone**
   ```bash
   git clone [https://github.com/your-repo/uav-ids-simulation.git](https://github.com/your-repo/uav-ids-simulation.git)
   cd uav-ids-simulation
Python DependenciesBashpip install -r requirements.txt
# 주요 라이브러리: fastapi, uvicorn, requests, pymavlink
PX4 & ROS SetupPX4 Autopilot 빌드 및 ROS Noetic 설치가 필요합니다. (공식 문서 참조)🚀 실행 방법 (Usage)전체 시뮬레이션은 데이터 수집 서버, 미들웨어, 시뮬레이터 순으로 실행해야 합니다.1. 수집 서버 실행 (Collector Server)네트워크 지표와 텔레메트리를 수집하는 중앙 서버를 가동합니다1.Bashpython collector.py
# Server runs on http://localhost:8000
2. 네트워크 미들웨어 및 물리 브리지 실행ns-3 기반의 링크 품질 계산 및 패킷 중계를 시작합니다2.Bash# 물리-네트워크 연동 브리지 (고도 정보 -> ns-3 입력)
python alt2positions.py

# 네트워크 미들웨어 (패킷 중계 및 셰이핑)
python udp_mw_ns3.py
3. ROS 및 MAVROS 실행드론의 상태 정보를 수집하여 서버로 전송합니다3.Bashroslaunch mavros px4.launch fcu_url:="udp://:14556@127.0.0.1:14550"
python ros_extra_pusher.py
4. PX4 SITL & QGroundControl 실행Bash# PX4 SITL (Gazebo)
cd ~/PX4-Autopilot
make px4_sitl gazebo

# QGroundControl (Connect to UDP port 14640, NOT 14550)
./QGroundControl.AppImage
주의: QGC에서 Comm Links 설정을 통해 14550 포트가 아닌 14640 포트로 접속해야 미들웨어가 정상 동작합니다4.⚔️ 공격 시뮬레이션 (Attack Simulation)학습 데이터의 다양성을 위해 attackctl.py를 사용하여 정상 트래픽 흐름에 제어된 공격을 주입할 수 있습니다5.1. 서비스 거부 공격 (DoS)대역폭 고갈 및 자원 소진을 유도합니다6.Bash# 20초간 800바이트 패킷 지속 주입
python attackctl.py dos --duration 20 --size 800 --rate 30
관측 지표: up_bytes 급증, 정상 명령 delay 증가 (Starvation)7.2. 하트비트 드롭 공격 (Heartbeat Drop)연결 상태를 교란하는 프로토콜 공격입니다8.Bash# 15초간 HEARTBEAT 메시지 60% 확률로 누락
python attackctl.py hb --duration 15 --probability 0.6
관측 지표: hb_hz (하트비트 주파수) 감소, heartbeat_gap_ms 분산 증가9.📊 데이터 파이프라인 및 API (Data Pipeline)수집된 데이터는 강화학습(AMAGO) 에이전트의 학습을 위해 정규화된 형태로 제공됩니다10101010.데이터 수집 구조 (Push/Pull)Push (/ingest): 미들웨어 및 ROS 노드가 1Hz 주기로 데이터를 서버로 전송1111.Pull (/obs/seq): 강화학습 에이전트가 과거 $k$개 시점의 시퀀스 데이터를 요청12121212.주요 수집 필드 (State Vector)CategoryFieldsDescriptionNetworkdelay_ms, loss_pct, up_bytes고도 상관관계 및 DoS 탐지용Dronealtitude_m, groundspeed_mps물리적 기동 상태 확인Protocolheartbeat_gap_ms, hb_hz연결 신뢰성 및 프로토콜 공격 탐지GNSSsatellites_used, fix_typeGPS 스푸핑/재밍 징후📝 Citation코드 스니펫@techreport{uav-ids-2024,
  title={Adaptive AI-based IDS for Heterogeneous UAV Networks},
  author={Your Name and Collaborators},
  year={2024},
  institution={Your Institution}
}
