# Adaptive AI-based IDS via Reinforcement Learning  
**UAV Domain Simulator & Dataset Generator**  
**이기종 네트워크 환경에서 동작하는 단일 RL 에이전트를 위한 고충실도 UAV 네트워크 시뮬레이터**

[UAV 네트워크 시뮬레이터 아키텍쳐  복사본.pptx](https://github.com/user-attachments/files/23757129/UAV.pptx)
*(위 이미지를 리포지토리 assets 폴더에 architecture_diagram.png로 업로드하세요. 제공된 다이어그램을 기반으로 한 최종 버전입니다.)*

## 프로젝트 개요 (Project Overview)

본 리포지토리는 논문  
**"Adaptive AI-based IDS via Reinforcement Learning"**  
(Byeongchang Kim – GIST, Jae-min Jung – Kyung Hee Univ., Yoo-hee Park, Ye-ji Lee, Sun-young Hwang, Shin-young Ryu – Korea Univ.)  
의 **UAV/IoT 도메인 시뮬레이터 및 데이터셋 생성기** 구현체입니다.

기존 IDS 연구는 정적인 벤치마크에서 99% 이상의 정확도를 보여주지만, 실제 동적이고 이기종 네트워크 환경(특히 UAV)에서는 성능이 급격히 떨어집니다. 본 연구는 **AMAGO 기반 model-based RL + Graph Neural Networks + Contrastive Reward Predictor + Generative Augmentation**을 통해 **단일 에이전트**가 UAV/IoT, 차량, 기업, 가정 등 모든 도메인에서 미지/진화하는 공격까지 탐지할 수 있도록 설계되었습니다.

저의 담당 파트는 **UAV 도메인**으로, PX4 SITL + ns-3 + 미들웨어 + FastAPI Collector를 완벽히 연동하여 **물리적 상태(고도, 속도)가 네트워크 품질(지연, 손실, 대역폭)에 실시간으로 영향을 미치는 고충실도 시뮬레이터**를 구축했습니다. 이를 통해 **완벽하게 동기화된 multi-modal 데이터** (RAW 패킷, 네트워크 지표, 드론 텔레메트리)를 대량 수집하는 시스템을 완성했습니다.

## 기존 연구의 주요 한계 (Background and Limitations)

선행 연구 분석 결과, 현실 세계에서의 IDS 일반화 실패를 유발하는 다섯 가지 핵심 한계를 도출했습니다:

1. 고정된 데이터셋과 토폴로지에 과도한 의존 – UAV처럼 이동성 있는 환경에서 성능 저하.
2. 단일 공격 유형(예: DDoS)에만 초점 – 복합 공격에 취약.
3. 제로데이 또는 진화하는 공격 패턴에 대한 적응성 부족 – 지도 학습의 본질적 한계.
4. 실세계 네트워크 동적성(노드 이동, 사용자 변화 등) 무시 – 오탐지 증가.
5. 레이블 데이터 의존 – 실시간 운영에서 레이블 확보 어려움, 미분류/적대적 트래픽 처리 불가.

본 연구는 위 한계를 **model-based RL + 동적 시뮬레이션 환경**으로 극복합니다.

## 시스템 아키텍처 (System Architecture – 2025년 11월 최종 버전)

<img width="1197" height="871" alt="image" src="https://github.com/user-attachments/assets/933edb3b-ed2e-4450-8403-0784d2624f19" />


QGroundControl (UDP 14640) ──→ Middleware (udp_mw_ns3.py) ──→ PX4 SITL (UDP 14540 Uplink)
PX4 SITL (UDP 14550 Downlink) ──→ Middleware ──→ QGroundControl (Dynamic Port)
│
│ (ns-3 계산: delay/loss/rate 적용 + POST /ingest)
│
MAVROS (Extra Downlink UDP 14556) ── Positions.txt (1Hz) ── ns-3 (mw-link-metrics)
│
└─ POST /ingest_extra ──→ Collector (FastAPI:8080)
(GET /obs/latest, /obs/seq for RL)
text핵심: 모든 MAVLink 패킷이 미들웨어를 경유하며, ns-3가 드론 고도 기반으로 네트워크 품질을 실시간 조정 – 실제 무선 링크와 유사한 동작 재현.

## 개발 타임라인 (Real Progress Logs)

| 날짜       | 내용                                      | 주요 성과 |
|------------|-------------------------------------------|-----------|
| 2024-10-07 | PX4 SITL ↔ QGC 기본 직통 연결             | SITL 베이스라인 안정화 (mavlink_main.cpp 등 핵심 모듈 이해) |
| 2024-10-08 | udp_mw_ns3.py 미들웨어 삽입                | 트래픽 중계 + 네트워크 불안정성 모사 준비 |
| 2024-10-09 | ns-3 (mw-link-metrics) 연동                | 고도 기반 실시간 delay/loss/rate 적용 (delay = 10 + h ms 등) |
| 2024-10-17 | FastAPI Collector 구축 (Push/Pull)         | /ingest, /ingest_extra, /obs/latest, /obs/seq 완성 – RL Pull 지원 |
| 2024-10-22 | RAW 패킷 수집 기능 완성 (tcpdump + scripts)| RUN_ID 기반 pcap + JSON 동기화, tshark CSV 변환 |
| 2024-11-02 | DoS/Flooding + Heartbeat Drop 공격 실험   | 네트워크 지표 변화 성공 캡처 (up_bytes 폭증, hb_hz 저하) |
| 2024-11-10 | GUIDE-main Dataset 분석 및 modality 분리   | MAVLink vs 센서 데이터 별 탐지 가능 공격 정리 (DoS vs GPS Spoofing) |

→ 2025년 11월 19일 기준 시뮬레이터 완전 안정화, 대량 데이터셋 확보.

## 핵심 기능 (All Implemented & Verified)

1. **물리-네트워크 실시간 연동**: 드론 고도 변화 → ns-3 계산 (positions.txt 1Hz) → 미들웨어 적용 (토큰 버킷 rate limiting, probabilistic drop).
2. **완벽 동기화 Multi-Modal 데이터 수집**:
   - RAW 패킷: tcpdump → .pcapng (MAVLink/UDP 캡처).
   - 네트워크 지표: delay/loss/rate/up/down_bytes → POST /ingest (1Hz).
   - 텔레메트리 + heartbeat_gap_ms: MAVROS → POST /ingest_extra (1Hz).
   - 동일 RUN_ID로 병합 → RL 학습에 즉시 활용 가능.
3. **쉬운 실험 워크플로**:
   ```bash
   ~/uav_tools/start_capture.sh run_001    # tcpdump 시작 + RUN_ID 설정
   # 비행 또는 공격 주입 (e.g., attackctl.py dos 20 800)
   ~/uav_tools/stop_capture.sh             # 자동 종료
   ~/uav_tools/pcap_to_csv.sh run_001.all.pcapng

공격 주입 예시 (attackctl.py):
DoS: up_bytes 급증 + 정상 패킷 지연 (토큰 버킷 고갈).
Heartbeat Drop: hb_hz 저하, 연결 불안정 재현 (60% 드롭 확률).

RL 에이전트 API: curl /obs/latest?k=10 – 최근 관측값 Pull.

탐지 가능 공격 (Based on GUIDE-main Analysis)

















Modality데이터 예시 / 탐지 가능 공격MAVLink 계층메시지 ID 시퀀스, IAT, 패킷율 – DoS/Flooding, Heartbeat Drop, 메시지 Drop/Injection, Link Spoofing센서/텔레메트리 계층altitude, EPH/EPV, GPS 위치, 자세 – GPS Spoofing/Jamming, 센서 교란 공격
시뮬레이터는 두 modality를 동시에 수집 → multi-modal anomaly detection 학습 지원.
현재 상태 (As of November 19, 2025)

시뮬레이터: 100% 안정화, Ubuntu 20.04 + PX4 + ROS Noetic + ns-3.35 통합.
수집 데이터: 정상 비행 + DoS + Heartbeat Drop (다양한 고도) – 250K+ rows, 15GB+ pcap.
실험 검증: 고도 상승 시 지연/손실 증가 재현, 공격 시그니처 명확 (up_bytes 폭증, hb_gap 변동).
다음 단계: GPS Spoofing 실험 (센서 주입 추가) + Parquet export → RL 훈련 연동.

빠른 시작 가이드 (From 빌드 과정 정리본)
Bash# 0) 초기화
pkill -f 'px4|qgroundcontrol|mavros|udp_mw_ns3|collector|ros_extra_pusher|uvicorn' 2>/dev/null || true
sudo lsof -nP -iUDP:14540 -iUDP:14550 -iUDP:14556 -iUDP:14558 -iUDP:14640 || true

# 1) PX4 SITL
cd ~/PX4-Autopilot && make px4_sitl gazebo

# 2) MAVROS (extra downlink)
source /opt/ros/noetic/setup.bash
roscore &
rosrun mavros mavros_node _fcu_url:=udp://0.0.0.0:14556@127.0.0.1:14540 &

# PX4 콘솔 (pxh>)에서
mavlink start -u 14558 -o 14556 -t 127.0.0.1 -m onboard

# 3) ns-3 feeder + Collector + Middleware
python3 ~/mw_ns3/alti2positions.py &
uvicorn collector:app --port 8080 &
python3 udp_mw_ns3.py &

# 4) QGroundControl → UDP 14640 연결

# 5) 실험 시작
~/uav_tools/start_capture.sh my_run_001
리포지토리 구조
text.
├── udp_mw_ns3.py          # 미들웨어 핵심
├── collector.py           # FastAPI 수집 서버
├── mw-link-metrics        # ns-3 계산기
├── alti2positions.py      # 고도 → positions.txt
├── ros_extra_pusher.py    # 텔레메트리 푸셔
├── attackctl.py           # 공격 주입 스크립트
├── uav_tools/             # start/stop_capture.sh, pcap_to_csv.sh
├── assets/                # architecture_diagram.png
├── pcaps/ & pcap_csv/     # 샘플 데이터
└── docs/                  # 진행 사항 PDF들
감사의 말 (Acknowledgments)
논문 "Adaptive AI-based IDS via Reinforcement Learning"의 일부입니다.
저자: Byeongchang Kim (GIST), Jae-min Jung (Kyung Hee Univ.), Yoo-hee Park et al. (Korea Univ.)
License: MIT
