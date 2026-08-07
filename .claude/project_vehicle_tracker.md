---
name: vehicle-tracker EC2 배포 현황
description: BSM 기반 차량 위치 관제 시스템 — EC2 배포 구성 및 환경 주의사항
type: project
originSessionId: a5726bf9-12cf-4b64-86e5-fc80d40279c4
---
EC2 13.209.88.22 에 vehicle-tracker:1.0.0 Docker 컨테이너로 배포됨 (호스트 포트 8081 → 컨테이너 8080).

**Why:** 차량 BSM(JSON/WebSocket) → EC2 서버 → Leaflet 웹페이지에서 실시간 차량 위치·속도·방향 관제.

**How to apply:** 서버 재배포·수정 시 아래 환경 사항 고려.

## 구성 파일 위치

- 서버: `server/` (Node.js Express+ws)
- 프론트엔드: `frontend/` (Leaflet 1.9.4 CDN)
- 배포: `deploy/docker-compose.prod.yml`, `deploy/env`
- ROS 포워더: `src/visualization/bsm_uploader/`
- 배포 메모: `deploy/README.md`

## EC2 환경 주의사항

- **포트 8080 점유**: nova-client(docker-proxy) 가 이미 사용 중 → 8081 사용
- **AWS CLI 미설치**: EC2에서 aws 명령 불가, SG 확인은 콘솔에서
- **NIC 이름**: `ens5` (eth0 아님) — tcpdump 시 `-i ens5`
- **로컬 Docker 없음**: WSL2 환경에 Docker 없음 → 빌드는 EC2에서 직접 (`docker build` on EC2)
- **SG**: `sg-0cf772580602f467a` (launch-wizard-2), 인바운드 TCP 8081 허용

## EC2 배포 절차 (무중단 갱신)

```bash
KEY=/home/sim/ec2-portable/nova.pem
rsync -avz -e "ssh -i $KEY" --exclude node_modules server/ ubuntu@13.209.88.22:~/vehicle-tracker/server/
rsync -avz -e "ssh -i $KEY" frontend/ ubuntu@13.209.88.22:~/vehicle-tracker/frontend/
ssh -i $KEY ubuntu@13.209.88.22 'cd ~/vehicle-tracker && sudo docker build -t vehicle-tracker:1.0.0 -f server/Dockerfile . && sudo docker compose --env-file env -f docker-compose.prod.yml up -d --no-deps vehicle-tracker'
```

## ROS 포워더 기동

```bash
roslaunch bsm_uploader bsm_uploader.launch
# vehicle_id=EV01, server_url=ws://13.209.88.22:8081/ws/ingest
# /sensors/gps/inspva + /sensors/v_can 둘 다 필요 (INSPVA 없으면 drop)
```
