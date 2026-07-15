# Vehicle Tracker — EC2 배포 메모

대상: EC2 13.209.88.22 (ubuntu) / `~/vehicle-tracker/` / 호스트 포트 8081

## 1. 사전 점검

- [ ] AWS Security Group 인바운드 TCP 8081 허용 (사용자 콘솔에서 처리)
- [ ] EC2 디스크 여유 ≥ 1 GB (`df -h /`)
- [ ] EC2 메모리 여유 ≥ 200 MB (`free -h`) — 컨테이너 memory limit 192M 설정됨

## 2. 로컬에서 이미지 빌드 + tar 추출

```bash
cd /home/sim/mcar
docker build -t vehicle-tracker:1.0.0 -f server/Dockerfile .
docker save vehicle-tracker:1.0.0 | gzip > /tmp/vehicle-tracker_1.0.0.tar.gz
```

## 3. EC2 로 업로드

```bash
KEY=/home/sim/ec2-portable/nova.pem
ssh -i $KEY ubuntu@13.209.88.22 'mkdir -p ~/vehicle-tracker'
scp -i $KEY /tmp/vehicle-tracker_1.0.0.tar.gz ubuntu@13.209.88.22:/tmp/
scp -i $KEY deploy/docker-compose.prod.yml deploy/env ubuntu@13.209.88.22:~/vehicle-tracker/
```

## 4. EC2 에서 적재 + 기동

```bash
ssh -i $KEY ubuntu@13.209.88.22
sudo docker load < /tmp/vehicle-tracker_1.0.0.tar.gz
sudo docker images | grep vehicle-tracker
rm /tmp/vehicle-tracker_1.0.0.tar.gz

cd ~/vehicle-tracker
sudo docker compose --env-file env -f docker-compose.prod.yml up -d
sudo docker compose -f docker-compose.prod.yml ps
sudo docker logs vehicle-tracker --tail 30
```

## 5. 검증

```bash
# EC2 내부
curl -sI http://127.0.0.1:8081/                       # 200
curl -s  http://127.0.0.1:8081/api/health             # {"ok":true,...}
curl -s  http://127.0.0.1:8081/api/vehicles           # {"server_ts_unix_ms":...,"vehicles":{}}

# 외부 (Security Group 허용 후 로컬에서)
curl -sI http://13.209.88.22:8081/
```

## 6. 무중단 갱신

```bash
# 1) 로컬 빌드 → IMAGE_VERSION 만 갱신 후 scp + load
# 2) EC2 에서:
cd ~/vehicle-tracker
sudo docker compose --env-file env -f docker-compose.prod.yml up -d --no-deps vehicle-tracker
```

## 7. 롤백 / 정지

```bash
# 정지 (볼륨/네트워크 보존)
sudo docker compose -f docker-compose.prod.yml stop

# 완전 제거 (이미지/네트워크 정리 — DB 없으므로 down 안전)
sudo docker compose -f docker-compose.prod.yml down
```

## 8. 차량 측 forwarder

차량 PC (`~/mcar/`):
```bash
catkin_make --pkg bsm_uploader
source devel/setup.bash
roslaunch bsm_uploader bsm_uploader.launch vehicle_id:=EV01
```
default `server_url`: `ws://13.209.88.22:8081/ws/ingest`
