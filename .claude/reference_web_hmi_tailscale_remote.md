---
name: reference-web-hmi-tailscale-remote
description: "web_hmi 를 Tailscale 경유 원격 PC 브라우저에서 보는 방법 — 코드 수정 불필요, NUC Tailscale IP 100.113.210.119 (2026-09-23 사용자 확인)"
metadata:
  node_type: memory
  type: reference
  originSessionId: a5af4f3e-ba4d-4955-905f-6d2a2505203f
  modified: 2026-09-23T14:06:05.373Z
---

사무실 NUC(`katech-office-nuc`, Tailscale IP `100.113.210.119`)에서 web_hmi 를 띄우면 Tailscale 로 연결된 원격 PC 브라우저에서 **코드 수정 없이** 볼 수 있다. 2026-09-23 사용자가 윈도우 PC `home`(Tailscale 100.87.52.100)에서 SSH 접속한 상태로 동작 확인.

- 접속 URL: `http://100.113.210.119:8088/index_threejs_f1.html`
- 동작 근거: `web_server.py` 가 `0.0.0.0:8088`, rosbridge 가 `0.0.0.0:9090` 바인딩 + 프론트가 ws 주소를 `window.location.hostname` 으로 구성 → 원격 호스트 주소로 자동 연결. NUC 의 ufw 는 비활성.
- SSH 세션에서 실행 시: DISPLAY 없음 → `open_browser:=false`. SSH 끊김에 roslaunch 가 같이 죽으므로 tmux(`/usr/local/bin/tmux`) 안에서 실행. bag 재생은 `web_hmi_replay.launch bag:=<파일명> open_browser:=false` (같은 포트). `bag:=` 은 경로가 아니라 `bag_dir`(기본 `~/bag_data`) 안의 **파일명만** — 다른 폴더는 `bag_dir:=$HOME/bag/<폴더>` — [[reference-tim-pedes-bag-replay]].
- 페이지는 뜨는데 연결 끊김 표시 → 9090 차단 의심. Tailscale 관리 콘솔 ACL 의 포트 제한 확인.
