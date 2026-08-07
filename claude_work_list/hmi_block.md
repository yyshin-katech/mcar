## 특정 타이밍에 주행 경로 HMI 블로킹 표시
- 하네스로 작업
- 차량이 link_548, 550, 552, 417 위에 있을때 아래 작업 동작
- web_hmi의 지도 영역에 다음 좌표들을 표시하고 붉은색 30%정도 투명한 박스생성
"regions": [
          {
            "anchor": {
              "lat": 373556181,
              "long": 1267288498
            }
          },
          {
            "anchor": {
              "lat": 373555997,
              "long": 1267288802
            }
          },
          {
            "anchor": {
              "lat": 373556350,
              "long": 1267288217
            }
          },
          {
            "anchor": {
              "lat": 373560281,
              "long": 1267292439
            }
          },
          {
            "anchor": {
              "lat": 373560450,
              "long": 1267292163
            }
          },
          {
            "anchor": {
              "lat": 373560100,
              "long": 1267292745
            }
          },
          {
            "anchor": {
              "lat": 373560100,
              "long": 1267292745
            }
          }
        ]
		
- 직진 주행 금지 표현
- 좌/우 진행할 수 있도록 화살표로 표시
- /v2x/tim_message/can_go_status 토픽에서 do_not_go_forward: True 데이터가 수신되면 web_hmi에 "전방 직진 주행 금지" 표시
- rviz에 stat_display 모듈로 동작하는 rviz -d workspace_config/ioniq_statdisplay.rviz로 실행하는 화면에서 "전방 직진 주행 금지" 팝업 생성
