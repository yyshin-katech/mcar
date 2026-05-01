# Ioniq5 HMI · A-1 (Solid Top-down · Amber) → ROS1 PyQt 구현 가이드

> 변형 캔버스의 **A · Vehicle View Style** 첫 번째 시안 — 차콜 + 앰버, 탑다운 차량 뷰 솔리드 — 을 ROS1 / Ubuntu 20.04 / Python 3.8 / PyQt5 환경에서 그대로 동작하는 패키지로 옮기는 절차서입니다. Claude CLI에 그대로 붙여 넣어 단계별로 실행하면 됩니다.

---

## 0. 사전 환경

| 항목 | 버전 | 비고 |
|---|---|---|
| OS | Ubuntu 20.04 LTS | |
| ROS | Noetic | `/opt/ros/noetic` |
| Python | 3.8.x | 시스템 기본 |
| PyQt5 | 5.14+ | `python3-pyqt5` |
| Catkin | catkin_tools 권장 | |

```bash
sudo apt update
sudo apt install -y \
  python3-pyqt5 python3-pyqt5.qtsvg python3-pyqt5.qtwebengine \
  python3-rospkg python3-rosdep python3-catkin-tools \
  ros-noetic-rospy ros-noetic-std-msgs ros-noetic-sensor-msgs \
  ros-noetic-geometry-msgs ros-noetic-nav-msgs
```

> **폰트**: Inter 와 JetBrains Mono 는 Ubuntu 20.04 기본 저장소에 없습니다. 부록 A 절차로 수동 설치하세요. 폰트가 없으면 `Noto Sans CJK KR` + `DejaVu Sans Mono` 로 자동 폴백하도록 코드에 명시합니다.

---

## 1. 디자인 시스템 요약 (구현 전 합의)

### 1.1 색상 토큰

| Token | Hex | 용도 |
|---|---|---|
| `bg-0` | `#0E0F11` | 페이지 배경 |
| `bg-1` | `#15171A` | 패널 |
| `bg-2` | `#1C1F23` | 카드 / stat |
| `bg-3` | `#232830` | 강조 셀 |
| `line-soft` | `#232830` | 1px 분할선 |
| `text-0` | `#F4F5F7` | 본문 |
| `text-1` | `#C8CCD2` | 보조 |
| `text-2` | `#8A9099` | 라벨 |
| `text-3` | `#5A616B` | 캡션 |
| `amber-0` | `#FFB547` | 메인 액센트 |
| `amber-1` | `#F59E2C` | 액센트 그라데이션 |
| `amber-2` | `#C97A14` | 어두운 액센트 |
| `red` | `#F87171` | 위험/REC |
| `green` | `#34D399` | OK |
| `glass` | `#15171A` | 차량 윈도우 |

### 1.2 타이포

- **Display**: `Inter 300/400/500/600` — 큰 숫자 / 라벨
- **Mono**: `JetBrains Mono 400/500` — 모든 측정치, 토픽명, 시간
- 모든 숫자는 `tabular-nums` 효과를 위해 폰트의 OpenType 기능 활성화

### 1.3 레이아웃 (1600 × 900)

```
┌──────────── topbar (64px) ────────────┐
│ brand         system tags        clock│
├─────┬──────────────────────────┬──────┤
│Left │       Center Stage       │Right │
│360px│        (탑다운 뷰)        │360px │
│     │  거리 링 + 감지 콘 +      │      │
│     │  Tesla 탑다운 차량 +      │      │
│     │  REC 배너                 │      │
├─────┴──────────────────────────┴──────┤
│           bottom bar (88px)            │
└────────────────────────────────────────┘
```

---

## 2. 패키지 스캐폴딩

```bash
cd ~/catkin_ws/src
catkin_create_pkg pyqt_hmi rospy std_msgs sensor_msgs geometry_msgs nav_msgs
cd pyqt_hmi
mkdir -p scripts/widgets scripts/utils launch resources/fonts resources/qss
touch scripts/main_display.py
chmod +x scripts/main_display.py
```

`package.xml` 에 의존성 추가:
```xml
<exec_depend>python3-pyqt5</exec_depend>
```

`CMakeLists.txt` 의 `catkin_install_python` 섹션 활성화:
```cmake
catkin_install_python(PROGRAMS
  scripts/main_display.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

---

## 3. 디자인 토큰 모듈

`scripts/utils/theme.py`

```python
# -*- coding: utf-8 -*-
"""Design tokens for A-1 (Solid top-down · Amber)."""
from PyQt5.QtGui import QColor, QFont, QFontDatabase

# ─── colors ─────────────────────────────────────────────────────────
BG_0 = QColor("#0E0F11")
BG_1 = QColor("#15171A")
BG_2 = QColor("#1C1F23")
BG_3 = QColor("#232830")
LINE = QColor("#232830")
TEXT_0 = QColor("#F4F5F7")
TEXT_1 = QColor("#C8CCD2")
TEXT_2 = QColor("#8A9099")
TEXT_3 = QColor("#5A616B")
AMBER_0 = QColor("#FFB547")
AMBER_1 = QColor("#F59E2C")
AMBER_2 = QColor("#C97A14")
RED   = QColor("#F87171")
GREEN = QColor("#34D399")
GLASS = QColor("#15171A")

# ─── fonts (with fallback) ──────────────────────────────────────────
def _pick(family_candidates, default):
    db = QFontDatabase()
    for fam in family_candidates:
        if fam in db.families():
            return fam
    return default

def display_font(size=14, weight=QFont.Normal):
    fam = _pick(["Inter", "Pretendard", "Noto Sans CJK KR"], "Sans Serif")
    f = QFont(fam, size, weight)
    f.setStyleStrategy(QFont.PreferAntialias)
    return f

def mono_font(size=12, weight=QFont.Normal):
    fam = _pick(["JetBrains Mono", "DejaVu Sans Mono", "Monospace"], "Monospace")
    f = QFont(fam, size, weight)
    f.setStyleHint(QFont.TypeWriter)
    return f

# ─── geometry ───────────────────────────────────────────────────────
WIN_W, WIN_H = 1600, 900
TOPBAR_H = 64
BOTTOM_H = 88
LEFT_W = 360
RIGHT_W = 360
RADIUS = 10
```

---

## 4. 전역 스타일시트

`resources/qss/app.qss` — 메인 윈도우에 `setStyleSheet(open(...).read())` 로 로드.

```css
QWidget { color: #F4F5F7; background: #0E0F11; }
QFrame#topbar, QFrame#bottom { background: #15171A; border-bottom: 1px solid #232830; }
QFrame#bottom { border-top: 1px solid #232830; border-bottom: none; }
QFrame#leftPanel, QFrame#rightPanel { background: #15171A; border-right: 1px solid #232830; }
QFrame#rightPanel { border-right: none; border-left: 1px solid #232830; }
QFrame#stage { background: #0E0F11; }

QLabel[role="section"] {
  color: #5A616B; font-family: "JetBrains Mono"; font-size: 10px;
  letter-spacing: 2px; padding: 10px 0;
}
QLabel[role="stat-k"] { color: #5A616B; font-family: "JetBrains Mono"; font-size: 10px; }
QLabel[role="stat-v"] { color: #F4F5F7; font-family: "Inter"; font-size: 22px; }

QFrame[role="card"] {
  background: #1C1F23; border: 1px solid #232830; border-radius: 10px;
}
QPushButton {
  background: #1C1F23; border: 1px solid #232830; border-radius: 8px;
  padding: 8px 14px; color: #F4F5F7;
}
QPushButton[variant="primary"] { background: #FFB547; color: #1A1206; border: none; }
QPushButton[variant="danger"]  { background: #1C1F23; color: #F87171; border: 1px solid #3A2024; }
QPushButton:hover { background: #232830; }
```

> Qt 5.14 의 `letter-spacing` 은 일부 위젯에서 무시됩니다. 라벨에는 코드에서 `setLetterSpacing(QFont.AbsoluteSpacing, 2)` 을 적용하세요 (theme 헬퍼에 추가).

---

## 5. 위젯 분해

### 5.1 `widgets/speed_gauge.py` — 원형 게이지 + 숫자

`QPainter` 로 270° 원호를 그리고, 중심에 숫자/단위/모드 라벨을 겹칩니다.

```python
# -*- coding: utf-8 -*-
import math
from PyQt5.QtCore import Qt, QRectF, pyqtSignal
from PyQt5.QtGui import QPainter, QPen, QColor, QLinearGradient
from PyQt5.QtWidgets import QWidget
from utils.theme import (AMBER_0, AMBER_2, BG_3, TEXT_0, TEXT_2, TEXT_3,
                         display_font, mono_font)

class SpeedGauge(QWidget):
    def __init__(self, parent=None, vmax=120):
        super().__init__(parent)
        self._v = 0.0
        self._vmax = vmax
        self._mode = "ECO"
        self.setMinimumSize(272, 272)

    def set_value(self, kmh: float):
        self._v = max(0.0, min(self._vmax, kmh))
        self.update()

    def set_mode(self, mode: str):
        self._mode = (mode or "").upper()
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w = h = min(self.width(), self.height())
        cx, cy = self.width() / 2, self.height() / 2
        r = w / 2 - 18

        # 0 km/h = 225°, 120 km/h = 225 + 270 = 495° (CCW positive in Qt = -16ths)
        start = 225 * 16
        span_total = -270 * 16
        progress = self._v / self._vmax

        # background track
        p.setPen(QPen(BG_3, 2))
        p.drawArc(QRectF(cx - r, cy - r, 2 * r, 2 * r), start, span_total)

        # progress arc with gradient
        grad = QLinearGradient(cx - r, cy, cx + r, cy)
        grad.setColorAt(0.0, AMBER_2)
        grad.setColorAt(1.0, AMBER_0)
        pen = QPen(grad, 3, Qt.SolidLine, Qt.RoundCap)
        p.setPen(pen)
        p.drawArc(QRectF(cx - r, cy - r, 2 * r, 2 * r),
                  start, int(span_total * progress))

        # ticks
        for i in range(0, self._vmax + 1, 10):
            a = math.radians(225 - 270 * (i / self._vmax))
            major = (i % 20 == 0)
            inner = r - (14 if major else 8)
            past = (i / self._vmax) <= progress
            x0 = cx + inner * math.cos(a); y0 = cy - inner * math.sin(a)
            x1 = cx + r * math.cos(a);     y1 = cy - r * math.sin(a)
            p.setPen(QPen(AMBER_0 if past else BG_3, 1.5 if major else 1))
            p.drawLine(int(x0), int(y0), int(x1), int(y1))

        # number
        p.setPen(TEXT_0)
        f = display_font(56, weight=300)
        p.setFont(f)
        text = str(int(round(self._v)))
        p.drawText(self.rect().adjusted(0, -8, 0, -8), Qt.AlignCenter, text)

        # unit + mode
        p.setPen(TEXT_3)
        p.setFont(mono_font(9))
        p.drawText(self.rect().adjusted(0, 56, 0, 56),
                   Qt.AlignHCenter | Qt.AlignVCenter, "KM / H")
        p.setPen(TEXT_2)
        p.drawText(self.rect().adjusted(0, 78, 0, 78),
                   Qt.AlignHCenter | Qt.AlignVCenter, f"{self._mode} • DRIVE")
```

ROS subscriber 연결:
```python
# main_display.py 안에서
gauge = SpeedGauge()
def on_vehicle(msg):  # geometry_msgs/TwistStamped 또는 사용자 정의
    kmh = msg.twist.linear.x * 3.6
    QMetaObject.invokeMethod(gauge, "set_value", Qt.QueuedConnection,
                             Q_ARG(float, kmh))
```

### 5.2 `widgets/drive_mode.py` — P / R / N / D 4-cell 토글

`QHBoxLayout` 안에 `QFrame` 4개. 활성 셀만 `background: #FFB547; color: #1A1206;`.

### 5.3 `widgets/stat_card.py` — 카드 (Battery / Range / Motor / Pack V / Lat / Lon)

라벨 + 값 두 줄. `QGridLayout` 으로 2×2.

### 5.4 `widgets/vehicle_view.py` — **탑다운 차량 + 거리 링 + 감지 콘**

핵심 위젯입니다. SVG 의 좌표를 Qt 단위로 그대로 옮깁니다.

```python
# -*- coding: utf-8 -*-
import math
from PyQt5.QtCore import Qt, QRectF, QPointF
from PyQt5.QtGui import (QPainter, QPainterPath, QPen, QColor, QLinearGradient,
                         QBrush, QFont)
from PyQt5.QtWidgets import QWidget
from utils.theme import (BG_0, BG_1, BG_2, BG_3, LINE, AMBER_0, AMBER_1,
                         RED, TEXT_3, mono_font)

class VehicleView(QWidget):
    """Center stage: range rings + detection cone + Tesla-style top-down car."""

    RING_RADII = [(80, "10m", False), (160, "25m", True),
                  (240, "50m", False), (320, "100m", False)]

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumHeight(560)

    # ─── helpers ────────────────────────────────────────────
    def _center(self):
        return QPointF(self.width() / 2, self.height() / 2)

    # ─── paint pipeline ─────────────────────────────────────
    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        # background
        p.fillRect(self.rect(), BG_0)
        c = self._center()
        p.translate(c)

        self._draw_rings(p)
        self._draw_detection_cone(p)
        self._draw_vehicle(p)

        p.resetTransform()
        self._draw_corners(p)

    def _draw_rings(self, p):
        p.save()
        for r, label, solid in self.RING_RADII:
            pen = QPen(QColor(255, 181, 71, 56) if solid
                       else QColor(200, 204, 210, 30),
                       1, Qt.SolidLine if solid else Qt.DashLine)
            p.setPen(pen)
            p.setBrush(Qt.NoBrush)
            p.drawEllipse(QPointF(0, 0), r, r)
            # label
            p.setPen(TEXT_3)
            p.setFont(mono_font(8))
            rect = QRectF(r - 22, -8, 44, 14)
            p.fillRect(rect, BG_0)
            p.drawText(rect, Qt.AlignCenter, label)
        p.restore()

    def _draw_detection_cone(self, p):
        """Forward amber cone + rear blue cone (top-down)."""
        p.save()
        # forward cone (up direction = -y)
        path = QPainterPath()
        path.moveTo(0, 0)
        path.lineTo(200, -260)
        path.arcTo(QRectF(-330, -330, 660, 660), 60, 60)
        path.closeSubpath()
        p.setBrush(QColor(255, 181, 71, 26))
        p.setPen(QPen(QColor(255, 181, 71, 90), 1))
        p.drawPath(path)

        # rear cone
        path = QPainterPath()
        path.moveTo(0, 0)
        path.lineTo(130, 200)
        path.arcTo(QRectF(-240, -240, 480, 480), -60, -60)
        path.closeSubpath()
        p.setBrush(QColor(96, 165, 250, 16))
        p.setPen(QPen(QColor(96, 165, 250, 64), 1))
        p.drawPath(path)
        p.restore()

    def _draw_vehicle(self, p):
        """Tesla-style top-down car body, drawn around (0,0)."""
        p.save()
        # body gradient
        body = QLinearGradient(0, -64, 0, 64)
        body.setColorAt(0.00, QColor("#E8EAED"))
        body.setColorAt(0.35, QColor("#F4F5F7"))
        body.setColorAt(0.65, QColor("#D5D8DC"))
        body.setColorAt(1.00, QColor("#A8ADB3"))

        # rounded body (slightly tapered)
        path = QPainterPath()
        path.moveTo(-22, -58)
        path.quadTo(-22, -64, -16, -64)
        path.lineTo(16, -64)
        path.quadTo(22, -64, 22, -58)
        path.lineTo(24, -30); path.lineTo(26, 0)
        path.lineTo(24, 30);  path.lineTo(22, 58)
        path.quadTo(22, 64, 16, 64)
        path.lineTo(-16, 64)
        path.quadTo(-22, 64, -22, 58)
        path.lineTo(-24, 30); path.lineTo(-26, 0)
        path.lineTo(-24, -30); path.closeSubpath()
        p.setBrush(QBrush(body))
        p.setPen(QPen(QColor(0, 0, 0, 64), 0.5))
        p.drawPath(path)

        # specular highlight
        p.setBrush(QColor(255, 255, 255, 38))
        p.setPen(Qt.NoPen)
        p.drawRect(QRectF(-10, -58, 20, 116))

        # glass
        glass = QLinearGradient(0, -36, 0, 42)
        glass.setColorAt(0, QColor("#1F2329"))
        glass.setColorAt(1, QColor("#0E1115"))
        p.setBrush(QBrush(glass))
        p.setPen(QPen(QColor(0, 0, 0, 100), 0.5))
        # front windshield trapezoid
        ws = QPainterPath()
        ws.moveTo(-18, -36); ws.lineTo(18, -36)
        ws.lineTo(14, -16);  ws.lineTo(-14, -16); ws.closeSubpath()
        p.drawPath(ws)
        # roof
        p.drawRoundedRect(QRectF(-14, -15, 28, 36), 2, 2)
        # rear windshield
        rs = QPainterPath()
        rs.moveTo(-14, 22); rs.lineTo(14, 22)
        rs.lineTo(18, 42);  rs.lineTo(-18, 42); rs.closeSubpath()
        p.drawPath(rs)

        # mirrors
        p.setBrush(QBrush(body))
        p.drawEllipse(QPointF(-24, -26), 4, 3)
        p.drawEllipse(QPointF( 24, -26), 4, 3)

        # door cut lines
        p.setPen(QPen(QColor(0, 0, 0, 38), 0.4))
        p.drawLine(-22, -10, 22, -10)
        p.drawLine(-22,  14, 22,  14)

        # signature lights
        p.setBrush(AMBER_0); p.setPen(Qt.NoPen)
        p.drawRect(QRectF(-18, -62, 36, 1.5))
        p.setBrush(QColor("#F87171"))
        p.drawRect(QRectF(-18, 60.5, 36, 1.5))

        # heading chevron above car
        p.setPen(QPen(AMBER_0, 1.4, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        p.setBrush(Qt.NoBrush)
        chev = QPainterPath()
        chev.moveTo(-6, -78); chev.lineTo(0, -84); chev.lineTo(6, -78)
        p.drawPath(chev)
        p.restore()

    def _draw_corners(self, p):
        p.setPen(TEXT_3)
        p.setFont(mono_font(9))
        p.drawText(QRectF(20, 20, 220, 16), Qt.AlignLeft, "FRONT · LIDAR + CAM")
        p.drawText(QRectF(20, self.height() - 36, 200, 16),
                   Qt.AlignLeft, "SCALE · 1 : 200")
        p.drawText(QRectF(self.width() - 220, self.height() - 36, 200, 16),
                   Qt.AlignRight, "PROJ · UTM 52N")
```

ROS 통합:
- `/objects` (예: `autoware_msgs/DetectedObjectArray` 또는 사용자 정의) 가 들어오면 `self._objects = ...` 에 저장 후 `self.update()`. paintEvent 안에서 `_draw_objects(p)` 를 추가해 `TrafficObjects` 박스+라벨을 그리세요.
- `/planning/path` 가 들어오면 핑크 폴리라인으로 `p.drawPolyline` 추가.

### 5.5 `widgets/sensor_list.py` — 토픽 + Hz + 상태 LED

`QTableWidget` 보다 `QFrame + QGridLayout` 한 줄씩이 더 잘 맞습니다 (회색 그리드 라인이 어울림). Hz 측정은 `rospy.Subscriber` 의 콜백 카운터로 1초 단위로 갱신하세요:

```python
from collections import deque
class TopicMonitor:
    def __init__(self):
        self.timestamps = deque(maxlen=200)
    def tick(self):
        self.timestamps.append(rospy.Time.now().to_sec())
    def hz(self):
        if len(self.timestamps) < 2: return 0.0
        dt = self.timestamps[-1] - self.timestamps[0]
        return (len(self.timestamps) - 1) / dt if dt > 0 else 0.0
```

### 5.6 `widgets/bag_banner.py` — REC 펄싱

빨간 점은 `QPropertyAnimation` 으로 `opacity` 0.4 ↔ 1.0 사이 1초 주기. `subprocess.Popen(["rosbag", "record", "-O", path, *topics])` 로 시작/종료.

---

## 6. 메인 윈도우 조립

`scripts/widgets/main_window.py`

```python
# -*- coding: utf-8 -*-
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QMainWindow, QFrame, QHBoxLayout, QVBoxLayout,
                             QGridLayout, QLabel, QPushButton, QSizePolicy)
from utils.theme import (WIN_W, WIN_H, TOPBAR_H, BOTTOM_H, LEFT_W, RIGHT_W,
                         display_font, mono_font, BG_1, AMBER_0, TEXT_2, TEXT_3)
from widgets.speed_gauge import SpeedGauge
from widgets.drive_mode import DriveModeRow
from widgets.stat_card import StatCard
from widgets.vehicle_view import VehicleView
from widgets.sensor_list import SensorList
from widgets.bag_banner import BagBanner

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Ioniq5 HMI · A-1")
        self.setFixedSize(WIN_W, WIN_H)

        # central layout: 3-col grid + topbar + bottom
        root = QFrame(); root.setObjectName("root")
        v = QVBoxLayout(root); v.setContentsMargins(0,0,0,0); v.setSpacing(0)

        v.addWidget(self._build_topbar(), 0)

        mid = QFrame()
        mh = QHBoxLayout(mid); mh.setContentsMargins(0,0,0,0); mh.setSpacing(0)
        mh.addWidget(self._build_left(),  0)
        mh.addWidget(self._build_stage(), 1)
        mh.addWidget(self._build_right(), 0)
        v.addWidget(mid, 1)

        v.addWidget(self._build_bottom(), 0)
        self.setCentralWidget(root)

        # demo timer (replace with ROS subs)
        self._demo_t = QTimer(self); self._demo_t.timeout.connect(self._demo_tick)
        self._demo_t.start(1000)

    # … (omitted: each _build_* returns a configured QFrame) …
```

(각 `_build_*` 의 전체 코드는 부록 B 에 첨부)

---

## 7. ROS 진입점

`scripts/main_display.py`

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os, sys, signal, rospy
from PyQt5.QtWidgets import QApplication

# add scripts dir to PYTHONPATH so utils/ widgets/ resolve
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))

from widgets.main_window import MainWindow

def main():
    rospy.init_node("ioniq5_hmi", anonymous=False, disable_signals=True)
    signal.signal(signal.SIGINT, signal.SIG_DFL)  # Ctrl-C in Qt
    app = QApplication(sys.argv)
    qss_path = os.path.join(os.path.dirname(__file__),
                            "..", "resources", "qss", "app.qss")
    if os.path.exists(qss_path):
        with open(qss_path) as f: app.setStyleSheet(f.read())
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
```

---

## 8. Launch 파일

`launch/hmi.launch`

```xml
<launch>
  <node pkg="pyqt_hmi" type="main_display.py" name="ioniq5_hmi"
        output="screen" required="true"/>
</launch>
```

빌드 + 실행:
```bash
cd ~/catkin_ws
catkin build pyqt_hmi
source devel/setup.bash
roslaunch pyqt_hmi hmi.launch
```

---

## 9. ROS 토픽 매핑 (예시)

| HMI 영역 | ROS 토픽 (예시) | 메시지 |
|---|---|---|
| 속도 게이지 | `/vehicle/twist` | `geometry_msgs/TwistStamped` |
| Drive 모드 | `/vehicle/gear` | `std_msgs/String` (P/R/N/D) |
| 배터리/주행거리 | `/vehicle/powertrain` | 사용자 정의 |
| GNSS | `/gnss/fix` | `sensor_msgs/NavSatFix` |
| 객체 박스 | `/perception/objects` | `autoware_msgs/DetectedObjectArray` 또는 `vision_msgs/Detection3DArray` |
| 계획 경로 | `/planning/path` | `nav_msgs/Path` |
| 시스템 상태 LED | `/diagnostics` | `diagnostic_msgs/DiagnosticArray` |
| Bag 알림 | `/hmi/bag/event` | `std_msgs/String` |

> Qt 메인 스레드 외부에서 위젯을 직접 만지지 마세요. ROS 콜백에서는 `pyqtSignal` 을 emit 하거나 `QMetaObject.invokeMethod(..., Qt.QueuedConnection)` 으로 메인 스레드에 위임하세요.

---

## 10. Claude CLI 워크플로

```bash
cd ~/catkin_ws/src/pyqt_hmi
claude  # 인터랙티브 모드 진입
```

CLI 안에서 단계별 프롬프트 (이 가이드 그대로 복붙):

1. *"§3 의 `theme.py` 를 `scripts/utils/theme.py` 로 만들어줘."*
2. *"§4 의 QSS 를 `resources/qss/app.qss` 로 저장해줘."*
3. *"§5.1 의 `SpeedGauge` 위젯을 만들고 임포트가 깨지지 않는지 확인해줘."*
4. *"§5.4 의 `VehicleView` 를 만들고 `python3 -c 'from widgets.vehicle_view import VehicleView'` 로 검증해줘."*
5. *"§6 의 메인 윈도우 + §7 의 진입점을 채우고 `roslaunch pyqt_hmi hmi.launch` 가 뜨는지 봐줘."*
6. *"각 위젯에 ROS subscriber 를 §9 매핑대로 붙여줘. 콜백은 `pyqtSignal` 로 메인 스레드에 위임."*

각 단계 후 `git add -A && git commit -m "step N"` 로 커밋, 문제가 생기면 `git diff` 로 롤백.

---

## 부록 A — Inter / JetBrains Mono 설치

Ubuntu 20.04 기본 저장소엔 둘 다 없습니다.

```bash
mkdir -p ~/.local/share/fonts

# Inter
wget -O /tmp/Inter.zip \
  https://github.com/rsms/inter/releases/download/v4.0/Inter-4.0.zip
unzip -j /tmp/Inter.zip "Inter Desktop/Inter-*.otf" -d ~/.local/share/fonts/

# JetBrains Mono
wget -O /tmp/JBMono.zip \
  https://download.jetbrains.com/fonts/JetBrainsMono-2.304.zip
unzip -j /tmp/JBMono.zip "fonts/ttf/*.ttf" -d ~/.local/share/fonts/

fc-cache -f
fc-list | grep -E "Inter|JetBrains" | head
```

설치 안 해도 `theme.py` 의 폴백으로 `Noto Sans CJK KR` + `DejaVu Sans Mono` 가 자동 적용됩니다.

## 부록 B — 메인 윈도우 빌더 전체 코드

(필요하면 별도 답변에 첨부 가능. 본문에 모두 넣으면 너무 길어집니다 — Claude CLI 에서 §10 프롬프트로 생성하세요.)

## 부록 C — 자주 마주치는 문제

| 증상 | 원인 / 해결 |
|---|---|
| 위젯이 안 뜨고 즉시 종료 | `init_node` 에서 `disable_signals=True` 빠짐 → SIGINT 가 ROS 와 Qt 둘 다 잡으려다 충돌 |
| 한글이 □ 로 표시 | `Noto Sans CJK KR` 미설치. `sudo apt install fonts-noto-cjk` |
| `QObject::moveToThread` 경고 | ROS 콜백에서 위젯 직접 호출. `pyqtSignal` 로 우회 |
| `ImportError: PyQt5.QtSvg` | `sudo apt install python3-pyqt5.qtsvg` |
| 게이지 텍스트가 흐리게 보임 | `setRenderHint(QPainter.TextAntialiasing)` 추가 |
| 16:9 비율이 안 맞음 | `setFixedSize(1600, 900)` 사용. 디스플레이가 더 작으면 `showMaximized()` + 내부 컨텐츠를 `QGraphicsView` 로 스케일 |

## 부록 D — Python 3.8 + Noetic 호환 메모

- f-string `=` 디버그 (`f"{x=}"`) 는 3.8 부터 사용 가능 — 그대로 OK
- `from __future__ import annotations` 는 권장하지 않음 (`rospy.Subscriber` 콜백 타입힌트 충돌)
- `PyQt5.sip` 임포트는 `from PyQt5 import sip` 가 표준 (5.11+)
- `signal.signal(SIGINT, SIG_DFL)` 는 반드시 `QApplication` 생성 **이전** 에 호출

---

## 끝.

§10 프롬프트 1번부터 순차로 실행하면 1시간 이내에 첫 화면이 뜹니다. 막히면 §부록 C 부터 확인하세요.
