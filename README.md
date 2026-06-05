# 2025 극한로봇 경진대회 — 개발 환경

ROS 2 Humble 기반 재난대응 로봇 프로젝트입니다. YOLO 비전으로 목표를 인식하고, Dynamixel 로봇팔과 자율주행 파워트레인이 협동해 임무를 수행합니다.
개발 환경은 Docker로 통일되어 있어, 레포를 clone하고 `docker compose up`만 하면 누구나 동일한 환경에서 작업할 수 있습니다.

## 시스템 한눈에

```
[카메라/LiDAR] ──▶ Jetson (ROS 2 Humble)
                     │  YOLO 인식 · Depth 3D 좌표 · SLAM
                     ▼
              Mission Manager (FSM)
              ├──▶ 로봇팔   : MoveIt 경로계획 → Dynamixel 서보
              └──▶ 파워트레인: 자율주행(Nav2) / CAN 모터
```

> 모든 ROS 2 명령은 **Docker 컨테이너 안**에서 실행합니다. 호스트는 `git`과 `docker compose`에만 씁니다.

---

## 1. 사전 요구사항

| 항목           | 버전       | 확인 명령어              |
| -------------- | ---------- | ------------------------ |
| Ubuntu         | 22.04 이상 | `lsb_release -a`         |
| Docker Engine  | 24.0 이상  | `docker --version`       |
| Docker Compose | v2 이상    | `docker compose version` |
| Git            | 아무 버전  | `git --version`          |

<details>
<summary>Docker 설치 (없을 경우)</summary>

```bash
# Docker 공식 설치 스크립트
curl -fsSL https://get.docker.com | sudo sh
# sudo 없이 docker 사용 (로그아웃 후 재로그인 필요)
sudo usermod -aG docker $USER
```
</details>

---

## 2. 빠른 시작

```bash
# 1) 레포 클론
git clone https://github.com/ksp118/extreme-robot.git
cd extreme-robot

# 2) 이미지 빌드 (첫 빌드는 베이스 이미지 다운로드로 10~20분)
docker compose build

# 3) 컨테이너 시작 — 환경에 맞는 것 하나만
xhost +local:docker && docker compose up -d                          # Ubuntu 네이티브
xhost +local:       && docker compose -f docker-compose.wsl.yml up -d # WSL2 (Windows)

# 4) 컨테이너 진입 (ROS 2 환경 자동 소싱됨)
docker exec -it ros2_humble bash
```

컨테이너 안에서 빌드·실행:

```bash
cd /root/ros2_ws
colcon build
source install/setup.bash
```

`./ros2_ws`는 호스트와 컨테이너가 공유합니다. 호스트에서 `ros2_ws/src/`를 수정하면 컨테이너에 즉시 반영됩니다.
빌드 산출물(`build/`, `install/`, `log/`)은 `.gitignore`로 제외되니, 각자 컨테이너에서 `colcon build` 하세요.

---

## 3. 패키지 구성 (`ros2_ws/src/`)

| 패키지 | 역할 |
| ------ | ---- |
| **dynamixel_control** | 핵심 런타임. `yolo_detection`(카메라+YOLO) → `yolo_bridge`(P제어) → `position_node`(XL430 서보 구동) 3노드 파이프라인 |
| **robot_arm_description** | 로봇팔 URDF(6축 + 그리퍼), `display.launch.py`(RViz 시각화) |
| **robot_arm_moveit_config** | MoveIt 경로계획 설정(SRDF/IK/컨트롤러), `demo.launch.py` |
| **pick_test_pkg** | 그리퍼 단독 테스트 노드(`pick_test_node`) |

> 각 패키지·노드의 상세 구조는 [`CLAUDE.md`](CLAUDE.md) 참고.

---

## 4. 실행법

### 4-1. 로봇팔 URDF 시각화

```bash
# 컨테이너 안에서
cd /root/ros2_ws
colcon build --packages-select robot_arm_description
source install/setup.bash
ros2 launch robot_arm_description display.launch.py
```

RViz와 joint_state_publisher_gui 창이 함께 뜹니다. **RViz가 처음 열리면 모델이 안 보이므로** 한 번만 아래 설정을 해주세요:

1. Displays → **Fixed Frame**을 `map` → `base_link`로 변경
2. 좌하단 **Add → RobotModel** 추가
3. RobotModel을 펼쳐 **Description Topic → Durability Policy**를 `Volatile` → `Transient Local`로 변경

설정 후 슬라이더로 각 관절을 움직여볼 수 있습니다.

### 4-2. MoveIt 경로계획 (시뮬레이션)

```bash
# 컨테이너 안에서
cd /root/ros2_ws
colcon build --packages-select robot_arm_description robot_arm_moveit_config
source install/setup.bash
ros2 launch robot_arm_moveit_config demo.launch.py
```

RViz **MotionPlanning** 패널에서 목표 자세를 정하고 **Plan & Execute**하면 경로가 계산·실행됩니다.
현재는 mock(가상) 하드웨어라 **실제 서보는 움직이지 않고** 시뮬상 관절만 동작합니다.

- Planning Group: `arm`(팔, base_link→link_6) / `gripper`(손가락)
- 목표 지정: 말단 마커 드래그 / Joints 탭 슬라이더 / Goal State 드롭다운(`home`, `<random valid>`)
- 마커가 빨간색 = IK 해 없음 또는 충돌 → 도달 가능 범위로 이동

### 4-3. YOLO 카메라-Dynamixel 추적 파이프라인

USB 카메라로 스마트폰을 감지하고 Dynamixel 모터가 카메라를 추적하는 파이프라인입니다.

```
카메라 → yolo_detection_node → /yolo/target_center
                                        ↓
               dynamixel_position_node ← yolo_to_dynamixel_bridge
```

`privileged: true` 설정 덕분에 USB 카메라(`/dev/video*`)와 Dynamixel(`/dev/ttyUSB0`)은 별도 설정 없이 컨테이너에서 바로 접근 가능합니다. 단, **컨테이너 시작 전에 USB 장치를 연결**해두어야 합니다.

#### 사전 확인

```bash
# 호스트 — USB 카메라 연결 확인
ls /dev/video*
# /dev/video0 ... 숫자가 클수록 최근 연결 장치 (보통 video2 또는 video3이 USB 카메라)

# 컨테이너 안 — 사용 가능한 카메라 인덱스 확인
python3 -c "
import cv2
for i in range(4):
    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
    print(f'video{i}:', cap.isOpened())
    cap.release()
"
# True가 나오는 인덱스 중 가장 큰 번호가 USB 카메라 (보통 2)
```

#### 빌드 및 실행

```bash
# 빌드
cd /root/ros2_ws
colcon build --packages-select dynamixel_control
source install/setup.bash
```

터미널을 3개 열고 (각 터미널에서 `docker exec -it ros2_humble bash` → `source /root/ros2_ws/install/setup.bash`):

```bash
# 터미널 1 — YOLO 감지 노드 (q 누르면 종료, headless면 -p show_window:=false)
ros2 run dynamixel_control yolo_detection --ros-args -p camera_device:=2

# 터미널 2 — YOLO-Dynamixel 브릿지
ros2 run dynamixel_control yolo_bridge

# 터미널 3 — Dynamixel 모터 제어
ros2 run dynamixel_control dynamixel_position
```

감지 결과 확인:

```bash
ros2 topic echo /yolo/target_center
# data: [320, 240]   ← 감지된 객체 중심 [x, y] 픽셀 좌표
```

#### 파라미터 (`yolo_detection_node`)

| 파라미터              | 기본값       | 설명                                        |
| --------------------- | ------------ | ------------------------------------------- |
| `camera_device`       | `0`          | `/dev/videoN`의 N 값 (보통 `2`)             |
| `image_width`         | `640`        | 카메라 캡처 해상도 너비 (px)                |
| `image_height`        | `480`        | 카메라 캡처 해상도 높이 (px)                |
| `model_path`          | `yolov8n.pt` | YOLO 모델 파일 경로                         |
| `target_class`        | `cell phone` | 감지할 COCO 클래스 이름                     |
| `conf_threshold`      | `0.5`        | 감지 신뢰도 임계값 (0.0 ~ 1.0)              |
| `publish_debug_image` | `true`       | 바운딩박스 이미지를 토픽으로 발행할지 여부  |
| `show_window`         | `true`       | 감지 윈도우 표시 여부 (headless 환경엔 false) |

```bash
ros2 run dynamixel_control yolo_detection --ros-args \
  -p camera_device:=2 \
  -p target_class:="cell phone" \
  -p conf_threshold:=0.4 \
  -p show_window:=false
```

#### 발행 토픽

| 토픽                       | 메시지 타입                  | 내용                                  |
| -------------------------- | ---------------------------- | ------------------------------------- |
| `/yolo/target_center`      | `std_msgs/Int32MultiArray`   | 감지된 객체 중심 좌표 `[x, y]` (px)   |
| `/yolo/detection_image`    | `sensor_msgs/Image`          | 바운딩박스가 그려진 디버그 이미지     |
| `/dynamixel/goal_position` | `std_msgs/Int32MultiArray`   | 모터 ID + 목표 위치 `[id, position]`  |

---

## 5. 개발 워크플로우

```bash
# 아침에 시작
git pull
xhost +local:docker && docker compose up -d            # (WSL2는 -f docker-compose.wsl.yml)
docker exec -it ros2_humble bash

# 작업 후 push (호스트에서)
git add . && git commit -m "feat: ..." && git push

# 작업 끝
docker compose down
```

소스코드(`ros2_ws/src/`)만 git으로 관리됩니다. `git pull` 후에는 컨테이너에서 다시 `colcon build` 하세요.

### Dockerfile이 변경된 경우 → 이미지 재빌드 필수

시스템 의존성(apt/pip 패키지)은 **재현성을 위해 Dockerfile에만** 추가합니다. 누군가 Dockerfile을 바꿔 push했다면(예: YOLO·MoveIt 의존성 추가) 반드시 재빌드하세요:

```bash
git pull
docker compose down
docker compose build      # 캐시가 꼬이면 docker compose build --no-cache
docker compose up -d
```

### 새 ROS 2 패키지 추가

```bash
# 직접 만든 패키지
cd /root/ros2_ws/src
ros2 pkg create --build-type ament_python my_package

# apt/pip 패키지는 Dockerfile에 추가 후 재빌드 (위 참고)
```

---

## 6. 트러블슈팅

**GUI 창이 안 뜸**
```bash
xhost +local:docker      # Ubuntu / WSL2는 xhost +local:
echo $DISPLAY            # 보통 :0 또는 :1
```

**WSL2에서 GUI가 갑자기 안 될 때** — WSLg가 죽은 경우. PowerShell(관리자)에서:
```powershell
wsl --shutdown
```
이후 WSL 터미널을 다시 열고 컨테이너를 재시작합니다.

**ros2 명령이 안 됨** — 소싱이 안 된 경우 수동으로:
```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

**colcon build 에러** — 의존성 누락 가능성:
```bash
cd /root/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

**permission denied (ros2_ws 내 파일)** — 호스트/컨테이너 UID 불일치. 호스트에서:
```bash
sudo chown -R $USER:$USER ros2_ws/
```

---

## 7. 브랜치 전략

`main`은 안정 버전만 유지하고, 기능별로 `feat/*` 브랜치에서 작업 후 PR로 merge합니다.

```bash
git checkout -b feat/vision     # 새 기능 브랜치
git add . && git commit -m "feat: 화재 타겟 인식"
git push -u origin feat/vision  # 이후 GitHub에서 PR
```
