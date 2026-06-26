# 작업 인수인계 지시서

> **대상**: 다음 Claude Code 세션  
> **최종 업데이트**: 2026-06-25  
> **기준 문서**: `/home/jo/ros2_ws/CLAUDE.md` (전체 통합 계획)  
> **레포 경로**: `/home/jo/ros2_ws/extreme-robot/`  
> **ROS2 소스**: `extreme-robot/ros2_ws/src/`

---

## 현재 완료된 작업

### 신규 패키지 (모두 빌드 완료)

| 패키지 | 위치 | 상태 |
|--------|------|------|
| `robot_arm_msgs` | `src/robot_arm_msgs/` | ✅ 빌드 완료 |
| `robot_arm_perception` | `src/robot_arm_perception/` | ✅ 빌드 완료 |
| `dynamixel_control` | `src/dynamixel_control/` | ✅ 기존 + 스켈레톤 추가 |

### robot_arm_msgs — 메시지 5개 정의 완료

```
msg/DetectedObject.msg         int32 class_id / string class_name / float32 confidence
                               geometry_msgs/Pose pose / sensor_msgs/RegionOfInterest bbox
msg/DetectedObjectArray.msg    std_msgs/Header header / DetectedObject[] objects
msg/ArrivalStatus.msg          Header / int32 mission_id / string status
msg/ChassisMode.msg            Header / string mode
msg/ArmStatus.msg              Header / int32 mission_id / string status
```

### robot_arm_perception — Phase 2 Step 1·2 완료

**파일**: `src/robot_arm_perception/robot_arm_perception/perception_node.py`

- **Step 1 완료**: YOLO 추론 → `class_id` / `class_name` / `confidence` / `bbox` 채우기
  - ultralytics YOLO 로드 (TensorRT `.engine` 캐시 지원, `_resolve_model()`)
  - RealSense D435i 파이프라인 (yolo_depth_3d.py 포팅, `_latest_frames()`)
  - `camera_mode` 파라미터: `realsense`(기본) / `test`(정지 이미지)
  - `/detected_objects` (`DetectedObjectArray`) publish, 30fps

- **Step 2 완료**: ArUco `estimatePoseSingleMarkers` → `pose` 채우기
  - ArUco DICT_5X5_100, marker_length=0.10m
  - 카메라 내부 파라미터: `config/camera_calibration.yaml` 로드
  - 매칭 로직: ArUco 마커 중심이 YOLO bbox 안에 있으면 해당 객체 pose 채움
  - rvec → quaternion: Shepperd 방법 (`_rvec_to_quat()`, scipy 불필요)
  - 마커 없는 객체는 `pose` 기본값(0) 유지

**설정 파일**: `src/robot_arm_perception/config/camera_calibration.yaml`
- ⚠️ 현재 D435i 공장 추정치. 실측 캘리브레이션 값으로 교체 필요
- ArUco `marker_class_map`: `{0: person, 1: cup}` → 실제 대상 클래스로 교체 필요

**검증 완료** (컨테이너 `ros2_humble` 내):
```
/detected_objects 토픽 echo:
  마커 있는 객체 → pose.position(≠0), pose.orientation(≠단위쿼터니언)
  마커 없는 객체 → pose 기본값(0,0,0,0,0,0,1) 유지
```

---

## 다음 작업 (Phase 2 Step 3)

### 목표: `/pick_target` 선별 로직 구현

**CLAUDE.md 원문**: "픽 대상 선별 로직 (클래스 + confidence 임계값 + '한 번에 하나' 규칙)"

#### 구현 위치

`src/robot_arm_perception/robot_arm_perception/perception_node.py`의 `PerceptionNode` 클래스에 추가.

현재 `__init__`에 주석으로 남겨둔 부분:
```python
# /pick_target은 Phase 2 Step 3(선별 로직)에서 추가
```

#### 구현 내용

1. **`/pick_target` 퍼블리셔 추가** (`__init__`에)
   ```python
   latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
   self.pub_pick = self.create_publisher(DetectedObject, '/pick_target', latched_qos)
   ```
   QoS는 반드시 `transient_local` (latched) — 도착 타이밍에 최신 타깃 유실 방지

2. **`/detected_objects`에서 픽 대상 선별** (`_loop()`의 `_fill_aruco_poses()` 호출 이후에)
   - 조건: ArUco pose가 채워진 객체 (즉 `pose.position.z != 0.0`)
   - 복수 후보 중 `confidence` 가장 높은 것 하나만 선택
   - 선택된 하나만 `pub_pick`으로 publish
   - 후보 없으면 publish 안 함 (이전 latched 값 유지)

3. **파라미터 추가** (`declare_parameter`에)
   ```python
   self.declare_parameter('pick_min_conf', 0.5)   # 픽 대상 최소 confidence
   ```

#### 참고: CLAUDE.md 설계 원칙
- "한 번에 하나만 집음 → `/pick_target` 단일 객체로 충분"
- "픽 대상 클래스 선별 → 하나만 publish"
- `transient_local` QoS: "도착 → 집어" 타이밍에 최신 타깃 안 놓치게

#### 테스트 방법

```bash
docker exec -it ros2_humble bash
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash

# 터미널 1: 노드 실행
ros2 run robot_arm_perception perception_node \
  --ros-args \
  -p camera_mode:=test \
  -p test_image_path:=/tmp/test_aruco_bus.png \
  -p model_path:=/root/ros2_ws/yolov8n.pt \
  -p conf_threshold:=0.3

# 터미널 2: 두 토픽 동시 확인
ros2 topic echo /detected_objects
ros2 topic echo /pick_target
```

확인 기준:
- `/detected_objects`: 전체 객체 배열, 마커 있는 것만 pose 非零
- `/pick_target`: 마커 있는 객체 중 confidence 최고 하나, pose 非零

---

## 그 이후 작업 (Phase 3 — FSM 통합)

### Phase 3 체크리스트 (CLAUDE.md §3 Phase 3)

- [ ] **로봇팔 FSM**: `/arrival_status` 수신 → `/pick_target` 읽기 → MoveIt 픽 시퀀스
  - 신규 노드: `src/robot_arm_perception/robot_arm_perception/arm_fsm_node.py` (또는 별도 패키지)
  - `/arrival_status` (ArrivalStatus) 구독
  - `status == 'ARRIVED_PICKUP'` 수신 시 FSM 전환
  - MoveIt2 `MoveGroupInterface` 로 픽 모션 실행

- [ ] **자세 락**: `/chassis_mode` 구독
  - `mode == 'CORNERING'` 또는 `'ROUGH_TERRAIN'` → 현재 관절각 유지 명령
  - `mode == 'DRIVING'` 복귀 시 언락
  - 구현 방식은 파워트레인 팀과 합의 필요 (오픈 이슈 5번)

- [ ] **완료 신호**: 픽 완료 시 `/arm_status`(ArmStatus, status='DONE') publish
  - 파워트레인이 이 신호 받아야 재출발 가능

- [ ] **파워트레인 연동**: `/detected_objects`에서 신호등/정지선/마커 필터링
  - 파워트레인 팀 쪽 작업이나 인터페이스 스펙은 우리가 정의

---

## 중요 설정값 (확정값 / 미확정값)

| 항목 | 값 | 상태 |
|------|-----|------|
| ArUco dictionary | `DICT_5X5_100` | ✅ 확정 |
| marker_length | `0.10m` (10cm) | ✅ 확정 |
| marker_id ↔ class | 고정 매핑 (yaml에서 설정) | ✅ 확정 |
| camera_matrix 출처 | 별도 캘리브레이션 yaml | ✅ 확정 (값은 미측정) |
| optical frame 이름 | `camera_color_optical_frame` | ⚠️ placeholder, 실값 확인 필요 |
| status 문자열 enum | `ARRIVED_PICKUP`, `DONE` 등 | ⚠️ 파워트레인 팀과 합의 필요 |
| ChassisMode 자세 락 구현 | 현재 각도 유지 vs 안전 자세 이동 | ⚠️ 합의 필요 |

---

## 빌드 방법 (컨테이너 내부)

```bash
docker exec -it ros2_humble bash
cd /root/ros2_ws
source /opt/ros/humble/setup.bash

# 신규 패키지만 빌드
colcon build --packages-select robot_arm_msgs robot_arm_perception
source install/setup.bash

# 또는 전체 빌드
colcon build
source install/setup.bash
```

## 파일 구조 스냅샷

```
extreme-robot/ros2_ws/src/
├── robot_arm_msgs/              ← 신규 (메시지 정의)
│   └── msg/
│       ├── DetectedObject.msg
│       ├── DetectedObjectArray.msg
│       ├── ArrivalStatus.msg
│       ├── ChassisMode.msg
│       └── ArmStatus.msg
├── robot_arm_perception/        ← 신규 (인식 노드)
│   ├── config/
│   │   └── camera_calibration.yaml   ← ⚠️ 실측값으로 교체 필요
│   └── robot_arm_perception/
│       └── perception_node.py        ← 핵심 파일
├── dynamixel_control/           ← 기존 + 스켈레톤 perception_node 추가
├── robot_arm_description/       ← 기존 (URDF)
├── robot_arm_moveit_config/     ← 기존 (MoveIt2)
└── pick_test_pkg/               ← 기존
```

## 오픈 이슈 (CLAUDE.md §5 참조)

1. **optical frame 실제 이름** 확인 (`ros2 run tf2_tools view_frames`)
2. **camera_calibration.yaml 실측** — RealSense Viewer 또는 `cv2.calibrateCamera`
3. **marker_class_map 실제 클래스 이름** — YOLO 학습 모델의 `model.names`와 일치해야 함
4. **status enum 합의** — 파워트레인 팀과 `ARRIVED_PICKUP`, `DONE` 등 문자열 통일
5. **자세 락 구현 방식** — 파워트레인 팀 합의 후 FSM에 반영
6. **ChassisMode → ArrivalStatus 트리거 순서** 합의
