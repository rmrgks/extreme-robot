# 작업 인수인계 지시서

> **대상**: 다음 Claude Code 세션  
> **최종 업데이트**: 2026-06-29 (Phase 3 착수 — 설계 문서 + 팔 FSM 스켈레톤)  
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

### robot_arm_perception — Phase 2 완료 (Step 1·2·3, markerless)

**파일**: `src/robot_arm_perception/robot_arm_perception/perception_node.py`

> ⚠️ **2026-06-29 설계 변경**: 대회 규정상 **타겟 객체에 ArUco 마커 부착 금지** 확인
> → 마커 기반 pose 추정 폐기, **markerless(YOLO seg + depth + 2D PCA)** 로 전환.
> ArUco/solvePnP 코드·`camera_calibration.yaml`·Phase 1 더미 스켈레톤
> (`dynamixel_control/perception_node.py`)은 혼동 방지 위해 **완전 삭제**함.

- **Step 1 완료**: YOLO **segmentation** 추론 → `class_id`/`class_name`/`confidence`/`bbox`/`mask`
  - ultralytics YOLO 로드 (TensorRT `.engine` 캐시 지원, `_resolve_model()`)
  - ⚠️ `model_path` 기본값 `yolov8n-seg.pt` — **반드시 seg 모델** (detection 모델이면 mask 없어 orientation 미산출)
  - RealSense D435i 파이프라인 (yolo_depth_3d.py 포팅, `_latest_frames()`)
  - `camera_mode` 파라미터: `realsense`(기본) / `test`(정지 이미지)
  - `/detected_objects` (`DetectedObjectArray`) publish, 30fps

- **Step 2 완료 (markerless pose)**:
  - **Translation**: 마스크 centroid color 픽셀 → depth 픽셀 투영 → depth 패치 median
    → deproject (`DepthCal`/`_deproject_centroid`, yolo_depth_3d.py 포팅, align 생략).
    카메라 내부파라미터는 RealSense 스트림 프로파일에서 직접 취득 (yaml 불필요·더 정확).
  - **Orientation**: 마스크 (u,v) 픽셀에 2D PCA (`_mask_pca_yaw_quat`) → 주축 각도를
    optical Z 축 yaw 로 근사 → quaternion `(0,0,sin θ/2,cos θ/2)`. depth 노이즈 무관.
  - 마스크 없거나 depth 측정불가 시: position 0 / orientation 단위쿼터니언 유지.

**설정 파일**: 없음. markerless 경로는 RealSense 스트림 intrinsics를 직접 사용하므로
별도 calibration yaml 불필요 (기존 `config/camera_calibration.yaml`은 삭제됨).

**검증 상태 (2026-06-29)**:
- ✅ `colcon build --packages-select robot_arm_msgs robot_arm_perception dynamixel_control` 성공
- ✅ 런타임 검증 (test 모드, `bus.jpg`, `yolov8n-seg.pt`): `/detected_objects`에 person 4개
  검출, 객체별로 **서로 다른 yaw quaternion**(2D PCA 정상 동작), test 모드라 position=0.
- ⚠️ **translation(depth median) 실측 검증은 RealSense D435i 하드웨어 필요** — 미수행.
  로직은 실카메라 검증된 `yolo_depth_3d.py` 포팅이라 하드웨어 연결 시 동작 기대.

---

### Phase 2 Step 3 — `/pick_target` 선별 로직 완료 (2026-06-29)

**파일**: `perception_node.py` — `_select_pick_target()` + `/pick_target` 퍼블리셔.

- `/pick_target` (`DetectedObject`) 퍼블리셔, **`transient_local`(latched) QoS** — 도착 타이밍 최신 타깃 유실 방지.
- 선별 조건 (3개 모두 만족하는 객체 중 **confidence 최고 1개**):
  1. `class_name ∈ pick_classes` (쉼표구분 **화이트리스트**, 빈값=후보없음 → 신호등/정지선 등 관찰 전용 자동 제외)
  2. `confidence ≥ pick_min_conf` (기본 0.5)
  3. `require_depth=True`(기본)면 `pose.position.z != 0.0` 필수 / `False`면 conf만 (test 검증용)
  - 후보 없으면 publish 안 함 (이전 latched 값 유지).
- **신규 파라미터**: `pick_classes`(필수), `pick_min_conf`(0.5), `require_depth`(True).

**검증 완료 (test 모드, bus.jpg, `require_depth:=false`, `pick_classes:=person`)**:
person 4 + airplane 검출 중 → `/pick_target`에 **confidence 최고 person(0.87)** 발행.
airplane(화이트리스트 제외)·person 0.46(min_conf 미달) 정상 탈락. 로그 `pick=person(0.87)` 확인.
⚠️ 실주행(`require_depth=True`)은 RealSense depth 필요.

---

### Phase 2 커밋 완료 (2026-06-29)

- 커밋 `22c25a1` `feat(perception): Phase 2 markerless 인식 파이프라인 구현` (브랜치 `Depth_LiDAR_RViz`, **push 안 함**).
  - perception_node markerless 전환 + `/pick_target` 선별 / ArUco·더미노드 제거 / setup.py 정리.
  - 개인 작업 문서 3종(`CLAUDE.md`·`WORK_STATUS.md`·`CLAUDE_Plan.md`) 추적 해제 + `.gitignore` 등록(비공유, 로컬 보존).
  - `ros2_ws/yolov8n-seg.pt`(6MB)는 커밋 제외(미추적).

---

## Phase 3 착수 — 로봇팔 FSM (2026-06-29, 진행 중)

### 설계 문서 작성: `PHASE3_FSM_설계.md` (레포 루트)

- 사용자 기존 FSM `2026_FSM_국방.drawio`(5구간 미션 FSM) 분석 반영.
- **핵심 발견**: 로봇팔 실작업은 **구간2(여름_구호물자운반)의 `ARM_GRASP_BOX`** 하나. 나머지 4구간은 팔이 IDLE+자세 락만.
- drawio(전체 로봇) → 팔 FSM 노드/파워트레인 노드 분리 매핑, §4 팔 상태표, §5 핸드셰이크, §6 구현 갭 정리.

### 구현 방식 결정 (사용자 확정 2026-06-29)

| 항목 | 결정 |
|------|------|
| A. 모션 경로 | **position_node 직접 제어** (MoveIt 실HW 인터페이스 안 씀) |
| B. 그리퍼 | **Dynamixel** — 전류로 파지/DROP 판정 |
| C. status enum | **보류** — 잠정값으로 두고 파워트레인 팀 합의 후 확정 |
| D. 구간4 제설 주체 | **미정** (팔로 치울지/밟고 갈지) |
| E. 95mm 박스 파지 | 그리퍼로 **가능** |

### 팔 FSM 스켈레톤 작성: `arm_fsm_node.py`

**파일**: `src/dynamixel_control/dynamixel_control/arm_fsm_node.py` (entry point `arm_fsm`, setup.py 등록 완료). 문법 검증 통과(AST), **빌드/런타임 미검증**.

- §4 상태표 12개 상태(`IDLE/PERCEIVE/PLAN/DESCEND/GRASP_CHECK/LIFT/CARRY/REGRASP/RELEASE/DONE/ABORT/LOCKED`) Enum + `_do_<state>()` 디스패치.
- 토픽 I/O: 구독 `/pick_target`(latched)·`/arrival_status`·`/chassis_mode`·`/dynamixel/state`, 발행 `/dynamixel/goal_position`·`/arm_status`.
- 전류 기반 파지/DROP 판정, 자세 락 인터럽트(거친지형/추종 모드), 재파지 루프(`max_regrasp` 초과 시 ABORT) — 골격 동작.
- status/mode 문자열은 파일 상단에 모아둔 **잠정값**(파워트레인 합의 전).

### Phase 3 선결 과제 / TODO (스켈레톤에 스텁으로 남김)

- [ ] **`_solve_ik()` 실제 구현** — (a) KDL/ikpy 직접 vs (b) MoveIt plan만 받아 관절각 추출. **다음 분기점.**
- [ ] **position_node에 그리퍼 ID 추가** — 현재 `DXL_IDS=[0,1,2]`(joint_1~3)뿐. `gripper_id` torque enable + `/dynamixel/state`에 포함해야 전류 피드백 들어옴.
- [ ] 그리퍼 open/close raw 값 + 전류 임계값(`grasp_current_thresh`/`drop_current_thresh`) **실측 캘리브**.
- [ ] DESCEND/LIFT 실제 궤적(저속 하강 0.02m/s 등) — 현재 타임아웃/전류로 단계 진행만.
- [ ] status enum 파워트레인 팀 합의(§6-C) → 파일 상단 상수 교체.
- [ ] 구간4 제설 주체 결정(D) — 팔이면 두 번째 파지류 동작 설계.

### 검증 (하드웨어 없이 mock)

```bash
cd /root/ros2_ws && colcon build --packages-select dynamixel_control robot_arm_msgs
source install/setup.bash && ros2 run dynamixel_control arm_fsm
# 다른 터미널
ros2 topic pub --once /pick_target robot_arm_msgs/DetectedObject '{class_name: box, confidence: 0.9, pose: {position: {z: 0.4}}}'
ros2 topic pub --once /arrival_status robot_arm_msgs/ArrivalStatus '{status: ARRIVED_PICKUP}'
ros2 topic echo /arm_status   # PERCEIVING→PLANNING→EXECUTING... 전이 확인
```

> ⚠️ `PHASE3_FSM_설계.md`·`arm_fsm_node.py`·setup.py 변경은 **아직 커밋 안 함**.

---

## 다음 작업 (Phase 3 — FSM 통합)

→ 아래 "그 이후 작업 (Phase 3)" 섹션 참조. 실행 명령 예시:

```bash
docker exec -it ros2_humble bash
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash

# 인식 노드 (실센서) — 픽 대상 클래스는 실제 타겟으로 교체
ros2 run robot_arm_perception perception_node --ros-args \
  -p model_path:=/root/ros2_ws/yolov8n-seg.pt \
  -p pick_classes:=<타겟클래스> -p pick_min_conf:=0.5

# 하드웨어 없이 선별까지 검증할 때 (test 모드)
BUS=/usr/local/lib/python3.10/dist-packages/ultralytics/assets/bus.jpg
ros2 run robot_arm_perception perception_node --ros-args \
  -p camera_mode:=test -p test_image_path:=$BUS \
  -p model_path:=/root/ros2_ws/yolov8n-seg.pt -p conf_threshold:=0.3 \
  -p pick_classes:=person -p require_depth:=false
# 확인: ros2 topic echo /detected_objects  /  ros2 topic echo /pick_target
```

---

## 그 이후 작업 (Phase 3 — FSM 통합)

### Phase 3 체크리스트 (CLAUDE.md §3 Phase 3)

- [~] **로봇팔 FSM**: `/arrival_status` 수신 → `/pick_target` 읽기 → 픽 시퀀스 *(스켈레톤 완료, 2026-06-29)*
  - ✅ 신규 노드 `src/dynamixel_control/dynamixel_control/arm_fsm_node.py` (위치: perception 아님 dynamixel_control)
  - ✅ `/arrival_status`(ArrivalStatus) 구독, `status=='ARRIVED_PICKUP'` 시 FSM 전환
  - 🔧 픽 모션: **MoveIt 대신 position_node 직접 제어**(결정 A) → `_solve_ik()` 실제 구현 + position_node 그리퍼 ID 추가 남음

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
| pose 추정 방식 | markerless (YOLO seg + depth median + 2D PCA yaw) | ✅ 확정 (2026-06-29 전환) |
| YOLO 모델 | segmentation 모델 (`yolov8n-seg.pt` 등) | ⚠️ seg 필수, 커스텀 학습 모델로 교체 예정 |
| ArUco 경로 | (삭제됨) | ❌ 대회 규정상 타겟 마커 금지 → 코드·yaml 완전 제거 |
| camera_matrix 출처 | RealSense 스트림 intrinsics 직접 사용 | ✅ 확정 (markerless는 yaml 불필요) |
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
├── robot_arm_perception/        ← 신규 (markerless 인식 노드)
│   └── robot_arm_perception/
│       └── perception_node.py        ← 핵심 파일 (YOLO seg + depth median + 2D PCA)
├── dynamixel_control/           ← 기존 (더미 perception_node 스켈레톤은 삭제됨)
├── robot_arm_description/       ← 기존 (URDF)
├── robot_arm_moveit_config/     ← 기존 (MoveIt2)
└── pick_test_pkg/               ← 기존
```

## 오픈 이슈 (CLAUDE.md §5 참조)

1. **optical frame 실제 이름** 확인 (`ros2 run tf2_tools view_frames`)
2. **커스텀 seg 모델** — 대회 타겟 클래스로 학습한 YOLO **segmentation** 모델로 교체 (현재 `yolov8n-seg.pt` COCO)
3. **2D PCA yaw 한계** — 객체가 이미지 평면 밖으로 크게 기울면 부정확 → 필요 시 3D PCA(마스크 erode+outlier 제거) 업그레이드
4. **status enum 합의** — 파워트레인 팀과 `ARRIVED_PICKUP`, `DONE` 등 문자열 통일
5. **자세 락 구현 방식** — 파워트레인 팀 합의 후 FSM에 반영
6. **ChassisMode → ArrivalStatus 트리거 순서** 합의
