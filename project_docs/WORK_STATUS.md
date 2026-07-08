# 작업 인수인계 지시서

> **대상**: 다음 Claude Code 세션  
> **최종 업데이트**: 2026-07-08 (브랜치 `Gripper_YOLO_FSM` — YOLO 인식 모델 Roboflow `best.pt` 교체 + 그리퍼 URDF 모듈화 gripper_a xacro 파싱 버그 수정·검증)  
> **기준 문서**: `/home/jo/ros2_ws/CLAUDE.md` (전체 통합 계획)  
> **레포 경로**: `/home/jo/ros2_ws/extreme-robot/`  
> **ROS2 소스**: `extreme-robot/ros2_ws/src/`

---

## YOLO 인식 모델 교체 — Roboflow 커스텀 학습 가중치 `best.pt` 적용 (2026-07-08, 브랜치 `Gripper_YOLO_FSM`)

기존 `perception_node`는 COCO 사전학습 `yolov8n-seg.pt`(사람/병/의자 등 범용 클래스)로 markerless pose를 뽑고 있었음. 대회 타겟 물체로 직접 라벨링·학습한 Roboflow 모델을 붙이는 작업.

- `ddkk0714/main`의 `b605666`(YOLO seg 학습 가중치 `best.pt` 추가 — `ros2_ws/src/robot_arm_perception/models/best.pt`, 6.2MB)을 그리퍼 커밋(`b4aa455`)과 함께 fast-forward로 받아옴. Notion "Roboflow 데이터 관리" 문서(라벨링→export→`best.pt` 로컬 배치 흐름)를 참고해 진행.
- `perception_node.py`의 `model_path` 파라미터 기본값을 `yolov8n-seg.pt` → `src/robot_arm_perception/models/best.pt`로 교체(커밋 `c3bc32e`). `CLAUDE.md`도 함께 갱신.
- **설계상 안전장치**: 코드가 이미 마스크 유무에 관계없이 동작하도록 짜여 있음 — seg 모델이면 markerless pose(translation+PCA yaw) 전체 활성, detection 전용 모델이면 마스크가 없어 bbox 중심 depth로 translation만 폴백하고 orientation은 스킵. 그래서 새 모델이 Instance Segmentation인지 Object Detection인지 몰라도 즉시 깨지진 않음.
- **미확인 — 다음 세션에서 컨테이너 안에서 확인 필요**:
  1. `python3 -c "from ultralytics import YOLO; m=YOLO('src/robot_arm_perception/models/best.pt'); print(m.task, m.names)"`로 이 모델이 `segment`인지 `detect`인지, 실제 클래스명이 뭔지 확인 (torch/ultralytics가 host엔 없어서 이번 세션에선 확인 못 함).
  2. 확인된 클래스명으로 `-p classes:='...' -p pick_classes:='...'`를 맞춰 실행 테스트(`ros2 run robot_arm_perception perception_node --ros-args -p model_path:=src/robot_arm_perception/models/best.pt ...`). 기존 COCO 클래스(`bottle`/`cell phone` 등)는 더 이상 안 맞을 가능성 높음.
  3. 실기 카메라로 대회 타겟 인식률이 기존 COCO 모델 대비 실제로 개선됐는지 실측.
- **커밋 여부**: 모델 교체(`c3bc32e`)는 커밋 완료. `models/best.pt` 자체(`b605666`)는 `ddkk0714/main`에서 받아온 상태로 이미 커밋됨.

---

## 그리퍼 URDF 모듈화 — gripper_a xacro 파싱 버그 수정·검증 (2026-07-08, 브랜치 `Gripper_YOLO_FSM`)

`main`에서 분기한 `Gripper_YOLO_FSM` 브랜치에 `ddkk0714/main`의 `b4aa455`(그리퍼 모듈화 — gripper_a URDF 추가, 5217073 위에 커밋)를 fast-forward로 받아옴. 이 커밋이 추가한 `urdf/grippers/gripper_a.xacro`(Fusion 360 fusion2urdf export 편입, `gripper_a_` prefix, 4절링크 닫힌 루프 단순화) + `meshes/grippers/gripper_a/`(mesh 16개) + `urdf/robot_arm.urdf.xacro`(신규, 몸체+그리퍼 xacro:include, `wrist_to_gripper` fixed joint)를 xacro로 실제 처리해 검증.

- **버그 발견·수정**: `gripper_a.xacro`에 `<robot>` 루트 태그가 없어 `xacro:include`가 `junk after document element`로 즉시 실패. 파일 앞뒤에 `<robot xmlns:xacro="..." name="gripper_a">...</robot>` 래퍼 추가로 해결(내용은 그대로).
- **검증 절차/결과** (컨테이너 내부, `docker exec ros2_humble`):
  1. `colcon build --packages-select robot_arm_description` — 성공
  2. `xacro robot_arm.urdf.xacro -o /tmp/robot_arm_out.urdf` — 에러 없이 처리됨 (link 57개, joint 61개)
  3. `check_urdf /tmp/robot_arm_out.urdf` — `Successfully Parsed XML`, 단일 트리 구조(중복 parent 없음) 확인
  4. 트리 확인: `...→ module_connector_5axis_Component41_1 → wrist_to_gripper(fixed) → gripper_a_base_link → ...`
  5. mesh 참조 57개(`package://robot_arm_description/...`) 전부 실제 파일로 해석됨 — 누락 0개
- **아직 안 한 것**: RViz 시각화 확인(수동), `wrist_to_gripper` origin 오프셋(x=147.544/y=0/z=239.50mm) CAD 재실측, `display.launch.py`/`robot_arm_moveit_config`를 이 xacro 경로로 배선.
- **커밋 여부**: `gripper_a.xacro`의 `<robot>` 래퍼 수정은 아직 미커밋 — 다음 세션(또는 이어서) 커밋 필요.

---

## HW-2~6 실하드웨어 테스트 완료 (2026-07-04, 커밋 `3bed8bd`)

Phase 3 문서화(아래 섹션들) 이후 실제 젯슨/서보/카메라로 진행한 하드웨어 검증 세션. 변경 6개 파일:

- **`Dockerfile` / `docker-compose.yml`**: 베이스 이미지를 `osrf/ros:humble-desktop-full` → `ros:humble-ros-base` + `ros-humble-desktop`로 분리하고 `linux/arm64` 플랫폼을 명시(젯슨 실기 배포용). `pyrealsense2`(pip) + gstreamer 풀세트(`gstreamer1.0-plugins-{base,good,bad,ugly}`, `-libav`, `libgstreamer*-dev`) 신규 설치 — 아래 `stream_node`용.
- **`moveit_dynamixel_bridge.py`**: `_enable_torque()`가 `bool` 반환하도록 변경, **토크 활성화에 성공한 ID만** `group_sync_read.addParam()`으로 등록. 이전엔 버스에 없는 서보(전원 미연결 등)가 하나만 있어도 SyncRead 대상 전체가 얽혀 있었는데, 실하드웨어에서 일부 관절 서보가 없거나 응답 없는 상태로도 나머지 서보는 정상 구동되도록 방어. `publish_joint_states()`도 `txRxPacket()` 결과값을 더 이상 체크하지 않고 응답 온 ID만 처리(일부 미응답 허용).
- **`camera_tf.launch.py`**: 카메라 2대 체계로 확장.
  - 전방 RGB-D(RealSense D435i, 차체 고정): `cam_x/y/z/roll/pitch/yaw` 기본값을 placeholder(0)에서 **CAD 실측값**으로 교체(`x=0.123, z=0.082, pitch=-0.26`).
  - **손목 RGB(그리퍼 위, 신규)**: `base_link → wrist_camera_link` static TF 추가, CAD 실측값 기준(`x=0.040, z=0.295`). 현재는 **홈 포즈 기준 static placeholder** — 팔이 움직이면 실제 카메라 위치와 어긋남. URDF 관절 통합은 여전히 후속 과제.
- **`perception_node.py`**: `/perception/debug_image`(`sensor_msgs/Image`) 퍼블리셔 신규 — 구독자 있을 때만(`get_subscription_count() > 0`) `_draw_debug()`로 마스크 반투명 오버레이 + bbox + `클래스명/conf/거리` 라벨을 그려 발행(pick 타겟=초록, 나머지=파란색).
- **`stream_node.py`(신규 노드, `robot_arm_perception`)**: `/perception/debug_image` 구독 → `gst-launch-1.0` 서브프로세스(rawvideoparse→x264enc zerolatency→mpegtsmux→**srtsink**)로 H.264/SRT 송신. 파라미터 `port`(기본 5000)/`fps`(15)/`bitrate_kbps`(3000)/`latency_ms`(60). PC 쪽에서 `recv_stream.sh <port> <JetsonIP>`로 수신(파워트레인 레포 스크립트). 프레임 크기 바뀌면 gst 프로세스 재시작, 파이프 끊기면 자동 재시작.
  - 실행: `ros2 run robot_arm_perception stream_node --ros-args -p host_ip:=<젯슨IP>` (entry point `setup.py` 등록 완료)

**검증 상태**: 커밋 메시지상 "HW-2~6 실하드웨어 테스트 완료"이나, 이 문서의 나머지 섹션(그리퍼 tick/전류 임계값 실측, 카메라 마운트 캘리브 등)이 갱신되지 않았으므로 어디까지 실측 완료됐는지는 다음 세션에서 재확인 필요. 회귀 확인 포인트: SyncRead 필터링 변경 후 정상 서보들의 `/joint_states` 발행 주기·값이 기존과 동일한지.

**진행 중(미커밋)**: 저장소 루트 `ros2_ws/`에 `check_servo.py`/`diag_servo.py`/`fix_servo.py`/`fix_servo2.py`/`move_servo.py` 임시 스크립트 존재 — ID 0 서보의 Operating Mode·Position Limit 이상 및 Hardware Error 복구(토크 OFF→리밋 재설정→리부트→토크 ON) 시도 흔적. 다음 세션에서 원인 파악 후 정리(성공했으면 삭제, 재현되면 `dynamixel_control`에 정식 유틸로 편입 검토).

---

## 현재 완료된 작업

### 신규 패키지 (모두 빌드 완료)

| 패키지 | 위치 | 상태 |
|--------|------|------|
| `robot_arm_msgs` | `src/robot_arm_msgs/` | ✅ 빌드 완료 |
| `robot_arm_perception` | `src/robot_arm_perception/` | ✅ 빌드 완료 |
| `dynamixel_control` | `src/dynamixel_control/` | ✅ `arm_fsm`(FSM+carry_pose) + `moveit_dynamixel_bridge`(effort+그리퍼 확장) |
| `robot_arm_description` | `src/robot_arm_description/` | ✅ `launch/camera_tf.launch.py` 추가(카메라→base_link static TF) |

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
| A. 모션 경로 | **MoveIt 단일 경로(결정 '가')** — FSM→MoveIt(IK·계획)→`arm_controller`→upstream `moveit_dynamixel_bridge`→서보. *(당초 position_node 직접(A)이었으나 upstream #9에 브릿지 존재 발견 → 가로 전환)* |
| B. 그리퍼 | **Dynamixel** — `/joint_states` effort(전류)로 파지/DROP 판정 |
| C. status enum | **보류** — 잠정값으로 두고 파워트레인 팀 합의 후 확정 |
| D. 구간4 제설 주체 | **미정** (팔로 치울지/밟고 갈지) |
| E. 95mm 박스 파지 | 그리퍼로 **가능** |

### 팔 FSM 스켈레톤 작성: `arm_fsm_node.py` (가 방향)

**파일**: `src/dynamixel_control/dynamixel_control/arm_fsm_node.py` (entry point `arm_fsm`, setup.py 등록 완료). **빌드 성공 + mock 스모크테스트 통과**(2026-06-29, 컨테이너).

- §4 상태표 12개 상태(`IDLE/PERCEIVE/PLAN/DESCEND/GRASP_CHECK/LIFT/CARRY/REGRASP/RELEASE/DONE/ABORT/LOCKED`) Enum + `_do_<state>()` 디스패치.
- 액추에이션(가): 팔=MoveIt `move_action`(MoveGroup, pose goal), 그리퍼=`/gripper_controller/follow_joint_trajectory`, 피드백=`/joint_states.effort`.
- 토픽 I/O: 구독 `/pick_target`(latched)·`/arrival_status`·`/chassis_mode`·`/joint_states`, 발행 `/arm_status`.
- effort 기반 파지/DROP 판정, 자세 락(진행 모션 취소+홀드), 재파지 루프 — 골격 동작.
- 스모크테스트: `IDLE→PERCEIVE→PLAN→DESCEND` 전이 확인, move_group 없으면 `move_action 미준비` 경고 후 대기(정상).

### Phase 3 선결 과제 / TODO (대부분 **브릿지 측**으로 이관)

- [x] **브릿지 effort(전류) 발행** *(2026-06-29 완료)* — `moveit_dynamixel_bridge`가 PRESENT_CURRENT(126,2 signed)~PRESENT_POSITION(132,4)을 연속 10바이트 SyncRead 블록으로 한 번에 읽어 `/joint_states`에 position+effort(**raw signed current**) 동시 발행. FSM이 effort로 파지/DROP 판정.
- [x] **브릿지에 그리퍼 실행 경로 추가** *(2026-06-29 완료)* — 같은 브릿지 노드에 `/gripper_controller/follow_joint_trajectory` 액션 서버 추가(단일 서보 양 핑거 미러링). 그리퍼 ID·미터↔틱 매핑·열림/닫힘 전부 파라미터화(`gripper_ids` 기본 [5], `gripper_open/close_tick` placeholder).
  - [ ] **남은 캘리브**: `gripper_open_tick`/`gripper_close_tick` 실측, 전류 임계값(`grasp_effort_thresh`=80·`drop_effort_thresh`=20 raw placeholder) 실측, `gripper_ids` 실제 ID 확정.
- [x] **TF** 카메라(`camera_color_optical_frame`)→`base_link` 연결 *(2026-06-29 완료)* — `robot_arm_description/launch/camera_tf.launch.py` 추가. 뎁스 카메라(베이스 고정) static TF 2단: `base_link→camera_link`(장착 오프셋, launch arg `cam_x/y/z·cam_roll/pitch/yaw`, placeholder=0) + `camera_link→camera_color_optical_frame`(REP-103 optical 회전 고정). tf2_echo로 체인·회전 검증 완료.
  - [ ] **남은 캘리브**: 장착 오프셋 실측값을 launch arg로 지정. **RGB 카메라(eye-in-hand)는 URDF 관절 통합 후속 과제.**
- [x] `_carry_pose()` 구현 *(2026-06-29 완료)* — TF(`base_frame`←`tip_link`)로 현재 TCP 자세 조회 → z+`lift_height`(기본 0.10m), orientation 유지. base_link(planning frame) 기준이라 MoveIt 바로 계획. TF 미가용 시 None→LIFT 스킵(graceful). 파라미터 `base_frame`/`lift_height` 추가, `tf2_ros` 의존 추가. 가짜 TF 스모크테스트 통과.
- [ ] status enum 파워트레인 팀 합의(§6-D) → 파일 상단 상수 교체.
- [ ] 구간4 제설 주체 결정(D); upstream 머지 시점 결정(브릿지 파일 필요).

### 검증 (하드웨어 없이 mock)

```bash
cd /root/ros2_ws && colcon build --packages-select robot_arm_msgs dynamixel_control
source install/setup.bash && ros2 run dynamixel_control arm_fsm
# 다른 터미널 — /pick_target은 transient_local이라 durability 맞춰야 전달됨
ros2 topic pub --qos-durability transient_local /pick_target robot_arm_msgs/DetectedObject \
  '{class_name: box, confidence: 0.9, pose: {position: {z: 0.4}, orientation: {w: 1.0}}}'
ros2 topic pub --once /arrival_status robot_arm_msgs/ArrivalStatus '{status: ARRIVED_PICKUP}'
# 기대: IDLE→PERCEIVE→PLAN→DESCEND (move_group 없으면 move_action 미준비 경고 후 대기)
```

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
  - 🔧 픽 모션: **MoveIt 단일 경로(결정 가)** → 브릿지 effort 발행 + 그리퍼 실행 경로 + TF 연결 남음

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
│       ├── perception_node.py        ← 핵심 파일 (YOLO seg + depth median + 2D PCA + debug_image)
│       └── stream_node.py            ← 신규 (debug_image → H.264/SRT 스트리밍, 2026-07-04)
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
