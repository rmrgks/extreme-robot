# 작업 인수인계 지시서

> **대상**: 다음 Claude Code 세션  
> **최종 업데이트**: 2026-07-08 (미커밋 — HW-8 그리퍼 반응 테스트 + Profile 과부하 트립 원인 규명. 이전: 2026-07-05 커밋 `3048f02` HW-7)  
> **기준 문서**: `/home/jo/ros2_ws/CLAUDE.md` (전체 통합 계획)  
> **레포 경로**: `/home/jo/ros2_ws/extreme-robot/`  
> **ROS2 소스**: `extreme-robot/ros2_ws/src/`

---

## HW-8 그리퍼 반응 테스트 + Profile 과부하 트립 원인 규명 (2026-07-08, 미커밋)

HW-7 다음 세션. 그리퍼 단독(병 인식→닫기/없음→열기) 반응 테스트를 실기로 진행하며 발견한
과부하 트립 문제의 원인을 규명하고 해결.

- **신규 스크립트** `ros2_ws/hw7_gripper_bottle_test.py` (컨테이너 내부 독립 실행 스크립트,
  `capture_pick.py`/`move_servo.py`와 같은 패키지 미편입 임시 테스트 — ros2_ws root, 미추적).
  `/pick_target`은 transient_local(latched)라 병이 사라져도 마지막 값이 남아 "없음"을 감지 못함
  → 매 프레임 발행되는 `/detected_objects`로 현재 프레임 기준 병 유무 판단. XL430 그리퍼에
  `moveit_dynamixel_bridge` 경유 없이 `dynamixel_sdk`로 직접 write.
- **그리퍼 서보 확정**: id=5, model=1060(XL430-W250), Operating Mode=3(Position Control).
  포지션 실측 조정 끝에 **닫힘 280°(tick 3186) / 열림 215°(tick 2446)**로 확정
  (`tick = round(deg/360*4096)`). `bridge.log`에 남아있던 "gripper id=5 토크 활성화 실패"는
  그 세션에 실장치 없이 컨테이너를 띄웠던 것으로 추정 — id=5 자체는 정상 확인.
- **perception_node 기본 해상도 불일치 발견**: 기본 `width=848,height=480`이 이 D435IF
  유닛 컬러 센서에서 미지원 조합이라 `RealSense init failed: Couldn't resolve requests`로
  실패. 이 카메라는 컬러 스트림이 424x240/640x480/1280x720/1920x1080만 지원(848x480은
  depth/IR 전용) — **640x480@30fps**로 띄워야 함.
- **핵심 발견 — 매 그리퍼 동작마다 토크 자동 해제(과부하 트립)**: 처음엔 원인 불명(Hardware
  Error Status가 읽을 때마다 0이라 안 보임)이었으나, Profile Acceleration/Velocity(주소
  108/112)가 **기본값 0(=최고속 즉시 이동)**이라 매 이동마다 순간 전류가 튀어 과부하 보호가
  걸리는 것으로 확인 — 재현율 100%(열림/닫힘 양방향 공통), 명령 후 0.3초 내 트립. Hardware
  Error Status는 트립 조건 해소 후 자동으로 0 복귀해 관찰 시점엔 안 보였을 뿐 실제로는
  발생하고 있었음.
  - **해결**: Profile Acceleration=25, Profile Velocity=80으로 설정 후 60초 반복 토글
    테스트에서 트립 0건 확인(accel=10/velocity=30도 안전하지만 더 느림, 도달 약 0.6초 vs
    25/80의 약 0.3~0.6초). 스크립트 기본값으로 반영.
  - 예기치 않은 토크 해제에 대한 방어로 2초 주기 하트비트(`_reassert_torque`)와 위치 명령
    직전 재활성화를 `_write_position`에 추가 — 근본 원인(Profile) 해결 후에도 안전망으로 유지.
- **실기 검증 완료**: bottle 인식 → 그리퍼 닫힘(280°) → bottle 사라짐 → 그리퍼 열림(215°),
  60초 연속 테스트에서 여러 차례 안정적으로 토글, 트립 없음. accel=0/velocity=0(최고속)으로
  되돌려 트립 재현도 별도 확인(양방향 100% 재현) 후 다시 안전 설정(25/80)으로 복구.
- **다음 세션 확인 포인트**:
  1. 그리퍼 각도(280°/215°)가 실제 파지 대상(병)에 맞는 stroke인지 재확인 (지금은 열림/닫힘
     반응 로직 검증 목적으로 임의 조정한 값).
  2. `moveit_dynamixel_bridge.py`의 `gripper_open_tick`(2400)/`gripper_close_tick`(2048)
     placeholder를 이번 실측값(2446/3186)으로 갱신할지, 그리고 그 브릿지의 즉시-이동 방식
     (Profile Accel/Velocity 미설정)에도 동일한 과부하 트립 위험이 있는지 점검 — 브릿지는
     아직 이 세션에서 발견한 Profile 이슈를 반영하지 않음.
  3. `hw7_gripper_bottle_test.py`는 현재 ros2_ws root의 미추적 독립 스크립트 — 계속 쓸 거면
     `dynamixel_control` 패키지 정식 유틸로 편입 검토.

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
→ **HW-7(2026-07-05, 커밋 `3048f02`)에서 정식 커밋됨** (`fix_servo2.py` 포함). 아래 HW-7 섹션 참고 — ID 0 이상 자체의 근본 원인 확인 여부는 미기록, 다음 세션 재확인 필요.

---

## HW-7 실하드웨어 픽 시퀀스 검증 및 analytic IK 우회 경로 추가 (2026-07-05, 커밋 `3048f02`)

HW-2~6 다음 세션. `arm_fsm`을 실제 서보로 처음 끝까지(인식→IK→하강→파지판정) 돌려본 세션. 핵심 발견은 **결정 '가'(MoveIt 단일 경로)가 현재 하드웨어에서 전제부터 깨져 있었다는 것**.

- **핵심 발견 — MoveIt 6DOF IK 원천 불가**: URDF/SRDF가 아직 팔 5축 중 `joint_1`~`joint_3` **3축만** 반영(CAD 미완성, WIP). 이 상태로 MoveIt `/compute_ik`를 호출하면 **현재 실제 tip pose에 대해서도 `NO_IK_SOLUTION`**이 반환됨을 실측 확인 — 3관절로는 위치+방향(6DOF) 목표를 만족시킬 자유도가 애초에 없음(자유도 3 < 목표 자유도 6).
- **대응 — analytic IK 우회 경로**: `arm_fsm_node.py`에 `ik_mode` 파라미터 신설, 기본값 `'analytic'`.
  - MoveGroup(MoveIt) 대신 FK 서비스(`/compute_fk`) + 수치 자코비안(finite-difference, Levenberg-Marquardt 유사 댐핑 최소자승)으로 **위치만** 맞추는 3DOF IK(`_solve_position_ik`/`_fk_tip`)를 구현, 결과를 `/arm_controller/joint_trajectory`에 직접 publish. 방향(orientation)은 이번엔 포기.
  - **폐기 아님 — 임시 우회**: 결정 '가'의 MoveGroup 경로(§6-A)는 코드에 그대로 남겨둠. URDF가 5축으로 확장되면 `ik_mode:='moveit'`로 전환해 즉시 재사용 가능.
  - `tip_link` 파라미터 기본값을 placeholder에서 실제 SRDF 값(`Link4_1_1`)으로 수정.
- **버그 수정 — FK 서비스 타임아웃**: `/compute_fk` 호출을 `_tick`(타이머 콜백) 안에서 `self`를 `spin_until_future_complete`하면, 이미 실행 중인 콜백을 재진입 spin하게 돼 응답을 못 받고 항상 타임아웃(독립 스크립트로는 2회 반복 만에 수렴하는데 노드 내부에서는 즉시 실패하는 걸로 실측 확인). → 별도 헬퍼 노드(`arm_fsm_fk_client`)로 FK 클라이언트를 분리해 우회.
- **서보 디버깅 스크립트 정식 커밋**: HW-2~6 세션에 미커밋 상태로 남아있던 `check_servo.py`/`diag_servo.py`/`fix_servo.py`/`fix_servo2.py`(ID 0 서보 Operating Mode·Position Limit 이상 및 Hardware Error 복구용) + 실행 스크립트 `run_perception.sh`가 이번 커밋에 반영됨.
- **실기 검증 결과**: bottle 인식 → analytic IK 계산 → 팔 하강 → 그리퍼 닫힘 → effort(전류) 기반 파지 판정까지 **실제 모터로 end-to-end 확인**. 단, 방향까지 맞추는 정밀 파지는 URDF가 5축으로 확장된 뒤(`ik_mode='moveit'` 전환 후)에야 가능.

**검증 상태**: analytic 3DOF 경로는 실기 동작 확인됨(위치만). `ik_mode='moveit'` 경로는 URDF 5축 확장 전까지 검증 보류(코드는 유지, 전환 스위치만 남음). **다음 세션 확인 포인트**: (1) 서보 스크립트가 이전 세션의 ID 0 이상을 실제로 해결했는지, (2) `check_servo.py` 등을 삭제할지 `dynamixel_control` 정식 유틸로 편입할지, (3) URDF 5축 확장 일정.

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
