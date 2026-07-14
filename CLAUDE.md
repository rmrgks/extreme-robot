# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Language

**Always respond to the user in Korean (한국어).** Documentation, commit messages, and PR descriptions are written in Korean too.

## Requirements (항상 참고)

**Before any design or implementation decision, consult `docs/requirements/요구사항.md`.** It is the consolidated digest of the competition rules, the team's Pipeline architecture doc, and meeting notes (raw sources in `docs/requirements/raw/`).

Treat it as a **draft, not a fixed spec** — the team is still converging and the docs contradict each other and the current code. Its §8 lists the live mismatches (DOF 5 vs 6, motor model XL430 vs XM540/XC430, direct-servo vs MoveIt2 path, CAN vs UDP/TCP, etc.) — check there before trusting any single number. Priority is a working implementation over faithfully following the docs.

## What this is

ROS 2 Humble workspace for the 2025 극한로봇 (Extreme Robot) competition: a robot arm that visually tracks a target with YOLO and drives Dynamixel servos to follow it. The dev environment is fully containerized so the team shares one identical setup. Documentation and commit messages are in Korean.

## Environment model (important)

**All ROS 2 commands run *inside* the Docker container, not on the host.** The host is only used for `git` and `docker compose`.

- `./ros2_ws` is bind-mounted to `/root/ros2_ws` in the container, so host edits to `ros2_ws/src/` appear instantly inside.
- **Only `ros2_ws/src/` is version-controlled.** Build outputs (`build/`, `install/`, `log/`) are gitignored — each developer runs `colcon build` in their own container.
- Compose files: `docker-compose.yml` (기본) + `docker-compose.gpu.yml` (Jetson GPU override — `-f`로 얹어 쓴다). WSL2 지원은 제거됨. 컨테이너는 `ros2_humble`, `privileged: true` + `network_mode: host` + **`ipc: host`**.
- **`ipc: host`는 파워트레인 연동에 필수다.** 파워트레인은 같은 Jetson의 **별도 컨테이너**에서 ROS 2 노드를 돌리고 DDS로만 통신한다. Fast-DDS는 같은 호스트면 공유메모리(`/dev/shm`)로 데이터를 보내는데 Docker는 컨테이너마다 별도 `/dev/shm`을 준다 → `ipc: host`가 없으면 **discovery는 되는데 데이터가 한 건도 안 오는 조용한 실패**가 난다. 양쪽 컨테이너 모두 필요하다.
- The container runs `privileged`, so host devices (e.g. the Dynamixel USB serial adapter at `/dev/ttyUSB0`, the camera) are reachable without explicit `devices:` mappings — but the hardware must actually be plugged into the host.

## Common commands

Start container + enter (host):
```bash
xhost +local:docker && docker compose up -d
# Jetson GPU 가속까지 쓸 때만
xhost +local:docker && docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d

docker exec -it ros2_humble bash   # ROS already sourced via .bashrc
```

Build & run (inside container):
```bash
cd /root/ros2_ws
colcon build
source install/setup.bash
```

- Build one package: `colcon build --packages-select <pkg>`
- Resolve missing deps before reporting a build failure: `rosdep install --from-paths src --ignore-src -r -y`
- Rebuild image after a `Dockerfile` change: `docker compose build` then `up -d`.

## Dependency policy

System dependencies go in the **`Dockerfile`**, not ad-hoc `apt install`, so the team's environment stays reproducible. Already installed there: `dynamixel-sdk`, `dynamixel-workbench` (apt), `joint-state-publisher-gui`, **`moveit`, `ros2-control`, `ros2-controllers`** (apt, for MoveIt + mock hardware/controllers), a full `gstreamer1.0` plugin set (`base`/`good`/`bad`/`ugly`/`libav` + dev headers, for `stream_node`'s SRT streaming), and `ultralytics` + `numpy<2` + `pyrealsense2` via pip (with `opencv-python` uninstalled so it doesn't clash with ROS's `cv_bridge` OpenCV).

> Base image is `ros:humble-ros-base` + `ros-humble-desktop` layered on top (split from the former `osrf/ros:humble-desktop-full` monolith), built for `linux/arm64` (`docker-compose.yml`) — this targets the Jetson deployment, not just dev laptops.

> Note: the Dynamixel libraries are pulled via apt (`ros-humble-dynamixel-sdk`, `ros-humble-dynamixel-workbench`) — **not** git submodules. An earlier broken submodule/gitlink for these was removed.

## 벤치 텔레옵 (파워트레인 없이 팔만 구동)

`bench.launch.py` — `joy_node` → `joystick_teleop` → `teleop_core` → `position_node`. 키보드 프론트엔드(`keyboard_teleop`)도 같은 토픽을 쓰므로 대체 가능하다.

- **격리는 launch 파일이 전부다.** `arm_fsm`을 안 띄우므로 계약 토픽(`/arm_status`·`/chassis_mode`·`/arrival_status`)이 **아예 생기지 않는다.** 노드 코드에 "테스트 모드면 건너뛰기" 분기는 **넣지 않았고, 넣지 말 것** — 안전 게이트에 스킵 분기가 있으면 실기에서 켜진 채 도는 사고가 난다(파워트레인도 같은 원칙: *"production source에 simulator 이름 분기를 넣지 않는다"*).
- ⚠️ **이 경로는 계약상 production 금지다.** `/dynamixel/goal_position` 직접 발행은 계약이 금지하는 *"direct dynamixel goal publisher"*이고, `home`(전 관절 0)은 금지된 *"all-zero home"*이다. **파워트레인과 "팔 단독 벤치 profile 허용"을 합의해야 한다**(그쪽은 자기 쪽에 `arm_gate_mode=arm_absent_field`라는 대칭 장치를 이미 뒀다).
- `JointJog.velocities`는 **rad/s**다(표준). `displacements`는 `jog_step_rad` 배수(키보드용). 예전엔 velocity가 "초당 jog_step 개수"로 해석돼 풀스틱이 0.05 rad/s밖에 안 나왔다 — 고쳤다.
- **velocity 프론트엔드는 매 발행마다 전 관절을 실어야 한다.** `teleop_core.on_jog`는 메시지에 없는 관절의 velocity를 0으로 만들지 않아, 움직이는 관절만 골라 보내면 **놓은 관절이 마지막 속도로 계속 돈다.**

### 그리퍼는 아직 이 경로에 배선되지 않았다

`moveit_dynamixel_bridge`가 이미 같은 서보(id 5)를 구동하므로, `position_node`에도 그리퍼를 넣으면 **같은 버스의 같은 서보를 두 노드가 만지게 된다**(계약이 금지하는 owner 중복). 게다가 표현이 세 갈래로 갈라져 있다 — URDF(`gripper_a_joint5~23`, continuous) / 브릿지(`left_finger_joint`, prismatic 미터 — **URDF에 없는 조인트**) / HW-8 스크립트(id 5, degree). **먼저 한 곳으로 정리한 뒤 배선할 것.**

> ⚠️ **배선할 때 반드시**: Dynamixel **Profile Acceleration(주소 108) / Velocity(112)를 25/80으로 설정**할 것. 기본값 `0`(=최고속 즉시 이동)이면 그리퍼가 움직일 때마다 순간 과전류로 **토크가 풀린다**(HW-8 실기 검증, 재현율 100%, 명령 후 0.3초 내 트립). Hardware Error Status는 트립 해소 후 0으로 복귀해 관찰 시점엔 안 보인다 — 이걸 모르면 원인을 못 찾는다.

## 파워트레인 계약 (중요)

파워트레인 팀([power-train-sw](https://github.com/lightminn/power-train-sw))과 **같은 Jetson의 별도 컨테이너**에서 각자 ROS 2 노드를 돌리고 **DDS로만** 통신한다. 워크스페이스를 서로 오버레이하지 않으며, 공유하는 것은 메시지 계약뿐이다. 파워트레인은 `robot_arm_msgs`의 `.msg`만 벤더링해 자기들이 직접 빌드한다(ROS 2는 wire에서 **패키지명 + 구조 해시**로 매칭하므로 동일한 `.msg`로 각자 빌드하면 붙는다).

- **값 어휘의 단일 출처는 `dynamixel_control/contract.py`** — 파워트레인의 `powertrain_ros/contract.py`와 짝이다. **여기 없는 status 문자열을 새로 만들지 말 것.** 파워트레인은 `contract.ARM_STATUSES` 밖의 값을 받으면 즉시 `CONTRACT_VIOLATION` + motion hold를 건다. 어휘 변경은 **양 팀 합의 사항**이다.
- QoS는 `dynamixel_control/qos_profiles.py`. heartbeat 계열은 **KeepLast 1**이다 — depth를 키우면 낡은 샘플이 큐에 쌓여 파워트레인의 신선도 판정이 어긋난다.
- **`/arm_status`는 `arm_fsm`의 heartbeat 타이머 한 곳에서만 발행한다.** 상태 핸들러는 `_set_status()`로 값만 바꾼다. 발행 경로가 둘이 되면 `header.stamp` 순서가 뒤집힐 수 있는데, 파워트레인은 stamp가 0.5초 이상 역행하면 **영구 latch**(프로세스 재시작 전까지 해제 불가)를 건다.
- **`MISSION_STOP`만이 팔 작업 허가다.** `DRIVING`을 포함한 나머지 mode는 전부 잠금(default-deny).
  - ⚠️ **`arm_fsm_node.py`는 아직 `DRIVING`에서 팔을 언락한다 — 계약 위반이자 안전 결함(주행 중 팔이 풀림).** 미수정 상태.
- **차가 움직이려면 팔이 `STOWED_LOCKED` 또는 `CARRYING_LOCKED`를 신선하게 발행해야 한다**(`contract.DRIVE_READY_STATUSES`). 그 외 status는 전부 주행 불가다.
  - ⚠️ **우리는 아직 이 둘을 발행하지 않는다 → 지금 붙이면 차가 출발하지 못한다.** 구현하려면 **접힘 자세(stow posture) 정의가 선행**이며, 계약상 `all-zero home`은 금지다.

### robot_arm_msgs (ament_cmake) — 공통 메시지 패키지
양팀(로봇팔·파워트레인)이 공유하는 커스텀 메시지 5개: `DetectedObject`(class_id/name/confidence/`geometry_msgs/Pose`/bbox), `DetectedObjectArray`(header + objects[]), `ArrivalStatus`, `ChassisMode`, `ArmStatus`. 인터페이스 상세는 `CLAUDE_Plan.md` §1 참고.

### robot_arm_perception (ament_python) — markerless 인식 노드
`perception_node` + `stream_node` 두 개. `perception_node`: RealSense D435i color+depth → YOLO **segmentation** 추론 → `/detected_objects`(`DetectedObjectArray`) 30Hz publish. **markerless pose**(대회 규정상 타겟 마커 부착 금지): translation은 마스크 centroid의 depth median deproject(`yolo_depth_3d.py` 로직 포팅, align 생략), orientation은 마스크 (u,v) 픽셀 **2D PCA** 주축각 → optical Z yaw quaternion. 카메라 intrinsics는 RealSense 스트림에서 직접 취득(calibration yaml 불필요). 또한 `/pick_target`(`DetectedObject`, transient_local latched)을 publish: `pick_classes` 화이트리스트 ∩ `pick_min_conf` 이상 ∩ depth 조건(`require_depth`) 만족 객체 중 confidence 최고 하나(신호등/정지선 등 관찰 전용은 화이트리스트로 자동 제외). 파라미터: `model_path`(**seg 모델이면 markerless pose 전체 활성, detection 전용이어도 bbox 중심 depth로 translation은 폴백** — 기본값이 2026-07-08부터 `models/best.pt`, 대회 타겟으로 Roboflow에서 커스텀 학습한 모델로 교체됨; 이전 COCO 사전학습 `yolov8n-seg.pt`는 `ros2_ws/` 루트에 그대로 남아있고 gitignore 대상), `camera_mode`(`realsense`|`test`), `conf_threshold`, `classes`/`pick_classes`(새 모델의 실제 클래스명에 맞게 실행 시 지정 필요 — 아직 미확인), `pick_min_conf`, `require_depth`, `frame_id` 등. ArUco 경로는 제거됨. 진행 상황은 `CLAUDE_Plan.md`·`WORK_STATUS.md`.

`perception_node`는 구독자가 있을 때만 `/perception/debug_image`(bbox·마스크·거리 오버레이, pick 타겟=초록/나머지=파란색)를 publish한다. `stream_node`는 이 토픽을 구독해 `gst-launch-1.0` 서브프로세스(x264enc→SRT)로 원격 PC에 H.264/SRT 스트리밍(`recv_stream.sh`로 수신) — 하드웨어 테스트 중 원격 모니터링용, `/pick_target` 등 제어 경로와 무관.

### dynamixel_control (ament_python) — the core runtime
Two runtimes share this package (entry points in `setup.py`): a **legacy YOLO→servo P-control pipeline** (3 nodes, below) and the **Phase 3 MoveIt/FSM pipeline** (`moveit_dynamixel_bridge` + `arm_fsm`) — the latter is the real 구간2 pick path.

```
yolo_detection ──/yolo/target_center──▶ yolo_bridge ──/dynamixel/goal_position──▶ position_node ──▶ physical XL430 servos
   (camera+YOLO)     [cx, cy]            (P-control)        [id, goal_pos]                          + /joint_states, /dynamixel/state
```

- `yolo_detection` (`yolo_detection_node.py`): opens the camera with `cv2.VideoCapture`, runs `ultralytics` YOLO, publishes the best target's pixel center to `/yolo/target_center`. **Does not use `rclpy.spin`** — it runs its own blocking `while rclpy.ok()` loop in `run()`; an OpenCV preview window (`show_window` param) needs X/GUI forwarding. Tunable params: `model_path`, `target_class`, `conf_threshold`, `camera_device`, etc.
- `yolo_bridge` (`yolo_to_dynamixel_bridge.py`): converts pixel error `cx - 320` into a goal position via simple proportional gain, publishes `[id=1, goal]` to `/dynamixel/goal_position`. Currently hardcoded to motor ID 1.
- `position_node` (`dynamixel_position_node.py`): touches hardware for the *legacy* pipeline. Talks to 5× XL430 (`DXL_IDS = [0..4]`) over `/dev/ttyUSB0` at 1 Mbps, protocol 2.0. Subscribes `/dynamixel/goal_position`, enables torque on startup, and at 10 Hz reads pos/vel/current/temp → publishes `/dynamixel/state` and a `/joint_states` (`JointState`) for RViz/MoveIt. Raw 0–4095 ↔ radians is approximated as `(raw-2048)*2π/4096`.

**MoveIt/FSM pipeline (Phase 3 — the real pick path; both nodes touch `/dev/ttyUSB0`/MoveIt, don't run alongside `position_node` on the same bus):**
- `moveit_dynamixel_bridge` (`moveit_dynamixel_bridge.py`): hardware node for the MoveIt path. Implements `/arm_controller/follow_joint_trajectory` + `/gripper_controller/follow_joint_trajectory` action servers, so MoveIt/`arm_fsm` execute on real servos (a lighter substitute for a full `ros2_control` HW interface). Reads `PRESENT_CURRENT`(126,2 signed)~`PRESENT_POSITION`(132,4) in one 10-byte SyncRead → publishes `/joint_states` with **position + effort (raw signed current)**. Gripper = single servo, both fingers mirrored; `gripper_ids`/`gripper_open_tick`/`gripper_close_tick`/`gripper_open_m`/`gripper_close_m` are params (empty `gripper_ids` disables the gripper → mock-friendly). Arm `JOINT_CONFIG` currently covers `joint_1..joint_3` (ids 0,1,2) — extend when arm DOF is finalized. **Only IDs whose torque-enable actually succeeds get registered in the SyncRead group** — a missing/unpowered servo no longer breaks readback for the rest of the bus.
- `arm_fsm` (`arm_fsm_node.py`): the 구간2 pick FSM (12 states `IDLE`~`LOCKED`, MoveIt 단일 경로 '가'). Subscribes `/pick_target`(latched)·`/arrival_status`·`/chassis_mode`·`/joint_states`, publishes `/arm_status`. Sends pose goals to MoveIt `move_action`; grasp/DROP decided from `/joint_states.effort` (raw-current thresholds). `_carry_pose()` looks up TF (`base_frame`←`tip_link`) for a base_link +Z lift (`lift_height`) → needs `tf2_ros` (in `package.xml`). Status string enums (`ARRIVED_PICKUP`/`DONE`/…) are **provisional, pending powertrain-team agreement**. Hardware-free smoke test: launch + mock-pub `/pick_target`(transient_local) + `/arrival_status` → expect `IDLE→PERCEIVE→PLAN→DESCEND` then a `move_action 미준비` warning (no move_group).
  - **IK note (HW-7, 2026-07-05):** the URDF currently models only 3 of the arm's 5 axes (`joint_1..joint_3`, CAD still WIP), so MoveIt's 6DOF pose IK returns `NO_IK_SOLUTION` even for the live tip pose — confirmed on real hardware, not just a planning-difficulty issue. Default `ik_mode='analytic'` bypasses MoveGroup: FK service (`/compute_fk`, called from a **separate helper node** `arm_fsm_fk_client` — calling it from `self` inside the `_tick` timer callback deadlocks via reentrant spin) + a finite-difference Jacobian solves position-only 3DOF IK, publishing straight to `/arm_controller/joint_trajectory` (orientation is dropped). The MoveGroup path (`ik_mode='moveit'`) is kept, not removed — switch back once the URDF covers all 5 axes. Real-hardware verified end-to-end: bottle detection → analytic IK → descend → gripper close → effort-based grasp check.

### robot_arm_description (ament_cmake)
Compiles nothing — `CMakeLists.txt` only installs `urdf/`, `launch/`, `rviz/`, `config/` to `share/`. Adding a resource dir requires adding it to the `install(DIRECTORY ...)` block.
- `urdf/robot_arm.urdf`: **as of the 2026-07-07 CAD export (5-DOF) it uses raw fusion2urdf link/joint names** (e.g. `Revolute 23`/`Revolute 29`/`Revolute 42`/`Revolute 48`/`Revolute 72`, links like `link1-1_1`, tip link `module_connector_5axis_Component41_1`) — **not** the `joint_1..joint_6`/`gripper_mount` naming this doc previously described (that matched an earlier URDF revision; see §8 for other live DOF/naming mismatches). It's a plain fully-expanded URDF (no xacro macros used), loaded as-is by `display.launch.py`.
- **그리퍼 모듈화 (신규, 2026-07-08 xacro 파싱 검증 완료):** `urdf/grippers/gripper_a.xacro`는 Fusion 360 fusion2urdf export를 편입한 첫 그리퍼 모듈 — 모든 link/joint에 `gripper_a_` prefix, mesh는 `meshes/grippers/gripper_a/`. Raw export가 평행 4절링크의 닫힌 루프를 동일 이름 중복 link(`link5`/`link5_2`/`link6`/`component53_1`)로 export해 URDF 트리 규칙을 위반했던 것을, 링크당 첫 조인트만 남기고 단순화(단일 트리로 변환 — 실제 평행 커플링 구속은 반영 안 됨, 시각화/충돌형상용). `urdf/robot_arm.urdf.xacro`가 몸체(`robot_arm.urdf`)와 `xacro:arg gripper`(기본 `gripper_a`)로 고른 그리퍼 xacro를 `xacro:include`해 `wrist_to_gripper` fixed joint(팔 끝단 `module_connector_5axis_Component41_1` 기준, CAD 실측 x=147.544mm/y=0/z=239.50mm — `base_link` 기준 — 에서 역산한 값, 재확인 필요)로 결합. 아직 어떤 launch 파일도 이 `.xacro`를 쓰도록 바뀌진 않음(`display.launch.py`는 여전히 `robot_arm.urdf` 그대로 로드) — `gripper_b` 등 추가나 실제 배선은 후속 과제.
  - **버그 수정 (2026-07-08):** 최초 커밋된 `gripper_a.xacro`에 `<robot>` 루트 태그가 없어(최상위에 `<link>`/`<joint>`가 나열된 조각 XML) `xacro:include`가 `junk after document element`로 실패했음 — `<robot xmlns:xacro="..." name="gripper_a">...</robot>` 래퍼를 앞뒤로 추가해 해결.
  - **검증 완료 (2026-07-08, 컨테이너 내부):** `colcon build --packages-select robot_arm_description` → `xacro robot_arm.urdf.xacro` 정상 처리(link 57/joint 61) → `check_urdf` `Successfully Parsed XML` + 단일 트리(중복 parent 없음) 확인 → `module_connector_5axis_Component41_1 → wrist_to_gripper → gripper_a_base_link` 결합 확인 → mesh 참조 57개 전부 실제 파일로 해석됨(누락 0). **RViz 시각화·`wrist_to_gripper` 오프셋 CAD 재실측·launch/moveit_config 배선은 미완료.**
- `launch/display.launch.py`: robot_state_publisher + joint_state_publisher_gui + rviz2. RViz launches with no saved config, so the model is invisible until you set Fixed Frame to `base_link`, add a RobotModel display, and set its Description Topic durability to `Transient Local` (see README).
- `launch/camera_tf.launch.py`: 카메라 2대분 static TF 발행. 전방 RGB-D(차체 고정): `base_link→camera_link`(장착 오프셋 launch arg `cam_x/y/z`·`cam_roll/pitch/yaw`, **CAD 실측값** 기본 `x=0.123, z=0.082, pitch=-0.26`) + `camera_link→camera_color_optical_frame`(REP-103 optical 회전 `-π/2,0,-π/2` 고정). 손목 RGB(그리퍼 위, 신규): `base_link→wrist_camera_link`(`wrist_cam_x/y/z`·`wrist_cam_roll/pitch/yaw`, CAD 실측값 기본 `x=0.040, z=0.295`) — **홈 포즈 기준 static placeholder**라 팔이 움직이면 실제 위치와 어긋남, URDF 관절 통합은 후속 과제. `perception_node`가 TF를 발행하지 않으므로, MoveIt이 `/pick_target`(camera frame) 목표를 `base_link`로 변환하려면 이 launch가 떠 있어야 함.
- `config/controllers.yaml`: an `arm_controller` (`joint_trajectory_controller`) over `joint_1`..`joint_6` + a `joint_state_broadcaster`, `update_rate: 100`.

### robot_arm_moveit_config (ament_cmake) — MoveIt 경로 계산용
Generated by MoveIt Setup Assistant; structure is complete and ready for motion planning. Use this package for path/trajectory planning.
- **Planning groups (`config/robot_arm.srdf`):** `arm` is the kinematic chain `base_link` → `link_6` (joint_1..joint_6); `gripper` group = `left_finger_joint`/`right_finger_joint`, end effector parented to `link_6`. Named state `home` = all arm joints at 0. Virtual joint `world` → `base_link` (fixed).
- **IK solver (`config/kinematics.yaml`):** KDL (`kdl_kinematics_plugin/KDLKinematicsPlugin`) for the `arm` group.
- **Controllers:** MoveIt sends `FollowJointTrajectory` to `arm_controller` and `gripper_controller` (`config/moveit_controllers.yaml`); the matching `ros2_control` controllers are in `config/ros2_controllers.yaml` (update_rate 100 Hz, position command interface).
- **`demo.launch.py` is mock-only, not real hardware.** `config/robot_arm.ros2_control.xacro` loads the `mock_components/GenericSystem` plugin (the SetupAssistant `FakeSystem`), so `demo.launch.py` plans against fake joints — it does **not** drive the physical Dynamixels, and you must **not** run it alongside the bridge (its mock `ros2_control_node` competes for `/joint_states` and `/arm_controller`). To execute MoveIt plans on **real servos**, run `move_group.launch.py` + `rsp.launch.py` and let `dynamixel_control`'s `moveit_dynamixel_bridge` act as the controller (it implements the `/arm_controller`+`/gripper_controller` action servers MoveIt drives — a lighter alternative to a full `ros2_control` HW interface).
- **MoveIt mock demo works (verified).** `ros-humble-moveit` + `ros-humble-ros2-control` + `ros-humble-ros2-controllers` are now in the Dockerfile. Run with:
  ```bash
  cd /root/ros2_ws && colcon build --packages-select robot_arm_description robot_arm_moveit_config
  source install/setup.bash && ros2 launch robot_arm_moveit_config demo.launch.py
  ```
  This brings up `move_group` + mock `ros2_control` + RViz MotionPlanning; all 3 controllers (`arm_controller`/`gripper_controller`/`joint_state_broadcaster`) go `active`. Plan & Execute in RViz drives the *mock* joints only (not real servos).
- **Two fixes were needed for the mock demo (don't regress):** (1) `urdf/robot_arm.urdf` had a Gazebo `ign_ros2_control/IgnitionSystem` ros2_control block that collided with MoveIt's mock `FakeSystem` and crashed `ros2_control_node` — it is now **commented out** (re-enable only for Gazebo Ignition). (2) `config/moveit_controllers.yaml` was missing `action_ns: follow_joint_trajectory` on both controllers, so MoveIt saw 0 controllers — now added.
- The MoveIt SRDF uses link names `link_1`..`link_6`/`gripper_base`; confirm these match `robot_arm_description/urdf/robot_arm.urdf` when editing the URDF, or planning/collision checks break.

### pick_test_pkg (ament_python)
Standalone gripper test: `pick_test_node` listens on `/fake_object_position` (`Point`) and sends a `FollowJointTrajectory` action to `/gripper_controller/follow_joint_trajectory` for `left_finger_joint`/`right_finger_joint`.

## Watch out for

- **Joint-count mismatches across files are a live source of bugs.** `position_node` publishes only `joint_1`..`joint_5`, but the URDF and `controllers.yaml` define `joint_1`..`joint_6` (plus gripper joints). Keep `DXL_IDS`/`JOINT_NAMES`, the URDF, and the controller config in sync when editing any one of them.
- Hardware nodes fail without the real devices: `position_node` / `moveit_dynamixel_bridge` need the servo bus on `/dev/ttyUSB0` (and must not share the bus — pick one runtime); `yolo_detection` / `perception_node` need a camera (RealSense for `perception_node`). All rely on `privileged` for device access.
- `wrist_camera_link`'s static TF (`camera_tf.launch.py`) is a **home-pose-only placeholder** — it does not move with the arm. Don't trust it for pick geometry once the arm has left home; real eye-in-hand tracking needs the wrist camera integrated as a URDF joint (not done yet).
- **`ros2 run`/`ros2 launch` leak child nodes:** `kill <PID>`/`Ctrl-C` often kills only the wrapper, leaving the python node or `static_transform_publisher` running (→ CPU spin, `/arm_status` noise, stale TF). Clean up with `pkill -f <node>` and verify via `ps aux | grep ros2`.
- Branch strategy: `main` stays stable; feature work on `feat/*` branches.
</content>
