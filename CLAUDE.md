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
- Two compose files exist: `docker-compose.yml` and `docker-compose.wsl.yml`. Both run `privileged: true` with `network_mode: host` and mount X11/WSLg sockets for GUI. The container is `ros2_humble`.
- The container runs `privileged`, so host devices (e.g. the Dynamixel USB serial adapter at `/dev/ttyUSB0`, the camera) are reachable without explicit `devices:` mappings — but the hardware must actually be plugged into the host.

## Common commands

Start container + enter (host):
```bash
# Ubuntu native
xhost +local:docker && docker compose up -d
# WSL2
xhost +local: && docker compose -f docker-compose.wsl.yml up -d

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

## Packages (`ros2_ws/src/`)

### robot_arm_msgs (ament_cmake) — 공통 메시지 패키지
양팀(로봇팔·파워트레인)이 공유하는 커스텀 메시지 5개: `DetectedObject`(class_id/name/confidence/`geometry_msgs/Pose`/bbox), `DetectedObjectArray`(header + objects[]), `ArrivalStatus`, `ChassisMode`, `ArmStatus`. 인터페이스 상세는 `CLAUDE_Plan.md` §1 참고.

### robot_arm_perception (ament_python) — markerless 인식 노드
`perception_node` + `stream_node` 두 개. `perception_node`: RealSense D435i color+depth → YOLO **segmentation** 추론 → `/detected_objects`(`DetectedObjectArray`) 30Hz publish. **markerless pose**(대회 규정상 타겟 마커 부착 금지): translation은 마스크 centroid의 depth median deproject(`yolo_depth_3d.py` 로직 포팅, align 생략), orientation은 마스크 (u,v) 픽셀 **2D PCA** 주축각 → optical Z yaw quaternion. 카메라 intrinsics는 RealSense 스트림에서 직접 취득(calibration yaml 불필요). 또한 `/pick_target`(`DetectedObject`, transient_local latched)을 publish: `pick_classes` 화이트리스트 ∩ `pick_min_conf` 이상 ∩ depth 조건(`require_depth`) 만족 객체 중 confidence 최고 하나(신호등/정지선 등 관찰 전용은 화이트리스트로 자동 제외). 파라미터: `model_path`(**seg 모델 필수**, 기본 `yolov8n-seg.pt`), `camera_mode`(`realsense`|`test`), `conf_threshold`, `pick_classes`, `pick_min_conf`, `require_depth`, `frame_id` 등. ArUco 경로는 제거됨. 진행 상황은 `CLAUDE_Plan.md`·`WORK_STATUS.md`.

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
- `urdf/robot_arm.urdf` now has **6 revolute joints** (`joint_1`..`joint_6`) plus a gripper: `gripper_mount` (fixed) and prismatic `left_finger_joint` / `right_finger_joint`. Includes `ros2_control` blocks.
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
