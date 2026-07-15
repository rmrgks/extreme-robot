"""로봇팔 미션 FSM 노드 (Phase 3, 구간2 구호물자 운반 중심).

설계 문서: `project_docs/PHASE3_FSM_설계.md` §4 상태표 / §5 핸드셰이크.
구현 방식(결정 '가', 2026-06-29): **MoveIt 단일 경로**.
  - 팔 모션: MoveIt `move_action`(MoveGroup)에 목표 pose 전송 → IK·경로계획은
    MoveIt이 수행 → MoveIt이 `arm_controller` FollowJointTrajectory로 실행 →
    upstream `moveit_dynamixel_bridge`가 실제 다이나믹셀 구동.
  - 그리퍼: `/gripper_controller/follow_joint_trajectory`(FollowJointTrajectory)로
    열림/닫힘 명령(그리퍼도 Dynamixel, 결정 B).
  - 전류 피드백: `/joint_states`의 effort 필드에서 그리퍼 관절 전류 → 파지/DROP 판정.

  ⚠️ 전제(브릿지 측 선행 작업, PHASE3_FSM_설계.md §6 → 결정 '가'로 이관):
    1) `moveit_dynamixel_bridge`가 `/joint_states`에 **effort(전류)** 를 채워야 함
       (현재는 position만 발행) — 안 그러면 파지/DROP 판정 불가.
    2) 그리퍼 관절 + `gripper_controller` 실행 경로를 브릿지/컨트롤러에 추가해야 함.
    3) 카메라 frame → planning frame(base_link) **TF**가 있어야 MoveIt이 목표를 변환.

상태 흐름(§4): IDLE → PERCEIVE → PLAN → DESCEND → GRASP_CHECK → LIFT → CARRY
  → (ARRIVED_DROP) RELEASE → DONE → IDLE / CARRY 중 DROP → REGRASP → (한계초과) ABORT
  LOCKED: 거친지형/추종 모드 → 진행 중 모션 취소 + 현재 자세 홀드.

⚠️ 스켈레톤: MoveGroup pose goal / 그리퍼 액션 / effort 판정 / FSM 골격은 구현.
   LIFT·CARRY 목표 pose, 임계값 캘리브, TF 연결은 TODO.
"""
from enum import Enum, auto

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration as RclpyDuration
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, PositionConstraint,
                             OrientationConstraint, BoundingVolume, RobotState)
from moveit_msgs.srv import GetPositionFK
from shape_msgs.msg import SolidPrimitive
from robot_arm_msgs.msg import ArrivalStatus, ChassisMode, ArmStatus, DetectedObject
from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset


# 2026-07-15 Isaac Sim 기반 재export(robotarm_urdf_20260711.urdf) 기준 — URDF 자체는
# 팔 5축(arm_joint_1~5)을 전부 반영하지만, analytic IK(FK+수치 자코비안)는 아직 앞의
# 3관절만 풀도록 남겨둠(HW-7 당시 6DOF pose goal이 NO_IK_SOLUTION이던 문제 회피용으로
# 도입된 3DOF 위치전용 IK — URDF가 3축만 있어서가 아니라 solver를 아직 5DOF로 확장 안
# 해서임, 방향은 여전히 무시). MoveGroup 경로(§6 결정 '가')는 남겨두되 ik_mode:='moveit'로
# 전환 가능하게만 유지.
ARM_JOINT_NAMES = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3']


# ──────────────────────────────────────────────
# status / mode 문자열 (⚠️ 잠정값 — 파워트레인 팀과 합의 후 확정, §3·§6-C)
# ──────────────────────────────────────────────
ARRIVED_PICKUP = 'ARRIVED_PICKUP'      # 박스 정렬 완료 → 집어
ARRIVED_DROP = 'ARRIVED_DROP'          # 하역 지점 도착 → 내려

ARM_IDLE = 'IDLE'
ARM_PERCEIVING = 'PERCEIVING'
ARM_PLANNING = 'PLANNING'
ARM_EXECUTING = 'EXECUTING'
ARM_CARRYING = 'CARRYING'
ARM_DONE = 'DONE'
ARM_FAILED = 'FAILED'

LOCK_MODES = {'CORNERING', 'ROUGH_TERRAIN', 'FOLLOW_LEAD'}
DRIVING_MODE = 'DRIVING'

# moveit_msgs/MoveItErrorCodes.SUCCESS
MOVEIT_SUCCESS = 1


class State(Enum):
    IDLE = auto()
    PERCEIVE = auto()
    PLAN = auto()
    DESCEND = auto()
    GRASP_CHECK = auto()
    LIFT = auto()
    CARRY = auto()
    REGRASP = auto()
    RELEASE = auto()
    DONE = auto()
    ABORT = auto()
    LOCKED = auto()


class ArmFsmNode(Node):
    def __init__(self):
        super().__init__('arm_fsm_node')

        # ── 파라미터 ──────────────────────────────
        # MoveIt
        self.declare_parameter('planning_group', 'arm')          # SRDF group
        # tip_link: arm_joint_5 이후 고정 조인트 체인의 마지막 링크(link_051) — 여기서
        # gripper_drive_joint(구동)와 gripper_linkage_base_fixed(그리퍼 고정 베이스)가 갈라짐.
        # 2026-07-15 Isaac Sim 재export(robotarm_urdf_20260711.urdf) 기준.
        self.declare_parameter('tip_link', 'link_051')            # 그리퍼 부모 링크
        self.declare_parameter('base_frame', 'base_link')        # planning frame (리프트 기준)
        self.declare_parameter('lift_height', 0.10)              # LIFT 시 base_link +Z [m]
        self.declare_parameter('pick_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('pos_tolerance', 0.01)            # [m]
        self.declare_parameter('orient_tolerance', 0.1)          # [rad]
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('vel_scale', 0.1)                 # 저속(파지 안전)
        self.declare_parameter('acc_scale', 0.1)
        # 'analytic'(기본, URDF 3관절 한정 수치 IK) | 'moveit'(URDF 5축 완성 후 전환)
        self.declare_parameter('ik_mode', 'analytic')
        self.declare_parameter('ik_max_iters', 8)
        self.declare_parameter('ik_tol', 0.01)          # [m] 위치 수렴 허용오차
        self.declare_parameter('ik_accept_tol', 0.03)   # [m] 최종 실패 판정 기준
        self.declare_parameter('arm_move_speed', 0.5)   # [rad/s] 직접명령 시 소요시간 추정용
        # 그리퍼 — gripper_type 이 gripper_presets.GRIPPER_PRESETS 의 기본값을 고르고,
        # 아래 개별 파라미터는 필요 시 CLI/런치로 여전히 개별 오버라이드 가능.
        self.declare_parameter('gripper_type', DEFAULT_GRIPPER)
        gripper_type = self.get_parameter('gripper_type').value
        gpreset = get_preset(gripper_type, self.get_logger())

        self.declare_parameter('gripper_joints', gpreset['gripper_joints'])
        self.declare_parameter('gripper_open', gpreset['gripper_open_m'])
        self.declare_parameter('gripper_close', gpreset['gripper_close_m'])
        # 전류(effort) 임계 — moveit_dynamixel_bridge 가 /joint_states.effort 에
        # raw signed PRESENT_CURRENT(XL430 기준 1단위≈2.69mA)를 발행. preset 값은 placeholder,
        # 실측 캘리브 필요(TODO): 무부하 파지 전류/낙하 시 전류를 측정해 임계값 설정.
        self.declare_parameter('grasp_effort_thresh', gpreset['grasp_effort_thresh'])
        self.declare_parameter('drop_effort_thresh', gpreset['drop_effort_thresh'])
        # 동작 제어
        self.declare_parameter('max_regrasp', 3)
        self.declare_parameter('gripper_action_time', gpreset['gripper_action_time'])  # [s]
        self.declare_parameter('tick_rate', 10.0)

        g = self.get_parameter
        self.planning_group = g('planning_group').value
        self.tip_link = g('tip_link').value
        self.base_frame = g('base_frame').value
        self.lift_height = g('lift_height').value
        self.pick_frame_id = g('pick_frame_id').value
        self.pos_tol = g('pos_tolerance').value
        self.orient_tol = g('orient_tolerance').value
        self.planning_time = g('planning_time').value
        self.vel_scale = g('vel_scale').value
        self.acc_scale = g('acc_scale').value
        self.ik_mode = g('ik_mode').value
        self.ik_max_iters = int(g('ik_max_iters').value)
        self.ik_tol = g('ik_tol').value
        self.ik_accept_tol = g('ik_accept_tol').value
        self.arm_move_speed = g('arm_move_speed').value
        self.gripper_type = gripper_type
        self.gripper_joints = list(g('gripper_joints').value)
        self.gripper_open = g('gripper_open').value
        self.gripper_close = g('gripper_close').value
        self.grasp_thresh = g('grasp_effort_thresh').value
        self.drop_thresh = g('drop_effort_thresh').value
        self.max_regrasp = g('max_regrasp').value
        self.gripper_action_time = g('gripper_action_time').value

        # ── 토픽/액션 I/O ─────────────────────────
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DetectedObject, '/pick_target', self._on_pick_target, latched)
        self.create_subscription(ArrivalStatus, '/arrival_status', self._on_arrival, 10)
        self.create_subscription(ChassisMode, '/chassis_mode', self._on_chassis_mode, 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)

        self.pub_status = self.create_publisher(ArmStatus, '/arm_status', 10)

        # TF: base_link ← tip_link 조회용 (LIFT 시 현재 TCP 기준 수직 리프트 계산)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._move = ActionClient(self, MoveGroup, 'move_action')          # MoveIt (ik_mode=='moveit')
        self._grip = ActionClient(self, FollowJointTrajectory,
                                  '/gripper_controller/follow_joint_trajectory')

        # analytic IK 경로 (ik_mode=='analytic', 기본): FK 서비스 + 직접 관절궤적 publish
        # ⚠️ FK 호출은 _tick(타이머 콜백) 안에서 블로킹 대기함 — self 를 spin하면 이미
        # 실행 중인 콜백을 재진입 spin 하게 되어 응답을 못 받고 타임아웃(실측 확인:
        # 독립 스크립트로는 2회 반복만에 수렴하는데 노드 내부에서는 즉시 실패).
        # 별도 헬퍼 노드/이그제큐터로 분리해서 우회.
        self._fk_node = rclpy.create_node('arm_fsm_fk_client')
        self._fk_client = self._fk_node.create_client(GetPositionFK, '/compute_fk')
        self._arm_traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._joint_position = {}          # joint_name -> position(rad), /joint_states 에서 갱신
        self._arm_move_deadline = None      # analytic 이동 완료 예상 시각

        # ── 내부 상태 ─────────────────────────────
        self.state = State.IDLE
        self._state_enter_t = self.get_clock().now()
        self._prev_state = None
        self.locked = False
        self.pick_target = None
        self.mission_id = 0
        self.regrasp_cnt = 0
        self._joint_effort = {}            # joint_name -> effort
        # 팔 모션(MoveIt) 진행 추적
        self._motion_state = 'idle'        # 'idle' | 'active' | 'done'
        self._motion_ok = False
        self._arm_goal_handle = None
        self._grip_sent = False            # 상태 진입 시 _transition에서 리셋

        period = 1.0 / g('tick_rate').value
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'arm_fsm_node started (MoveIt 경로, state=IDLE, gripper_type={self.gripper_type})'
        )

    # ── 콜백 ───────────────────────────────────

    def _on_pick_target(self, msg):
        self.pick_target = msg

    def _on_arrival(self, msg):
        self.mission_id = msg.mission_id
        if msg.status == ARRIVED_PICKUP and self.state == State.IDLE:
            self._transition(State.PERCEIVE)
        elif msg.status == ARRIVED_DROP and self.state == State.CARRY:
            self._transition(State.RELEASE)

    def _on_chassis_mode(self, msg):
        if msg.mode in LOCK_MODES and not self.locked:
            self.locked = True
            if self.state not in (State.IDLE, State.LOCKED):
                self._prev_state = self.state
                self._cancel_arm_motion()
                self._transition(State.LOCKED)
        elif msg.mode == DRIVING_MODE and self.locked:
            self.locked = False
            if self.state == State.LOCKED:
                # TODO: 취소된 모션 복구 정밀화 — 스켈레톤은 직전 상태 재진입
                self._transition(self._prev_state or State.IDLE)

    def _on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.effort):
                self._joint_effort[name] = msg.effort[i]
            if i < len(msg.position):
                self._joint_position[name] = msg.position[i]

    # ── FSM tick ───────────────────────────────

    def _tick(self):
        if (self._motion_state == 'active' and self._arm_move_deadline is not None
                and self.get_clock().now() >= self._arm_move_deadline):
            self._motion_state = 'done'
            self._arm_move_deadline = None
        handler = getattr(self, f'_do_{self.state.name.lower()}', None)
        if handler:
            handler()

    def _do_idle(self):
        pass

    def _do_perceive(self):
        self._publish_status(ARM_PERCEIVING)
        if self.pick_target is None:
            return
        if self.pick_target.pose.position.z == 0.0:   # depth 무효 (Phase 2 require_depth 기준)
            self.get_logger().warn('pick_target depth 무효 — 대기')
            return
        self._transition(State.PLAN)

    def _do_plan(self):
        """파지 목표로 이동 시작. 디스패치만 하고 DESCEND에서 대기."""
        self._publish_status(ARM_PLANNING)
        if self.ik_mode == 'moveit':
            grasp_pose = self._grasp_pose()
            if grasp_pose is None:
                self._publish_status(ARM_FAILED)
                self._transition(State.IDLE)
                return
            self._begin_arm_move(grasp_pose)
            self._transition(State.DESCEND)
            return

        target = self._grasp_target_xyz()
        if target is None or not self._move_to_xyz(target):
            self._publish_status(ARM_FAILED)
            self._transition(State.IDLE)
            return
        self._transition(State.DESCEND)

    def _do_descend(self):
        """MoveIt 모션 결과 대기 (저속 실행 = 하강 포함). TODO: 접촉 시 arm effort 감시."""
        self._publish_status(ARM_EXECUTING)
        if self._motion_state == 'active':
            return
        ok = self._motion_ok
        self._motion_state = 'idle'
        self._transition(State.GRASP_CHECK if ok else State.IDLE)
        if not ok:
            self._publish_status(ARM_FAILED)

    def _do_grasp_check(self):
        """그리퍼 닫고 effort(전류)로 파지 판정."""
        if not self._grip_sent:
            self._send_gripper(self.gripper_close)
            self._grip_sent = True
            return
        if self._elapsed() < self.gripper_action_time:
            return
        if self._gripper_effort() >= self.grasp_thresh:
            self.regrasp_cnt = 0
            self._transition(State.LIFT)
        else:
            self.get_logger().warn('파지 실패(effort 미달) → 재계획')
            self._transition(State.PLAN)

    def _do_lift(self):
        """수직 리프트 → 운반 자세 (base_link +Z, 현재 tip TF 기준)."""
        self._publish_status(ARM_EXECUTING)
        if self._motion_state == 'active':
            return
        if self._motion_state == 'done':
            self._motion_state = 'idle'
            self._transition(State.CARRY)
            return

        # 'idle' — 리프트 모션 시작
        if self.ik_mode == 'moveit':
            lift_pose = self._carry_pose()
            if lift_pose is None:
                self.get_logger().warn('carry pose 미구현/TF 실패 — 스킵하고 CARRY 진입')
                self._transition(State.CARRY)
                return
            self._begin_arm_move(lift_pose)
            return

        target = self._lift_target_xyz()
        if target is None or not self._move_to_xyz(target):
            self.get_logger().warn('LIFT 목표 계산/이동 실패 — 스킵하고 CARRY 진입')
            self._transition(State.CARRY)

    def _do_carry(self):
        """운반 중 그리퍼 effort 감시 → 급감 시 DROP."""
        self._publish_status(ARM_CARRYING)
        if self._elapsed() < 0.5:          # 진입 직후 dwell (오탐 방지)
            return
        if self._gripper_effort() < self.drop_thresh:
            self.get_logger().warn('DROP 감지 (effort 급감)')
            self._transition(State.REGRASP)
        # ARRIVED_DROP 수신은 _on_arrival에서 RELEASE로 전이

    def _do_regrasp(self):
        self.regrasp_cnt += 1
        self._publish_status(ARM_FAILED)
        if self.regrasp_cnt > self.max_regrasp:
            self.get_logger().error('재파지 한계 초과 → ABORT')
            self._transition(State.ABORT)
        else:
            self._transition(State.PERCEIVE)   # 파워트레인 ARRIVED_PICKUP 재발행 가정

    def _do_release(self):
        self._publish_status(ARM_EXECUTING)
        if not self._grip_sent:
            self._send_gripper(self.gripper_open)
            self._grip_sent = True
            return
        if self._elapsed() > self.gripper_action_time:
            self._transition(State.DONE)

    def _do_done(self):
        self._publish_status(ARM_DONE)
        self.pick_target = None
        self._transition(State.IDLE)

    def _do_abort(self):
        self._publish_status(ARM_FAILED)
        self._transition(State.IDLE)

    def _do_locked(self):
        # 현재 자세 홀드: MoveIt에 새 goal 안 보냄 → 브릿지가 torque로 마지막 위치 유지.
        pass

    # ── MoveIt 팔 모션 ─────────────────────────

    def _begin_arm_move(self, pose_stamped):
        self._motion_state = 'active'
        self._motion_ok = False
        if not self._move.server_is_ready():
            self.get_logger().warn('move_action 서버 미준비 — MoveIt(move_group) 실행 확인')
        goal = self._build_move_group_goal(pose_stamped)
        self._move.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('MoveGroup goal 거부됨')
            self._motion_state = 'done'
            self._motion_ok = False
            return
        self._arm_goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_arm_result)

    def _on_arm_result(self, future):
        result = future.result().result
        self._motion_ok = (result.error_code.val == MOVEIT_SUCCESS)
        self._motion_state = 'done'
        self._arm_goal_handle = None

    def _cancel_arm_motion(self):
        if self._arm_goal_handle is not None:
            self._arm_goal_handle.cancel_goal_async()
            self._arm_goal_handle = None
        self._arm_move_deadline = None
        self._motion_state = 'idle'

    def _build_move_group_goal(self, pose_stamped):
        """목표 pose → MoveGroup goal (plan & execute). tip_link를 pose로 이동."""
        req = MotionPlanRequest()
        req.group_name = self.planning_group
        req.num_planning_attempts = 5
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale

        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = self.tip_link
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.pos_tol]
        region.primitives.append(sphere)
        region.primitive_poses.append(pose_stamped.pose)
        pc.constraint_region = region
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = pose_stamped.header
        oc.link_name = self.tip_link
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = self.orient_tol
        oc.absolute_y_axis_tolerance = self.orient_tol
        oc.absolute_z_axis_tolerance = self.orient_tol
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req
        # planning_options 기본값: plan_only=False → 계획 후 컨트롤러로 실행
        return goal

    def _grasp_pose(self):
        """/pick_target(DetectedObject, frame 정보 없음) → PoseStamped(pick_frame_id)."""
        if self.pick_target is None:
            return None
        ps = PoseStamped()
        ps.header.frame_id = self.pick_frame_id    # DetectedObject엔 header 없음 → 파라미터 사용
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = self.pick_target.pose
        return ps

    def _carry_pose(self):
        """현재 TCP(tip_link)를 base_link 기준 +Z 로 들어올린 운반 자세.

        파지 직후의 실제 말단 자세를 TF(base_frame←tip_link)로 조회 → z 에 lift_height
        를 더하고 orientation 은 유지(박스 자세 보존). base_frame 이 planning frame 이라
        MoveIt 이 바로 계획 가능. TF 미가용 시 None → 호출부(_do_lift)가 LIFT 스킵.
        """
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'carry_pose TF 조회 실패 ({self.base_frame} <- {self.tip_link}): {e}')
            return None

        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        t = tf.transform.translation
        ps.pose.position.x = t.x
        ps.pose.position.y = t.y
        ps.pose.position.z = t.z + self.lift_height
        ps.pose.orientation = tf.transform.rotation
        return ps

    # ── analytic IK (ik_mode=='analytic', URDF 3관절 한정) ─────

    def _current_arm_joint_positions(self):
        return [self._joint_position.get(j, 0.0) for j in ARM_JOINT_NAMES]

    def _grasp_target_xyz(self):
        """/pick_target(카메라 프레임) → base_frame 기준 (x,y,z). 방향은 무시(3DOF 한계)."""
        if self.pick_target is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.pick_frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f'grasp target TF 조회 실패: {e}')
            return None
        out = do_transform_pose(self.pick_target.pose, tf)
        return (out.position.x, out.position.y, out.position.z)

    def _lift_target_xyz(self):
        """현재 tip 위치(base_frame)에서 +Z lift_height 만큼 든 목표."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(f'lift target TF 조회 실패: {e}')
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z + self.lift_height)

    def _fk_tip(self, q):
        """3관절 각도 q=[j1,j2,j3] → tip_link 위치(base_frame) np.array, 실패 시 None."""
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.fk_link_names = [self.tip_link]
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = list(ARM_JOINT_NAMES)
        req.robot_state.joint_state.position = [float(v) for v in q]
        if not self._fk_client.service_is_ready():
            return None
        future = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self._fk_node, future, timeout_sec=1.0)
        res = future.result()
        if res is None or not res.pose_stamped:
            return None
        p = res.pose_stamped[0].pose.position
        return np.array([p.x, p.y, p.z])

    def _solve_position_ik(self, target_xyz, q_init):
        """FK + 수치 자코비안(finite-difference) 로 위치만 맞추는 3DOF IK.

        ARM_JOINT_NAMES가 앞 3관절(arm_joint_1~3)만 써서 MoveIt 6DOF pose IK 대신 이 방식을
        기본으로 씀(HW-7 실측 확인, compute_ik가 현재 실제 tip pose에도 NO_IK_SOLUTION 반환하던
        문제 회피 — URDF 자체는 5축 다 있음, solver가 아직 5DOF로 확장 안 됨). 방향은 포기하고
        위치만 댐핑 최소자승(Levenberg-Marquardt 유사)으로 반복 수렴.
        """
        q = np.array(q_init, dtype=float)
        target = np.array(target_xyz, dtype=float)
        eps = 0.05
        lam = 0.01
        max_step = 0.4

        p = self._fk_tip(q)
        if p is None:
            return None

        for _ in range(self.ik_max_iters):
            err = target - p
            if np.linalg.norm(err) < self.ik_tol:
                return q.tolist()
            J = np.zeros((3, 3))
            for i in range(3):
                dq = np.zeros(3)
                dq[i] = eps
                p2 = self._fk_tip(q + dq)
                if p2 is None:
                    return None
                J[:, i] = (p2 - p) / eps
            delta = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err)
            norm = np.linalg.norm(delta)
            if norm > max_step:
                delta *= max_step / norm
            q = q + delta
            p = self._fk_tip(q)
            if p is None:
                return None

        if np.linalg.norm(target - p) < self.ik_accept_tol:
            return q.tolist()
        return None

    def _move_to_xyz(self, target_xyz):
        """target_xyz(base_frame) 로 analytic IK 계산 → /arm_controller/joint_trajectory 직접 발행."""
        q_current = self._current_arm_joint_positions()
        solution = self._solve_position_ik(target_xyz, q_current)
        if solution is None:
            self.get_logger().warn(f'analytic IK 실패 — 목표 도달 불가: {target_xyz}')
            return False

        delta = max(abs(a - b) for a, b in zip(solution, q_current))
        duration = max(1.0, min(5.0, delta / max(self.arm_move_speed, 0.05)))

        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in solution]
        pt.time_from_start = Duration(sec=int(duration))
        traj.points.append(pt)
        self._arm_traj_pub.publish(traj)

        self._motion_state = 'active'
        self._motion_ok = True
        self._arm_move_deadline = self.get_clock().now() + RclpyDuration(
            seconds=duration + 0.5)
        self.get_logger().info(
            f'analytic IK: {[round(v, 3) for v in solution]} rad, {duration:.1f}s 예상')
        return True

    # ── 그리퍼 ─────────────────────────────────

    def _send_gripper(self, position):
        """gripper_controller에 FollowJointTrajectory 단일 점 전송 (fire-and-forget)."""
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(position)] * len(self.gripper_joints)
        pt.time_from_start = Duration(sec=int(self.gripper_action_time))
        traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        if not self._grip.server_is_ready():
            self.get_logger().warn('gripper_controller 액션 서버 미준비')
        self._grip.send_goal_async(goal)

    def _gripper_effort(self):
        """대표 핑거 관절의 effort(전류) 절댓값 — /joint_states effort에서."""
        return abs(self._joint_effort.get(self.gripper_joints[0], 0.0))

    # ── 공통 ───────────────────────────────────

    def _elapsed(self):
        return (self.get_clock().now() - self._state_enter_t).nanoseconds * 1e-9

    def _transition(self, new_state):
        self.get_logger().info(f'{self.state.name} → {new_state.name}')
        self.state = new_state
        self._state_enter_t = self.get_clock().now()
        self._grip_sent = False

    def _publish_status(self, status):
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_id = self.mission_id
        msg.status = status
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
