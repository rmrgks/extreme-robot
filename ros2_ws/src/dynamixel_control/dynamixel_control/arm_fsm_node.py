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

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy

from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (MotionPlanRequest, Constraints, PositionConstraint,
                             OrientationConstraint, BoundingVolume)
from shape_msgs.msg import SolidPrimitive
from robot_arm_msgs.msg import ArrivalStatus, ChassisMode, ArmStatus, DetectedObject


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
        self.declare_parameter('tip_link', 'link_6')             # 그리퍼 부모 링크
        self.declare_parameter('pick_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('pos_tolerance', 0.01)            # [m]
        self.declare_parameter('orient_tolerance', 0.1)          # [rad]
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('vel_scale', 0.1)                 # 저속(파지 안전)
        self.declare_parameter('acc_scale', 0.1)
        # 그리퍼 (prismatic finger, 단위 m — 실측 캘리브 필요)
        self.declare_parameter('gripper_joints', ['left_finger_joint', 'right_finger_joint'])
        self.declare_parameter('gripper_open', 0.02)
        self.declare_parameter('gripper_close', 0.0)
        # 전류(effort) 임계 — /joint_states effort 단위, 실측 필요(TODO)
        self.declare_parameter('grasp_effort_thresh', 0.2)
        self.declare_parameter('drop_effort_thresh', 0.05)
        # 동작 제어
        self.declare_parameter('max_regrasp', 3)
        self.declare_parameter('gripper_action_time', 1.0)       # 그리퍼 동작 시간 [s]
        self.declare_parameter('tick_rate', 10.0)

        g = self.get_parameter
        self.planning_group = g('planning_group').value
        self.tip_link = g('tip_link').value
        self.pick_frame_id = g('pick_frame_id').value
        self.pos_tol = g('pos_tolerance').value
        self.orient_tol = g('orient_tolerance').value
        self.planning_time = g('planning_time').value
        self.vel_scale = g('vel_scale').value
        self.acc_scale = g('acc_scale').value
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
        self._move = ActionClient(self, MoveGroup, 'move_action')          # MoveIt
        self._grip = ActionClient(self, FollowJointTrajectory,
                                  '/gripper_controller/follow_joint_trajectory')

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
        self.get_logger().info('arm_fsm_node started (MoveIt 경로, state=IDLE)')

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

    # ── FSM tick ───────────────────────────────

    def _tick(self):
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
        """MoveIt에 파지 pose 목표 전송(IK·경로계획 위임). 디스패치만 하고 DESCEND에서 대기."""
        self._publish_status(ARM_PLANNING)
        grasp_pose = self._grasp_pose()
        if grasp_pose is None:
            self._publish_status(ARM_FAILED)
            self._transition(State.IDLE)
            return
        self._begin_arm_move(grasp_pose)
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
        """수직 리프트 → 운반 자세. TODO: base_link +Z 리프트 pose(TF 필요)."""
        self._publish_status(ARM_EXECUTING)
        lift_pose = self._carry_pose()
        if lift_pose is None:
            self.get_logger().warn('TODO: LIFT/CARRY 목표 pose 미구현 — 스킵하고 CARRY 진입')
            self._transition(State.CARRY)
            return
        if self._motion_state == 'idle':
            self._begin_arm_move(lift_pose)
            return
        if self._motion_state == 'active':
            return
        self._motion_state = 'idle'
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
        # TODO: 파지 pose에서 base_link +Z로 들어올린 운반 자세 (TF 변환 필요).
        return None

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
