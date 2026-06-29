#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncWrite, GroupSyncRead


ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000
DEVICENAME = "/dev/ttyUSB0"

DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095
DXL_CENTER_POSITION = 2048

TICKS_PER_RAD = 4096.0 / (2.0 * math.pi)


JOINT_CONFIG = {
    "joint_1": {"id": 0, "center": 2048, "direction": 1},
    "joint_2": {"id": 1, "center": 2048, "direction": 1},
    "joint_3": {"id": 2, "center": 2048, "direction": 1},
}


class MoveItDynamixelBridge(Node):
    def __init__(self):
        super().__init__("moveit_dynamixel_bridge")

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            raise RuntimeError(f"Failed to open port: {DEVICENAME}")

        if not self.port_handler.setBaudRate(BAUDRATE):
            raise RuntimeError(f"Failed to set baudrate: {BAUDRATE}")

        self.group_sync_write = GroupSyncWrite(
            self.port_handler,
            self.packet_handler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )

        self.group_sync_read = GroupSyncRead(
            self.port_handler,
            self.packet_handler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )

        for joint_name, config in JOINT_CONFIG.items():
            dxl_id = config["id"]

            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_ENABLE,
            )

            if result != 0 or error != 0:
                self.get_logger().warn(
                    f"Torque enable failed: {joint_name}, id={dxl_id}, "
                    f"result={result}, error={error}"
                )
            else:
                self.get_logger().info(f"Torque enabled: {joint_name} -> id {dxl_id}")

            self.group_sync_read.addParam(dxl_id)

        self.trajectory_sub = self.create_subscription(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            self.trajectory_callback,
            10,
        )

        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self.execute_follow_joint_trajectory,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.feedback_timer = self.create_timer(0.05, self.publish_joint_states)

        self.get_logger().info("MoveIt Dynamixel bridge started")

    def rad_to_tick(self, joint_name, rad):
        config = JOINT_CONFIG[joint_name]
        tick = config["center"] + config["direction"] * rad * TICKS_PER_RAD
        tick = int(round(tick))
        return max(DXL_MINIMUM_POSITION_VALUE, min(DXL_MAXIMUM_POSITION_VALUE, tick))

    def tick_to_rad(self, joint_name, tick):
        config = JOINT_CONFIG[joint_name]
        return (tick - config["center"]) / (config["direction"] * TICKS_PER_RAD)

    def int_to_little_endian_4bytes(self, value):
        return [
            value & 0xFF,
            (value >> 8) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 24) & 0xFF,
        ]

    def goal_callback(self, goal_request):
        self.get_logger().info("Received FollowJointTrajectory goal")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested")
        return CancelResponse.ACCEPT

    def execute_follow_joint_trajectory(self, goal_handle):
        trajectory = goal_handle.request.trajectory

        self.get_logger().info(
            f"Executing FollowJointTrajectory with {len(trajectory.points)} points"
        )

        self.trajectory_callback(trajectory)

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory sent to Dynamixel motors"
        return result

    def trajectory_callback(self, msg):
        if not msg.points:
            return

        point = msg.points[-1]

        if len(msg.joint_names) != len(point.positions):
            self.get_logger().warn("JointTrajectory names/positions length mismatch")
            return

        self.group_sync_write.clearParam()

        for joint_name, rad in zip(msg.joint_names, point.positions):
            if joint_name not in JOINT_CONFIG:
                self.get_logger().warn(f"Unknown joint from MoveIt: {joint_name}")
                continue

            dxl_id = JOINT_CONFIG[joint_name]["id"]
            goal_tick = self.rad_to_tick(joint_name, rad)
            param_goal_position = self.int_to_little_endian_4bytes(goal_tick)

            ok = self.group_sync_write.addParam(dxl_id, param_goal_position)
            if not ok:
                self.get_logger().warn(f"Failed to add sync write param: id={dxl_id}")

            self.get_logger().info(
                f"{joint_name} -> id {dxl_id}: {rad:.3f} rad -> {goal_tick}"
            )

        result = self.group_sync_write.txPacket()
        if result != 0:
            self.get_logger().warn(f"GroupSyncWrite failed: result={result}")

        self.group_sync_write.clearParam()

    def publish_joint_states(self):
        result = self.group_sync_read.txRxPacket()
        if result != 0:
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for joint_name, config in JOINT_CONFIG.items():
            dxl_id = config["id"]

            available = self.group_sync_read.isAvailable(
                dxl_id,
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION,
            )
            if not available:
                continue

            tick = self.group_sync_read.getData(
                dxl_id,
                ADDR_PRESENT_POSITION,
                LEN_PRESENT_POSITION,
            )

            msg.name.append(joint_name)
            msg.position.append(self.tick_to_rad(joint_name, tick))

        self.joint_state_pub.publish(msg)

    def destroy_node(self):
        for config in JOINT_CONFIG.values():
            dxl_id = config["id"]
            self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_DISABLE,
            )

        self.port_handler.closePort()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MoveItDynamixelBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
