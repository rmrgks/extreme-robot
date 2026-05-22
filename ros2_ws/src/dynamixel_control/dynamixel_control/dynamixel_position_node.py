import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import JointState

from dynamixel_sdk import PortHandler, PacketHandler


# =========================
# Dynamixel 기본 설정
# =========================
DEVICENAME = "/dev/ttyUSB0"
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

# =========================
# XL430 Control Table 주소
# =========================
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116

ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_TEMPERATURE = 146

TORQUE_ENABLE = 1

# 사용하는 모터 ID
DXL_IDS = [0, 1, 2, 3, 4]

# URDF joint 이름과 모터 ID 순서를 맞춤
JOINT_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
]


class DynamixelPositionNode(Node):
    def __init__(self):
        super().__init__("dynamixel_position_node")

        # 포트/패킷 핸들러 생성
        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        # 포트 열기
        if not self.port_handler.openPort():
            self.get_logger().error(f"Failed to open port: {DEVICENAME}")
            raise RuntimeError("Failed to open Dynamixel port")

        # Baudrate 설정
        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error(f"Failed to set baudrate: {BAUDRATE}")
            raise RuntimeError("Failed to set baudrate")

        self.get_logger().info("Dynamixel port opened")

        # 모든 모터 토크 ON
        for dxl_id in DXL_IDS:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_ENABLE,
            )

            if result != 0:
                self.get_logger().error(
                    f"Torque enable failed ID {dxl_id}: "
                    f"{self.packet_handler.getTxRxResult(result)}"
                )
            elif error != 0:
                self.get_logger().error(
                    f"Torque enable error ID {dxl_id}: "
                    f"{self.packet_handler.getRxPacketError(error)}"
                )
            else:
                self.get_logger().info(f"Torque enabled : ID {dxl_id}")

        # 위치 명령 구독
        # 메시지 형식: [모터ID, 목표위치]
        self.subscription = self.create_subscription(
            Int32MultiArray,
            "/dynamixel/goal_position",
            self.goal_callback,
            10,
        )

        # 모터 상태 publish
        # 데이터 형식: [id, position, velocity, current, temperature, ...]
        self.state_pub = self.create_publisher(
            Int32MultiArray,
            "/dynamixel/state",
            10,
        )

        # RViz / MoveIt용 joint_states publish
        self.joint_state_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        # 0.1초마다 상태 읽기
        self.timer = self.create_timer(0.1, self.read_state)

        self.get_logger().info("Dynamixel node started")

    def goal_callback(self, msg):
        """목표 위치 명령을 받아 Dynamixel에 전송."""
        if len(msg.data) < 2:
            self.get_logger().error("Message must be [id, goal_position]")
            return

        dxl_id = int(msg.data[0])
        goal_position = int(msg.data[1])

        if dxl_id not in DXL_IDS:
            self.get_logger().error(f"Unknown Dynamixel ID: {dxl_id}")
            return

        goal_position = max(0, min(4095, goal_position))

        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            dxl_id,
            ADDR_GOAL_POSITION,
            goal_position,
        )

        if result != 0:
            self.get_logger().error(
                f"Goal position failed ID {dxl_id}: "
                f"{self.packet_handler.getTxRxResult(result)}"
            )
        elif error != 0:
            self.get_logger().error(
                f"Goal position error ID {dxl_id}: "
                f"{self.packet_handler.getRxPacketError(error)}"
            )
        else:
            self.get_logger().info(
                f"Goal sent -> ID:{dxl_id} POS:{goal_position}"
            )

    def read_state(self):
        """모터의 현재 위치/속도/전류/온도를 읽어서 publish."""
        state_msg = Int32MultiArray()
        state_data = []

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = []
        joint_msg.position = []
        joint_msg.velocity = []
        joint_msg.effort = []

        for index, dxl_id in enumerate(DXL_IDS):
            position, result, error = self.packet_handler.read4ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_PRESENT_POSITION,
            )

            if result != 0:
                self.get_logger().warn(
                    f"Position read failed ID {dxl_id}: "
                    f"{self.packet_handler.getTxRxResult(result)}"
                )
                continue

            if error != 0:
                self.get_logger().warn(
                    f"Position read error ID {dxl_id}: "
                    f"{self.packet_handler.getRxPacketError(error)}"
                )
                continue

            velocity, _, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_PRESENT_VELOCITY,
            )

            current, _, _ = self.packet_handler.read2ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_PRESENT_CURRENT,
            )

            temperature, _, _ = self.packet_handler.read1ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_PRESENT_TEMPERATURE,
            )

            # raw position 0~4095를 radian으로 근사 변환
            # 2048을 중앙, 한 바퀴를 2pi로 가정
            rad = (int(position) - 2048) * (2.0 * math.pi / 4096.0)

            state_data.extend([
                int(dxl_id),
                int(position),
                int(velocity),
                int(current),
                int(temperature),
            ])

            joint_msg.name.append(JOINT_NAMES[index])
            joint_msg.position.append(rad)
            joint_msg.velocity.append(float(velocity))
            joint_msg.effort.append(float(current))

        state_msg.data = state_data

        self.state_pub.publish(state_msg)
        self.joint_state_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)

    node = DynamixelPositionNode()

    try:
        rclpy.spin(node)
    finally:
        node.port_handler.closePort()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
