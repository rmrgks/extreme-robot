import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

from dynamixel_sdk import PortHandler, PacketHandler


DEVICENAME = "/dev/ttyUSB0"   # Docker 안에서 실제 포트 확인 필요
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116

TORQUE_ENABLE = 1


class DynamixelPositionNode(Node):

    def __init__(self):
        super().__init__("dynamixel_position_node")

        self.port_handler = PortHandler(DEVICENAME)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort():
            self.get_logger().error(f"Failed to open port: {DEVICENAME}")
            raise RuntimeError("Failed to open Dynamixel port")

        if not self.port_handler.setBaudRate(BAUDRATE):
            self.get_logger().error(f"Failed to set baudrate: {BAUDRATE}")
            raise RuntimeError("Failed to set baudrate")

        self.get_logger().info("Dynamixel port opened")

        for dxl_id in [0, 1, 2, 3, 4]:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler,
                dxl_id,
                ADDR_TORQUE_ENABLE,
                TORQUE_ENABLE
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

        self.subscription = self.create_subscription(
            Int32MultiArray,
            "/dynamixel/goal_position",
            self.goal_callback,
            10
        )

        self.get_logger().info("Dynamixel node started")

    def goal_callback(self, msg):
        if len(msg.data) < 2:
            self.get_logger().error("Message must be [id, goal_position]")
            return

        dxl_id = int(msg.data[0])
        goal_position = int(msg.data[1])

        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler,
            dxl_id,
            ADDR_GOAL_POSITION,
            goal_position
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


def main(args=None):
    rclpy.init(args=args)

    node = DynamixelPositionNode()

    rclpy.spin(node)

    node.port_handler.closePort()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
