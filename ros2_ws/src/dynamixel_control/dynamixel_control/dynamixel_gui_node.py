import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

import tkinter as tk


class DynamixelGuiNode(Node):
    def __init__(self):
        super().__init__('dynamixel_gui_node')

        self.publisher = self.create_publisher(
            Int32MultiArray,
	    '/dynamixel/goal_position',
            10
        )

        self.root = tk.Tk()
        self.root.title("Dynamixel Multi Motor GUI")

        self.motor_ids = [0, 1, 2, 3, 4]
        self.entries = {}

        for row, motor_id in enumerate(self.motor_ids):
            tk.Label(self.root, text=f"ID {motor_id} Position").grid(row=row, column=0)

            entry = tk.Entry(self.root)
            entry.insert(0, "2048")
            entry.grid(row=row, column=1)

            self.entries[motor_id] = entry

            button = tk.Button(
                self.root,
                text="Send",
                command=lambda mid=motor_id: self.send_position(mid)
            )
            button.grid(row=row, column=2)

        all_button = tk.Button(
            self.root,
            text="Send All",
            command=self.send_all_positions
        )
        all_button.grid(row=len(self.motor_ids), column=0, columnspan=3)

    def send_position(self, motor_id):
        position = int(self.entries[motor_id].get())

        msg = Int32MultiArray()
        msg.data = [motor_id, position]

        self.publisher.publish(msg)
        self.get_logger().info(f"Published ID={motor_id}, position={position}")

    def send_all_positions(self):
        for motor_id in self.motor_ids:
            self.send_position(motor_id)

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)
            self.root.update()


def main(args=None):
    rclpy.init(args=args)

    node = DynamixelGuiNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
