import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import RegionOfInterest
from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray


DUMMY_OBJECTS = [
    {'class_id': 0, 'class_name': 'target_box', 'confidence': 0.92,
     'x': 0.0, 'y': 0.0, 'z': 1.5, 'u1': 200, 'v1': 150, 'u2': 440, 'v2': 330},
    {'class_id': 1, 'class_name': 'traffic_light', 'confidence': 0.85,
     'x': 0.3, 'y': -0.1, 'z': 2.0, 'u1': 50, 'v1': 30, 'u2': 120, 'v2': 100},
]
PICK_CLASS = 'target_box'


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('publish_rate', 10.0)

        latched_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.pub_objects = self.create_publisher(
            DetectedObjectArray, '/detected_objects', 10)
        self.pub_pick = self.create_publisher(
            DetectedObject, '/pick_target', latched_qos)

        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info('perception_node started (DUMMY MODE)')

    def _make_object(self, d: dict) -> DetectedObject:
        obj = DetectedObject()
        obj.class_id = d['class_id']
        obj.class_name = d['class_name']
        obj.confidence = d['confidence']

        obj.pose = Pose(
            position=Point(x=d['x'], y=d['y'], z=d['z']),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

        roi = RegionOfInterest()
        roi.x_offset = d['u1']
        roi.y_offset = d['v1']
        roi.width = d['u2'] - d['u1']
        roi.height = d['v2'] - d['v1']
        obj.bbox = roi

        return obj

    def _publish(self):
        frame_id = self.get_parameter('frame_id').value
        now = self.get_clock().now().to_msg()

        array_msg = DetectedObjectArray()
        array_msg.header.stamp = now
        array_msg.header.frame_id = frame_id

        pick_candidate = None
        for d in DUMMY_OBJECTS:
            obj = self._make_object(d)
            array_msg.objects.append(obj)
            if d['class_name'] == PICK_CLASS and pick_candidate is None:
                pick_candidate = obj

        self.pub_objects.publish(array_msg)
        self.get_logger().info(
            f'/detected_objects: {len(array_msg.objects)} objects '
            f'({", ".join(o.class_name for o in array_msg.objects)})'
        )

        if pick_candidate is not None:
            self.pub_pick.publish(pick_candidate)
            self.get_logger().debug(
                f'/pick_target: {pick_candidate.class_name} '
                f'z={pick_candidate.pose.position.z:.2f}m '
                f'conf={pick_candidate.confidence:.2f}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
