"""RealSense D435i + YOLO + ArUco → DetectedObjectArray ROS2 퍼블리셔.

Phase 2 Step 1: YOLO 추론 → class_id / class_name / confidence / bbox
Phase 2 Step 2: ArUco estimatePoseSingleMarkers → pose (position + orientation)

pose 채우기 규칙:
  - ArUco 마커가 감지되고 YOLO bbox 안에 마커 중심이 들어오는 객체
      → tvec(position) + rvec→quaternion(orientation) 으로 채움
  - 마커 없는 관찰 대상 (신호등/정지선 등)
      → position/orientation 기본값(0) 유지

camera_mode 파라미터:
  realsense  실제 RealSense D435i (기본값)
  test       test_image_path 정지 이미지 반복 사용 (하드웨어 없이 검증)
"""
import os
import threading
import time

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import Pose, Point, Quaternion
from sensor_msgs.msg import RegionOfInterest
from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray


# ──────────────────────────────────────────────
# TensorRT 엔진 캐시 (yolo_depth_3d.py resolve_model 포팅)
# ──────────────────────────────────────────────

def _resolve_model(path: str, backend: str, height: int, width: int) -> str:
    if backend == 'pt' or path.endswith('.engine'):
        return path
    h32 = ((height + 31) // 32) * 32
    w32 = ((width + 31) // 32) * 32
    base = os.path.splitext(path)[0]
    cached = f'{base}_{h32}x{w32}_fp16.engine'
    if os.path.exists(cached):
        print(f'[perception] reusing cached engine: {cached}')
        return cached
    print(f'[perception] exporting TensorRT FP16 engine (imgsz={h32}x{w32}) — 5~10분 소요')
    from ultralytics import YOLO as _YOLO
    engine = _YOLO(path).export(format='engine', half=True, imgsz=(h32, w32))
    if engine != cached and os.path.exists(str(engine)):
        os.rename(str(engine), cached)
    return cached


# ──────────────────────────────────────────────
# RealSense 헬퍼 (yolo_depth_3d.py latest_frames 포팅)
# ──────────────────────────────────────────────

def _latest_frames(pipe):
    frames = pipe.wait_for_frames()
    while True:
        nxt = pipe.poll_for_frames()
        if nxt.size() == 0:
            return frames
        frames = nxt


# ──────────────────────────────────────────────
# rvec → quaternion (Shepperd method, scipy 불필요)
# ──────────────────────────────────────────────

def _rvec_to_quat(rvec: np.ndarray) -> tuple[float, float, float, float]:
    """Rodrigues 회전 벡터 → 쿼터니언 (x, y, z, w)."""
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


# ──────────────────────────────────────────────
# 노드
# ──────────────────────────────────────────────

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('backend', 'pt')          # 'pt' | 'trt'
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('classes', '')            # 쉼표구분 필터, 빈값=전체
        self.declare_parameter('width', 848)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('camera_mode', 'realsense')  # 'realsense' | 'test'
        self.declare_parameter('test_image_path', '')

        # calibration yaml 기본 경로: 패키지 share 디렉토리
        share_dir = get_package_share_directory('robot_arm_perception')
        default_calib = os.path.join(share_dir, 'config', 'camera_calibration.yaml')
        self.declare_parameter('calibration_yaml', default_calib)

        model_path = self.get_parameter('model_path').value
        backend = self.get_parameter('backend').value
        self._w = self.get_parameter('width').value
        self._h = self.get_parameter('height').value
        self._fps = self.get_parameter('fps').value
        self._camera_mode = self.get_parameter('camera_mode').value

        # YOLO 모델 로드
        from ultralytics import YOLO
        resolved = _resolve_model(model_path, backend, self._h, self._w)
        self.model = YOLO(resolved)
        self.get_logger().info(f'YOLO loaded: {resolved}')

        # ArUco 초기화 (calibration yaml 로드)
        self._aruco_ready = False
        self._load_aruco_config()

        # 카메라 초기화
        self._pipe = None
        self._test_img = None
        if self._camera_mode == 'realsense':
            self._init_realsense()
        else:
            self._init_test_image()

        # 퍼블리셔
        self.pub_objects = self.create_publisher(
            DetectedObjectArray, '/detected_objects', 10)
        # /pick_target은 Phase 2 Step 3(선별 로직)에서 추가

        # 추론 루프 스레드
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ── ArUco 초기화 ────────────────────────────

    def _load_aruco_config(self):
        yaml_path = self.get_parameter('calibration_yaml').value
        if not os.path.exists(yaml_path):
            self.get_logger().warn(
                f'calibration_yaml not found: {yaml_path} — ArUco pose 비활성')
            return
        try:
            with open(yaml_path) as f:
                cfg = yaml.safe_load(f)

            cm = cfg['camera_matrix']
            self._cam_matrix = np.array([
                [cm['fx'],  0.0,       cm['ppx']],
                [0.0,       cm['fy'],  cm['ppy']],
                [0.0,       0.0,       1.0],
            ], dtype=np.float64)
            self._dist_coeffs = np.array(cfg['dist_coeffs'], dtype=np.float64)

            ac = cfg['aruco']
            dict_id = getattr(cv2.aruco, ac['dictionary'])
            self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
            self._aruco_params = cv2.aruco.DetectorParameters_create()
            self._marker_length = float(ac['marker_length'])
            self._marker_class_map = {
                int(k): str(v) for k, v in ac['marker_class_map'].items()
            }

            self._aruco_ready = True
            self.get_logger().info(
                f'ArUco ready: {ac["dictionary"]}, '
                f'marker_length={self._marker_length}m, '
                f'class_map={self._marker_class_map}')
        except Exception as e:
            self.get_logger().error(f'ArUco config load failed: {e}')

    # ── 카메라 초기화 ──────────────────────────

    def _init_realsense(self):
        try:
            import pyrealsense2 as rs
            self._rs = rs
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.depth, self._w, self._h, rs.format.z16, self._fps)
            cfg.enable_stream(rs.stream.color, self._w, self._h, rs.format.bgr8, self._fps)
            pipe.start(cfg)
            self._pipe = pipe
            self.get_logger().info(
                f'RealSense started ({self._w}x{self._h} @ {self._fps}fps)')
        except Exception as e:
            self.get_logger().error(f'RealSense init failed: {e}')
            raise

    def _init_test_image(self):
        test_path = self.get_parameter('test_image_path').value
        if test_path and os.path.exists(test_path):
            img = cv2.imread(test_path)
            if img is not None:
                self._test_img = cv2.resize(img, (self._w, self._h))
                self.get_logger().info(f'Test image loaded: {test_path}')
                return
        self._test_img = np.zeros((self._h, self._w, 3), dtype=np.uint8)
        self.get_logger().warn('camera_mode=test: 빈 프레임 사용 (test_image_path 미지정)')

    # ── ArUco 마커 매칭 및 pose 채우기 ────────

    def _fill_aruco_poses(self, objects: list[DetectedObject], color_img: np.ndarray):
        """ArUco 검출 → YOLO bbox와 매칭 → pose(position+orientation) 채우기."""
        if not self._aruco_ready or not objects:
            return

        corners, ids, _ = cv2.aruco.detectMarkers(
            color_img, self._aruco_dict, parameters=self._aruco_params)
        if ids is None:
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self._marker_length, self._cam_matrix, self._dist_coeffs)

        for i, marker_id in enumerate(ids.flatten()):
            # 마커 중심 픽셀 좌표
            cx = float(corners[i][0][:, 0].mean())
            cy = float(corners[i][0][:, 1].mean())

            # YOLO bbox 중 마커 중심을 포함하는 것 탐색
            for obj in objects:
                x1 = obj.bbox.x_offset
                y1 = obj.bbox.y_offset
                x2 = x1 + obj.bbox.width
                y2 = y1 + obj.bbox.height
                if not (x1 <= cx <= x2 and y1 <= cy <= y2):
                    continue

                # pose 채우기
                tx, ty, tz = tvecs[i][0]
                obj.pose.position = Point(x=float(tx), y=float(ty), z=float(tz))

                qx, qy, qz, qw = _rvec_to_quat(rvecs[i][0])
                obj.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

                self.get_logger().debug(
                    f'ArUco ID={marker_id} → {obj.class_name} '
                    f'pos=({tx:.3f},{ty:.3f},{tz:.3f}) '
                    f'quat=({qx:.3f},{qy:.3f},{qz:.3f},{qw:.3f})')
                break  # bbox 하나에 마커 하나만 매칭

    # ── 클래스 필터 ────────────────────────────

    def _get_cls_filter(self):
        classes_str = self.get_parameter('classes').value
        if not classes_str.strip():
            return None
        name_to_id = {v: k for k, v in self.model.names.items()}
        ids = []
        for name in classes_str.split(','):
            name = name.strip()
            if name in name_to_id:
                ids.append(name_to_id[name])
            else:
                self.get_logger().warn(f'Unknown class "{name}" — 무시')
        return ids or None

    # ── 추론 루프 ──────────────────────────────

    def _loop(self):
        idx = 0
        interval = 1.0 / self._fps

        while self._running and rclpy.ok():
            t0 = time.time()

            color_img = self._grab_frame()
            if color_img is None:
                time.sleep(0.01)
                continue

            conf = self.get_parameter('conf_threshold').value
            cls_filter = self._get_cls_filter()
            frame_id = self.get_parameter('frame_id').value

            # ① YOLO 추론
            results = self.model.predict(
                color_img, conf=conf, classes=cls_filter, verbose=False)

            array_msg = DetectedObjectArray()
            array_msg.header.stamp = self.get_clock().now().to_msg()
            array_msg.header.frame_id = frame_id

            for box in results[0].boxes:
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])

                obj = DetectedObject()
                obj.class_id = int(box.cls[0])
                obj.class_name = self.model.names[obj.class_id]
                obj.confidence = float(box.conf[0])
                obj.pose = Pose()  # 기본값 — 마커 없으면 그대로

                roi = RegionOfInterest()
                roi.x_offset = x1
                roi.y_offset = y1
                roi.width = x2 - x1
                roi.height = y2 - y1
                obj.bbox = roi

                array_msg.objects.append(obj)

            # ② ArUco로 마커 붙은 객체의 pose 채우기
            self._fill_aruco_poses(array_msg.objects, color_img)

            self.pub_objects.publish(array_msg)

            # 30프레임마다 로그
            if idx % 30 == 0:
                if array_msg.objects:
                    summary = ', '.join(
                        f'{o.class_name}({o.confidence:.2f})'
                        f'{"[posed]" if o.pose.position.z != 0.0 else ""}'
                        for o in array_msg.objects
                    )
                    self.get_logger().info(
                        f'[{idx}] {len(array_msg.objects)} objects: {summary}')
                else:
                    self.get_logger().info(f'[{idx}] (검출 없음)')
            idx += 1

            if self._camera_mode == 'test':
                elapsed = time.time() - t0
                if interval - elapsed > 0:
                    time.sleep(interval - elapsed)

    def _grab_frame(self):
        if self._camera_mode == 'realsense':
            try:
                frames = _latest_frames(self._pipe)
                color_frame = frames.get_color_frame()
                if not color_frame:
                    return None
                return np.asanyarray(color_frame.get_data())
            except Exception as e:
                self.get_logger().error(f'Frame grab error: {e}')
                return None
        else:
            return self._test_img.copy()

    # ── 정리 ───────────────────────────────────

    def destroy_node(self):
        self._running = False
        self._thread.join(timeout=2.0)
        if self._pipe is not None:
            self._pipe.stop()
        super().destroy_node()


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
