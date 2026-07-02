"""RealSense D435i + YOLO(seg) → DetectedObjectArray ROS2 퍼블리셔 (markerless).

대회 규정상 타겟 객체에 마커 부착이 금지되어 pose 추정을 markerless 로 구현한다.

Phase 2 Step 1: YOLO segmentation 추론 → class_id / class_name / confidence / bbox / mask
Phase 2 Step 2 (markerless pose):
  - Translation: 마스크 centroid color 픽셀 → depth 픽셀 투영 → depth 패치 median
      → deproject (yolo_depth_3d.py 로직 포팅). 정렬(align) 생략, 검출별 투영만.
  - Orientation: 마스크 (u,v) 픽셀에 2D PCA → 주축 각도를 카메라 광축(Z) 기준
      yaw 로 근사 → quaternion. depth 노이즈와 무관하게 안정적인 1축 근사.

좌표계(camera color optical frame): X=오른쪽, Y=아래, Z=전방 [m] (REP-103 optical).
yaw 는 optical Z 축 회전 → quaternion (0, 0, sin(θ/2), cos(θ/2)).

camera_mode 파라미터:
  realsense  실제 RealSense D435i (기본값) — depth 기반 translation 활성
  test       test_image_path 정지 이미지 반복 (하드웨어 없이 검증) — translation 0,
             orientation(2D PCA)만 채워짐
"""
import math
import os
import threading
import time

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

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
# color↔depth 투영 파라미터 (yolo_depth_3d.py DepthCal 포팅)
# 전체 정렬(rs.align) 대신 검출별 color→depth 픽셀 투영만 수행 → Orin Nano
# 기준 5.5fps→30fps, 좌표 오차 평균 11mm (센서 노이즈 이내).
# ──────────────────────────────────────────────

class DepthCal:
    def __init__(self, rs, profile, dmin: float = 0.1, dmax: float = 10.0):
        dprof = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        cprof = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.di = dprof.get_intrinsics()
        self.ci = cprof.get_intrinsics()
        self.c2d = cprof.get_extrinsics_to(dprof)
        self.d2c = dprof.get_extrinsics_to(cprof)
        self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.dmin, self.dmax = dmin, dmax


def _deproject_centroid(rs, depth_frame, depth_img: np.ndarray,
                        cu: float, cv_: float, r: int,
                        cal: DepthCal):
    """color 픽셀 (cu,cv) → depth 픽셀 투영 → 패치 median depth → color (X,Y,Z)[m].

    정렬 없이 동작. depth 측정 불가(투영 화면밖/유효픽셀 부족) 시 None.
    """
    dpx = rs.rs2_project_color_pixel_to_depth_pixel(
        depth_frame.get_data(), cal.scale, cal.dmin, cal.dmax,
        cal.di, cal.ci, cal.c2d, cal.d2c, [cu, cv_])
    dx, dy = int(round(dpx[0])), int(round(dpx[1]))
    h, w = depth_img.shape
    if not (0 <= dx < w and 0 <= dy < h):
        return None
    patch = depth_img[max(dy - r, 0):dy + r, max(dx - r, 0):dx + r]
    valid = patch[patch > 0]
    if valid.size < 5:
        return None
    z = float(np.median(valid)) * cal.scale
    pt_d = rs.rs2_deproject_pixel_to_point(cal.di, [float(dx), float(dy)], z)
    return tuple(rs.rs2_transform_point_to_point(cal.d2c, pt_d))


# ──────────────────────────────────────────────
# 2D 마스크 PCA → yaw 쿼터니언
# ──────────────────────────────────────────────

def _mask_pca_yaw_quat(xs: np.ndarray, ys: np.ndarray):
    """마스크 픽셀 (xs=u, ys=v) 에 PCA → 주축 각도 → optical Z 회전 quaternion.

    이미지 좌표 x=오른쪽, y=아래 는 optical frame X,Y 와 같은 방향이므로 주축
    각도 θ=atan2(dy,dx) 가 곧 광축(Z) 기준 회전이 된다. 주축 방향은 ±180°
    모호성이 있으나(긴 축은 양방향 동일) 그리퍼 접근각엔 영향 없다.
    반환: (qx, qy, qz, qw). 점이 부족하면 None.
    """
    if xs.size < 10:
        return None
    pts = np.column_stack([xs.astype(np.float64), ys.astype(np.float64)])
    pts -= pts.mean(axis=0)
    cov = np.cov(pts.T)
    evals, evecs = np.linalg.eigh(cov)         # 오름차순 고유값
    major = evecs[:, int(np.argmax(evals))]    # 최대 분산 방향 = 객체 긴 축
    theta = math.atan2(float(major[1]), float(major[0]))
    return (0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0))


# ──────────────────────────────────────────────
# 노드
# ──────────────────────────────────────────────

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.declare_parameter('model_path', 'yolov8n-seg.pt')  # seg 모델 필수
        self.declare_parameter('backend', 'pt')          # 'pt' | 'trt'
        self.declare_parameter('conf_threshold', 0.4)
        self.declare_parameter('classes', '')            # 쉼표구분 필터, 빈값=전체
        self.declare_parameter('width', 848)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_color_optical_frame')
        self.declare_parameter('camera_mode', 'realsense')  # 'realsense' | 'test'
        self.declare_parameter('test_image_path', '')

        # 픽 대상 선별 (Phase 2 Step 3)
        self.declare_parameter('pick_classes', '')      # 쉼표구분 화이트리스트(필수). 빈값=후보없음
        self.declare_parameter('pick_min_conf', 0.5)    # 픽 후보 최소 confidence
        self.declare_parameter('require_depth', True)   # True=depth pose(z!=0) 필수, False=conf만(test용)
        self._warned_no_pick_classes = False

        model_path = self.get_parameter('model_path').value
        backend = self.get_parameter('backend').value
        self._w = self.get_parameter('width').value
        self._h = self.get_parameter('height').value
        self._fps = self.get_parameter('fps').value
        self._camera_mode = self.get_parameter('camera_mode').value

        # YOLO 모델 로드 (segmentation)
        from ultralytics import YOLO
        resolved = _resolve_model(model_path, backend, self._h, self._w)
        self.model = YOLO(resolved)
        self.get_logger().info(f'YOLO(seg) loaded: {resolved}')

        # 카메라 초기화 (realsense 모드면 DepthCal 생성)
        self._rs = None
        self._pipe = None
        self._depth_cal = None
        self._test_img = None
        if self._camera_mode == 'realsense':
            self._init_realsense()
        else:
            self._init_test_image()
            self.get_logger().warn(
                'camera_mode=test: depth 없음 → translation 0, orientation(PCA)만 채워짐')

        # 퍼블리셔
        self.pub_objects = self.create_publisher(
            DetectedObjectArray, '/detected_objects', 10)
        # /pick_target: transient_local(latched) — "도착→집어" 타이밍에 최신 타깃 유실 방지
        latched_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_pick = self.create_publisher(
            DetectedObject, '/pick_target', latched_qos)

        # 추론 루프 스레드
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    # ── 카메라 초기화 ──────────────────────────

    def _init_realsense(self):
        try:
            import pyrealsense2 as rs
            self._rs = rs
            pipe = rs.pipeline()
            cfg = rs.config()
            cfg.enable_stream(rs.stream.depth, self._w, self._h, rs.format.z16, self._fps)
            cfg.enable_stream(rs.stream.color, self._w, self._h, rs.format.bgr8, self._fps)
            profile = pipe.start(cfg)
            self._pipe = pipe
            # color↔depth 투영 파라미터 (RealSense 내부파라미터 직접 사용)
            self._depth_cal = DepthCal(rs, profile)
            self.get_logger().info(
                f'RealSense started ({self._w}x{self._h} @ {self._fps}fps), depth pose 활성')
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

            color_img, depth_frame, depth_img = self._grab_frame()
            if color_img is None:
                time.sleep(0.01)
                continue

            conf = self.get_parameter('conf_threshold').value
            cls_filter = self._get_cls_filter()
            frame_id = self.get_parameter('frame_id').value

            # ① YOLO segmentation 추론
            results = self.model.predict(
                color_img, conf=conf, classes=cls_filter, verbose=False)
            r0 = results[0]
            masks = None if r0.masks is None else r0.masks.data.cpu().numpy()

            array_msg = DetectedObjectArray()
            array_msg.header.stamp = self.get_clock().now().to_msg()
            array_msg.header.frame_id = frame_id

            for i, box in enumerate(r0.boxes):
                x1, y1, x2, y2 = (int(v) for v in box.xyxy[0])

                obj = DetectedObject()
                obj.class_id = int(box.cls[0])
                obj.class_name = self.model.names[obj.class_id]
                obj.confidence = float(box.conf[0])
                obj.pose = Pose()  # 기본값(position 0)
                obj.pose.orientation.w = 1.0  # 유효 단위 쿼터니언

                roi = RegionOfInterest()
                roi.x_offset = x1
                roi.y_offset = y1
                roi.width = x2 - x1
                roi.height = y2 - y1
                obj.bbox = roi

                # ② markerless pose: 마스크 기반 translation(depth) + orientation(2D PCA)
                binmask = self._get_binmask(masks, i)
                self._fill_markerless_pose(
                    obj, binmask, (x1, y1, x2, y2), depth_frame, depth_img)

                array_msg.objects.append(obj)

            self.pub_objects.publish(array_msg)

            # ③ 픽 대상 선별 → /pick_target (후보 없으면 publish 안 함 = latched 유지)
            pick = self._select_pick_target(array_msg.objects)
            if pick is not None:
                self.pub_pick.publish(pick)

            if idx % 30 == 0:
                if array_msg.objects:
                    summary = ', '.join(
                        f'{o.class_name}({o.confidence:.2f})'
                        f'{"[posed]" if o.pose.position.z != 0.0 else ""}'
                        for o in array_msg.objects
                    )
                    pick_str = (f' | pick={pick.class_name}({pick.confidence:.2f})'
                                if pick is not None else '')
                    self.get_logger().info(
                        f'[{idx}] {len(array_msg.objects)} objects: {summary}{pick_str}')
                else:
                    self.get_logger().info(f'[{idx}] (검출 없음)')
            idx += 1

            if self._camera_mode == 'test':
                elapsed = time.time() - t0
                if interval - elapsed > 0:
                    time.sleep(interval - elapsed)

    def _select_pick_target(self, objects):
        """픽 후보 중 confidence 최고 1개 선택.

        후보 조건: class_name ∈ pick_classes(화이트리스트) AND confidence ≥ pick_min_conf
        AND (require_depth=False 또는 depth pose 채워짐 z!=0). 후보 없으면 None.
        """
        pick_classes = self.get_parameter('pick_classes').value
        whitelist = {c.strip() for c in pick_classes.split(',') if c.strip()}
        if not whitelist:
            if not self._warned_no_pick_classes:
                self.get_logger().warn(
                    'pick_classes 미설정 → /pick_target 후보 없음. 집을 클래스를 지정하라.')
                self._warned_no_pick_classes = True
            return None

        min_conf = self.get_parameter('pick_min_conf').value
        require_depth = self.get_parameter('require_depth').value

        best = None
        for obj in objects:
            if obj.class_name not in whitelist:
                continue
            if obj.confidence < min_conf:
                continue
            if require_depth and obj.pose.position.z == 0.0:
                continue
            if best is None or obj.confidence > best.confidence:
                best = obj
        return best

    def _get_binmask(self, masks, i):
        """i번째 객체의 이진 마스크(원본 해상도). seg 모델 아니면 None."""
        if masks is None or i >= len(masks):
            return None
        m = masks[i]
        if m.shape != (self._h, self._w):
            m = cv2.resize(m, (self._w, self._h), interpolation=cv2.INTER_NEAREST)
        return m > 0.5

    def _fill_markerless_pose(self, obj, binmask, box, depth_frame, depth_img):
        """translation = 마스크 centroid depth median deproject, orientation = 2D PCA yaw."""
        x1, y1, x2, y2 = box

        # 마스크 픽셀 좌표 (없으면 bbox 중심으로 폴백)
        if binmask is not None and binmask.any():
            ys, xs = np.nonzero(binmask)
            cu, cv_ = float(xs.mean()), float(ys.mean())
            quat = _mask_pca_yaw_quat(xs, ys)
            if quat is not None:
                qx, qy, qz, qw = quat
                obj.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        else:
            cu, cv_ = (x1 + x2) / 2.0, (y1 + y2) / 2.0

        # translation: realsense 모드에서만 (depth 필요)
        if depth_frame is None or self._depth_cal is None:
            return
        r = max(4, min(x2 - x1, y2 - y1) // 6)
        xyz = _deproject_centroid(
            self._rs, depth_frame, depth_img, cu, cv_, r, self._depth_cal)
        if xyz is not None:
            obj.pose.position = Point(x=float(xyz[0]), y=float(xyz[1]), z=float(xyz[2]))

    def _grab_frame(self):
        """(color_img, depth_frame, depth_img) 반환. test 모드는 depth None."""
        if self._camera_mode == 'realsense':
            try:
                frames = _latest_frames(self._pipe)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame:
                    return None, None, None
                color_img = np.asanyarray(color_frame.get_data())
                depth_img = (np.asanyarray(depth_frame.get_data())
                             if depth_frame else None)
                return color_img, depth_frame, depth_img
            except Exception as e:
                self.get_logger().error(f'Frame grab error: {e}')
                return None, None, None
        else:
            return self._test_img.copy(), None, None

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
