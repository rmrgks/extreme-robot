"""카메라 static TF 발행 — 전방 RGB-D(RealSense D435i) + 손목 RGB.

perception_node 는 pyrealsense2 를 직접 쓰므로(realsense-ros 드라이버 미사용) 아무도
TF 를 내지 않는다. /pick_target 의 frame_id='camera_color_optical_frame' 을 MoveIt 이
base_link(planning frame)로 변환하려면 이 체인이 TF 트리에 있어야 한다 (Phase3 §6-E).

전방 RGB-D (차체 고정):
  base_link ──(CAD 오프셋)──▶ camera_link
  camera_link ──(REP-103 optical 회전, 고정)──▶ camera_color_optical_frame

손목 RGB (그리퍼 위, 팔에 장착):
  base_link ──(CAD 오프셋, 홈 포즈 기준)──▶ wrist_camera_link
  NOTE: 추후 URDF 관절로 통합 필요 — 현재는 홈 포즈 기준 static placeholder.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# REP-103 optical frame: body(X전방·Y좌·Z상) → optical(Z전방·X우·Y하)
OPTICAL_ROLL = -math.pi / 2.0
OPTICAL_PITCH = 0.0
OPTICAL_YAW = -math.pi / 2.0


def generate_launch_description():
    args = [
        # ── 전방 RGB-D 카메라 (차체 고정, CAD 실측값) ──
        DeclareLaunchArgument('cam_x',     default_value='0.123'),
        DeclareLaunchArgument('cam_y',     default_value='0.0'),
        DeclareLaunchArgument('cam_z',     default_value='0.082'),
        DeclareLaunchArgument('cam_roll',  default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='-0.26'),
        DeclareLaunchArgument('cam_yaw',   default_value='0.0'),

        # ── 손목 RGB 카메라 (그리퍼 위, CAD 실측값) ──
        DeclareLaunchArgument('wrist_cam_x',     default_value='0.040'),
        DeclareLaunchArgument('wrist_cam_y',     default_value='0.0'),
        DeclareLaunchArgument('wrist_cam_z',     default_value='0.295'),
        DeclareLaunchArgument('wrist_cam_roll',  default_value='0.0'),
        DeclareLaunchArgument('wrist_cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('wrist_cam_yaw',   default_value='0.0'),
    ]

    # ── 전방 RGB-D: base_link → camera_link ──
    front_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    # ── 전방 RGB-D: camera_link → camera_color_optical_frame (REP-103 고정) ──
    front_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_optical',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll',  str(OPTICAL_ROLL),
            '--pitch', str(OPTICAL_PITCH),
            '--yaw',   str(OPTICAL_YAW),
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_color_optical_frame',
        ],
    )

    # ── 손목 RGB: base_link → wrist_camera_link (홈 포즈 기준 static) ──
    wrist_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_wrist_camera_link',
        arguments=[
            '--x', LaunchConfiguration('wrist_cam_x'),
            '--y', LaunchConfiguration('wrist_cam_y'),
            '--z', LaunchConfiguration('wrist_cam_z'),
            '--roll',  LaunchConfiguration('wrist_cam_roll'),
            '--pitch', LaunchConfiguration('wrist_cam_pitch'),
            '--yaw',   LaunchConfiguration('wrist_cam_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'wrist_camera_link',
        ],
    )

    return LaunchDescription(args + [front_mount_tf, front_optical_tf, wrist_mount_tf])
