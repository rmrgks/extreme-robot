"""뎁스 카메라(RealSense D435i) static TF 발행.

perception_node 는 pyrealsense2 를 직접 쓰므로(realsense-ros 드라이버 미사용) 아무도
TF 를 내지 않는다. /pick_target 의 frame_id='camera_color_optical_frame' 을 MoveIt 이
base_link(planning frame)로 변환하려면 이 체인이 TF 트리에 있어야 한다 (Phase3 §6-E).

체인 (뎁스 카메라 = 베이스 고정):
  base_link ──(장착 오프셋, placeholder·캘리브 과제)──▶ camera_link
  camera_link ──(REP-103 optical 회전, 고정)──▶ camera_color_optical_frame

장착 오프셋은 launch arg(cam_x/y/z, cam_roll/pitch/yaw)로 노출. 실측 전까지 0 placeholder.
  예) ros2 launch robot_arm_description camera_tf.launch.py cam_x:=0.1 cam_z:=0.3 cam_pitch:=0.5

NOTE: 2번 카메라(RGB, eye-in-hand)는 팔 링크에 장착 → URDF 관절로 통합해야 함(후속 과제).
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# REP-103 optical frame: body(X전방·Y좌·Z상) → optical(Z전방·X우·Y하)
# 고정 회전 rpy = (-pi/2, 0, -pi/2)
OPTICAL_ROLL = -math.pi / 2.0
OPTICAL_PITCH = 0.0
OPTICAL_YAW = -math.pi / 2.0


def generate_launch_description():
    args = [
        DeclareLaunchArgument('camera_link', default_value='camera_link'),
        DeclareLaunchArgument('optical_frame', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('parent_frame', default_value='base_link'),
        # base_link → camera_link 장착 오프셋 (placeholder=0, 실측 과제)
        DeclareLaunchArgument('cam_x', default_value='0.0'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.0'),
        DeclareLaunchArgument('cam_roll', default_value='0.0'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0'),
        DeclareLaunchArgument('cam_yaw', default_value='0.0'),
    ]

    # 1단: base_link → camera_link (물리 장착, 캘리브)
    mount_tf = Node(
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
            '--frame-id', LaunchConfiguration('parent_frame'),
            '--child-frame-id', LaunchConfiguration('camera_link'),
        ],
    )

    # 2단: camera_link → camera_color_optical_frame (REP-103 optical, 고정)
    optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_optical',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll', str(OPTICAL_ROLL),
            '--pitch', str(OPTICAL_PITCH),
            '--yaw', str(OPTICAL_YAW),
            '--frame-id', LaunchConfiguration('camera_link'),
            '--child-frame-id', LaunchConfiguration('optical_frame'),
        ],
    )

    return LaunchDescription(args + [mount_tf, optical_tf])
