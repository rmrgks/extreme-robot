#!/bin/bash
ros2 run robot_arm_perception perception_node --ros-args -p model_path:=/root/ros2_ws/yolov8n-seg.pt -p camera_mode:=realsense -p pick_classes:=bottle -p pick_min_conf:=0.5 -p require_depth:=true
