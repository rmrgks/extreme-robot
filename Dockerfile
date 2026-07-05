FROM ros:humble-ros-base

# 한글 로케일 설정
RUN apt-get update && apt-get install -y locales \
    && locale-gen ko_KR.UTF-8 \
    && update-locale LANG=ko_KR.UTF-8

ENV LANG=ko_KR.UTF-8

# 필수 패키지 설치
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-turtlesim \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-joint-state-publisher-gui \
    ros-humble-dynamixel-sdk \
    ros-humble-dynamixel-workbench \
    python3-serial \
    python3-pip \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install "numpy<2" ultralytics pyrealsense2 && pip3 uninstall -y opencv-python opencv-python-headless

# MoveIt (로봇팔 경로계획) + ros2_control (mock 하드웨어/컨트롤러)
# - ros-humble-moveit: move_group, OMPL, KDL IK, RViz MotionPlanning 플러그인
# - ros-humble-ros2-control: controller_manager, mock_components/GenericSystem
# - ros-humble-ros2-controllers: joint_trajectory_controller, joint_state_broadcaster
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-configs-utils \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 환경 자동 소싱
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws

CMD ["bash"]
