FROM osrf/ros:humble-desktop-full

# 한글 로케일 설정
RUN apt-get update && apt-get install -y locales \
    && locale-gen ko_KR.UTF-8 \
    && update-locale LANG=ko_KR.UTF-8

ENV LANG=ko_KR.UTF-8

# 필수 패키지 설치
RUN apt-get update && apt-get install -y \
    ros-humble-turtlesim \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-joint-state-publisher-gui \
    python3-pip \
    nano \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 환경 자동 소싱
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws

CMD ["bash"]
