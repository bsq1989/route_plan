ARG ROS_BASE=ros:jazzy-ros-base
FROM ${ROS_BASE}

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       ros-jazzy-diff-drive-controller \
       ros-jazzy-joint-state-broadcaster \
       ros-jazzy-mujoco-ros2-control \
       ros-jazzy-robot-state-publisher \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
       ros-jazzy-joint-state-topic-hardware-interface \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/mujoco-dev
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "/workspace/mujoco-dev/ros2/tracer.launch.py"]
