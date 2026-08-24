FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Europe/Brussels
ENV LANG=en_US.UTF-8

# Base utilities + locale
RUN apt-get update && apt-get install -y --no-install-recommends \
    tzdata locales \
    ca-certificates curl gnupg2 lsb-release \
    sudo udev \
    python3 python3-pip \
  && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8 && update-locale LANG=en_US.UTF-8

# Python dependencies
RUN pip3 install --no-cache-dir \
    numpy \
    pyserial \
    crc \
    matplotlib \
    scipy \
    esptool

# ROS 2 apt repo
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
      http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list

# ROS base + build tools + GUI/display libs for matplotlib TkAgg + RViz2
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rviz2 \
    ros-humble-rviz-default-plugins \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    python3-tk \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxft2 \
    libfreetype6 \
    fontconfig \
  && rm -rf /var/lib/apt/lists/*

# Create a non-root user
ARG USERNAME=docker
ARG USER_UID=1000
ARG USER_GID=1000

# Create the user and add to sudoers
RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -aG sudo,dialout ${USERNAME} && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

# rosdep init (system-wide)
RUN rosdep init || true

USER ${USERNAME}
WORKDIR /tutorial_3dsensors_ws

# Pre-configure matplotlib to use TkAgg backend
RUN mkdir -p /home/${USERNAME}/.config/matplotlib && \
    echo "backend: TkAgg" > /home/${USERNAME}/.config/matplotlib/matplotlibrc

# Auto-source ROS
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc

# Auto-source the tactile robot workspace
RUN echo "source /Hands-on-tutorial-on-3D-tactile-sensor-integration-in-robotic-fingers/Codes/ros2_ws/install/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "export AMENT_PREFIX_PATH=/Hands-on-tutorial-on-3D-tactile-sensor-integration-in-robotic-fingers/Codes/ros2_ws/install/ros2_tactile_robot:\$AMENT_PREFIX_PATH" >> /home/${USERNAME}/.bashrc

CMD ["/bin/bash"]
