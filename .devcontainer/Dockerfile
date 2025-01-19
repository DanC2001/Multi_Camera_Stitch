ARG ROS_DISTRO=humble
#----------------------------------------------------------------------------------------------------
# Dockerfile for ROS 2 Humble + OpenCV + Python + X11 forwarding + dev user
#----------------------------------------------------------------------------------------------------
FROM osrf/ros:${ROS_DISTRO}-desktop-full

# Avoid interactive prompts
ARG DEBIAN_FRONTEND=noninteractive

# Build arguments for creating a non-root user
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=1000

# Install core dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    build-essential \
    python3-pip \
    python3-opencv \
    libgtk-3-dev \
    libglu-dev \
    x11-apps \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir colcon-common-extensions

# Create a non-root user with sudo privileges
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Source ROS 2 setup by default in new shells
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc

# Optional: you can switch directly to the new user here
# but with Dev Containers, you can also do it via devcontainer.json.
# USER $USERNAME

# Create a default workspace directory (owned by dev user)
WORKDIR /workspaces/ros2_ws

CMD ["bash"]
