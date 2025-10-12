
# ============================================================================
# Isaac ROS base (Jetson / aarch64)
# Find latest image tags here:
#   https://catalog.ngc.nvidia.com/orgs/nvidia/teams/isaac/containers/ros
# ============================================================================
    ARG BASE_IMAGE=nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_adc428c7077de4984a00b63c55903b0a
    FROM ${BASE_IMAGE}
    ARG BASE_IMAGE=ros:humble

    USER root
    SHELL ["/bin/bash", "-c"]
    ARG DEBIAN_FRONTEND=noninteractive

    # ------------------------------------------------------------------------------
    # 1) Create build-time args for user/group, so the container user matches your host
    # ------------------------------------------------------------------------------
    ARG USER_ID=1000
    ARG GROUP_ID=1000
    ARG USERNAME=vortex

    # ------------------------------------------------------------------------------
    # Set up workspace paths
    # ------------------------------------------------------------------------------
    ENV WORKSPACE=/ros_ws
    WORKDIR $WORKSPACE

    # ------------------------------------------------------------------------------
    # Update apt and install required dependencies.
    # ------------------------------------------------------------------------------
    ARG ROS_DISTRO
    ENV ROS_DISTRO=${ROS_DISTRO}

    RUN apt-get update && \
        apt-get install -y \
            git \
            python3-rosdep \
            python3-vcstool \
            ros-${ROS_DISTRO}-ros-core \
            python3-pip

    # ------------------------------------------------------------------------------
    # Copy the entire workspace into the Docker image.
    # ------------------------------------------------------------------------------
    COPY . .


    # ------------------------------------------------------------------------------
    # Run rosdep to install missing dependencies.
    # ------------------------------------------------------------------------------
    RUN apt-get update
    RUN rosdep update && rosdep install --from-paths src/ --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

    # ------------------------------------------------------------------------------
    # 7) Create a non-root user matching the host UID/GID and allow sudo
    # ------------------------------------------------------------------------------
    RUN if ! getent group ${GROUP_ID} >/dev/null; then \
            groupadd --gid ${GROUP_ID} ${USERNAME}; \
        fi && \
        if ! id -u ${USER_ID} >/dev/null 2>&1; then \
            useradd --uid ${USER_ID} --gid ${GROUP_ID} -m -s /bin/bash ${USERNAME}; \
        fi && \
        apt-get update && apt-get install -y sudo && \
        echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers && \
        apt-get clean && rm -rf /var/lib/apt/lists/*
        
    # ------------------------------------------------------------------------------
    # 8) Switch to non-root user (fixes the "I have no name!" prompt)
    # ------------------------------------------------------------------------------
    USER ${USERNAME}
    WORKDIR /home/${USERNAME}

    # ------------------------------------------------------------------------------
    # Default command
    # ------------------------------------------------------------------------------
    CMD ["bash"]