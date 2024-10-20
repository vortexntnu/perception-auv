FROM ros:humble
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /root/ros2_ws/src
COPY . /root/ros2_ws/src/
WORKDIR /root/ros2_ws

# Remove any existing build or install directories to clean up old builds
RUN rm -rf build install log

# Update and install necessary packages including dependencies
RUN apt-get update && apt-get install -y \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install ROS dependencies using rosdep
RUN apt-get update && rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y

RUN pip3 install -U setuptools==58.5.3

# Source ROS setup files before building and ensure vortex-asv is built
RUN source /opt/ros/humble/setup.bash && \
    colcon build --packages-select perception_setup

# Build the rest of the project
RUN source /opt/ros/humble/setup.bash && \
    colcon build

RUN sudo apt install -y ros-humble-joy

RUN echo "source /root/ros2_ws/install/setup.bash" >> ~/.bashrc

CMD . ~/.bashrc && source /opt/ros/humble/setup.bash && . /root/ros2_ws/install/setup.bash && ros2 launch perception_setup perception.launch.py

# Instead of asv_setup, use perception_setup OG perception.launch.py
