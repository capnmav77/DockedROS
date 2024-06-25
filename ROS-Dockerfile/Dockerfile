# Use the official ROS 1 Docker image as the base image
FROM ros:melodic

# Set the locale
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Install additional packages if needed
# For example:
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# (Optional) Set up your workspace or additional configurations here
# For example, create a catkin workspace:
ENV CATKIN_WS /root/catkin_ws
RUN mkdir -p $CATKIN_WS/src

# (Optional) Clone your ROS packages into the workspace
# RUN git clone <your_ros_package_repository> $CATKIN_WS/src

# (Optional) Build your workspace
WORKDIR $CATKIN_WS
RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; \
                  catkin_init_workspace src"

RUN /bin/bash -c "source /opt/ros/melodic/setup.bash; \
                  catkin_make"

# Source the setup script for ROS environment setup
RUN echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
RUN echo "source $CATKIN_WS/devel/setup.bash" >> ~/.bashrc

# Start a bash session by default
CMD ["bash"]
