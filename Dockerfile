# Use the official Ubuntu 22.04 LTS (Jammy) image as the base
FROM ubuntu:22.04

# Set the locale
ENV LANG=C.UTF-8 LC_ALL=C.UTF-8

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y tzdata && \
    rm -rf /var/lib/apt/lists/*

# Install required packages
RUN apt-get update && apt-get install -y \
    software-properties-common \
    lsb-release \
    sudo \
    gnupg2 \
    curl \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

# Set up sources for ROS 2 Humble 
RUN apt-get update && apt-get install -y \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && export LANG=en_US.UTF-8 \
    && apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble and Gazebo - will install the dependencies so chill for half an hour
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*


# Source the ROS 2 setup script 
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create a workspace directory
RUN mkdir -p /ros2_ws/src

# Set the working directory
WORKDIR /home

# installing OMPL
    # Copy the install script
    COPY gazebo_ros/OMPL/install-ompl-ubuntu.sh /
    # Give elevated permissions
    RUN chmod +x /install-ompl-ubuntu.sh
    # Run the install script
    RUN /install-ompl-ubuntu.sh


# Copy the entrypoint script
COPY entrypoint.sh /

# Make the entrypoint script executable
RUN chmod +x /entrypoint.sh

# Set the entrypoint script as the command to run
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]