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
    openssh-server \    
    python3 \
    python3-pip \
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


# Install Colcon dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-apt \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Install Colcon using pip
RUN python3 -m pip install -U colcon-common-extensions

# Source the ROS 2 setup script 
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Create a workspace directory
RUN mkdir -p /ros2_ws/src

# installing OMPL
# Copy the install script
COPY gazebo_ros/OMPL/install-ompl-ubuntu.sh /
# Give elevated permissions
RUN chmod +x /install-ompl-ubuntu.sh
# Run the install script
RUN /install-ompl-ubuntu.sh

# Set the volume for persistence
VOLUME /ros2_ws/src

# Copy the entrypoint script
COPY entrypoint.sh /

# Make the entrypoint script executable
RUN chmod +x /entrypoint.sh

#exposing the port
RUN mkdir /var/run/sshd \
    && echo 'root:password' | chpasswd \
    && sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config \
    && sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/' /etc/ssh/sshd_config \
    && ssh-keygen -A

# Expose SSH port
EXPOSE 22

# Set the entrypoint script as the command to run
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
