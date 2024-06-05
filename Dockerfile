FROM nvidia/cuda:11.3.1-base-ubuntu20.04

WORKDIR /home

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y tzdata && \
    rm -rf /var/lib/apt/lists/*

# --- PYTHON3 INSTALLATION ---

# Install Python 3.8

RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt-get update && apt-get install -y python3.8

RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.8 1

# Install pytorch

RUN apt-get install -y python3-pip

RUN pip3 install \
    numpy==1.24.3 \
    matplotlib==3.7.1 \
    pandas==2.0.2 \
    pyqtgraph==0.12.4 \
    PyQt5==5.14.1 \
    torch==1.10.0+cu113 -f https://download.pytorch.org/whl/cu113/torch_stable.html

# --- GAZEBO INSTALLATION ---

RUN apt-get update && apt-get install -y wget

# install Gazebo
RUN curl -sSL http://get.gazebosim.org | sh

# Download basic gazebo models manually instead of complete (slow) download
ENV GAZEBO_MODEL_DATABASE_URI ""
RUN mkdir -p /root/.gazebo/models
WORKDIR /root/.gazebo/models
RUN wget https://raw.githubusercontent.com/osrf/gazebo_models/master/ground_plane/model.sdf -P ./ground_plane
RUN wget https://raw.githubusercontent.com/osrf/gazebo_models/master/ground_plane/model.config -P ./ground_plane
RUN wget https://raw.githubusercontent.com/osrf/gazebo_models/master/sun/model.sdf -P ./sun
RUN wget https://raw.githubusercontent.com/osrf/gazebo_models/master/sun/model.config -P ./sun

# --- ROS2 FOXY INSTALLATION ---

# Set locale
RUN apt-get update && apt-get install -y locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8

# Add ROS 2 apt repository
RUN apt-get update && apt-get install -y software-properties-common curl
RUN add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 packages
RUN apt-get update && apt-get install -y ros-foxy-ros-base python3-argcomplete

RUN apt-get update && apt-get install -y python3-rosdep2 python3-tk

RUN apt-get install -y ros-foxy-gazebo-ros-pkgs

# --- INITIALIZE APPLICATION ---

WORKDIR /home/gazebo_ros

# RUN apt-get install -y nano tmux

# --- OMPL installation ---
RUN apt-get install -y libompl-dev ompl-demos 



# Set up ~/.bashrc file
RUN echo \
    "\n"\
    "source /opt/ros/foxy/setup.bash\n"\
    "# ROS2 domain id for network communication, machines with the same ID will receive each others' messages\n"\
    "export ROS_DOMAIN_ID=1\n"\
    "# Source the workspace\n"\
    "source /opt/ros/foxy/setup.bash\n"\
    "# Allow gazebo to find our models\n"\
    "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/.gazebo/models\n"\
    >> ~/.bashrc

CMD ["bash"]
