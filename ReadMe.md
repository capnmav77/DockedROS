# Welcome to DockedRobotics

Below are the different versions that can be used as base templates

### Ros Foxy Version Steps [Cuda support] :

#### Pre-installation Steps:

1. **Nvidia Container Toolkit Setup:**
   - Set up the package repository and GPG key.
     ```bash
     set distribution $(. /etc/os-release;echo $ID$VERSION_ID) \
            && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
            && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
                  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
                  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
     ```
   - Install the toolkit.
     ```bash
     sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
     ```
   - Configure the runtime.
     ```bash
     sudo nvidia-ctk runtime configure --runtime=docker
     ```

2. **Install Xhost for GUI:**
   ```bash
   xhost +local:docker
   ```

#### Containerizing:

1. **Build the Dockerfile:**
   ```bash
   docker build -t my_ros_container .
   ```

2. **Run the Docker Container:**
   ```bash
   docker run --gpus all -it --net=host \
       --env NVIDIA_DRIVER_CAPABILITIES=all \
       --env="DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       --volume="/path/to/host/directory:/home/gazebo_ros" \
       my_ros_container
   ```

### Ros Humble Version Steps [No CUDA Support]:

#### Pre-installation:

1. **Install Xhost for GUI:**
   ```bash
   xhost +local:docker
   ```

#### Containerizing:

1. **Build the Dockerfile:**
   ```bash
   docker build -t my_ros_container .
   ```

2. **Run the Docker Container:**
   ```bash
   docker run -it --net=host --privileged \
       --env="DISPLAY" \
       --env="QT_X11_NO_MITSHM=1" \
       --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
       -v "/home/neo/Documents/FREDBOTS2.0/Bot_Volume/:/persistent_data/" \
       ros2-humble-gazebo
   ```

#### Inside the Container:

- The container contains an Ubuntu image with ROS2 and Gazebo installed.
- Running scripts and performing required tasks should be done in the `/persistent_data` folder, as it is directly attached to the user's directory.
- Alternatively, you can create a Docker volume to attach it to.
