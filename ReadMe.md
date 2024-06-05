# ROS Humble and Gazebo 11 [dockerized]

## Old version steps:
## Pre installation steps: 

1. Nvidia container Toolkit : 
first setup the package repo and gpg key: 
```
set distribution $(. /etc/os-release;echo $ID$VERSION_ID) \
            && curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
            && curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
                  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
                  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```
install the toolkit : 
```
sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
```

configure the runtime : 
```
sudo nvidia-ctk runtime configure --runtime=docker
```

install xhost for GUI
```
xhost +local:docker
```

## Containerizing >

1. build the dockerfile: 
```
docker build -t my_ros_container .
```

2. Run the docker container : 
```
docker run --gpus all -it --rm --net=host \
    --env NVIDIA_DRIVER_CAPABILITIES=all \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/path/to/host/directory:/home/gazebo_ros" \
    my_ros_container

```


## New Version Steps [ no cuda support :alert: ]
### preinstallation
skip the nvidia-toolkit installation
1. install xhost for GUI
```
xhost +local:docker
```

## Containerizing > 

1. build the dockerfile: 
```
docker build -t my_ros_container .
```

2. Run the docker container : 
```
docker run -it --rm --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="/path/to/host/directory:/home/gazebo_ros" \
    ros2-humble-gazebo
```


## inside the container:
1. installing ompl: 
- get the volume of gazebo_ros into the container while building by using the command : 
```
docker run -it --rm --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume=".:/home/gazebo_ros" \
    ros2-humble-gazebo
```

    NOTE : the above makes the current directory as a volume of your docker container 

- navigate into the directory with install-ompl-ubuntu.sh script and run the command : 
```
chmod +x *
./install-ompl-ubuntu.sh --python
```

