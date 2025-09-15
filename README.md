# marr_gz

Ubuntu Noble Numbat (LTS 24.04)
https://releases.ubuntu.com/noble/

ROS2 Jazzy (LTS 2024-2029)
https://docs.ros.org/en/jazzy/

Gazebo Harmonic (LTS 2023-2028) 
https://gazebosim.org/docs/harmonic/



# Install

* docker engine

https://docs.docker.com/engine/

Install docker engine (not docker Desktop!!!)  (tested on v. 19.03, 20.10) 

Usually, this should work on Ubuntu distributions
    
        sudo apt install docker.io

or install from binaries

        https://docs.docker.com/engine/install/binaries/

See also 
[Post-installation steps for Linux](https://docs.docker.com/install/linux/linux-postinstall/).
In particular, add your user to the `docker` group and log out and in again, before proceeding.

        sudo usermod -aG docker $USER
        
Install docker compose plugin  (tested on v2.37)

https://docs.docker.com/compose/install/linux/

    DOCKER_CONFIG=${DOCKER_CONFIG:-$HOME/.docker}
    mkdir -p $DOCKER_CONFIG/cli-plugins
    curl -SL https://github.com/docker/compose/releases/download/v2.37.0/docker-compose-linux-x86_64 -o $DOCKER_CONFIG/cli-plugins/docker-compose
    chmod +x $DOCKER_CONFIG/cli-plugins/docker-compose


* For Nvidia drivers, install nvidia-docker2

[Nvidia docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
        

# Build

## Option 1: build images

    cd docker
    ./build.bash
    
In case of problems with building the image (possibly due to updates in the apt keys),
update the docker images and build from scratch (it will take time and bandwidth!)

    docker pull ros:jazzy-ros-base-noble
    ./build.bash no-cache

# Option 2: pull images from dockerhub

    cd docker
    ./pull.bash


# Run

## Option 1: local graphic card

This option autodetects if nvidia drivers are present and uses nvidia runtime in docker

    cd docker
    ./run.bash

## Option 2: vnc 

    cd docker
    ./run.bash vnc

Use a browser on `http://localhost:3000` to see the simulation.



# Launch

Run from the container, i.e., from tmux windows opened by the `run` script.

## Option 1. Using robot configuration

Copy `marr_gz/config/launch_config_template.yaml` in a new config file (e.g. `my_launch.yaml`)

Edit `my_launch.yaml` to configure your simulation.

Robot configuration parameters:

    robot: 
      base: [wheeled|reacher]
      dof: [2..6]            # dof of reacher base
      arms: [false|true]     # side arms for wheeled base
      pantilt: [false|true]  # pan-tilt for wheeled base (camera will be mounted here)
      imu: [false|true]      # for both wheeled and reacher
      lidar: [false|true]    # for both wheeled and reacher
      camera: [false|rgb|rgbd]  # for both wheeled and reacher
      arms_control_interface: [effort|velocity|control]   # reacher and wheeled arms


Note: you can create many config launch files for different configurations.

Launch the simulator and ROS nodes

    cd /opt/marr_gz/scripts
    python marr_launch.py ../config/my_launch.yaml


## Option 2. Using ros2 launch file

    cd ros2_ws
    colcon build
    ros2 launch marr_gz marr_robot.launch.py <config_paraneters>

The list of configurable parameters is given by

    ros2 launch marr_gz marr_robot.launch.py -s

Example:

    ros2 launch marr_gz marr_robot.launch.py robot_type:=reacher dof:=3 control_interface:=velocity





# Models available

## two-wheeled base

- wheeled: 2-wheeled robot

## reacher arm

- reacher:  2 to 6 dof arm

## actuators

- pan-tilt: for the wheeled base (cameras will be mounted here)

## sensors

- IMU
- lidar
- RGB camera
- RGBD camera

On the wheeled base, IMU and lidar are placed on the base, while cameras on the pan-tilt unit, if present, otherwise on the base as well.

On the reacher base, sensors are mounted on the end-effector.


# Controllers

Available controllers:

- diffdrive for wheels
- effort|veocity|position for arms

Check running controllers

    ros2 control list_controllers



# Test controllers

## wheeled robot

Launch a wheeled robot (it will automatically launch a diffdrive controller)

    ros2 launch marr_gz marr_robot.launch.py robot_type:=wheeled

Run the test control (on another window terminal - use `Ctrl-b c` in tmux to create a new window)
    cd /opt/marr_gz/script
    python test_control.py wheeled_diffdrive 

## reacher robot

Launch a reacher robot with any control interface and run the test

1. position control interface

    ros2 launch marr_gz marr_robot.launch.py robot_type:=reacher dof:=2 control_interface:=position

    cd /opt/marr_gz/script
    python test_control.py arm_position 


2. velocity control interface

    ros2 launch marr_gz marr_robot.launch.py robot_type:=reacher dof:=2 control_interface:=velocity

    cd /opt/marr_gz/script
    python test_control.py arm_velocity


3. effort control interface

    ros2 launch marr_gz marr_robot.launch.py robot_type:=reacher dof:=2 control_interface:=effort

    cd /opt/marr_gz/script
    python test_control.py arm_effort
   

