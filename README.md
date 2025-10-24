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

## Standard procedure

Pull the system image and build the user image

    cd docker
    ./build.bash
    

# Building from scratch

In case of problems with building the image (due to updates in the apt keys, libraries, etc.),
update the docker images and build from scratch (it will take time and bandwidth!)

    cd docker
    ./build_from_scratch.bash


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
      base: wheeled|reacher
      dof: 2..6            # dof of reacher base
      arms: false|true     # side arms for wheeled base
      pantilt: false|true  # pan-tilt for wheeled base (camera will be mounted here)
      imu: false|true      # for both wheeled and reacher
      lidar: false|true    # for both wheeled and reacher
      camera: false|rgb|rgbd  # for both wheeled and reacher
      arms_control_interface: effort|velocity|position   # reacher and wheeled arms


Example:

    robot: 
      base: reacher
      dof: 3
      imu: false
      lidar: false
      camera: rgb
      arms_control_interface: position


Note: you can create many config launch files for different configurations.

Launch the simulator and ROS nodes

    cd marr_gz/scripts
    python marr_launch.py ../config/my_launch.yaml


## Option 2. Using ros2 launch file

    cd ros2_ws
    colcon build
    ros2 launch marr_gz marr_robot.launch.py <config_paraneters>

The list of configurable parameters is given by

    ros2 launch marr_gz marr_robot.launch.py -s

Examples:

    ros2 launch marr_gz marr_robot.launch.py robot_type:=wheeled

    ros2 launch marr_gz marr_robot.launch.py robot_type:=wheeled arms:=True control_interface:=velocity

    ros2 launch marr_gz marr_robot.launch.py robot_type:=reacher dof:=3 control_interface:=velocity





# Models available

## two-wheeled base

- wheeled: 2-wheeled robot

## reacher arm

- reacher:  2 to 6 dof arm

## actuators

- pan-tilt: for the wheeled base (cameras will be mounted here)
- amrs: two 1 dof arms for wheeled robot

## sensors

- IMU
- lidar
- RGB camera
- RGBD camera

On the wheeled base, IMU and lidar are placed on the base, while cameras on the pan-tilt unit, if present, otherwise on a fixed support.

On the reacher base, sensors are placed on the end-effector.

# Controllers

Available controllers:

- differential drive for wheeled robot
- effort|veocity|position controllers for arms

Check running controllers

    ros2 control list_controllers



## Test controllers

Launch a robot, as described above.

Test the controllers of the robot model.
Run the test control (on another window terminal - use `Ctrl-b c` in tmux to create a new window)

    cd marr_gz/script
    python test_control.py

You should see all the joints moving.


## Manual tests

wheeled robot

    ros2 launch marr_gz marr_robot.launch.py robot_type:=wheeled

test

    cd marr_gz/script
    python test_control.py -fn ddrive 


wheeled robot with arms

position control interface

    ros2 launch marr_gz marr_robot.launch.py robot_type:=wheeled arms:=True control_interface:=position

test

    cd marr_gz/script
    python test_control.py -fn arm_position 


reacher robot position control interface

    ros2 launch marr_gz marr_robot.launch.py robot_type:=reacher dof:=2 control_interface:=position

test

    cd marr_gz/script
    python test_control.py -fn arm_position 


reacher robot velocity control interface

    ros2 launch marr_gz marr_robot.launch.py robot_type:=reacher dof:=2 control_interface:=velocity

test

    cd marr_gz/script
    python test_control.py -fn arm_velocity


reacher robot effort control interface

    ros2 launch marr_gz marr_robot.launch.py robot_type:=reacher dof:=2 control_interface:=effort

test

    cd marr_gz/script
    python test_control.py -fn arm_effort
   

# Sensors

Use Rviz to see sensor data

    ros2 run rviz2 rviz2
    
Predefined rviz files are available in the `config` folder, e.g.

    cd marr_gz/config
    ros2 run rviz2 rviz2 -d wheeled.rviz


