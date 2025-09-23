import os, sys
import yaml


if len(sys.argv)<2:
    print(f"Use: python {sys.argv[0]} <launch_config_file.yaml>")
    sys.exit(1)

configfile = sys.argv[1]

with open(configfile) as cf:
    try:
        config = yaml.safe_load(cf)
    except yaml.YAMLError as exc:
        print(exc)
        sys.exit(1)

print(config)

rconfig = config['robot']

world_file_arg = f"world_file:=empty.world"

rbase = rconfig['base']


def get_config(key):
    if key in rconfig.keys():
        return rconfig[key]
    else:
        return None

def get_arg(value,key):

    s = ""
    v = get_config(key)
    if v is not None:
        v = rconfig[key]
        s = f"{value}:={v}"

    return s 


robot_type_arg = get_arg("robot_type","base")
robot_name_arg = "robot_name:=robot1"
control_interface_arg = get_arg("control_interface", "arms_control_interface")
imu_arg = get_arg("imu","imu")
lidar_arg = get_arg("lidar","lidar")
rgb_arg = "rgb:=True" if get_config("camera")=="rgb" else ""
rgbd_arg = "rgbd:=True" if get_config("camera")=="rgbd" else ""

if rbase=='wheeled':

    arms_arg = f"arms:={rconfig['arms']}"
    pantilt_arg = f"pantilt:={rconfig['pantilt']}"

    launch_cmd = f"ros2 launch marr_gz marr_robot.launch.py {world_file_arg} {robot_type_arg} " + \
      f"{robot_name_arg} {control_interface_arg} {imu_arg} {lidar_arg} " + \
      f"{rgb_arg} {rgbd_arg} {arms_arg} {pantilt_arg}" 


elif rbase=='reacher':

    try:
        dof = int(rconfig['dof'])
    except KeyError:
        print("Warning!!! Use 'dof' field in config file to set arm dof (default to 2) !!!")
        dof = 2
    if dof<2 or dof>6:
        print(f"Reacher with {dof} dof not implemented.")
        sys.exit(1)
    dof_arg = f"dof:={dof}"

    launch_cmd = f"ros2 launch marr_gz marr_robot.launch.py {world_file_arg} {robot_type_arg} {dof_arg} " + \
      f"{robot_name_arg} {control_interface_arg} {imu_arg} {lidar_arg} " + \
      f"{rgb_arg} {rgbd_arg}" 

else:
    print(f"Unknown robot base {rbase}")
    sys.exit(1)

os.system(f"cd ~/ros2_ws && colcon build && {launch_cmd}")

