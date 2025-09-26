from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.actions import RegisterEventHandler, SetEnvironmentVariable, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution, PythonExpression
from launch.conditions import IfCondition

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    gz_args = LaunchConfiguration('gz_args', default='[]')
    world_file = LaunchConfiguration('world_file')   
    robot_type = LaunchConfiguration('robot_type')
    dof = LaunchConfiguration('dof')
    robot_name = LaunchConfiguration('robot_name')
    control_interface = LaunchConfiguration('control_interface')
    imu = LaunchConfiguration('imu')
    lidar = LaunchConfiguration('lidar')
    rgb = LaunchConfiguration('rgb')
    rgbd = LaunchConfiguration('rgbd')
    arms = LaunchConfiguration('arms')
    pantilt = LaunchConfiguration('pantilt')

    pkg_share = FindPackageShare('marr_gz')


    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='If true, use simulated clock.'
    )

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value=TextSubstitution(text='empty.world'),
        description='Gazebo world to launch.'
    )

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value=TextSubstitution(text='wheeled'),
        description='Type of the robot to spawn [wheeled|reacher].'
    )

    dof_arg = DeclareLaunchArgument(
        'dof',
        default_value=TextSubstitution(text='2'),
        description='DOF of reacher arm [2...6]'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value=TextSubstitution(text='robot1'),
        description='Name (i.e., prefix) of the robot to spawn.'
    )

    control_interface_arg = DeclareLaunchArgument(
        'control_interface',
        default_value=TextSubstitution(text='position'),
        description='control interface [effort|velocity|position]'
    )

    imu_arg = DeclareLaunchArgument(
        'imu',
        default_value=TextSubstitution(text='False'),
        description='Enable IMU sensor'
    )

    lidar_arg = DeclareLaunchArgument(
        'lidar',
        default_value=TextSubstitution(text='False'),
        description='Enable lidar sensor'
    )

    rgb_arg = DeclareLaunchArgument(
        'rgb',
        default_value=TextSubstitution(text='False'),
        description='Enable RGB camera'
    )

    rgbd_arg = DeclareLaunchArgument(
        'rgbd',
        default_value=TextSubstitution(text='False'),
        description='Enable RGBD camera'
    )

    arms_arg = DeclareLaunchArgument(
        'arms',
        default_value=TextSubstitution(text='False'),
        description='Enable robot arms for wheeled robot'
    )

    pantilt_arg = DeclareLaunchArgument(
        'pantilt',
        default_value=TextSubstitution(text='False'),
        description='Enable pan-tilt device for wheeled robot (to mount cameras on)'
    )


    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [ pkg_share, 'urdf', 
                    PythonExpression( ["'", robot_type, ".urdf.xacro'" ]) ]
            ),
            ' ',
            PythonExpression( ["'use_imu:=", imu, "'" ]),
            ' ',
            PythonExpression( ["'use_lidar:=", lidar, "'" ]),
            ' ',
            PythonExpression( ["'use_rgb:=", rgb, "'" ]),
            ' ',
            PythonExpression( ["'use_rgbd:=", rgbd, "'" ]),
            ' ',
            PythonExpression( ["'use_arms:=", arms, "'" ]),
            ' ',
            PythonExpression( ["'use_pantilt:=", pantilt, "'" ]),
            ' ',
            PythonExpression( ["'dof:=", dof, "'" ]),
        ]
    )

    robot_description = {'robot_description': robot_description_content}
    
    has_wheels = PythonExpression(["'", robot_type, "' == 'wheeled'" ])

    has_arms = PythonExpression([arms, " or ", "'", robot_type, "' == 'reacher'" ])

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', robot_type, '-allow_renaming', 'true',
                   '-z', '0.25', '-Y', '0.00' ],
    )

    detach_cmd = ExecuteProcess(
        cmd=[ "gz topic -t '/B1/detach' -m gz.msgs.Empty -p 'unused: true'" ],
        shell=True,
        condition=IfCondition(has_arms),
    )

    robot_controllers_file = PythonExpression(["'", robot_type, "' + ('", dof, "' if '", robot_type, "' == 'reacher' else '')", " + '_controllers.yaml'" ])

    robot_controllers = PathJoinSubstitution(
        [ pkg_share, 'config', robot_controllers_file ]
    )

    pantilt_controllers = PathJoinSubstitution(
        [ pkg_share, 'config', 'pantilt_controllers.yaml' ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    ddrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'ddrive_controller',
            '--param-file',
            robot_controllers,
            ],
        condition=IfCondition(has_wheels),
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            PythonExpression( ["'arm_", control_interface, "_controller'" ] ),
            '--param-file',
            robot_controllers,
            ],
        condition=IfCondition(has_arms),
    )

    pan_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            PythonExpression( ["'pan_", control_interface, "_controller'" ] ),
            '--param-file',
            pantilt_controllers,
            ],
        condition=IfCondition(pantilt),
    )

    tilt_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            PythonExpression( ["'tilt_", control_interface, "_controller'" ] ),
            '--param-file',
            pantilt_controllers,
            ],
        condition=IfCondition(pantilt),
    )

    # Check ga service info
    #   gz service -s <service> -i
    # Chech ROS2 service interface
    #   ros2 interface show ros_gz_interfaces/srv/SpawnEntity

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/world/default/create@ros_gz_interfaces/srv/SpawnEntity@gz.msgs.EntityFactory@gz.msgs.Boolean',
                   '/world/default/remove@ros_gz_interfaces/srv/DeleteEntity@gz.msgs.Entity@gz.msgs.Boolean',
            PythonExpression( ["'/model/", robot_type, "/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'"] ), # ground truth
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/scan/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth_image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera/depth_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            ],
        output='screen'
    )



    '''
    SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', [ EnvironmentVariable('GZ_SIM_RESOURCE_PATH'),
        PathJoinSubstitution([pkg_share, 'models']) ]
    ),
    SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', [ EnvironmentVariable('GZ_SIM_RESOURCE_PATH'),
        PathJoinSubstitution([pkg_share, 'worlds']) ]
    ),
    '''

    return LaunchDescription([
        # Launch Arguments
        use_sim_time_arg,
        world_file_arg,
        robot_type_arg, dof_arg,
        robot_name_arg,
        control_interface_arg,
        imu_arg, lidar_arg, rgb_arg, rgbd_arg,
        arms_arg, pantilt_arg,

        # Launch gazebo environment
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            launch_arguments = [ 
                ('gz_args', [gz_args, ' -r -v 1 ', 
                    PathJoinSubstitution([pkg_share, 'worlds/', world_file]) ]),
                ('on_exit_shutdown', 'True' ),
             ] ),

        LogInfo(msg=PythonExpression(["'Robot controllers file: ",robot_controllers_file,"'"])),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[ddrive_controller_spawner, arm_controller_spawner, pan_controller_spawner, tilt_controller_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[detach_cmd],
            )
        ),
       
        bridge,
        robot_state_publisher,
        gz_spawn_entity,

    ])

