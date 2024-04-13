from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
"""from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource"""


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="false",  # 默认不启动RViz
            description="Start RViz2 automatically with this launch file.",
        )
    )
    
    ydlidar_params = PathJoinSubstitution([
            FindPackageShare("usv_hardware"),
            "config",
            "ydlidar_x3.yaml", 
        ]
    )

    ydlidar_node = LifecycleNode(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[ydlidar_params],
        namespace='/'
    )

    # Initialize Arguments
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("usv_hardware"), "urdf", "usv.urdf.xacro"]
            ),
            " ",
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("usv_hardware"),
            "config",
            "controller.yaml", 
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ('/controller_manager/robot_description', '/robot_description'),
            ],
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description]
    )

    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["usv_controller", "--controller-manager=/controller_manager"],
    )
    
    ekf_config = PathJoinSubstitution([
        FindPackageShare("usv_hardware"),
        "config",
        "robot_localization_parameters.yaml",  # 确保这个文件存在并且包含了正确的EKF配置
    ])
    
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter",
        output="screen",
        parameters=[ekf_config],
    )
    
    slam_toolbox_params = PathJoinSubstitution([
        FindPackageShare("usv_hardware"),
        "config",
        "mapper_params_online_async.yaml",  # 确保这个文件存在并且包含了正确的SLAM配置
    ])
    
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_toolbox_params],
    )
    
    serial = Node(
        package="serial",
        executable="ros_serial",
        name="ros_serial",
        output="screen",
    )

    """twist_mux_params = PathJoinSubstitution([
        FindPackageShare("usv_hardware"),
        "config",
        "twist_mux.yaml",  # 确保这个文件存在并且包含了正确的SLAM配置
    ])
    
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        name="twist_mux",
        output="screen",
        parameters=[twist_mux_params],
    )"""
    
    
    nodes = [
        serial,
        ekf_node,
        control_node,
        robot_state_pub_node,
        robot_controller_spawner,
        slam_toolbox,
        ydlidar_node,
    ]

    """# 包含navigation_launch.py
    include_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('usv_hardware'), 'bringup', 'navigation_launch.py'
            ])
        )
    )
    
    rrt2D_launch = PathJoinSubstitution([
        FindPackageShare("usv_hardware"),
        "bringup",
        "rrt2Dlaunch.xml",  # 确保这个文件存在并且包含了正确的SLAM配置
    ])

    # 包含RRT_2D.py
    include_RRT_2D_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(rrt2D_launch)
    )"""

    return LaunchDescription(
        declared_arguments + nodes
    )

