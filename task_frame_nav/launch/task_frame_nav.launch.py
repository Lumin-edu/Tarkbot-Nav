import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    action_share = get_package_share_directory('airobot_action')
    fast_lio_share = get_package_share_directory('fast_lio')
    livox_share = get_package_share_directory('livox_ros_driver2')
    tarkbot_share = get_package_share_directory('tarkbot_robot')
    nav2_share = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    nav2_params = LaunchConfiguration('nav2_params')
    fast_lio_config = LaunchConfiguration('fast_lio_config')
    save_pcd = LaunchConfiguration('save_pcd')
    body_x = LaunchConfiguration('body_to_base_x')
    body_y = LaunchConfiguration('body_to_base_y')
    body_z = LaunchConfiguration('body_to_base_z')
    body_roll = LaunchConfiguration('body_to_base_roll')
    body_pitch = LaunchConfiguration('body_to_base_pitch')
    body_yaw = LaunchConfiguration('body_to_base_yaw')
    task_frame = LaunchConfiguration('task_frame')
    nav2_use_composition = PythonExpression([
        "'True' if '", use_composition,
        "'.lower() in ('true', '1', 'yes', 'on') else 'False'",
    ])

    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='false'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz with Task-frame navigation displays'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                action_share,
                'task_frame_nav',
                'config',
                'task_frame_nav.rviz'),
            description='Path to the RViz configuration file'),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=os.path.join(
                action_share,
                'task_frame_nav',
                'config',
                'nav2_task_frame.yaml')),
        DeclareLaunchArgument(
            'fast_lio_config',
            default_value=os.path.join(
                fast_lio_share, 'config', 'mid360.yaml')),
        DeclareLaunchArgument('save_pcd', default_value='false'),
        DeclareLaunchArgument('body_to_base_x', default_value='0.145194959'),
        DeclareLaunchArgument('body_to_base_y', default_value='0.0'),
        DeclareLaunchArgument('body_to_base_z', default_value='-0.137544261'),
        DeclareLaunchArgument('body_to_base_roll', default_value='0.0'),
        DeclareLaunchArgument(
            'body_to_base_pitch',
            default_value='-0.812450766803360'),
        DeclareLaunchArgument('body_to_base_yaw', default_value='0.0'),
        DeclareLaunchArgument('task_frame', default_value='task_frame'),
        DeclareLaunchArgument('robot_type', default_value='r20_fwd'),
        DeclareLaunchArgument('invert_angular_z', default_value='false'),
        DeclareLaunchArgument('robot_port', default_value='/dev/tarkbot_base'),
        DeclareLaunchArgument(
            'livox_config',
            default_value=os.path.join(
                livox_share, 'config', 'MID360_config.json')),
        DeclareLaunchArgument(
            'livox_bd_code', default_value='livox0000000001'),
        DeclareLaunchArgument('livox_frame', default_value='livox_frame'),
        DeclareLaunchArgument('auto_goal', default_value='false'),
        DeclareLaunchArgument('goal_x', default_value='0.0'),
        DeclareLaunchArgument('goal_y', default_value='0.0'),
    ]

    # MID360 custom messages are expected by FAST-LIO with lidar_type=1.
    livox_node = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': LaunchConfiguration('livox_frame'),
            'lvx_file_path': '/home/livox/livox_test.lvx',
            'user_config_path': LaunchConfiguration('livox_config'),
            'cmdline_input_bd_code': LaunchConfiguration('livox_bd_code'),
        }],
    )

    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            tarkbot_share, 'launch', 'robot.launch.py')),
        launch_arguments={
            'base_frame': 'base_link',
            'odom_frame': 'odom',
            'odom_topic': 'odom',
            'cmd_vel_topic': 'cmd_vel',
            'robot_type': LaunchConfiguration('robot_type'),
            'invert_angular_z': LaunchConfiguration('invert_angular_z'),
            'robot_port': LaunchConfiguration('robot_port'),
            # FAST-LIO + the static leveling transform are the sole TF path.
            'pub_odom_tf': 'false',
        }.items(),
    )

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        output='screen',
        parameters=[
            fast_lio_config,
            {
                'use_sim_time': use_sim_time,
                # A long-running navigation process should not retain an
                # unbounded PCD save buffer unless explicitly requested.
                'pcd_save.pcd_save_en': save_pcd,
            },
        ],
    )

    leveling_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base_link_leveling_tf',
        output='screen',
        arguments=[
            '--x', body_x,
            '--y', body_y,
            '--z', body_z,
            '--roll', body_roll,
            '--pitch', body_pitch,
            '--yaw', body_yaw,
            '--frame-id', 'body',
            '--child-frame-id', 'base_link',
        ],
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'namespace': '',
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            # Keep every Nav2 global_frame in sync with the frame selected by
            # task_frame:=...; the checked-in YAML uses task_frame as its
            # default, but a launch override must rewrite it as well.
            'params_file': RewrittenYaml(
                source_file=nav2_params,
                param_rewrites={'global_frame': task_frame},
                convert_types=True),
            'use_composition': nav2_use_composition,
            'use_respawn': 'false',
            'container_name': 'nav2_container',
            'log_level': 'info',
        }.items(),
    )

    task_node = Node(
        package='airobot_action',
        executable='task_frame_nav_node',
        name='task_frame_nav',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': '/Odometry',
            'odom_frame': 'camera_init',
            'body_frame': 'body',
            'capture_delay': 3.0,
            'body_to_base_x': body_x,
            'body_to_base_y': body_y,
            'body_to_base_z': body_z,
            'body_to_base_roll': body_roll,
            'body_to_base_pitch': body_pitch,
            'body_to_base_yaw': body_yaw,
            'task_frame': task_frame,
            'auto_goal': LaunchConfiguration('auto_goal'),
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
        }],
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='task_frame_nav_rviz',
        output='screen',
        arguments=['-d', RewrittenYaml(
            source_file=rviz_config,
            param_rewrites={'Fixed Frame': task_frame},
            convert_types=True)],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription(arguments + [
        livox_node,
        robot_launch,
        fast_lio_node,
        leveling_tf,
        task_node,
        nav2_launch,
        rviz_node,
    ])
