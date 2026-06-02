from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, LogInfo, OpaqueFunction, GroupAction,
    AppendEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory
import os


WORLD_WAYPOINTS = {
    'turtlebot3_world':   [2.6, -0.5,  2.6, 2.0,  1.6, 2.0,  1.6, -0.5],

    'loop_closure_world': [
         1.5,  0.0,
         1.5,  1.5,
        -1.5,  1.5,
        -1.5, -1.5,
         1.5, -1.5,
         1.5,  1.5,
        -4.0,  1.5,
        -4.0,  4.0,
         0.0,  4.0,
         4.0,  4.0,
         4.0,  0.0,
         4.0, -4.0,
         0.0, -4.0,
        -4.0, -4.0,
        -4.0,  1.5,
        -1.5,  1.5,
        -1.5,  0.0,
         0.0,  0.0,
    ],

    'corridor_world': [
         6.5,  0.0,
         6.5,  2.0,
         3.0,  2.0,
        -0.5,  2.0,
        -4.0,  2.0,
        -7.5,  2.0,
        -7.5,  0.0,
       -11.5,  0.0,
       -11.5,  2.0,
        -7.5,  2.0,
        -7.5,  0.0,
         0.0,  0.0,
    ],

    'dynamic_world': [
         3.2,  0.0,
         3.2, -3.2,
        -3.2, -3.2,
        -3.2,  0.0,
         3.2,  0.0,
         3.2,  3.2,
        -3.2,  3.2,
        -3.2,  0.0,
         0.0,  0.0,
    ],

    'dynamic_world_static': [
         3.2,  0.0,
         3.2, -3.2,
        -3.2, -3.2,
        -3.2,  0.0,
         3.2,  0.0,
         3.2,  3.2,
        -3.2,  3.2,
        -3.2,  0.0,
         0.0,  0.0,
    ],
}

WORLD_SPAWN = {
    'turtlebot3_world':     ('-2.0', '-0.5'),
    'loop_closure_world':   ('0.0',  '0.0'),
    'corridor_world':       ('0.0',  '0.0'),
    'dynamic_world':        ('0.0',  '0.0'),
    'dynamic_world_static': ('0.0',  '0.0'),
}


def _worlds_sdf():
    pkg = get_package_share_directory('slam_benchmark')
    return {
        'turtlebot3_world':     None,
        'loop_closure_world':   os.path.join(pkg, 'worlds', 'loop_closure_world.sdf'),
        'corridor_world':       os.path.join(pkg, 'worlds', 'corridor_world.sdf'),
        'dynamic_world':        os.path.join(pkg, 'worlds', 'dynamic_world.sdf'),
        'dynamic_world_static': os.path.join(pkg, 'worlds', 'dynamic_world_static.sdf'),
    }


WORLDS = _worlds_sdf()


def generate_launch_description():

    args = [
        DeclareLaunchArgument('world', default_value='turtlebot3_world',
            description='turtlebot3_world | loop_closure_world | corridor_world | dynamic_world | dynamic_world_static'),
        DeclareLaunchArgument('laps', default_value='1',
            description='Number of laps for map coverage'),
        DeclareLaunchArgument('map_save_path',
            default_value='~/slam_thesis_ws/maps/map',
            description='Path prefix for map_saver_cli (informational)'),
        DeclareLaunchArgument('use_rviz', default_value='true',
            description='Launch RViz to visualise mapping progress'),
    ]

    world         = LaunchConfiguration('world')
    laps          = LaunchConfiguration('laps')
    map_save_path = LaunchConfiguration('map_save_path')
    use_rviz      = LaunchConfiguration('use_rviz')

    tb3_launch_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    )

    gazebo_default = GroupAction([
        SetRemap(src='/tf', dst='/tf_bridge'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'turtlebot3_world.launch.py')
            ),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'turtlebot3_world'"])
            )
        ),
    ])

    lx, ly = WORLD_SPAWN['loop_closure_world']
    gazebo_loop = GroupAction([
        SetRemap(src='/tf', dst='/tf_bridge'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r -s -v2 {WORLDS["loop_closure_world"]}',
                'on_exit_shutdown': 'true',
            }.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'loop_closure_world'"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'loop_closure_world'"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'loop_closure_world'"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={'x_pose': lx, 'y_pose': ly}.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'loop_closure_world'"])
            )
        ),
    ])

    cx, cy = WORLD_SPAWN['corridor_world']
    gazebo_corridor = GroupAction([
        SetRemap(src='/tf', dst='/tf_bridge'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r -s -v2 {WORLDS["corridor_world"]}',
                'on_exit_shutdown': 'true',
            }.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'corridor_world'"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'corridor_world'"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'corridor_world'"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={'x_pose': cx, 'y_pose': cy}.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'corridor_world'"])
            )
        ),
    ])

    dx, dy = WORLD_SPAWN['dynamic_world']
    gazebo_dynamic = GroupAction([
        SetRemap(src='/tf', dst='/tf_bridge'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r -s -v2 {WORLDS["dynamic_world"]}',
                'on_exit_shutdown': 'true',
            }.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'dynamic_world'"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'dynamic_world'"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'dynamic_world'"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={'x_pose': dx, 'y_pose': dy}.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'dynamic_world'"])
            )
        ),
    ])

    ds_x, ds_y = WORLD_SPAWN['dynamic_world_static']
    gazebo_dynamic_static = GroupAction([
        SetRemap(src='/tf', dst='/tf_bridge'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': f'-r -s -v2 {WORLDS["dynamic_world_static"]}',
                'on_exit_shutdown': 'true',
            }.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'dynamic_world_static'"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': '-g -v2', 'on_exit_shutdown': 'true'}.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'dynamic_world_static'"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'robot_state_publisher.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'dynamic_world_static'"])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(tb3_launch_dir, 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={'x_pose': ds_x, 'y_pose': ds_y}.items(),
            condition=IfCondition(
                PythonExpression(["'", world, "' == 'dynamic_world_static'"])
            )
        ),
    ])

    tf_relay = Node(
        package='slam_benchmark', executable='tf_noise_injector',
        name='tf_noise_injector',
        parameters=[{'noise_percentage': 0, 'seed': 0}],
        output='screen'
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        )
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=[
            '-d', os.path.join(
                get_package_share_directory('slam_toolbox'),
                'config', 'slam_toolbox_default.rviz'
            )
        ],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    def make_navigator(context, *args, **kwargs):
        world_val = LaunchConfiguration('world').perform(context)
        waypoints = WORLD_WAYPOINTS.get(world_val, WORLD_WAYPOINTS['turtlebot3_world'])
        node = Node(
            package='slam_benchmark', executable='waypoint_navigator',
            name='waypoint_navigator',
            parameters=[{
                'use_sim_time': True,
                'linear_speed': 0.22,
                'angular_speed': 0.8,
                'start_delay': 5.0,
                'laps': LaunchConfiguration('laps'),
                'waypoints': waypoints,
            }],
            output='screen'
        )
        return [node]

    log_start = LogInfo(msg=[
        '\n============================================\n',
        '  CREATE MAP\n',
        '  World:    ', world, '\n',
        '  Laps:     ', laps, '\n',
        '  When done, save the map with:\n',
        '    ros2 run nav2_map_server map_saver_cli -f <map_save_path>\n',
        '============================================\n',
    ])

    log_reminder = LogInfo(msg=[
        '\n[create_map] Map building in progress.\n',
        '  Save when done:\n',
        '    ros2 run nav2_map_server map_saver_cli -f ', map_save_path, '\n',
    ])

    return LaunchDescription([
        *args,
        gz_resource_path,
        log_start,

        gazebo_default,
        gazebo_loop,
        gazebo_corridor,
        gazebo_dynamic,
        gazebo_dynamic_static,

        TimerAction(period=10.0, actions=[tf_relay, slam_toolbox]),
        TimerAction(period=10.0, actions=[rviz]),
        TimerAction(period=20.0, actions=[OpaqueFunction(function=make_navigator)]),
        TimerAction(period=2.0, actions=[log_reminder]),
    ])
