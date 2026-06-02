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
}

WORLD_SPAWN = {
    'turtlebot3_world':   ('-2.0', '-0.5'),
    'loop_closure_world': ('0.0',  '0.0'),
    'corridor_world':     ('0.0',  '0.0'),
    'dynamic_world':      ('0.0',  '0.0'),
}

WORLD_GZ_NAMES = {
    'turtlebot3_world':   'default',
    'loop_closure_world': 'loop_closure_world',
    'corridor_world':     'corridor_world',
    'dynamic_world':      'dynamic_world',
}


def _worlds_sdf():
    pkg = get_package_share_directory('slam_benchmark')
    return {
        'turtlebot3_world':   None,
        'loop_closure_world': os.path.join(pkg, 'worlds', 'loop_closure_world.sdf'),
        'corridor_world':     os.path.join(pkg, 'worlds', 'corridor_world.sdf'),
        'dynamic_world':      os.path.join(pkg, 'worlds', 'dynamic_world.sdf'),
    }


WORLDS = _worlds_sdf()


def generate_launch_description():

    args = [
        DeclareLaunchArgument('algorithm', default_value='slam_toolbox',
            description='slam_toolbox or amcl'),
        DeclareLaunchArgument('noise_pct', default_value='0',
            description='0, 5, 10, or 20'),
        DeclareLaunchArgument('world', default_value='turtlebot3_world',
            description='turtlebot3_world | loop_closure_world | corridor_world | dynamic_world'),
        DeclareLaunchArgument('run_id', default_value='0pct_001',
            description='Run identifier e.g. 0pct_001'),
        DeclareLaunchArgument('laps', default_value='3',
            description='Number of laps'),
        DeclareLaunchArgument('duration', default_value='300',
            description='Max duration in seconds'),
        DeclareLaunchArgument('seed', default_value='200',
            description='Random seed'),
        DeclareLaunchArgument('map_file', default_value='',
            description='Map YAML for AMCL'),
        DeclareLaunchArgument('output_dir',
            default_value='/tmp/slam_benchmark_results',
            description='Output directory'),
        DeclareLaunchArgument('use_rviz', default_value='false',
            description='Launch RViz'),
    ]

    algorithm  = LaunchConfiguration('algorithm')
    noise_pct  = LaunchConfiguration('noise_pct')
    world      = LaunchConfiguration('world')
    run_id     = LaunchConfiguration('run_id')
    laps       = LaunchConfiguration('laps')
    duration   = LaunchConfiguration('duration')
    seed       = LaunchConfiguration('seed')
    map_file   = LaunchConfiguration('map_file')
    output_dir = LaunchConfiguration('output_dir')
    use_rviz   = LaunchConfiguration('use_rviz')

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

    tf_noise_injector = Node(
        package='slam_benchmark', executable='tf_noise_injector',
        name='tf_noise_injector',
        parameters=[{'noise_percentage': noise_pct, 'seed': seed}],
        output='screen'
    )

    def make_gt_publisher(context, *args, **kwargs):
        world_val = LaunchConfiguration('world').perform(context)
        gz_world = WORLD_GZ_NAMES.get(world_val, 'default')
        return [Node(
            package='slam_benchmark', executable='gt_publisher',
            name='ground_truth_publisher',
            parameters=[{'model_name': 'waffle', 'world_name': gz_world}],
            output='screen',
        )]

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch', 'online_async_launch.py'
            )
        ),
        launch_arguments={
            'slam_params_file': os.path.join(
                get_package_share_directory('slam_benchmark'),
                'config', 'slam_toolbox_params.yaml'
            )
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", algorithm, "' == 'slam_toolbox'"])
        )
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=[
            '-d', os.path.join(
                get_package_share_directory('slam_benchmark'),
                'config', 'slam_benchmark.rviz'
            )
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", algorithm, "' == 'slam_toolbox' or '", use_rviz, "' == 'true'"
            ])
        )
    )

    def make_amcl_stack(context, *args, **kwargs):
        """AMCL + map_server + lifecycle_manager with spawn-matched initial pose."""
        if LaunchConfiguration('algorithm').perform(context) != 'amcl':
            return []
        world_val    = LaunchConfiguration('world').perform(context)
        map_file_val = LaunchConfiguration('map_file').perform(context)
        sx, sy = WORLD_SPAWN.get(world_val, ('0.0', '0.0'))

        amcl_node = Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            parameters=[{
                'use_sim_time': True,
                'base_frame_id': 'base_footprint',
                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                'max_particles': 2000, 'min_particles': 500,
                'scan_topic': '/scan', 'tf_broadcast': True,
                'set_initial_pose': True,
                'initial_pose_x': float(sx),
                'initial_pose_y': float(sy),
                'initial_pose_a': 0.0,
            }],
            output='screen',
        )
        map_server_node = Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'use_sim_time': True, 'yaml_filename': map_file_val}],
            output='screen',
        )
        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{
                'use_sim_time': True, 'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
            output='screen',
        )
        return [amcl_node, map_server_node, lifecycle_manager]

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

    recorder = Node(
        package='slam_benchmark', executable='trajectory_recorder',
        name='trajectory_recorder',
        parameters=[{
            'output_dir': output_dir,
            'run_id': run_id,
            'algorithm': algorithm,
            'noise_pct': noise_pct,
            'world': world,
            'duration_sec': duration,
        }],
        output='screen'
    )

    log_start = LogInfo(msg=[
        '\n============================================\n',
        '  SLAM BENCHMARK RUN\n',
        '  Algorithm: ', algorithm, '\n',
        '  Noise:     ', noise_pct, '%\n',
        '  World:     ', world, '\n',
        '  Run ID:    ', run_id, '\n',
        '  Laps:      ', laps, '\n',
        '  Duration:  ', duration, 's (max)\n',
        '  Seed:      ', seed, '\n',
        '============================================\n',
    ])

    return LaunchDescription([
        *args,
        gz_resource_path,
        log_start,

        gazebo_default,
        gazebo_loop,
        gazebo_corridor,
        gazebo_dynamic,

        TimerAction(period=5.0, actions=[tf_noise_injector]),
        TimerAction(period=5.0, actions=[OpaqueFunction(function=make_gt_publisher)]),

        TimerAction(period=12.0, actions=[
            slam_toolbox,
            OpaqueFunction(function=make_amcl_stack),
        ]),

        TimerAction(period=16.0, actions=[rviz]),
        TimerAction(period=13.0, actions=[recorder]),
        TimerAction(period=20.0, actions=[OpaqueFunction(function=make_navigator)]),
    ])
