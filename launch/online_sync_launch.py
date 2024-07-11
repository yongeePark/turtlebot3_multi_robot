import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    # default_params_file = os.path.join(get_package_share_directory("slam_toolbox"),
    default_params_file = os.path.join(get_package_share_directory("turtlebot3_multi_robot"),
                                       'config', 'mapper_params_online_sync.yaml')


    namespace = LaunchConfiguration('namespace')

    frame_prefix = LaunchConfiguration('frame_prefix')

    declare_frame_prefix_cmd = DeclareLaunchArgument(
        'frame_prefix',
        default_value='robot',
        description='Top-level namespace')


    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='robot',
        description='Top-level namespace')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    # declare_robot_params_file_cmd = DeclareLaunchArgument(
    #     'robot_params_file',
    #     default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
    #     description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

    # If the provided param file doesn't have slam_toolbox params, we must pass the
    # default_params_file instead. This could happen due to automatic propagation of
    # LaunchArguments. See:
    # https://github.com/ros-planning/navigation2/pull/2243#issuecomment-800479866
    has_node_params = HasNodeParams(source_file=params_file,
                                    node_name='slam_toolbox')

    actual_params_file = PythonExpression(['"', params_file, '" if ', has_node_params,
                                           ' else "', default_params_file, '"'])

    log_param_change = LogInfo(msg=['provided params_file ',  params_file,
                                    ' does not contain slam_toolbox parameters. Using default: ',
                                    default_params_file],
                               condition=UnlessCondition(has_node_params))

    remappings = [('/tf', 'tf'), 
              ('/tf_static', 'tf_static')]

    print("BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB Namespace : ",namespace)
    param_substitutions = { 
        'use_sim_time': use_sim_time,
        'base_frame': frame_prefix
        } 
  
    configured_params = RewrittenYaml( 
        source_file=default_params_file, 
        root_key=namespace,
        param_rewrites=param_substitutions, 
        convert_types=True) 

    
    print(str(actual_params_file))
    start_sync_slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        namespace=namespace,
        parameters=[configured_params],
        remappings=[
            ('/scan', 'scan'),
            ('/map', 'map'),
        ],
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_frame_prefix_cmd)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(log_param_change)
    ld.add_action(start_sync_slam_toolbox_node)

    return ld
