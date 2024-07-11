# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example for spawing multiple robots in Gazebo.

This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command

from launch_ros.actions import Node
from launch.substitutions import PythonExpression, PathJoinSubstitution


def generate_launch_description():
    # Get the launch directory
    # a
    bringup_dir = get_package_share_directory('turtlebot3_multi_robot')
    nav_bringup_dir = get_package_share_directory('nav2_bringup')

    launch_dir = os.path.join(bringup_dir, 'launch')
    turtlebot3_multi_robot = get_package_share_directory('turtlebot3_multi_robot')

    frame_prefix = LaunchConfiguration('frame_prefix')

    # Names and poses of the robots
    robots = [
        {'name': 'robot1', 'x_pose': '-1.5', 'y_pose': '0.3', 'z_pose': '0.01'},
        {'name': 'robot2', 'x_pose': '-2.5', 'y_pose': '0.5', 'z_pose': '0.01'},
        {'name': 'robot3', 'x_pose': '-4.5', 'y_pose': '0.4', 'z_pose': '0.01'}
        ]

    # Simulation settings
    world = LaunchConfiguration('world')
    simulator = LaunchConfiguration('simulator')

    # On this example all robots are launched with the same settings
    map_yaml_file = LaunchConfiguration('map')

    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='true')

    # Declare the launch arguments
    declare_frame_prefix_cmd = DeclareLaunchArgument(
        'frame_prefix',
        default_value='robot',
        description='Full path to world file to load')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # default_value=os.path.join(bringup_dir, 'worlds', 'multi_robot_world.world'),
        default_value=os.path.join(bringup_dir, 'worlds', 'turtlebot3_house.world'),
        description='Full path to world file to load')

    declare_simulator_cmd = DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    declare_robot_params_file_cmd = DeclareLaunchArgument(
        'robot_params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )


    # declare_params_file_cmd = DeclareLaunchArgument(
    #     'nav_params_file',
    #     default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
    #     description='Full path to the ROS2 parameters file to use for all launched nodes')

    # declare_robot1_params_file_cmd = DeclareLaunchArgument(
    #     'robot1_params_file',
    #     default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_1.yaml'),
    #     description='Full path to the ROS2 parameters file to use for robot1 launched nodes')

    # declare_robot2_params_file_cmd = DeclareLaunchArgument(
    #     'robot2_params_file',
    #     default_value=os.path.join(bringup_dir, 'params', 'nav2_multirobot_params_2.yaml'),
    #     description='Full path to the ROS2 parameters file to use for robot2 launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='false',
        description='Automatically startup the stacks')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_namespaced_view.rviz'),
        description='Full path to the RVIZ config file to use.')

    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    # Start Gazebo with plugin providing the robot spawing service
    
    start_gazebo_cmd = ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    xacro_file_path = os.path.join(turtlebot3_multi_robot, 'urdf', 'turtlebot3_waffle.urdf.xacro')

    
    # Define commands for spawing the robots into Gazebo
    spawn_robots_cmds = []
    for robot in robots:
        namespace = [ '/' + robot['name']]
        robot_prefix = robot['name']
        robot_desc = Command(['xacro ', str(xacro_file_path), ' frame_prefix:=', robot_prefix, ' topic_prefix:=', robot_prefix])

        spawn_robots_cmds.append(
            Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    namespace=robot_prefix,
                    parameters=[{
                        'use_sim_time': True,
                        # 'publish_frequency': 10.0,
                        'robot_description': robot_desc,
                        'frame_prefix': 
                            PythonExpression(["'", robot_prefix, "/'"])
                            # PythonExpression(["'", robot_prefix, "/'"])
                    }],
                )
        )
        spawn_robots_cmds.append(
            Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', PathJoinSubstitution([robot_prefix, 'waffle']),
                        '-topic', PathJoinSubstitution([robot_prefix, 'robot_description']),
                        '-x', robot['x_pose'],
                        '-y', robot['y_pose'],
                        '-z', '0.01'
                    ],
                    parameters=[{
                        'use_sim_time': True,
                    }],
                    output='screen',
                )
        )


        # spawn_robots_cmds.append(
        #     IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource(os.path.join(nav_bringup_dir, 'launch',
        #                                                    'spawn_tb3_launch.py')),
        #         launch_arguments={
        #                           'x_pose': TextSubstitution(text=str(robot['x_pose'])),
        #                           'y_pose': TextSubstitution(text=str(robot['y_pose'])),
        #                           'z_pose': TextSubstitution(text=str(robot['z_pose'])),
        #                           'robot_name': robot['name'],
        #                           'turtlebot_type': TextSubstitution(text='waffle')
        #                           }.items()))

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        # params_file = LaunchConfiguration(robot['name'] + '_params_file')
        params_file = LaunchConfiguration('robot_params_file')
        group = GroupAction([
            # IncludeLaunchDescription(
            #     PythonLaunchDescriptionSource(
            #             os.path.join(launch_dir, 'rviz_launch.py')),
            #     condition=IfCondition(use_rviz),
            #     launch_arguments={
            #                       'namespace': TextSubstitution(text=robot['name']),
            #                       'use_namespace': 'True',
            #                       'rviz_config': rviz_config_file}.items()
            #                       ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(bringup_dir,
                                                           'launch',
                                                           'nav2_bringup',
                                                           'tb3_simulation_launch.py')),
                launch_arguments={'namespace': robot['name'],
                                  'use_namespace': 'True',
                                  'map': map_yaml_file,
                                  'use_sim_time': 'True',
                                  'params_file': params_file,
                                  'default_bt_xml_filename': default_bt_xml_filename,
                                  'slam': 'True',
                                  'autostart': autostart,
                                  'use_rviz': 'False',
                                  'use_simulator': 'False',
                                  'headless': 'False',
                                  'frame_prefix': robot['name']+'/base_footprint',
                                  'use_robot_state_pub': use_robot_state_pub}.items()),

            Node(package = "tf2_ros",
                name=robot['name'],
                executable = "static_transform_publisher",
                arguments = ["0", "0", "0", "0", "0", "0", "odom", robot['name']+"/odom"],
                parameters=[{
                    'use_sim_time': True
                }]
            ),

            LogInfo(
                condition=IfCondition(log_settings),
                msg=['Launching ', robot['name']]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' map yaml: ', map_yaml_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' params yaml: ', params_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' behavior tree xml: ', default_bt_xml_filename]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' rviz config file: ', rviz_config_file]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' using robot state pub: ', use_robot_state_pub]),
            LogInfo(
                condition=IfCondition(log_settings),
                msg=[robot['name'], ' autostart: ', autostart])
        ])

        nav_instances_cmds.append(group)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_frame_prefix_cmd)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_robot_params_file_cmd)
    # ld.add_action(declare_robot2_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_robot_state_pub_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    for simulation_instance_cmd in nav_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld