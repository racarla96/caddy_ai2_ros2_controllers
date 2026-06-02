"""
controllers.launch.py — real-robot controller bringup for Caddy AI2.

Starts:
  1. ros2_control_node   — controller manager (no hardware interface needed;
                           both driver controllers own their CAN bus directly)
  2. steering_driver_controller spawner
  3. traction_driver_controller spawner
  4. bicycle_cmd_relay   — converts /cmd_vel → per-driver ~/reference topics
                           using the bicycle kinematic model
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Minimal robot_description for the controller_manager.
# No <ros2_control> hardware tags: our controllers own the CAN bus directly.
_MINIMAL_URDF = """\
<?xml version="1.0"?>
<robot name="caddy_ai2">
  <link name="base_footprint"/>
</robot>
"""


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context)
    desc_share = get_package_share_directory('caddy_ai2_ros2_description')
    ctrl_share = get_package_share_directory('caddy_ai2_ros2_controllers')

    # Physical parameters (wheelbase, max_steer_angle, …)
    params_file = os.path.join(desc_share, 'bringup', 'config', 'robot_params.yaml')
    with open(params_file) as f:
        robot_params = yaml.safe_load(f)

    controllers_yaml = os.path.join(ctrl_share, 'config', 'controllers.yaml')

    # --------------------------------------------------------------------- #
    # ros2_control_node (controller manager)                                 #
    # --------------------------------------------------------------------- #
    node_controller_manager = Node(
        package='ros2_control',
        executable='ros2_control_node',
        namespace=namespace,
        parameters=[
            {'robot_description': _MINIMAL_URDF},
            controllers_yaml,
        ],
        output='screen',
    )

    # --------------------------------------------------------------------- #
    # Controller spawners                                                     #
    # Spawners wait automatically for the controller_manager service.        #
    # --------------------------------------------------------------------- #
    steering_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            'steering_driver_controller',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    traction_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=[
            'traction_driver_controller',
            '--param-file', controllers_yaml,
        ],
        output='screen',
    )

    # --------------------------------------------------------------------- #
    # Bicycle model cmd_vel relay                                            #
    # Converts /cmd_vel (Twist) → steering + traction ~/reference topics.   #
    # --------------------------------------------------------------------- #
    bicycle_relay = Node(
        package='caddy_ai2_ros2_controllers',
        executable='bicycle_cmd_relay',
        namespace=namespace,
        parameters=[{
            'wheelbase':       robot_params['wheelbase'],
            'max_steer_angle': robot_params['max_steer_angle'],
        }],
        remappings=[
            # /cmd_vel is the standard nav2 velocity topic; keep it global
            ('cmd_vel', '/cmd_vel'),
        ],
        output='screen',
    )

    return [
        node_controller_manager,
        bicycle_relay,
        # Give the controller_manager a short head start before spawning
        TimerAction(
            period=1.0,
            actions=[steering_spawner, traction_spawner],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='ROS namespace for this robot instance',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
