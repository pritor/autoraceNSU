from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    proj = Node(
        package='autorace_core_CVlization',
        executable='image_projector',
        remappings=[('/color/image_output', '/color/image_projected')]
    )
    # comp = Node(
    #     package='autorace_core_CVlization',
    #     executable='image_compensation',
    #     remappings=[('/color/image', '/color/image_projected'), ('/color/image_output', '/color/image_projected_comp')]
    # )
    find = Node(
        package='autorace_core_CVlization',
        executable='lane_finder',
        remappings=[('/color/image', '/color/image_projected')]
    )

    img = Node(
        package='autorace_core_CVlization',
        executable='signs_analyzer')

    state = Node(
        package="autorace_core_CVlization",
        executable="state_decider"
    )
    autorace = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_bringup'),
                'launch',
                'autorace_2023.launch.py'
            ])
        ]),
    )
    referee = Node(
        package="referee_console",
        executable="mission_autorace_2023_referee"
    )
    return LaunchDescription([
        autorace,
        referee,
        proj,
        # comp,
        find,
        img,
        state
    ])
