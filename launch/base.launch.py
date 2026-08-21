from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rtk_accuracy_test",
            executable="bx100_test_recorder",
            name="rtk_test_recorder",
            output="screen",
            parameters=[PathJoinSubstitution([
                FindPackageShare("rtk_accuracy_test"), "config", "base.yaml"
            ])],
        )
    ])
