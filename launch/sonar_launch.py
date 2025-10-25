from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sonar',
            namespace='utux/sensor',
            executable='sonar_pub',
            name='SonarPublisher'
        )

        Node(
            package='sonar',
            namespace='utux/sensor',
            executable='sonar_sub',
            name='SonarSubscriber')
    ])