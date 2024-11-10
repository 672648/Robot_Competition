import launch
import launch_ros

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='controller',
            namespace='tb3_0',
            name='minimal_publisher'),
        launch_ros.actions.Node(
            package='multi_robot_challenge_23',
            executable='controller',
            namespace='tb3_1',
            name='minimal_publisher'),
        ])


