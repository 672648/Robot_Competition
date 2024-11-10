import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_wall_follow',
            name='bug2_wall_follow',
            namespace='bug1'),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_go_to_point',
            name='bug2_go_to_point',
            namespace='bug2'),

        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_controller',
            name='bug2_controller'),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_navigation',
            name='robot_controller'),
  ])