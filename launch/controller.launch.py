import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # tb3_0 Nodes
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_wall_follow',
            name='bug2_wall_follow',
            namespace='tb3_0'
        ),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_go_to_point',
            name='go_to_point',
            namespace='tb3_0'
        ),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_controller_competition',
            name='bug2_controller',
            namespace='tb3_0'
        ),

        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='robot_controller',
            name='bug2_controller',
            namespace='tb3_0'
        ),
        
        # tb3_1 Nodes
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_wall_follow',
            name='bug2_wall_follow',
            namespace='tb3_1'
        ),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_go_to_point',
            name='go_to_point',
            namespace='tb3_1'
        ),
        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='bug2_controller_competition',
            name='bug2_controller',
            namespace='tb3_1'
        ),

        launch_ros.actions.Node(
            package='bug2_navigation',
            executable='robot_controller',
            name='bug2_controller',
            namespace='tb3_1'
        ),
    ])