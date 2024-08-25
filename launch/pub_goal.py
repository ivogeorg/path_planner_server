import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='path_planner_server',  # Replace with your package name
            executable='nav_to_pose_action_client',  # Replace with your node's executable name
            name='nav_to_pose_action_client',  # Optionally, give the node a name
            output='screen',  # Output logs to the screen
            # Add any additional arguments or parameters as needed
            # arguments=['--your_arg', 'value'],
            # parameters=[{'param_name': 'param_value'}],
        ),
    ])