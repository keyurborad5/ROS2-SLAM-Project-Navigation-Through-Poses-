"""
Launch file for the automated_vehicle package
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    """
    Main function for the launch file
    """
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # find the parameter file
    parameter_file = os.path.join(
        get_package_share_directory('group17_final'),
        'config',
        'waypoint_params.yaml'
    )
    
    my_robot_node = Node(
        package="group17_final",
        executable="my_robot_node",
        parameters=[parameter_file,
                    {"use_sim_time": use_sim_time},
                    ],    # parameter file
        
    )

    ld = LaunchDescription()
    ld.add_action(my_robot_node)
    return ld
