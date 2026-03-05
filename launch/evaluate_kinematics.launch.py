import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Dynamically locate the yaml config file
    #    This MUST be inside generate_launch_description and before the Nodes
    config_file = os.path.join(
        get_package_share_directory('planar_arm_kinematics'),
        'config',
        'kinematics.yaml'
    )

    # 2. Declare the arguments so they can be passed via command line
    theta1_arg = DeclareLaunchArgument('theta1', default_value='0.6', description='Joint angle 1 (rad)')
    theta2_arg = DeclareLaunchArgument('theta2', default_value='-1.2', description='Joint angle 2 (rad)')
    theta3_arg = DeclareLaunchArgument('theta3', default_value='-0.2', description='Joint angle 3 (rad)')

    solver_arg = DeclareLaunchArgument('solver_backend', default_value='AnalyticalSolver', description='Which backend to use, example: AnalyticalSolver or PinocchioSolver')

    # 3. Define the FK Publisher Node
    fk_node = Node(
        package='planar_arm_kinematics',
        executable='fk_publisher_node',
        name='fk_publisher_node',
        parameters=[{
            'theta1': LaunchConfiguration('theta1'),
            'theta2': LaunchConfiguration('theta2'),
            'theta3': LaunchConfiguration('theta3'),
            'yaml_filepath': config_file,
            'solver_backend': LaunchConfiguration('solver_backend')
        }],
        output='screen'
    )

    # 4. Define the IK Subscriber Node
    ik_node = Node(
        package='planar_arm_kinematics',
        executable='ik_subscriber_node',
        name='ik_subscriber_node',
        parameters=[{
            'yaml_filepath': config_file,
            'solver_backend': LaunchConfiguration('solver_backend')
        }],
        output='screen'
    )

    # 5. Return the complete description
    return LaunchDescription([
        theta1_arg,
        theta2_arg,
        theta3_arg,
        solver_arg,
        fk_node,
        ik_node
    ])