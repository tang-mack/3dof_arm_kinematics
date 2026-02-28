from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare the arguments so they can be passed via command line
    theta1_arg = DeclareLaunchArgument('theta1', default_value='0.0', description='Joint angle 1 (rad)')
    theta2_arg = DeclareLaunchArgument('theta2', default_value='0.0', description='Joint angle 2 (rad)')
    theta3_arg = DeclareLaunchArgument('theta3', default_value='0.0', description='Joint angle 3 (rad)')

    # 2. Define the FK Publisher Node
    fk_node = Node(
        package='planar_arm_kinematics',
        executable='fk_publisher_node',
        name='fk_publisher_node',
        parameters=[{
            'theta1': LaunchConfiguration('theta1'),
            'theta2': LaunchConfiguration('theta2'),
            'theta3': LaunchConfiguration('theta3'),
        }],
        output='screen'
    )

    # 3. Define the IK Subscriber Node
    ik_node = Node(
        package='planar_arm_kinematics',
        executable='ik_subscriber_node',
        name='ik_subscriber_node',
        output='screen'
    )

    # 4. Return the complete description
    return LaunchDescription([
        theta1_arg,
        theta2_arg,
        theta3_arg,
        fk_node,
        ik_node
    ])