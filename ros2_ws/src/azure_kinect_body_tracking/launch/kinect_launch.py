from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Use FindPackageShare to dynamically locate the package directory
    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('azure_kinect_body_tracking'), 'launch', 'kinect.rviz']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=rviz_config_path,
            description='Full path to the RVIZ config file to use'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
        ),
        
        Node(
            package='azure_kinect_body_tracking',
            executable='body_tracking_node',
            name='body_tracking_node',
            output='screen'
        ),
        
        Node(
            package='azure_kinect_body_tracking',
            executable='joint_angle_node',
            name='joint_angle_node',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
