from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'),

        Node(  #主要借用cartographer_ros中的odom融合：将 /wheel_odom与/imu 融合成/odom
            package = 'cartographer_ros',
            executable = 'cartographer_odom_preproc',
            name='cartographer_odom_preproc',
            parameters = [
                {'use_sim_time': use_sim_time}],
        ),

        Node(
            package='grid_mapping', 
            executable='grid_mapping_ros', 
            name='grid_mapping_ros',
            output='screen', 
            parameters=[{'use_sim_time':use_sim_time}]
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='odom_to_base_footprint',
        #     arguments=['0','0','0','0','0','0','odom','base_footprint']
        # ),
        Node(
            package='grid_mapping',
            executable='odom_tf_pub',
            name='odom_base_footprint_publisher',
            parameters=[{'use_sim_time':use_sim_time}]
        ),
    ])