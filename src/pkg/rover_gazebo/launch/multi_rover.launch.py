import os
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument


def generate_launch_description():
    world = LaunchConfiguration('world')


    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='chepalle.sdf',
        description='World file to use in Gazebo')
    
    gz_world_arg = PathJoinSubstitution([
        get_package_share_directory('rover_gazebo'), 'worlds', world])

    # Include the gz sim launch file  
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args" : gz_world_arg 
        }.items()
    )

    slave=IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rover_gazebo'),
                    'launch',
                    'rover_gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': 'slave',
                'frame_prefix': 'slave/',
                'x':'-2.0',
                'y':'5.5',
                
            }.items()
    )

    master=IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('rover_gazebo'),
                    'launch',
                    'rover_gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': 'master',
                'frame_prefix': 'master/',
                'tf_prefix' :'master',
                'x':'0.0',
                'y':'0.0',
                
            }.items()
    )

    
    
    # gz_ros2_bridge = Node(
    #     package="ros_gz_bridge",
    #     executable="parameter_bridge",
    #     arguments=[
    #         "/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist",
    #         "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
    #         "/odom/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
    #         "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",          # @ is for a 1 to 1 conversion, [ is for a 1 to multiples conversion
    #         '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
    #         '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
    #         '/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
    #         '/color/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    #         '/depth/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
    #         '/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
    #         '/depth/color/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
    #     ],
    # )

    


    return LaunchDescription([
        
        master, 
        slave,     
        declare_world_cmd,
        # gz_ros2_bridge,
        gz_sim,
    ])
