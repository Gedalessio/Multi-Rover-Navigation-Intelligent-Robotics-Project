import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from launch.conditions import IfCondition
from launch.substitutions import PythonExpression

def generate_launch_description():
    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf = os.path.join(get_package_share_directory(
        'rover_description_pkg'), 'urdf', 'rover.xacro')
    # world = LaunchConfiguration('world')
    # frame_prefix = LaunchConfiguration("frame_prefix")
    tf_prefix = LaunchConfiguration("tf_prefix")   
    x=LaunchConfiguration('x')
    y=LaunchConfiguration('y')
    namespace = LaunchConfiguration('namespace') 

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value='',
            description="Prefix of the joint names, useful for "
            "multi-robot setup. If changed than also joint names in the controllers' configuration "
            "have to be updated.",
        )
    )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "frame_prefix",
    #         default_value='rover/',
    #         description="Prefix of the joint names, useful for "
    #         "multi-robot setup. If changed than also joint names in the controllers' configuration "
    #         "have to be updated.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "x",
    #         default_value='',
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'namespace',
    #         default_value='rover',
    #         description='Namespace of launched nodes, useful for multi-robot setup. \
    #                      If changed than also the namespace in the controllers \
    #                      configuration needs to be updated. Expected format "<ns>/".',
    #     )
    # )
  

    robot_desc = ParameterValue(Command(
            ['xacro ', urdf,
             " ",            
            "frame_prefix:=",  
            namespace," ",
            'tf_prefix:=',namespace,
            " ",
            'namespace:=',
            namespace
            ]),value_type=str)
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    # declare_world_cmd = DeclareLaunchArgument(
    #     'world',
    #     default_value='maze.sdf',
    #     description='World file to use in Gazebo')
    
    # gz_world_arg = PathJoinSubstitution([
    #     get_package_share_directory('rover_gazebo'), 'worlds', world])

    # # Include the gz sim launch file  
    # gz_sim_share = get_package_share_directory("ros_gz_sim")
    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")),
    #     launch_arguments={
    #         "gz_args" : gz_world_arg 
    #     }.items()
    # )
    
    # Spawn Rover Robot
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        arguments=[
            "-topic", ['/',namespace,"/robot_description"],
            "-name", namespace,
            "-allow_renaming", "true",
            "-z", "0.1",
            "-y",y,
            "-x",x,
           
        ]
    )

    gz_ros2_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name=[namespace, '_gz_bridge'],
            arguments=[          
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                ['/model/', namespace, '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
                ['/model/', namespace, '/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'],
                ['/model/', namespace, '/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'],
                ['/world/maze/model/',namespace,'/link/base_footprint/sensor/imu/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'],
                ['/world/maze/model/',namespace,'/link/base_footprint/sensor/gpu_lidar/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'],
                ['/world/maze/model/', namespace, '/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'],
                ['/model/', namespace, '/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'],
                ['/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'],
                
               
                # "/master/odom/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                # "/slave/odom/wheels@nav_msgs/msg/Odometry@ignition.msgs.Odometry",
                # "/master/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",          # @ is for a 1 to 1 conversion, [ is for a 1 to multiples conversion
                # "/slave/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",          # @ is for a 1 to 1 conversion, [ is for a 1 to multiples conversion
                # '/master/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                # '/slave/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
                # '/master/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                # '/slave/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                # '/slave/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
                # '/master/imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU',
                

               
            ],
            remappings=[
                            (['/model/', namespace, '/cmd_vel'],['/', namespace , '/cmd_vel']),
                            (['/model/', namespace, '/odometry'],['/', namespace, '/odom']),                         
                            (['/world/maze/model/', namespace, '/joint_state'],['/', namespace, '/joint_states']),                      
                            (['/world/maze/model/',namespace,'/link/base_footprint/sensor/gpu_lidar/scan'],['/',namespace,'/scan']),
                            (['/world/maze/model/',namespace,'/link/base_footprint/sensor/imu/imu'],['/',namespace,'/imu']),
                            (['/model/', namespace, '/pose'],['/', namespace, '/pose']),
                        ],
            
        
    )

    gz_ros2_bridge_camera = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name='gz_camera_bridge',
        condition=IfCondition(PythonExpression(["'", namespace, "' == 'master'"])),
        arguments=[
            'master/color/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            'master/depth/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            'master/color/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            'master/depth/color/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '--ros-args',
            '-r','/master/color/image_raw:=videocamera',
        ],
        output='screen',
    )

    
    common_frame_publisher = Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='common_frame',
                output='screen',
                namespace=namespace,
                remappings=[],
                parameters=[{'use_sim_time': use_sim_time}],
                arguments=[LaunchConfiguration('x'), LaunchConfiguration('y'), '0', '0', '0', '0', 'global', [namespace , '/map']])


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

    # Robot state publisher
    params = {'use_sim_time': use_sim_time, 
              'robot_description': robot_desc, 
              
              "frame_prefix":[namespace,'/']}
    start_robot_state_publisher_cmd = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=namespace,
            output='both',
            parameters=[params],
            arguments=[])

    # Create the launch description and populate
    ld = LaunchDescription(declared_arguments)

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    # ld.add_action(declare_world_cmd)

    # Launch Gazebo
    # ld.add_action(gz_sim)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)
    ld.add_action(gz_ros2_bridge_camera)
    ld.add_action(common_frame_publisher)

    # Launch Robot State Publisher
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
