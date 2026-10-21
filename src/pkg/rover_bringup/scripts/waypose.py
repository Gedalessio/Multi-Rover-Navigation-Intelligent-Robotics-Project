#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
from tf_transformations import euler_from_quaternion
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from rclpy.time import Time
from geometry_msgs.msg import Pose


class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('pose_subscriber')

        self.subscription = self.create_subscription(
            PoseStamped,
            '/aruco_single/pose',
            self.pose_callback,
            10
        )
        self.get_logger().info('Sottoscritto al topic PoseStamped!')

        # TF buffer e listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.received_poses = []         # robot poses
        self.received_aruco_poses = []   # aruco poses

        self.waypoint_pub = self.create_publisher(PoseStamped, '/master/discovered_waypoints', 10)

    def get_robot_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'global',         # frame di riferimento
                'master/base_footprint',   # frame del robot
                now
            )
            return trans.transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Errore TF: {str(e)}")
            return None
        
    def get_aruco_camera_pose(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                'global',         # frame di riferimento
                'aruco_marker_frame',   # frame del robot
                now
            )
            return trans.transform
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Errore TF: {str(e)}")
            return None
# 'master/camera_rgb_optical_frame'

    def transform_to_pose(self,transform):
        pose = Pose()
        pose.position.x = transform.translation.x
        pose.position.y = transform.translation.y
        pose.position.z = transform.translation.z
        pose.orientation = transform.rotation
        return pose 
    
    def save_poses_to_yaml(self):
        waypoints = []
        for idx, pose in enumerate(self.received_poses):
            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w
            roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
            goal = {
                "name": f"goal{idx + 1}",
                "position": {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": 0.0
                },
                "orientation": {
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw
                }
            }
            waypoints.append(goal)
            self.publish_waypoint(goal)
            self.get_logger().info(f"Aruco salvato e pubblicato. Robot era in ({robot_pose.position.x:.2f}, {robot_pose.position.y:.2f})")



        data = {"waypoints": waypoints}
        package_path = get_package_share_directory('rover_bringup')
        file_path = os.path.join(package_path, 'config', 'waypoints.yaml')

        with open(file_path, "w") as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

        self.get_logger().info("Waypoints salvati su waypoints.yaml")

 

    def publish_waypoint(self, pose: Pose):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "global"
        pose_stamped.pose = pose
        self.waypoint_pub.publish(pose_stamped)
        self.get_logger().info("Waypoint pubblicato su /master/discovered_waypoints")



    def pose_callback(self, aruco_msg: PoseStamped):
        flag = True
        robot_pose_transf = self.get_robot_pose()
        robot_pose=self.transform_to_pose(robot_pose_transf)
        # aruco_camera_transf=self.get_aruco_camera_pose()
        # aruco_camera_pose=self.transform_to_pose(aruco_camera_transf)
        
        aruco_x = aruco_msg.pose.position.x
        aruco_y = aruco_msg.pose.position.y
        rov_pos_x= robot_pose.position.x
        rov_pos_y = robot_pose.position.y

        if not self.received_aruco_poses:
            self.get_logger().info('Nessuna posa salvata, salvo la prima.')
            flag=True
        else:
            for i, pose in enumerate(self.received_aruco_poses[:]):
                saved_x = pose.pose.position.x
                saved_y = pose.pose.position.y
                distanza = math.sqrt((aruco_x - saved_x)**2 + (aruco_y - saved_y)**2)
                self.get_logger().info(f"Distanza dalla posa salvata: {distanza:.3f} m")

                if distanza <= 1.0:
                    flag = False
                    self.get_logger().info(f"Pose troppo vicina, non la salvo.")
                    

        if flag:
            self.received_poses.append(robot_pose)
            self.received_aruco_poses.append(aruco_msg)
            self.publish_waypoint(robot_pose)
            self.get_logger().info(f"Aruco salvato in posizione nuova. Robot era in ({robot_pose.position.x:.2f}, {robot_pose.position.y:.2f})")
            # self.get_logger().info(f"Aruco camera ({aruco_camera_pose.position.x:.2f}, {aruco_camera_pose.position.y:.2f}, {aruco_camera_pose.position.z:.2f})")

            

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
