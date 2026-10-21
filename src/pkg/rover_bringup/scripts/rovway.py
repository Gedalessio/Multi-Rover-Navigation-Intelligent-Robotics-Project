#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion
import yaml
import os
import math
from collections import deque

from ament_index_python.packages import get_package_share_directory


class PoseSubscriber(Node):

    def __init__(self):
        super().__init__('pose_subscriber_sync')

        self.aruco_sub = self.create_subscription(
            PoseStamped, '/aruco_single/pose', self.aruco_callback, 10)
        self.rover_sub = self.create_subscription(
            PoseStamped, '/master/pose', self.rover_callback, 10)

        # Buffer per messaggi in arrivo
        self.aruco_buffer = deque()
        self.rover_buffer = deque()

        # Tolleranza sincronizzazione in secondi
        self.time_tolerance = 0.1

        self.received_poses = []
        self.received_aruco_poses = []

        self.get_logger().info('Subscriber sincronizzati manualmente avviati!')

    def to_sec(self, stamp):
        return stamp.sec + stamp.nanosec * 1e-9

    def aruco_callback(self, msg: PoseStamped):
        self.get_logger().info('Subscriber aruco')
        self.aruco_buffer.append(msg)
        self.try_sync()

    def rover_callback(self, msg: PoseStamped):
        self.rover_buffer.append(msg)
        self.get_logger().info('Subscriber rover')
        self.try_sync()

    def try_sync(self):
        while self.aruco_buffer and self.rover_buffer:
            aruco_msg = self.aruco_buffer[0]
            rover_msg = self.rover_buffer[0]

            t_aruco = self.to_sec(aruco_msg.header.stamp)
            t_rover = self.to_sec(rover_msg.header.stamp)

            diff = abs(t_aruco - t_rover)
            if diff <= self.time_tolerance:
                # Messaggi sincronizzati, rimuovi da buffer e processa
                self.aruco_buffer.popleft()
                self.rover_buffer.popleft()
                self.pose_callback(aruco_msg, rover_msg)
            else:
                # Scarta il messaggio più vecchio
                if t_aruco < t_rover:
                    self.aruco_buffer.popleft()
                else:
                    self.rover_buffer.popleft()

    def save_poses_to_yaml(self):
        waypoints = []
        for idx, pose in enumerate(self.received_poses):
            qx = pose.pose.orientation.x
            qy = pose.pose.orientation.y
            qz = pose.pose.orientation.z
            qw = pose.pose.orientation.w
            roll, pitch, yaw = euler_from_quaternion([qx, qy, qz, qw])
            goal = {
                "name": f"goal{idx + 1}",
                "position": {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "z": 0.0
                },
                "orientation": {
                    "roll": roll,
                    "pitch": pitch,
                    "yaw": yaw
                }
            }
            waypoints.append(goal)

        data = {"waypoints": waypoints}
        package_path = get_package_share_directory('rover_bringup')
        file_path = os.path.join(package_path, 'config', 'waypoints.yaml')

        with open(file_path, "w") as outfile:
            yaml.dump(data, outfile, default_flow_style=False)

        self.get_logger().info("Waypoints salvati su waypoints.yaml")

    def pose_callback(self, aruco_msg: PoseStamped, robot_msg: PoseStamped):
        flag = True
        aruco_x = aruco_msg.pose.position.x
        aruco_y = aruco_msg.pose.position.y
        rov_pos_x= robot_msg.pose.position.x
        rov_pos_y = robot_msg.pose.position.y

        if not self.received_aruco_poses:
            self.get_logger().info('Nessuna posa salvata, salvo la prima.')
        else:
            for i, pose in enumerate(self.received_aruco_poses[:]):
                saved_x = pose.pose.position.x
                saved_y = pose.pose.position.y
                distanza = math.sqrt((aruco_x - saved_x)**2 + (aruco_y - saved_y)**2)
                self.get_logger().info(f"Distanza dalla posa salvata: {distanza:.3f} m")

                if distanza <= 1.0:
                    flag = False
                    self.get_logger().info(f"Pose troppo vicina, non la salvo.")
                    distanza_nuova=math.sqrt((aruco_x - rov_pos_x)**2 + (aruco_y - rov_pos_y)**2)
                    if distanza_nuova <= distanza:
                        flag=True
                        del self.received_aruco_poses[i]
                        del self.received_poses[i]
                        break
                        

                    

        if flag:
            self.received_poses.append(robot_msg)
            self.received_aruco_poses.append(aruco_msg)
            self.save_poses_to_yaml()
            self.get_logger().info(f"Aruco salvato in posizione nuova. Robot era in ({robot_msg.pose.position.x:.2f}, {robot_msg.pose.position.y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()