#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, MapMetaData
from rclpy.qos import QoSProfile, DurabilityPolicy

class MapRelayNode(Node):
    def __init__(self):
        super().__init__('map_relay_node')

        # QoS compatibile con map_server e amcl: TRANSIENT_LOCAL
        transient_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Sottoscrizione alla mappa e metadati dal master
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/master/map',
            self.map_callback,
            transient_qos
        )
        self.map_metadata_sub = self.create_subscription(
            MapMetaData,
            '/master/map_metadata',
            self.map_metadata_callback,
            transient_qos
        )

        # Publisher per slave con QoS TRANSIENT_LOCAL
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/slave/map',
            transient_qos
        )
        self.map_metadata_pub = self.create_publisher(
            MapMetaData,
            '/slave/map_metadata',
            transient_qos
        )

        self.get_logger().info("Map relay node started. Bridging /master/map → /slave/map")

    def map_callback(self, msg):
        self.map_pub.publish(msg)

    def map_metadata_callback(self, msg):
        self.map_metadata_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MapRelayNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
