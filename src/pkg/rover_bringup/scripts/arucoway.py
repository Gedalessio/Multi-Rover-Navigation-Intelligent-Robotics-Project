#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from collections import deque
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration


class ReactiveWaypointFollower(Node):
    def __init__(self):
        super().__init__('reactive_waypoint_follower')

        self.navigator = BasicNavigator(namespace='slave')
        self.navigator.waitUntilNav2Active(localizer='smoother_server')

        self.waypoint_queue = deque()
        self.is_navigating = False
        self.returning = False

        # Posizione fissa a cui tornare
        self.home_pose = PoseStamped()
        self.home_pose.header.frame_id = 'global'
        self.home_pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        self.home_pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # Sottoscrizione al topic dei waypoint
        self.create_subscription(PoseStamped, '/master/discovered_waypoints', self.waypoint_callback, 10)

        # Timer per gestire i waypoint
        self.create_timer(0.5, self.check_and_navigate)

    def waypoint_callback(self, msg):
        self.get_logger().info(f"[Nuovo waypoint] x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}")
        self.waypoint_queue.append(msg)

    def check_and_navigate(self):
        if not self.is_navigating and self.waypoint_queue:
            next_goal = self.waypoint_queue.popleft()
            self.start_navigation(next_goal)

    def start_navigation(self, pose):
        self.get_logger().info(f"[Inizio navigazione] Verso x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
        self.navigator.goToPose(pose)
        self.is_navigating = True
        self.returning = False
        self.nav_start_time = self.navigator.get_clock().now()
        self.create_timer(1.0, self.check_nav_status)

    def start_return_to_home(self):
        self.get_logger().info("[Ritorno] Inizio ritorno alla posizione fissa x=-0.6, y=0.0")
        self.navigator.goToPose(self.home_pose)
        self.is_navigating = True
        self.returning = True
        self.nav_start_time = self.navigator.get_clock().now()

    def check_nav_status(self):
        if not self.is_navigating:
            return

        if self.navigator.isTaskComplete():
            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                if not self.returning:
                    self.get_logger().info('[Raggiunto] Waypoint completato con successo. Attendo 3 secondi...')
                    self.is_navigating = False
                    # Timer di 3 secondi prima del ritorno
                    self.create_timer(8.0, self.delayed_return)
                else:
                    self.get_logger().info('[Ritorno completato] Tornato alla posizione iniziale fissa.')
                    self.is_navigating = False
            elif result == TaskResult.CANCELED:
                self.get_logger().warn('[Annullato] Goal cancellato.')
                self.is_navigating = False
            elif result == TaskResult.FAILED:
                self.get_logger().error('[Fallito] Goal non riuscito.')
                self.is_navigating = False
            else:
                self.get_logger().warn('[Errore] Stato di ritorno non valido.')
                self.is_navigating = False
        else:
            now = self.navigator.get_clock().now()
            if now - self.nav_start_time > Duration(seconds=1000):
                self.navigator.cancelTask()
                self.get_logger().warn('[Timeout] Navigazione troppo lunga, annullata.')
                self.is_navigating = False

    def delayed_return(self):
        if not self.is_navigating:
            self.start_return_to_home()


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveWaypointFollower()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
