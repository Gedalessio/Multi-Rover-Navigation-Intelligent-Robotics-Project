#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from tf_transformations import quaternion_from_euler
from ament_index_python.packages import get_package_share_directory
import os
import yaml



waypoints_yaml_path=os.path.join(get_package_share_directory('rover_bringup'), "config", "waypoints.yaml")
with open(waypoints_yaml_path, 'r') as yaml_file:
        yaml_content = yaml.safe_load(yaml_file)

# waypoints = yaml.safe_load('''
# waypoints:
#   - position:
#       x: 2.0
#       y: 0.0
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: -0.0055409271259092485
#       w: 0.9999846489454652
#   - position:
#       x: -1.8789787292480469
#       y: 0.0
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.010695864295550759
#       w: 0.9999427976074288
#   - position:
#       x: 0.0
#       y: 0.6118782758712769
#       z: 0.0
#     orientation:
#       x: 0.0
#       y: 0.0
#       z: 0.01899610435153287
#       w: 0.9998195577300264
# ''')

def main():
    rclpy.init()
    navigator = BasicNavigator(namespace='slave')

    def create_pose(transform):
        name= transform["name"]
        pose = PoseStamped()
        pose.header.frame_id = 'global'
        pose.header.stamp = navigator.get_clock().now().to_msg()
        pose.pose.position.x = transform["position"]["x"]
        pose.pose.position.y = transform["position"]["y"]
        pose.pose.position.z = transform["position"]["z"]
        roll = transform["orientation"]["roll"]
        pitch = transform["orientation"]["pitch"]
        yaw = transform["orientation"]["yaw"]
        quaternion = quaternion_from_euler(roll, pitch, yaw)

        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose, name
        # return pose


    # goals = list(map(create_pose, yaml_content["waypoints"]))
    # goal_poses = []

    # strategy=yaml_content["mode"][0]["strategy"]
    # print(f"La strategia è: {strategy}")
    
    # if strategy == "path":
    #     target_name1= "Goal_3"
    #     for pose, name in goals:
    #         if name == target_name1:
    #             goal_poses.append(pose)
    
    #     target_name2= "Goal_4"
    #     for pose, name in goals:
    #         if name == target_name2:
    #             goal_poses.append(pose)

    #     target_name3= "Goal_2"
    #     for pose, name in goals:
    #         if name == target_name3:
    #             goal_poses.append(pose)

    #     target_name4= "Goal_1"
    #     for pose, name in goals:
    #         if name == target_name4:
    #             goal_poses.append(pose)

    # else:
    #     target_nameex= "explore"
    #     for pose, name  in goals:
    #         if name == target_nameex:
    #             goal_poses.append(pose)   
    # 
         
    goals = list(map(create_pose, yaml_content["waypoints"]))
    # goal_poses=goals 
    goal_poses = []

    # # Aggiungi tutte le pose, ignorando la strategia
    goal_poses = [pose for pose, name in goals]

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer="smoother_server")

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    nav_start = navigator.get_clock().now()
    navigator.followWaypoints(goal_poses)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()

        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' +
                  str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
            now = navigator.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()