#! /usr/bin/env python3

import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # Helper module
 
'''
Navigates a robot through goal poses.
'''
def main():
 
  # Start the ROS 2 Python Client Library
  rclpy.init()
 
 
  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()
 
  # Set the robot's initial pose if necessary
  # initial_pose = PoseStamped()
  # initial_pose.header.frame_id = 'map'
  # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
  # initial_pose.pose.position.x = -6.5
  # initial_pose.pose.position.y = -4.2
  # initial_pose.pose.position.z = 0.0
  # initial_pose.pose.orientation.x = 0.0
  # initial_pose.pose.orientation.y = 0.0
  # initial_pose.pose.orientation.z = 0.0
  # initial_pose.pose.orientation.w = 1.0
  # navigator.setInitialPose(initial_pose)
 
  # Activate navigation, if not autostarted. This should be called after setInitialPose()
  # or this will initialize at the origin of the map and update the costmap with bogus readings.
  # If autostart, you should `waitUntilNav2Active()` instead.
  # navigator.lifecycleStartup()
 
  # Wait for navigation to fully activate. Use this line if autostart is set to true.
  navigator.waitUntilNav2Active()
 
  # If desired, you can change or load the map as well
  # navigator.changeMap('/path/to/map.yaml')
 
  # You may use the navigator to clear or obtain costmaps
  # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
  # global_costmap = navigator.getGlobalCostmap()
  # local_costmap = navigator.getLocalCostmap()
 
  # Set the robot's goal poses
  goal_poses = []
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -7.8
  goal_pose.pose.position.y = 3.197
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 0.707
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 14.0
  goal_pose.pose.position.y = 3.197
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 0.707
  goal_poses.append(goal_pose)
  
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 14.0
  goal_pose.pose.position.y = 1.169
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 0.707
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -7.8
  goal_pose.pose.position.y = 1.176
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 0.707
  goal_poses.append(goal_pose)
   
 
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -7.8
  goal_pose.pose.position.y = -1.176
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 0.707
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 14.0
  goal_pose.pose.position.y = -1.169
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 0.707
  goal_poses.append(goal_pose)
  
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 14.0
  goal_pose.pose.position.y = -3.197
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 0.707
  goal_poses.append(goal_pose)
    
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -7.8
  goal_pose.pose.position.y = -3.197
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 0.707
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = -7.8
  goal_pose.pose.position.y = -5.218
  goal_pose.pose.orientation.z = 1.0
  goal_pose.pose.orientation.w = 0.707
  goal_poses.append(goal_pose)
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 14.0
  goal_pose.pose.position.y = -5.218
  goal_pose.pose.orientation.z = 1.0
  goal_pose.pose.orientation.w = 0.707
  goal_poses.append(goal_pose)
 
  # sanity check a valid path exists
  # path = navigator.getPathThroughPoses(initial_pose, goal_poses)
 
  # Go through the goal poses
  nav_start = navigator.get_clock().now()
  navigator.followWaypoints(goal_poses)
  
  i = 0
  # Keep doing stuff as long as the robot is moving towards the goal poses
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
      print('Executing current waypoint: '+ str(feedback.current_waypoint + 1)+ '/'+ str(len(goal_poses)))
      now = navigator.get_clock().now()
 
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
 
  # Close the ROS 2 Navigation Stack
  navigator.lifecycleShutdown()
 
  exit(0)
 
if __name__ == '__main__':
  main()