#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator,position_x,position_y,orientation_z):
    q_x,q_y,q_z,q_w = tf_transformations.quaternion_from_euler(0.0,0.0,orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def main():
    rclpy.init()
    nav = BasicNavigator()
    
    # set initial pose
    initial_pose = create_pose_stamped(nav,0.0,0.0,0.0)
    nav.setInitialPose(initial_pose)

    # wait for nav2 to be active
    nav.waitUntilNav2Active()

    # set NAV2 waypoint follow
    goal_pose1 = create_pose_stamped(nav, 1.8, 1.5, 1.57)         # P1
    goal_pose2 = create_pose_stamped(nav, 2.0, -7.0, -1.57)       # P2 
    goal_pose3 = create_pose_stamped(nav, -3.0, 2.5, 1.57)        # P3
    goal_pose4 = create_pose_stamped(nav, -1.0, -5.0, 1.57)       # P4
    #goal_pose5 = create_pose_stamped(nav, -1.0, -8.0, 1.57)      # P5
    waypoints = [goal_pose1, goal_pose2, goal_pose3,goal_pose4] 

    for waypoint in waypoints:
        # Navigate to the current waypoint
        nav.goToPose(waypoint)
        # Wait for the robot to reach the waypoint
        while not nav.isTaskComplete():
            rclpy.spin_once(nav)

    rclpy.shutdown()

if __name__ == "__main__":
    main()