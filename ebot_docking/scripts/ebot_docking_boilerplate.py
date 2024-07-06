#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import  tf_transformations
from ebot_docking.srv import DockSw  # Import custom service message
import math
from linkattacher_msgs.srv import AttachLink  # Import link attacher service
from linkattacher_msgs.srv import DetachLink

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        self.count = 0

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements (for left)
        self.ultrasonic_rl_sub = self.create_subscription(Range, 'ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        # Add another one here
        # Subscribe to ultrasonic sensor data for distance measurements (for right)
        self.ultrasonic_rr_sub = self.create_subscription(Range, 'ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        # Create a service client for the link attacher service
        self.link_attach_cli = self.create_client(AttachLink, '/ATTACH_LINK')

        while not self.link_attach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link attacher service not available, waiting again...')
        
        # Create a service client for the link detacher service
        self.link_detach_cli = self.create_client(DetachLink, '/DETACH_LINK')
        
        while not self.link_detach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Link detacher service not available, waiting again...')

        # Initialize all  flags and parameters here
        self.is_docking = False
        self.dock_aligned = True         
        self.linear_dock = False 
        self.orientation_dock = False
        self.orientation = 0.0
        self.robot_pose = [0.0, 0.0, 0.0]   # x,y,yaw
        self.usrleft_value = 0.0
        self.usrright_value = 0.0

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    # Callback function for the left ultrasonic sensor
    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    # Callback function for the right ultrasonic sensor
    def ultrasonic_rr_callback(self,msg):
        self.usrright_value = msg.range

    # Utility function to normalize angles within the range of -π to π (OPTIONAL)
    def normalize_angle(self, angle):
        if angle > math.pi:
            angle -= 2*math.pi
        elif angle < -math.pi:
            angle += 2*math.pi

        return angle

    # Main control loop for managing docking behavior

    def controller_loop(self):

        # The controller loop manages the robot's linear and angular motion control to achieve docking alignment and execution

        if self.is_docking:
            if self.robot_pose[1]>0:
        # Check if orientation alignment is needed
                if not self.dock_aligned:
                    # Implement control logic for orientation change
                    diff = self.orientation - self.robot_pose[2]
                    diff = self.normalize_angle(diff)
                    target_orient = Twist()
                    if diff > 0.02:
                        target_orient.angular.z = diff
                        self.cmd_vel_publisher_.publish(target_orient)
                    else:
                        # Orientation aligned, set the flag
                        self.dock_aligned = True
                        self.get_logger().info("Orientation aligned!")
                        # Optional: Add a delay if needed before linear movement
                        time.sleep(1)

                elif self.dock_aligned:
                    # Robot is oriented correctly, now implement control logic for linear motion along x
                    target_distance = (self.usrleft_value + self.usrright_value) / 2.0
                    target_vel = Twist()
                    if target_distance > 0.2:
                        target_vel.linear.x -= 0.5
                        self.cmd_vel_publisher_.publish(target_vel)
                    else:
                        # Linear position aligned, stop the robot
                        target_vel.linear.x = 0.0
                        self.cmd_vel_publisher_.publish(target_vel)
                        # Docking process complete, set flags accordingly
                        self.is_docking = False
                        self.dock_aligned = True
                        self.get_logger().info("Linear position aligned, docking complete!")

            elif self.robot_pose[1]<0:
                if not self.dock_aligned:
                    # Implement control logic for orientation change
                    diff = self.orientation - self.robot_pose[2]
                    diff = self.normalize_angle(diff)
                    target_orient = Twist()
                    if abs(diff) > 0.02:
                        target_orient.angular.z = diff
                        self.cmd_vel_publisher_.publish(target_orient)
                    else:
                        # Orientation aligned, set the flag
                        self.dock_aligned = True
                        self.get_logger().info("Orientation aligned!")
                        # Optional: Add a delay if needed before linear movement
                        time.sleep(1)

                elif self.dock_aligned:
                    # Robot is oriented correctly, now implement control logic for linear motion along x
                    dist_x = 0.5 - self.robot_pose[0]
                    print(f"dist_x: {dist_x}")
                    dist_y = -2.455 - self.robot_pose[1]
                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    #print(distance)
                    target_vel = Twist()
                    if dist_x > 0.2:
                        target_vel.linear.x -= distance*0.25
                        self.cmd_vel_publisher_.publish(target_vel)
                    else:
                        # Linear position aligned, stop the robot
                        target_vel.linear.x = 0.0
                        self.cmd_vel_publisher_.publish(target_vel)
                        # Docking process complete, set flags accordingly
                        self.is_docking = False
                        self.dock_aligned = True
                        self.get_logger().info("Linear position aligned, docking complete!")


    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):

        # Extract desired docking parameters from the service request
        self.linear_dock = request.linear_dock
        self.orientation_dock = request.orientation_dock
        self.orientation = request.orientation
        # Reset flags and start the docking process
        self.is_docking = True
        self.dock_aligned = False

        # Log a message indicating that docking has started
        self.get_logger().info("Docking started!")

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())

        # Wait until the robot is aligned for docking
        while not self.dock_aligned:
            self.get_logger().info("Waiting for alignment...")
            rate.sleep()

        # Set the service response indicating success
        response.success = True
        response.message = "Docking control initiated"
        time.sleep(5)
        self.count += 1
        if self.count == 1:
            self.attach_link_to_robot()
        else:
            self.detach_link_to_robot()
        return response
    
    def attach_link_to_robot(self):
        # Call the link attacher service
        req = AttachLink.Request()
        req.model1_name = 'ebot'
        req.link1_name = 'ebot_base_link'
        req.model2_name = 'rack1'  
        req.link2_name = 'link'
        future = self.link_attach_cli.call_async(req)

        while rclpy.ok():
            if future.done():
                if future.result() is not None:
                    self.get_logger().info('Link attached successfully.')
                else:
                    self.get_logger().error('Failed to attach.')
                break
            time.sleep(0.1)

    def detach_link_to_robot(self):
        req = DetachLink.Request()
        req.model1_name = 'ebot'
        req.link1_name = 'ebot_base_link'
        req.model2_name = 'rack1'  
        req.link2_name = 'link'
        future = self.link_detach_cli.call_async(req)

        while rclpy.ok():
            if future.done():
                if future.result() is not None:
                    self.get_logger().info('Link detached successfully.')
                else:
                    self.get_logger().error('Failed to detach.')
                break
            time.sleep(0.1)


# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()