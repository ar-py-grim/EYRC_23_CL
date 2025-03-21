#!/usr/bin/env python3
"""
Example of using MoveIt 2 Servo to perform a circular motion in the yz plane,
including a service call to /servo_node/start_servo before sending Twist commands.

Usage:
    ros2 run pymoveit2 ex_servo_yz_with_service.py
"""

import math
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import Trigger
from pymoveit2.robots import ur5

# Global twist message and parameters
__twist_msg = TwistStamped()
__twist_msg.header.frame_id = ur5.base_link_name()
radius = 0.01        # Radius of circular motion in meters
angular_speed = 0.5  # Angular speed in radians per second
start_time = time.time()

def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_servo")
    callback_group = ReentrantCallbackGroup()

    # Publisher for TwistStamped commands
    twist_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    # Create a service client for /servo_node/start_servo
    servo_client = node.create_client(Trigger, "/servo_node/start_servo", callback_group=callback_group)

    # Wait for the service to become available
    node.get_logger().info("Waiting for /servo_node/start_servo service...")
    while not servo_client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info("Still waiting for /servo_node/start_servo service...")

    node.get_logger().info("Service /servo_node/start_servo is now available.")

    # Call the service
    request = Trigger.Request()
    future = servo_client.call_async(request)

    # Spin until we get a response
    rclpy.spin_until_future_complete(node, future)

    # Check the service response
    if future.result() is not None and future.result().success:
        node.get_logger().info(f"Servo started successfully: {future.result().message}")
    else:
        node.get_logger().error("Failed to start servo. Aborting.")
        rclpy.shutdown()
        return

    def servo_circular_motion():
        """Move in a circular motion in the yz plane using Servo."""
        elapsed = (time.time() - start_time)
        phase = (elapsed * angular_speed) % (2.0 * math.pi)
        vy = radius * math.cos(phase)
        vz = radius * math.sin(phase)
        __twist_msg.header.stamp = node.get_clock().now().to_msg()
        __twist_msg.twist.linear.x = 0.0
        __twist_msg.twist.linear.y = vy
        __twist_msg.twist.linear.z = vz

        # Optional: update angular velocity if you want rotation
        # __twist_msg.twist.angular.z = angular_speed

        twist_pub.publish(__twist_msg)
        node.get_logger().info(f"Publishing twist: linear=(0.0, {vy:.2f}, {vz:.2f})")

    # Create a timer to call servo_circular_motion at 50 Hz (every 0.02 seconds)
    node.create_timer(0.02, servo_circular_motion, callback_group=callback_group)

    # Use a multi-threaded executor to spin the node.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node due to KeyboardInterrupt")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
