#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import  tf_transformations
from ebot_docking.srv import DockSw              # Import custom service message
import math
from linkattacher_msgs.srv import AttachLink     # Import link attacher service
from linkattacher_msgs.srv import DetachLink     # Import link detacher service
from nav2_msgs.srv import ClearCostmapAroundRobot
from example_interfaces.msg import Int16


# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')

        self.count = 0

        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to imu data for robot orientation information
        self.imu_sub = self.create_subscription(Imu, 'imu', self.imu_callback, 10)


        # Subscribe to odometry data for robot pose information
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements (for left)
        self.ultrasonic_rl_sub = self.create_subscription(Range, 'ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)

        # Subscribe to ultrasonic sensor data for distance measurements (for right)
        self.ultrasonic_rr_sub = self.create_subscription(Range, 'ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)

        # Create a publisher for sending status of the robot to the arm
        self.status_publisher_ = self.create_publisher(Int16, "status", 10)

        # Create a service client for the clear local costmap service
        self.clear_local_cli = self.create_client(ClearCostmapAroundRobot, '/local_costmap/clear_around_local_costmap')

        # Create a service client for the clear global costmap service
        self.clear_global_cli = self.create_client(ClearCostmapAroundRobot, '/global_costmap/clear_around_global_costmap')

        while not self.clear_local_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Clear Local cost map service not available, waiting again...')

        while not self.clear_global_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Clear Global cost map service not available, waiting again...')

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
        self.target_orientation = 0.0
        self.robot_pose = [0.0, 0.0]                # x,y
        self.current_yaw = 0.0                      # yaw
        self.usrleft_value = 0.0
        self.usrright_value = 0.0
        self.target_rack_name = None

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)
    

    def imu_callback(self,msg):
        orientation_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(orientation_list)
        #print(f"from sim. topic is: {self.current_yaw}, from hardware topic: {self.current_yaw + 3.14}")
        print(f"yaw from imu topic is: {self.current_yaw}")


    # Callback function for odometry data
    def odometry_callback(self, msg):
        # Extract and update robot pose information from odometry message
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
    

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
        #print(f"value of counter: {self.count}")

        if self.is_docking:

            #print(f"value of count: {self.count}")

            # reach rack3
            
            # if self.count == 0:

            #     if not self.dock_aligned:

            #         # Implement control logic for orientation change
            #         # print(f"target orientation: {self.target_orientation} \n")
            #         # print(f"target rack: {self.target_rack_name} \n")
            #         # print(f"current yaw: {self.current_yaw} \n")
            #         diff = self.target_orientation - self.current_yaw
            #         # print(f"orientation difference: {diff} \n")
            #         diff = self.normalize_angle(diff)
            #         #print(f"normalized orientation difference: {diff} \n")
            #         target_orient = Twist()

            #         if abs(diff) > 0.02:

            #             target_orient.angular.z = diff
            #             self.cmd_vel_publisher_.publish(target_orient)

            #         else:

            #             # Orientation aligned, set the flag
            #             self.dock_aligned = True
            #             self.get_logger().info("Orientation aligned!")

            #             # Optional: Add a delay if needed before linear movement
            #             time.sleep(1)

            #     elif self.dock_aligned:

            #         # Robot is oriented correctly, now implement control logic for linear motion along x

            #         target_distance = (self.usrleft_value + self.usrright_value) / 2.0
            #         #print(f"target_distance: {target_distance}")
            #         target_vel = Twist()

            #         if target_distance > 0.15:

            #             target_vel.linear.x -= 0.4
            #             self.cmd_vel_publisher_.publish(target_vel)
                    
            #         else:

            #             # Linear position aligned, stop the robot
            #             target_vel.linear.x = 0.0                 
            #             self.cmd_vel_publisher_.publish(target_vel)
            #             # Docking process complete, set flags accordingly
            #             self.is_docking = False
            #             self.dock_aligned = True
            #             #self.get_logger().info("Linear position aligned, docking complete!")
               

            #    #bring rack3 towards arm
                        
            # elif self.count == 1:

            #     if not self.dock_aligned:

            #         # Implement control logic for orientation change
            #         #print(f"target orientation: {self.target_orientation} \n")
            #         # print(f"target rack: {self.target_rack_name} \n")
            #         # print(f"current yaw: {self.current_yaw} \n")
            #         diff = self.target_orientation - self.current_yaw
            #         # print(f"orientation difference: {diff} \n")
            #         diff = self.normalize_angle(diff)
            #         #print(f"normalized orientation difference: {diff} \n")
            #         target_orient = Twist()

            #         if abs(diff) > 0.02:

            #             target_orient.angular.z = diff
            #             self.cmd_vel_publisher_.publish(target_orient)

            #         else:

            #             # Orientation aligned, set the flag
            #             self.dock_aligned = True
            #             #self.get_logger().info("Orientation aligned!")

            #             # Optional: Add a delay if needed before linear movement
            #             time.sleep(1)


            #     elif self.dock_aligned:

            #         # Robot is oriented correctly, now implement control logic for linear motion along x
            #         print("moving rack3 towards arm")

            #         dist_x = 1.6 - self.robot_pose[0]   
            #         dist_y = -1.75 - self.robot_pose[1] 

            #         distance = math.sqrt(dist_x**2 + dist_y**2)
            #         #print(f"dist_x: {dist_x}")
            #         #print(f"dist_y: {dist_y}")
            #         print(f"distance: {distance}")
            #         target_vel = Twist()


            #         if dist_x > 0.0:

            #             target_vel.linear.x = (-2.0) * distance
            #             self.cmd_vel_publisher_.publish(target_vel)
                    
            #         else:

            #             # Linear position aligned, stop the robot
            #             target_vel.linear.x = 0.0                 
            #             self.cmd_vel_publisher_.publish(target_vel)
            #             # Docking process complete, set flags accordingly
            #             self.is_docking = False
            #             self.dock_aligned = True
            #             #self.get_logger().info("Linear position aligned, docking complete!")
            

            # reach rack1 
            
            if self.count == 0:

                #print("moving  towards rack1")

                if not self.dock_aligned:

                    # Implement control logic for orientation change

                    # print(f"target orientation: {self.target_orientation} \n")
                    # print(f"target rack: {self.target_rack_name} \n")
                    # print(f"current yaw: {self.current_yaw} \n")
                    diff = self.target_orientation - self.current_yaw
                    # print(f"orientation difference: {diff} \n")
                    diff = self.normalize_angle(diff)
                    #print(f"normalized orientation difference: {diff} \n")
                    target_orient = Twist()

                    if abs(diff) > 0.02:

                        target_orient.angular.z = diff 
                        self.cmd_vel_publisher_.publish(target_orient)

                    else:
                        
                        # Orientation aligned, set the flag
                        self.dock_aligned = True
                        self.get_logger().info("Orientation aligned!")

                        # Optional: Add a delay if needed before linear movement
                        #time.sleep(0.5)

                elif self.dock_aligned:

                    # Robot is oriented correctly, now implement control logic for linear motion along x

                    target_distance = (self.usrleft_value + self.usrright_value) / 2.0
                    #print(f"target_distance: {target_distance}")
                    target_vel = Twist()

                    if target_distance > 0.15:

                        target_vel.linear.x -= 0.4
                        self.cmd_vel_publisher_.publish(target_vel)
                    
                    else:

                        # Linear position aligned, stop the robot
                        target_vel.linear.x = 0.0                 
                        self.cmd_vel_publisher_.publish(target_vel)
                        # Docking process complete, set flags accordingly
                        self.is_docking = False
                        self.dock_aligned = True

                        self.get_logger().info("Linear position aligned, docking complete!")
                        

            # bring rack1 towards arm

            elif self.count == 1:

                # msg = Int16()

                if not self.dock_aligned:

                    # Implement control logic for orientation change

                    #self.get_logger().info(f"self.orientation: {self.current_yaw}")
                    diff = self.target_orientation - self.current_yaw
                    diff = self.normalize_angle(diff)
                    #print(f"normalized angle diff: {diff}")
                    target_orient = Twist()

                    if abs(diff) > 0.01:

                        target_orient.angular.z = diff
                        self.cmd_vel_publisher_.publish(target_orient)

                    else:
                         
                        # Orientation aligned, set the flag
                        self.dock_aligned = True
                        self.get_logger().info("Orientation aligned!")
                        # Optional: Add a delay if needed before linear movement
                        #time.sleep(0.5)


                elif self.dock_aligned:

                    # Robot is oriented correctly, now implement control logic for linear motion along x

                    dist_x = 0.82 - self.robot_pose[0]   
                    dist_y = -2.455 - self.robot_pose[1] 

                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    #print(f"dist_x: {dist_x}")
                    #print(f"dist_y: {dist_y}")
                    #print(f"distance left: {distance}")
                    target_vel = Twist()


                    if dist_x > 0.01:

                        target_vel.linear.x -= distance*0.6
                        self.cmd_vel_publisher_.publish(target_vel)
                    
                    else: 
                         
                        # Linear position aligned, stop the robot
                        target_vel.linear.x = 0.0                 
                        self.cmd_vel_publisher_.publish(target_vel)
                        # Docking process complete, set flags accordingly
                        self.is_docking = False
                        self.dock_aligned = True
                        self.get_logger().info("Linear position aligned, docking complete!")
            

            # towards rack2

            elif self.count == 2:

                if not self.dock_aligned:

                    # Implement control logic for orientation change

                    #print(f"target orientation for rack2: {self.target_orientation} \n")
                    diff = self.target_orientation - self.current_yaw
                    # print(f"orientation difference: {diff} \n")
                    diff = self.normalize_angle(diff)
                    #print(f"normalized orientation difference: {diff} \n")
                    target_orient = Twist()

                    if abs(diff) > 0.02:

                        target_orient.angular.z = diff
                        self.cmd_vel_publisher_.publish(target_orient)

                    else:
                        
                        # Orientation aligned, set the flag
                        self.dock_aligned = True
                        self.get_logger().info("Orientation aligned!")

                        # Optional: Add a delay if needed before linear movement
                        #time.sleep(0.5)

                elif self.dock_aligned:

                    # Robot is oriented correctly, now implement control logic for linear motion along x

                    target_distance = (self.usrleft_value + self.usrright_value) / 2.0
                    #print(f"target_distance: {target_distance}")
                    target_vel = Twist()

                    if target_distance > 0.15:

                        target_vel.linear.x -= 0.4
                        self.cmd_vel_publisher_.publish(target_vel)
                    
                    else:

                        # Linear position aligned, stop the robot
                        target_vel.linear.x = 0.0                 
                        self.cmd_vel_publisher_.publish(target_vel)
                        # Docking process complete, set flags accordingly
                        self.is_docking = False
                        self.dock_aligned = True

                        # self.attach_link_to_robot(self.target_rack_name)
                        # print("calling clear map service")
                        # self.clear_local_map()
                        # self.clear_global_map()
                        self.get_logger().info("Linear position aligned, docking complete!")
            

           # bringing rack2 towards arm

            elif self.count == 3:


                if not self.dock_aligned:

                    # Implement control logic for orientation change

                    diff = self.target_orientation - self.current_yaw
                    diff = self.normalize_angle(diff)
                    #print(f"normalized angle diff: {diff}")
                    target_orient = Twist()

                    if abs(diff) > 0.02:

                        target_orient.angular.z = diff
                        self.cmd_vel_publisher_.publish(target_orient)

                    else:
                        
                        #target_orient.angular.z = 0.0
                        # Orientation aligned, set the flag
                        self.dock_aligned = True
                        self.get_logger().info("Orientation aligned!")
                        # Optional: Add a delay if needed before linear movement
                        #time.sleep(0.5)


                elif self.dock_aligned:

                    # Robot is oriented correctly, now implement control logic for linear motion along x

                    dist_x = 1.65 - self.robot_pose[0]   
                    dist_y = -2.455 - self.robot_pose[1] 

                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    #print(f"dist_x: {dist_x}")
                    #print(f"dist_y: {dist_y}")
                    print(f"distance left for rack2: {distance}")
                    target_vel = Twist()


                    if dist_x > 0.0:

                        target_vel.linear.x -= distance * 0.2
                        self.cmd_vel_publisher_.publish(target_vel)
                    
                    else:

                        # Linear position aligned, stop the robot
                        target_vel.linear.x = 0.0                 
                        self.cmd_vel_publisher_.publish(target_vel)
                        # Docking process complete, set flags accordingly
                        self.is_docking = False
                        self.dock_aligned = True
                        self.get_logger().info("Linear position aligned, docking complete!")

            else:
                
                target_vel = Twist()
                target_vel.linear.x = 0.0                  
                self.cmd_vel_publisher_.publish(target_vel)


    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):

        msg = Int16()

        # Extract desired docking parameters from the service request
        self.linear_dock = request.linear_dock
        self.orientation_dock = request.orientation_dock
        self.target_orientation = request.orientation
        self.target_rack_name = request.rack_no
        # print(f"initial value of self.count: {self.count}")
        # self.get_logger().info(f"initial state flags of is_docking: {self.is_docking} & dock_aligned: {self.is_docking}")
        # self.get_logger().info(f"target rack name: {self.target_rack_name} & orient by: {self.target_orientation}")


        # Reset flags and start the docking process
        self.is_docking = True
        self.dock_aligned = False
        # self.get_logger().info(f"final state flags of is_docking: {self.is_docking} & dock_aligned: {self.is_docking}")

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
        print(f"next value of self.count: {self.count}")
        
        if self.count == 1:             # towards rack1

            self.attach_link_to_robot(self.target_rack_name)
            msg.data = 0
            self.status_publisher_.publish(msg)
            # print("calling clear map service")
            # self.clear_local_map()
            # self.clear_global_map()


        elif self.count == 2:                # bring rack1 towards arm
            
            msg.data = 1     
            self.status_publisher_.publish(msg)
            self.detach_link_to_robot(self.target_rack_name)
            #print(f"robot stopped: {msg.data}")
            

        elif self.count == 3:               # towards rack2

            self.attach_link_to_robot(self.target_rack_name)
            msg.data = 0
            self.status_publisher_.publish(msg)
            
            # print("calling clear map service")
            # self.clear_local_map()
            # self.clear_global_map()
        

        elif self.count == 4:                # bring rack2 towards arm
            
            msg.data = 1
            self.status_publisher_.publish(msg)
            self.detach_link_to_robot(self.target_rack_name)
            # print(f"robot stopped: {msg.data}")
        
        else:

            msg.data = 0
            self.status_publisher_.publish(msg)

        return response
    
    
    def clear_local_map(self):

        req = ClearCostmapAroundRobot.Request()
        req.reset_distance = 0.4
        future = self.clear_local_cli.call_async(req)
        while rclpy.ok():
            if future.done():
                if future.result() is not None:
                    self.get_logger().info('Local Costmap cleared')
                    # return True
                else:
                    self.get_logger().error('Failed...')
                break
            time.sleep(0.1)
    

    def clear_global_map(self):

        req = ClearCostmapAroundRobot.Request()
        req.reset_distance = 1.0
        future = self.clear_global_cli.call_async(req)
        while rclpy.ok():
            if future.done():
                if future.result() is not None:
                    self.get_logger().info('Global Costmap cleared')
                    # return True
                else:
                    self.get_logger().error('Failed...')
                break
            time.sleep(0.1)


    def attach_link_to_robot(self, rack_name):
        # Call the link attacher service
        req = AttachLink.Request()
        req.model1_name = 'ebot'
        req.link1_name = 'ebot_base_link' 
        req.model2_name = rack_name                                
        req.link2_name = 'link'
        future = self.link_attach_cli.call_async(req)

        while rclpy.ok():
            if future.done():
                if future.result() is not None:
                    self.get_logger().info('Link attached successfully.')
                    # return True
                else:
                    self.get_logger().error('Failed to attach.')
                break
            time.sleep(0.1)


    def detach_link_to_robot(self, rack_name):
        req = DetachLink.Request()
        req.model1_name = 'ebot'
        req.link1_name = 'ebot_base_link'
        req.model2_name = rack_name                                           
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
    executor = MultiThreadedExecutor(3)
    executor.add_node(my_robot_docking_controller)
    executor.spin()
    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
