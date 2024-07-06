#!/usr/bin/env python3

import math
from copy import deepcopy
#from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
import rclpy
import time
from std_msgs.msg import Float64MultiArray
from example_interfaces.msg import Int16
import tf2_ros
from rclpy.node import Node
#from threading import Thread
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import TwistStamped
from pymoveit2.robots import ur5
from linkattacher_msgs.srv import AttachLink
from linkattacher_msgs.srv import DetachLink



class ServoMotionNode(Node):

    def __init__(self):

        super().__init__("test_servo")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        self.index_pub = self.create_publisher(Int16, "index",10)

        # recieve aruco id and coordinates, published from aruco_task4c.py
        self.tf_subscriber = self.create_subscription(Float64MultiArray,"aruco_tf_array",self.callback_tf_array,10)
        self.__twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        #self.status_sub = self.create_subscription(Int16, "status", self.callback_status,10)

        # Create MoveIt 2 interface

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group)
        

        # Initialize message based on passed arguments
        self.__twist_msg = TwistStamped()
        self.__twist_msg.header.frame_id = ur5.base_link_name()
        self.linear_speed_scale = 1.0
        self.angular_speed_scale = 1.0
        self.__twist_msg.twist.linear.x = self.linear_speed_scale
        self.__twist_msg.twist.linear.y = self.linear_speed_scale
        self.__twist_msg.twist.linear.z = self.linear_speed_scale
        self.__twist_msg.twist.angular.x = self.angular_speed_scale
        self.__twist_msg.twist.angular.y = self.angular_speed_scale
        self.__twist_msg.twist.angular.z = self.angular_speed_scale

        # Parameters

        self.initial_tool0_to_base_link_tf = None
        self.distance_threshold = 0.08
        self.distance_back_threshold = 0.2
        self.distance_st_threshold = 0.01
        #self.status = None
        self.index = 0
        self.count = 0
        self.box_names = None
        self.target_positions1 = None
        self.target_positions2 = None
        self._is_servoing = True
        self.create_timer(0.02, self.execute)                           # 50 Hz
        self.state = "initial"
        self.home_pickpos = [0.0, -2.398, 2.43, -3.15, -1.58, 3.15]
        self.drop_pos = [-0.027, -1.803, -1.3658, -3.039, -1.52, 3.15]
        self.drop_pos2 = [- 5.299702279648599e-05, -2.35, -0.71, -3.14, -1.588, 3.14]
    
    # def callback_status(self,msg):

    #     self.status = msg.data
    #     print(f"status of bot: {self.status}")

    def callback_tf_array(self,msg):

        #self.box_count = int(msg.data[len(msg.data)-1])
        #print(f" no. of boxes: {self.box_count}")

        if len(msg.data)>0:
            
            self.box_names = [f'box{int(msg.data[0])}']
            self.target_positions1 = [msg.data[1:4]]
            self.target_positions2 = [msg.data[4:8]]
            self.count = int(msg.data[len(msg.data)-1])
            print(f"targte_pos1: {self.target_positions1} and target_pos2: {self.target_positions2} for box_name: {self.box_names} \n")


    def get_tool0_tf(self):

        """Get the current position of the tool0 frame"""

        try:
            tf = self.tf_buffer.lookup_transform(ur5.base_link_name(), ur5.end_effector_name(), time=rclpy.time.Time())
            return tf
        except:
            return None
    

    def calc_distance(self, current_position, target_position):

        return  math.sqrt((current_position.x - target_position[0]) ** 2 +
                          (current_position.y - target_position[1]) ** 2 +
                          (current_position.z - target_position[2]) ** 2)
    

    def calc_x_distance(self, current_position, target_position):

        return  math.sqrt((current_position.x - target_position[0]) ** 2)
    

    def calc_y_distance(self, current_position, target_position):

        return  math.sqrt((current_position.y - target_position[1]) ** 2)
    

    def linear_velocity(self, target_position):

        tool0_tf = self.get_tool0_tf()

        # Check if the initial transform is available
        if tool0_tf is not None:

            vel = [target_position[0]-tool0_tf.transform.translation.x, 
                   target_position[1]-tool0_tf.transform.translation.y,
                   target_position[2]-tool0_tf.transform.translation.z]
            
        return vel
    

    def servo_motion(self,target_position):

        distance = 0.0
        tool0_current_tf = self.get_tool0_tf()
        twist_msg = deepcopy(self.__twist_msg)
        twist_msg.header.stamp = self.get_clock().now().to_msg()

        if tool0_current_tf is not None:

                # self.oriented = True
                distance = self.calc_distance(tool0_current_tf.transform.translation, target_position)
                velocity = self.linear_velocity(target_position)

                if distance >= self.distance_threshold:
                    
                # Move toward the target position
                    twist_msg.twist.linear.x *= velocity[0]
                    twist_msg.twist.linear.y *= velocity[1]
                    twist_msg.twist.linear.z *= velocity[2]
                    twist_msg.twist.angular.x *= 0.0
                    twist_msg.twist.angular.y *= 0.0
                    twist_msg.twist.angular.z *= 0.0
                    self.__twist_pub.publish(twist_msg)
                    #print("Servoing...")
                    self.get_logger().info(f"depth: {distance}")
                    
                else:

                    twist_msg.twist.linear.x *= 0.0
                    twist_msg.twist.linear.y *= 0.0
                    twist_msg.twist.linear.z *= 0.0
                    twist_msg.twist.angular.x *= 0.0
                    twist_msg.twist.angular.y *= 0.0
                    twist_msg.twist.angular.z *= 0.0
                    self.__twist_pub.publish(twist_msg)            
        
        return distance
       


    def x_servo_motion(self,target_position):

        x_distance =  0.0 
        twist_msg = deepcopy(self.__twist_msg)
        tool0_current_tf = self.get_tool0_tf()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        if tool0_current_tf is not None:

                # self.oriented = True
                x_distance = self.calc_x_distance(tool0_current_tf.transform.translation, target_position)
                if x_distance >= self.distance_st_threshold:

                    # Move toward the target position

                    twist_msg.twist.linear.x *= 1.0
                    twist_msg.twist.linear.y *= 0.0
                    twist_msg.twist.linear.z *= 0.0
                    twist_msg.twist.angular.x *= 0.0
                    twist_msg.twist.angular.y *= 0.0
                    twist_msg.twist.angular.z *= 0.0
                    self.__twist_pub.publish(twist_msg)
                    #print("Servoing...")
                    self.get_logger().info(f"x_distnce left: {x_distance}")
                    

                else:

                    twist_msg.twist.linear.x = 0.0
                    twist_msg.twist.linear.y = 0.0
                    twist_msg.twist.linear.z = 0.0
                    twist_msg.twist.angular.x = 0.0
                    twist_msg.twist.angular.y = 0.0
                    twist_msg.twist.angular.z = 0.0
                    self.__twist_pub.publish(twist_msg)
        
        return x_distance
    


    def y_servo_motion(self,target_position):

        y_distance =  0.0 
        twist_msg = deepcopy(self.__twist_msg)
        tool0_current_tf = self.get_tool0_tf()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        if tool0_current_tf is not None:

                # self.oriented = True
                y_distance = self.calc_y_distance(tool0_current_tf.transform.translation, target_position)
                if y_distance >= self.distance_st_threshold:

                    if target_position[1] < 0:

                    # Move toward the target position

                        twist_msg.twist.linear.x *= 0.0
                        twist_msg.twist.linear.y *= -1.2
                        twist_msg.twist.linear.z *= 0.0
                        twist_msg.twist.angular.x *= 0.0
                        twist_msg.twist.angular.y *= 0.0
                        twist_msg.twist.angular.z *= 0.0
                        self.__twist_pub.publish(twist_msg)
                        #print("Servoing...")
                        self.get_logger().info(f"y_distnce left: {y_distance}")

                    else:

                        twist_msg.twist.linear.x *= 0.0
                        twist_msg.twist.linear.y *= 1.2
                        twist_msg.twist.linear.z *= 0.0
                        twist_msg.twist.angular.x *= 0.0
                        twist_msg.twist.angular.y *= 0.0
                        twist_msg.twist.angular.z *= 0.0
                        self.__twist_pub.publish(twist_msg)
                        #print("Servoing...")
                        self.get_logger().info(f"y_distnce left: {y_distance}")
                    

                else:

                    twist_msg.twist.linear.x = 0.0
                    twist_msg.twist.linear.y = 0.0
                    twist_msg.twist.linear.z = 0.0
                    twist_msg.twist.angular.x = 0.0
                    twist_msg.twist.angular.y = 0.0
                    twist_msg.twist.angular.z = 0.0
                    self.__twist_pub.publish(twist_msg)
        
        return y_distance
    
    
    
    def execute(self):
        
        msg1 = Int16()
        msg1.data = self.count
        # print(f"status of bot: {self.status}")
        # if self.status == 1:
        #     msg1.data = self.index

        # elif self.status == 0:

        #     msg1.data = -1
        
        self.index_pub.publish(msg1)
        #print(f"publishing index: {msg1.data}")

        if None in (self.box_names, self.target_positions1, self.target_positions2):
            self.get_logger().warning("Not all necessary data is available. Skipping execution.")
            return
        

        # if len(self.target_positions1) < 3 and len(self.target_positions2) < 3:
        #     return

        for box_name, target_position1, target_position2 in zip(self.box_names,self.target_positions1, self.target_positions2):
        
            if target_position2[0] < 0.3 and target_position2[1] < 0:                   # right rack

                if self.state == "initial":
                    joint_position = [[-1.404, -2.407, 2.44, -3.176, -1.72, 3.187]]
                    self.joint_pos(joint_position, box_name)
                    self.state = "xz_servoing"
            

                elif self.state == "xz_servoing":
                        xz_distance = self.servo_motion(target_position1)
                        if xz_distance <= self.distance_threshold:
                            self.state = "y_servoing"
                    

                elif self.state == "y_servoing":
                    y_distance = self.y_servo_motion(target_position2)

                    if y_distance <= self.distance_st_threshold:
                        self.attach_link_service_client(box_name)
                        time.sleep(1.0)
                        self.joint_pos([self.home_pickpos, self.drop_pos2 ,self.home_pickpos], box_name)
                        #self.joint_pos([self.home_pickpos, self.drop_pos2], box_name)
                        self.state = "finished"
                

                elif self.state == "finished":

                    #print(f"{box_name} is dropped safely...")
                    self.state = "initial"
                    self.index +=1
                    self.box_names = None
                    self.target_positions1 = None
                    self.target_positions2 = None

             
            elif target_position2[0] > 0.3:                                 # front rack
                                                    
                #print("moving to front rack")

                if self.state == "initial":
                    self.joint_pos([self.home_pickpos], box_name)
                    self.state = "xz_servoing"
                

                elif self.state == "xz_servoing":
                    distance = self.servo_motion(target_position1)
                    if distance <= self.distance_threshold:
                        self.state = "x_servoing"
                        #time.sleep(1.0)                  

                elif self.state == "x_servoing":
                        
                        x_distance = self.x_servo_motion(target_position2)
                        if x_distance <= self.distance_st_threshold:
                            #time.sleep(2.0)
                            self.attach_link_service_client(box_name)
                            time.sleep(1.0)
                            self.joint_pos([self.home_pickpos, self.drop_pos,self.home_pickpos], box_name)
                            self.state = "finished"
                                

                elif self.state == "finished":

                    #print(f"{box_name} is dropped safely...")
                    self.state = "initial"
                    self.index += 1
                    self.box_names = None
                    self.target_positions1 = None
                    self.target_positions2 = None

                        
            else:                                                             # left rack 

                if self.state == "initial":        

                    joint_position = [0.645, -2.27, 2.712, -3.6, -0.675, 3.15]
                    self.joint_pos([self.home_pickpos ,joint_position], box_name)
                    self.state = "xz_servoing"
            
                elif self.state == "xz_servoing":
                        
                        xz_distance = self.servo_motion(target_position1)
                        if xz_distance <= self.distance_threshold:
                            self.state = "y_servoing"
                    
                elif self.state == "y_servoing":

                    y_distance = self.y_servo_motion(target_position2)
                    if y_distance <= self.distance_st_threshold:  
                        self.attach_link_service_client(box_name)
                        time.sleep(2.0) 
                        self.joint_pos([self.home_pickpos, self.drop_pos,self.home_pickpos], box_name)
                        self.state = "finished"
            

                elif self.state == "finished":

                    #print(f"{box_name} is dropped safely...")
                    self.state = "initial"
                    self.index +=1
                    self.box_names = None
                    self.target_positions1 = None
                    self.target_positions2 = None



    # def joint_pos(self,joint_positions,box_name):
        
    #     for joint_position in joint_positions:

    #         # Move to joint configuration
    #         self.get_logger().info(f"Moving to joint_position")
    #         self.moveit2.move_to_configuration(joint_positions=joint_position)
    #         time.sleep(5.5)

    #         if joint_position == self.drop_pos or joint_position == self.drop_pos2:
    #             self.get_logger().info("Gripper turning OFF")
    #             self.detach_link_service_client(box_name)
    #             time.sleep(2.0)
    

    def joint_pos(self,joint_positions,box_name):
        
        for joint_position in joint_positions:

            # Move to joint configuration
            self.get_logger().info(f"Moving to joint_position")
            self.moveit2.move_to_configuration(joint_positions=joint_position)
            self.moveit2.wait_until_executed()

            if joint_position == self.drop_pos or joint_position == self.drop_pos2:
                self.get_logger().info("Gripper turning OFF")
                self.detach_link_service_client(box_name)
                time.sleep(2.0)


    def detach_link_service_client(self,box_name):

        gripperoff_control = self.create_client(DetachLink, '/GripperMagnetOFF')
        while not gripperoff_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gripper service not available, waiting again...')

        req = DetachLink.Request()
        # req.model1_name = self.box_names[self.i]              
        req.model1_name = box_name                              # Specify the box name
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'
        gripperoff_control.call_async(req)


    def attach_link_service_client(self, box_name):

        gripperon_control = self.create_client(AttachLink, '/GripperMagnetON')
        while not gripperon_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Gripper service not available, waiting again...')

        req = AttachLink.Request()
        #req.model1_name = self.box_names[self.j]               
        req.model1_name = box_name                              # Specify the box name
        req.link1_name = 'link'
        req.model2_name = 'ur5'
        req.link2_name = 'wrist_3_link'
        gripperon_control.call_async(req)
        
def main():

    rclpy.init()
    node = ServoMotionNode()
    executor = MultiThreadedExecutor(3)
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    # rclpy.spin(node)
    # rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()
