#!/usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		    Cosmo Logistic (CL) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script should be used to implement Task 1A of Cosmo Logistic (CL) Theme (eYRC 2023-24).
*
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:        CL#1691
# Author List:		Arpit Gandhi
# Filename:		    task1a.py
# Functions:      detect_aruco, colorimagecb, depthimagecb, process_image,
#			            [ Comma separated list of functions in this file ]
# Nodes:		      Add your publishing and subscribing node
#                 Example:
#			            Publishing Topics  - /tf
#                 Subscribing Topics - /camera/aligned_depth_to_color/image_raw


################### IMPORT MODULES #######################

from copy import deepcopy
import rclpy
import cv2 
import math
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
import tf_transformations
from sensor_msgs.msg import Image

##################### FUNCTION DEFINITIONS #######################


'''def calculate_rectangle_area(coordinates):

    area = None
    width = None
    print(coordinates)
    p0 = coordinates[0]
    p1 = coordinates[1]
    p2 = coordinates[2]
    p3 = coordinates[3] 

    diag1_mag = np.linalg.norm(np.array(p0) - np.array(p2))
    diag2_mag = np.linalg.norm(np.array(p1) - np.array(p3))
    
    dot_product = np.dot(np.array(p0) - np.array(p2), np.array(p1) - np.array(p3))
    theta = np.arccos(dot_product / (diag1_mag*diag2_mag))

    area = abs(0.5*np.sin(theta)*diag2_mag*diag1_mag)
    width = np.linalg.norm(np.array(p0)-np.array(p1))

    return area, width
    '''


def detect_aruco(image):

  aruco_area_threshold = 1500
  cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
  dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])
  size_of_aruco_m = 0.15
  center_aruco_list = []
  distance_from_rgb_list = []
  angle_aruco_list = []
  width_aruco_list = []
  ids = []

  try:

    gray_image = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    detectorParams = cv2.aruco.DetectorParameters()
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)
    corners, ids,_= detector.detectMarkers(gray_image)
    ids_copy = deepcopy(ids)
    copy_corners = deepcopy(corners)
    cv2.aruco.drawDetectedMarkers(image, corners, ids_copy)
    ids_copy = ids_copy.flatten()

    #print(f"flattened ids:- {ids_copy}, type_flattened ids:- {type(ids_copy)}")

    for (markerCorner, markerID) in zip(corners, ids_copy):

      corners = markerCorner.reshape(4, 2)
      #print(f"zipped markercorner:- {corners}, {type(corners)}")
      #corners = corners.flatten()
      #print(f"flattened markercorner:- {corners}")
      #print(f"1st element markercorner:- {corners[0]}")

      (topLeft, topRight, bottomRight, bottomLeft) = corners
      topRight = (int(topRight[0]), int(topRight[1]))
      bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
      bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
      topLeft = (int(topLeft[0]), int(topLeft[1]))

      width = math.dist(topLeft,topRight)
      length = math.dist(topRight , bottomRight)
      width_aruco_list.append(width)
      area = width*length

      # if area < aruco_area_threshold:
      #   np.delete(ids_copy , markerID)
      #   np.delete(corners,np.where(corners==markerCorner))

      cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
      cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
      cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
      cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

      aruco_x_center = int((topLeft[0] + bottomRight[0]) / 2.0)
      aruco_y_center = int((topLeft[1] + bottomRight[1]) / 2.0)
      center_aruco_list.append((aruco_x_center,aruco_y_center))

    objPoints = np.zeros((4, 3), dtype=np.float32)
    objPoints[0] = [-size_of_aruco_m/2, size_of_aruco_m/2, 0]
    objPoints[1] = [size_of_aruco_m/2, size_of_aruco_m/2, 0]
    objPoints[2] = [size_of_aruco_m/2, -size_of_aruco_m/2, 0]
    objPoints[3] = [-size_of_aruco_m/2, -size_of_aruco_m/2, 0]
    tvecs=[]

    for i in range(len(ids_copy)): 

      corner = copy_corners[i][0]
      #print(corner, type(corner), corner.shape)

      # rvec:- rotational vector
      # tvec:- translational vector

      _,rvec,tvec = cv2.solvePnP(objPoints,corner,cam_mat, dist_mat)

    # Calculate the distance from the camera to the marker

      distance = np.linalg.norm(tvec)
      distance_from_rgb_list.append(distance)
      tvecs.append(tvec)
      angle_aruco_list.append(rvec)
          
    # Draw coordinate axes for each marker
    for i in range(len(ids_copy)):

      cv2.drawFrameAxes(image, cam_mat, dist_mat, angle_aruco_list[i], tvecs[i], 1)

  
  except:

    print("No Aruco detected !!")

  #print(angle_aruco_list, type(angle_aruco_list),np.shape(angle_aruco_list))

  return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids


class ImageSubscriber(Node):
  
  def __init__(self):

    super().__init__('image_subscriber')

    ############ Topic SUBSCRIPTIONS ############

    self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb,10)
    self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb,10)
    self.tf_pub = self.create_publisher(Float64MultiArray,"aruco_tf_array",10)

    ############ Constructor VARIABLES/OBJECTS ############

    image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
    self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
    self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
    self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
    self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
    self.timer = self.create_timer(image_processing_rate,self.process_image)      # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
    
    self.cv_image = None                          # colour raw image variable (from colorimagecb())
    self.depth_image = None                       # depth image variable (from depthimagecb())
   
  def colorimagecb(self, data):

    '''
    Description:    Callback function for colour camera raw topic.
                    Use this function to receive raw image data and convert to CV2 image

    Args:
        data (Image):    Input coloured raw image frame received from image_raw camera topic

    Returns:
    '''
    
  
  # Convert the ROS Image message to a cv2 image
    
    self.cv_image = self.bridge.imgmsg_to_cv2(data)
    

  def depthimagecb(self, data):

    '''
    Description:    Callback function for aligned depth camera topic. 
                    Use this function to receive image depth data and convert to CV2 image

    Args:
        data (Image):    Input depth image frame received from aligned depth camera topic

    Returns:
    '''

    try:
    # Convert the ROS Image message to a cv2 image
        self.depth_image = self.bridge.imgmsg_to_cv2(data)

    except CvBridgeError as e:
        print(e)


  def process_image(self):

    arr = []
    tf_array = []
    sizeCamX = 1280
    sizeCamY = 720
    centerCamX = 640 
    centerCamY = 360
    focalX = 931.1829833984375
    focalY = 931.1829833984375

    #print(f"type- {type(self.cv_image)}, shape- {np.shape(self.cv_image)}")
    aruco_center_list, _, angle_list, _, ids_list = detect_aruco(self.cv_image)

    # Using list comprehension to flatten the list
    angle_list = [[item[0] for item in sublist] for sublist in angle_list]
    #print(f"updated angle_list- {angle_list}, {np.shape(angle_list)}")

    try: 

      for i in range(len(ids_list)):

        #print(f"center of obj_{ids_list[i]} is- {aruco_center_list[i]}")
        #print(angle_list[i])
        angles = []
        
        for angle in angle_list[i]:
          angle = (0.788*angle) - ((angle**2)/3160)
          #print(angle)
          angles.append(angle)

        cX = (aruco_center_list[i][0])
        cY = (aruco_center_list[i][1])
        #print(f"cx: {cX}, cY: {cY}")

        depth = (self.depth_image[cY,cX])/1000
    
        if depth < 6.00:

          arr1 = []
          #self.get_logger().info(f'depth of {ids_list[i]} is {depth} m')
          x1 = depth * (sizeCamX - cX - centerCamX) / (focalX)
          y1 = depth * (sizeCamY - cY - centerCamY) / (focalY)
          z1 = depth

          #print(f"corrected coordinates of id {ids_list[i]}  x: {z1}, y: {x1}, z: {y1}")

          #cv2.circle(self.cv_image, (cX,cY), 5, (0, 0, 255), -1)
          #cv2.putText(self.cv_image, 'center', (cX,cY), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA) 
    
          cv2.imshow("Color Image",self.cv_image)
          cv2.waitKey(1)
          x_rot_quaterion = tf_transformations.quaternion_about_axis(np.pi/2,(1,0,0))
          z_rot_quaterion = tf_transformations.quaternion_about_axis(np.pi/2,(0,0,1))
          z_clkpi_rot_quaterion = tf_transformations.quaternion_about_axis(-np.pi,(0,0,1))

          if z1 > 1.3:

            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'camera_link'                     
            t.child_frame_id = 'cl#1691_cam_' + str(ids_list[i])
            t.transform.translation.x = z1 
            t.transform.translation.y = x1 
            t.transform.translation.z = y1

            # to make tf's parallel to tf of base_link value was obtained using:-
            # ros2 run tf2_ros tf2_echo world camera_link
            
            q_x,q_y,q_z,q_w = tf_transformations.quaternion_from_euler(0.0,-0.261,0.0)
            original_quaterion = [q_x,q_y,q_z,q_w]
            new_quaternion = tf_transformations.quaternion_multiply(original_quaterion, z_rot_quaterion)
            new_quaternion = tf_transformations.quaternion_multiply(new_quaternion, x_rot_quaterion)
            t.transform.rotation.x = new_quaternion[0] 
            t.transform.rotation.y = new_quaternion[1]  
            t.transform.rotation.z = new_quaternion[2] 
            t.transform.rotation.w = new_quaternion[3] 
            self.br.sendTransform(t)

          elif z1 < 1.3 and x1 < 0:

            t1 = TransformStamped()
            t1.header.stamp = self.get_clock().now().to_msg() 
            t1.header.frame_id = 'camera_link'                     
            t1.child_frame_id = 'cl#1691_cam_' + str(ids_list[i])
            t1.transform.translation.x = z1 
            t1.transform.translation.y = x1 
            t1.transform.translation.z = y1
            q_x,q_y,q_z,q_w = tf_transformations.quaternion_from_euler(0.0,-0.261,0.0)
            original_quaterion = [q_x,q_y,q_z,q_w]
            new_quaternion = tf_transformations.quaternion_multiply(original_quaterion, x_rot_quaterion)
            t1.transform.rotation.x = new_quaternion[0] 
            t1.transform.rotation.y = new_quaternion[1]  
            t1.transform.rotation.z = new_quaternion[2]
            t1.transform.rotation.w = new_quaternion[3]
            self.br.sendTransform(t1)
          
          else:
            
            t2 = TransformStamped()
            t2.header.stamp = self.get_clock().now().to_msg() 
            t2.header.frame_id = 'camera_link'                     
            t2.child_frame_id = 'cl#1691_cam_' + str(ids_list[i])
            t2.transform.translation.x = z1 
            t2.transform.translation.y = x1 
            t2.transform.translation.z = y1
            q_x,q_y,q_z,q_w = tf_transformations.quaternion_from_euler(0.0,-0.261,0.0)
            original_quaterion = [q_x,q_y,q_z,q_w]
            new_quaternion = tf_transformations.quaternion_multiply(original_quaterion, z_clkpi_rot_quaterion)
            new_quaternion = tf_transformations.quaternion_multiply(new_quaternion, x_rot_quaterion)
            t2.transform.rotation.x = new_quaternion[0] 
            t2.transform.rotation.y = new_quaternion[1]  
            t2.transform.rotation.z = new_quaternion[2]
            t2.transform.rotation.w = new_quaternion[3]
            self.br.sendTransform(t2)

          t3 = self.tf_buffer.lookup_transform('base_link','cl#1691_cam_' + str(ids_list[i]),rclpy.time.Time())
          new_x = t3.transform.translation.x
          new_y = t3.transform.translation.y
          new_z = t3.transform.translation.z
          new_qx = t3.transform.rotation.x
          new_qy = t3.transform.rotation.y
          new_qz = t3.transform.rotation.z
          new_qw = t3.transform.rotation.w

          t4 = TransformStamped()
          t4.header.stamp = self.get_clock().now().to_msg() 
          t4.header.frame_id = 'base_link'                     
          t4.child_frame_id = 'cl#1691_base_' + str(ids_list[i])

          t4.transform.translation.x = new_x
          t4.transform.translation.y = new_y
          t4.transform.translation.z = new_z
          t4.transform.rotation.x = new_qx
          t4.transform.rotation.y = new_qy
          t4.transform.rotation.z = new_qz
          t4.transform.rotation.w = new_qw
          self.br.sendTransform(t4)
          arr1.append([ids_list[i],new_x,new_y,new_z,new_qx,new_qy,new_qz,new_qw])
        arr.append(arr1)

        if len(arr) == len(ids_list):

          tf_array.append(arr)
          #print(f"tf_Array: {tf_array}, type: {type(tf_array)} \n")

    except:

      #print("No Aruco detected !!")
      pass
    
  
    msg = Float64MultiArray()

    flattened_list = [item for sublist1 in tf_array for sublist2 in sublist1 for sublist3 in sublist2 for item in sublist3]

    # Convert values to float64
    flattened_list = [float(value) for value in flattened_list]
    # Check for valid float range
    flattened_list = [max(min(value, 1.79e308), -1.79e308) for value in flattened_list]
    msg.data = flattened_list
    self.tf_pub.publish(msg)

  
  
def main(args=None):
  
  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':

  main()
  