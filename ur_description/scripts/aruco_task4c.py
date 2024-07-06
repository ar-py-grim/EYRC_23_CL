#!/usr/bin/env python3


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
from example_interfaces.msg import Int16


##################### FUNCTION DEFINITIONS #######################


def calculate_rectangle_area(coordinates):

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
    


def detect_aruco(image):

  # aruco_area_threshold = 1500
  aruco_area_threshold = 9000
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
      #print(f"marker {markerID} zipped markercorner:- {corners}, {type(corners)}")
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
      #width_aruco_list.append(width)
      area = width*length
      # print(f"area of aruco {markerID} is {area}")

      # if area > aruco_area_threshold:
      #   np.delete(ids , markerID)
      #   np.delete(corners,np.where(corners==markerCorner))
      
      # print(f"updated ids_list: {ids}")
      width_aruco_list.append(width)
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

    #print("No Aruco detected !!")
    pass

  #print(angle_aruco_list, type(angle_aruco_list),np.shape(angle_aruco_list))

  return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids


class ImageSubscriber(Node):
  
  def __init__(self):

    super().__init__('image_subscriber')

    ############ Topic SUBSCRIPTIONS ############

    self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb,10)
    self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb,10)

    # publish aruco ids & coordinates, subscribed in arm_task4c.py
    self.tf_pub = self.create_publisher(Float64MultiArray,"aruco_tf_array", 10)

    # get aruco id count, published from arm_task4c.py
    self.index_sub = self.create_subscription(Int16, "index", self.callback_index, 10)

    # check bot is carrying rack or not, published from dock_task4c.py script
    self.status_sub = self.create_subscription(Int16, "status", self.callback_status, 10) 

    ############ Constructor VARIABLES/OBJECTS ############
     
    image_processing_rate = 0.02                        # 50 Hz
    self.bridge = CvBridge()                                                      # initialise CvBridge object for image conversion
    self.tf_buffer = tf2_ros.buffer.Buffer()                                      # buffer time used for listening transforms
    self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
    self.br = tf2_ros.TransformBroadcaster(self)                                  # object as transform broadcaster to send transform wrt some frame_id
    self.timer = self.create_timer(image_processing_rate,self.process_image)      # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
    
    self.cv_image = None                       # colour raw image variable (from colorimagecb())
    self.depth_image = None                    # depth image variable (from depthimagecb())
    self.index = None                          # index variable (from callback_index())
    self.status = 0                            # status variable (from callback_status())
  

  def callback_status(self,msg1):

    self.status = msg1.data


  def callback_index(self,msg):

    self.index = msg.data

   
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

    #arr = []
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
    #print(f"flattened angle_list- {angle_list}, {np.shape(angle_list)}")

    try: 
      arr = []
      correct_id_list = []

      for i in range(len(ids_list)):

        #print(ids_list)

        #print(f"center of obj_{ids_list[i]} is- {aruco_center_list[i]}")
        #print(angle_list[i])
        angles = []
        
        for angle in angle_list[i]:
          angle = (0.788*angle) - ((angle**2)/3160)
          #print(angle)
          angles.append(angle)
        
        #print(f"roll: {angles[0]}, pitch: {angles[1]}, yaw: {angles[2]} of {ids_list[i]}")

        cX = (aruco_center_list[i][0])
        cY = (aruco_center_list[i][1])
        #print(f"cx: {cX}, cY: {cY}")

        depth = (self.depth_image[cY,cX])/1000

        if depth < 6.00 and depth > 0:

          arr1 = []
          correct_id_list.append(ids_list[i][0])
          #print(f"corrected ids_list: {correct_id_list}")
          #self.get_logger().info(f'depth of {ids_list[i]} is {depth} m')
          x1 = depth * (sizeCamX - cX - centerCamX) / (focalX)
          y1 = depth * (sizeCamY - cY - centerCamY) / (focalY)
          z1 = depth

          #print(f"corrected coordinates of id {ids_list[i]}  x: {z1}, y: {x1}, z: {y1}")
          #print(f"corrected angles of id {ids_list[i]}  {angles}")

          #cv2.circle(self.cv_image, (cX,cY), 5, (0, 0, 255), -1)
          #cv2.putText(self.cv_image, 'center', (cX,cY), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2, cv2.LINE_AA) 
    
          cv2.imshow("Color Image",self.cv_image)
          cv2.waitKey(1)
          original_quaterion = tf_transformations.quaternion_from_euler(0.0, 0.0, angles[2])

            # to make tf's parallel to tf of base_link value was obtained using:-
            # ros2 run tf2_ros tf2_echo world camera_link
          

          t = TransformStamped()
          t.header.stamp = self.get_clock().now().to_msg()
          t.header.frame_id = 'camera_link'                     
          t.child_frame_id = 'cam_' + str(ids_list[i][0])
          t.transform.translation.x = z1 
          t.transform.translation.y = x1 
          t.transform.translation.z = y1
          t.transform.rotation.x = original_quaterion[0]
          t.transform.rotation.y = original_quaterion[1]
          t.transform.rotation.z = original_quaterion[2]
          t.transform.rotation.w = original_quaterion[3]
          self.br.sendTransform(t)

          x_rot_quaterion = tf_transformations.quaternion_about_axis(np.pi/2,(1,0,0))
          z_rot_quaterion = tf_transformations.quaternion_about_axis(np.pi/2,(0,0,1))
          z_clkpi_quat = tf_transformations.quaternion_about_axis(-np.pi, (0,0,1))
          t1 = self.tf_buffer.lookup_transform('base_link','cam_' + str(ids_list[i][0]),rclpy.time.Time())
          new_quat = [t1.transform.rotation.x, t1.transform.rotation.y, t1.transform.rotation.z, t1.transform.rotation.w]

          # make tf of boxes of  parallel to base_link: ros2 run tf2_ros tf2_echo world camera_link
          cam_quat = tf_transformations.quaternion_from_euler(0.0, -0.261, 0.0)
          new_quaterion = tf_transformations.quaternion_multiply(new_quat, cam_quat)

          if z1 > 1.3:                                       # for middle rack
            
            t2 = TransformStamped()
            t2.header.stamp = self.get_clock().now().to_msg() 
            t2.header.frame_id = 'base_link'    
            t2.child_frame_id = 'obj_' + str(ids_list[i][0])
            quat1 = tf_transformations.quaternion_multiply(new_quaterion, z_rot_quaterion)
            quat1 = tf_transformations.quaternion_multiply(quat1, x_rot_quaterion)
            t2.transform.translation.x = t1.transform.translation.x
            t2.transform.translation.y = t1.transform.translation.y
            t2.transform.translation.z = t1.transform.translation.z
            t2.transform.rotation.x = quat1[0]
            t2.transform.rotation.y = quat1[1]  
            t2.transform.rotation.z = quat1[2] 
            t2.transform.rotation.w = quat1[3] 
            self.br.sendTransform(t2)
            #print(f"euler angles: {tf_transformations.euler_from_quaternion(new_quaternion)} for {ids_list[i]}")
            
            t3 = TransformStamped()
            t3.header.stamp = self.get_clock().now().to_msg()
            t3.header.frame_id = 'base_link'                     
            t3.child_frame_id = 'r' + str(ids_list[i][0])
            t3.transform.translation.x = t2.transform.translation.x - 0.1
            t3.transform.translation.y = t2.transform.translation.y
            t3.transform.translation.z = t2.transform.translation.z
            t3.transform.rotation.x = t2.transform.rotation.x
            t3.transform.rotation.y = t2.transform.rotation.y
            t3.transform.rotation.z = t2.transform.rotation.z
            t3.transform.rotation.w = t2.transform.rotation.w
            self.br.sendTransform(t3)


          elif x1 < 0 and z1 < 1.3:                         # for right rack
            
            t2 = TransformStamped()
            t2.header.stamp = self.get_clock().now().to_msg() 
            t2.header.frame_id = 'base_link'                     
            t2.child_frame_id = 'obj_' + str(ids_list[i][0])
            t2.transform.translation.x = t1.transform.translation.x
            t2.transform.translation.y = t1.transform.translation.y
            t2.transform.translation.z = t1.transform.translation.z
            z_quat = tf_transformations.quaternion_about_axis(1.575, (0, 0, 1))
            y_quat = tf_transformations.quaternion_about_axis(0.269, (0, 1, 0))
            x_quat = tf_transformations.quaternion_about_axis(0.27, (1, 0, 0))

            # to make tf parallel to base_link
            quat2 = tf_transformations.quaternion_multiply(new_quaterion, z_quat)
            quat2 = tf_transformations.quaternion_multiply(quat2, x_quat)
            quat2 = tf_transformations.quaternion_multiply(quat2, y_quat)

            # to convert tf to coodinate system of base_link
            quat2 = tf_transformations.quaternion_multiply(quat2, z_clkpi_quat)
            quat2 = tf_transformations.quaternion_multiply(quat2, x_rot_quaterion)
            t2.transform.rotation.x = quat2[0] 
            t2.transform.rotation.y = quat2[1]  
            t2.transform.rotation.z = quat2[2]
            t2.transform.rotation.w = quat2[3] 
            self.br.sendTransform(t2)

            t3 = TransformStamped()
            t3.header.stamp = self.get_clock().now().to_msg()
            t3.header.frame_id = 'base_link'                     
            t3.child_frame_id = 'r' + str(ids_list[i][0])
            t3.transform.translation.x = t2.transform.translation.x 
            t3.transform.translation.y = t2.transform.translation.y + 0.1
            t3.transform.translation.z = t2.transform.translation.z
            t3.transform.rotation.x = t2.transform.rotation.x
            t3.transform.rotation.y = t2.transform.rotation.y
            t3.transform.rotation.z = t2.transform.rotation.z
            t3.transform.rotation.w = t2.transform.rotation.w
            self.br.sendTransform(t3)
          
            
          else:                                        # for left rack
             
            x_rot_quat = tf_transformations.quaternion_about_axis(-0.27, (1,0,0))
            y_rot_quat = tf_transformations.quaternion_about_axis(-0.27, (0,1,0))
            
            
            t2 = TransformStamped()
            t2.header.stamp = self.get_clock().now().to_msg() 
            t2.header.frame_id = 'base_link'                     
            t2.child_frame_id = 'obj_' + str(ids_list[i][0])

            # to make tf parallel
            quat3 = tf_transformations.quaternion_multiply(new_quaterion, z_clkpi_quat)
            quat3 = tf_transformations.quaternion_multiply(quat3, y_rot_quat)
            quat3 = tf_transformations.quaternion_multiply(quat3, x_rot_quat)

            # to convert tf to coodinate system of base_link
            quat3 = tf_transformations.quaternion_multiply(quat3, z_rot_quaterion)
            quat3 = tf_transformations.quaternion_multiply(quat3, x_rot_quaterion)

            t2.transform.translation.x = t1.transform.translation.x 
            t2.transform.translation.y = t1.transform.translation.y 
            t2.transform.translation.z = t1.transform.translation.z
            t2.transform.rotation.x = quat3[0]
            t2.transform.rotation.y = quat3[1]
            t2.transform.rotation.z = quat3[2]
            t2.transform.rotation.w = quat3[3]
            self.br.sendTransform(t2)

            t3 = TransformStamped()
            t3.header.stamp = self.get_clock().now().to_msg()
            t3.header.frame_id = 'base_link'                     
            t3.child_frame_id = 'r' + str(ids_list[i][0])
            t3.transform.translation.x = t2.transform.translation.x 
            t3.transform.translation.y = t2.transform.translation.y - 0.1
            t3.transform.translation.z = t2.transform.translation.z
            t3.transform.rotation.x = t2.transform.rotation.x
            t3.transform.rotation.y = t2.transform.rotation.y
            t3.transform.rotation.z = t2.transform.rotation.z
            t3.transform.rotation.w = t2.transform.rotation.w
            self.br.sendTransform(t3)


          t4 = self.tf_buffer.lookup_transform('base_link','r' + str(ids_list[i][0]),rclpy.time.Time())
          new_x1 = t4.transform.translation.x
          new_y1 = t4.transform.translation.y
          new_z1 = t4.transform.translation.z
          t5 = self.tf_buffer.lookup_transform('base_link','obj_' + str(ids_list[i][0]),rclpy.time.Time())
          new_x2 = t5.transform.translation.x
          new_y2 = t5.transform.translation.y
          new_z2 = t5.transform.translation.z
          arr1.append([ids_list[i][0], new_x1, new_y1, new_z1, new_x2, new_y2, new_z2, len(correct_id_list)])
          #arr1.append([ids_list[i][0], new_x1, new_y1, new_z1, new_x2, new_y2, new_z2, 1])

        arr.append(arr1)
        #arr.append([ids_list[i][0], new_x1, new_y1, new_z1, new_x2, new_y2, new_z2, len(ids_list)])
        #print(f"arr: {arr}, len: {len(arr)} \n")
        #print(f"len of ids_list: {len(ids_list)}")

        if len(arr) == len(correct_id_list):

          tf_array.append(arr)
          #print(f"tf_Array: {tf_array} \n")

    except:

      #print("No Aruco detected !!")
      pass   
    

    msg = Float64MultiArray()

    flattened_list = [item for sublist1 in tf_array for sublist2 in sublist1 for sublist3 in sublist2 for item in sublist3]

    # Convert values to float64
    flattened_list = [float(value) for value in flattened_list]
    flattened_list = [round(element, 3) for element in flattened_list]
    flattened_list = [flattened_list[i:i+8] for i in range(0, len(flattened_list), 8)]
    flattened_list = sorted(flattened_list, key=lambda x: x[0])
    #print(flattened_list, len(flattened_list))

    if len(flattened_list) > 0 and self.index is not None and flattened_list is not None:

      flattened_list = sorted(flattened_list, key=lambda x: x[0])
      # print(flattened_list, len(flattened_list))

      if self.index <= len(flattened_list) and self.index is not None:

        msg.data = flattened_list[0]   
        self.tf_pub.publish(msg)
      
      else:
        msg.data = []
        self.tf_pub.publish(msg)

      # if self.status == 1:
        
      #   #print(f"robot has stopped: {self.status}")
      #   msg.data = flattened_list[0]   
      #   self.tf_pub.publish(msg)
      #   #print(msg.data)           
      
      # elif self.status == 0:
      #   msg.data = []
      #   self.tf_pub.publish(msg)
      #   self.index  = None
        
      print(f"currently publishing: {msg.data}")

    # else:

    #   msg.data = []
    #   #self.tf_pub.publish(msg)
    #   self.index = None


def main(args=None):

  rclpy.init(args=args)
  image_subscriber = ImageSubscriber()
  rclpy.spin(image_subscriber)
  image_subscriber.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':

  main()
  
    