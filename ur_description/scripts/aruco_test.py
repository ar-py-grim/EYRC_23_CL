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
import time


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
    theta = np.arccos(dot_product / (diag1_mag * diag2_mag))

    area = abs(0.5 * np.sin(theta) * diag2_mag * diag1_mag)
    width = np.linalg.norm(np.array(p0) - np.array(p1))

    return area, width
    


def detect_aruco(image):
    aruco_area_threshold = 9000
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    dist_mat = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
    size_of_aruco_m = 0.15
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []

    try:
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        detectorParams = cv2.aruco.DetectorParameters()
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)
        corners, ids, _ = detector.detectMarkers(gray_image)
        ids_copy = deepcopy(ids)
        copy_corners = deepcopy(corners)
        cv2.aruco.drawDetectedMarkers(image, corners, ids_copy)
        ids_copy = ids_copy.flatten()

        for (markerCorner, markerID) in zip(corners, ids_copy):
            corners = markerCorner.reshape(4, 2)
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            width = math.dist(topLeft, topRight)
            length = math.dist(topRight, bottomRight)
            area = width * length

            width_aruco_list.append(width)
            cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)

            aruco_x_center = int((topLeft[0] + bottomRight[0]) / 2.0)
            aruco_y_center = int((topLeft[1] + bottomRight[1]) / 2.0)
            center_aruco_list.append((aruco_x_center, aruco_y_center))

        objPoints = np.zeros((4, 3), dtype=np.float32)
        objPoints[0] = [-size_of_aruco_m / 2, size_of_aruco_m / 2, 0]
        objPoints[1] = [size_of_aruco_m / 2, size_of_aruco_m / 2, 0]
        objPoints[2] = [size_of_aruco_m / 2, -size_of_aruco_m / 2, 0]
        objPoints[3] = [-size_of_aruco_m / 2, -size_of_aruco_m / 2, 0]
        tvecs = []

        for i in range(len(ids_copy)): 
            corner = copy_corners[i][0]
            _, rvec, tvec = cv2.solvePnP(objPoints, corner, cam_mat, dist_mat)
            distance = np.linalg.norm(tvec)
            distance_from_rgb_list.append(distance)
            tvecs.append(tvec)
            angle_aruco_list.append(rvec)
          
        for i in range(len(ids_copy)):
            cv2.drawFrameAxes(image, cam_mat, dist_mat, angle_aruco_list[i], tvecs[i], 1)

    except:
        pass

    return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids


class ImageSubscriber(Node):
  
    def __init__(self):
        super().__init__('image_subscriber')

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)
        self.tf_pub = self.create_publisher(Float64MultiArray, "aruco_tf_array", 10)
        self.index_sub = self.create_subscription(Int16, "index", self.callback_index, 10)
        self.status_sub = self.create_subscription(Int16, "status", self.callback_status, 10)

        image_processing_rate = 0.02  # 50 Hz
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(image_processing_rate, self.process_image)
    
        self.cv_image = None
        self.depth_image = None
        self.index = None
        self.status = 0

        self.last_detected_aruco = {}
        self.aruco_timeout = 2.0  # seconds

    def callback_status(self, msg1):
        self.status = msg1.data

    def callback_index(self, msg):
        self.index = msg.data

    def colorimagecb(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data)

    def depthimagecb(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError as e:
            print(e)

    def process_image(self):
        tf_array = []
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375

        aruco_center_list, _, angle_list, _, ids_list = detect_aruco(self.cv_image)
        angle_list = [[item[0] for item in sublist] for sublist in angle_list]

        current_time = time.time()

        try: 
            arr = []
            correct_id_list = []

            for i in range(len(ids_list)):
                angles = []
                for angle in angle_list[i]:
                    angle = (0.788 * angle) - ((angle ** 2) / 3160)
                    angles.append(angle)
                
                cX = aruco_center_list[i][0]
                cY = aruco_center_list[i][1]
                depth = (self.depth_image[cY, cX]) / 1000

                if depth < 6.00 and depth > 0:
                    arr1 = []
                    correct_id_list.append(ids_list[i][0])
                    x1 = depth * (sizeCamX - cX - centerCamX) / focalX
                    y1 = depth * (sizeCamY - cY - centerCamY) / focalY
                    z1 = depth

                    cv2.imshow('image', self.cv_image)
                    cv2.waitKey(1)

                    t = TransformStamped()
                    t.header.stamp = self.get_clock().now().to_msg()
                    t.header.frame_id = 'camera_color_optical_frame'
                    t.child_frame_id = 'aruco'+str(ids_list[i][0])

                    t.transform.translation.x = float(x1)
                    t.transform.translation.y = float(y1)
                    t.transform.translation.z = float(z1)

                    q = tf_transformations.quaternion_from_euler(angles[0], angles[1], angles[2])
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]

                    self.br.sendTransform(t)

                    arr1.append(ids_list[i][0])
                    arr1.append(x1)
                    arr1.append(y1)
                    arr1.append(z1)

                    tf_array.append(arr1)

                    # Update last detected time and transform
                    self.last_detected_aruco[ids_list[i][0]] = (current_time, t)

            # Check for any ArUco markers not currently detected but within the timeout window
            for aruco_id, (last_time, transform) in self.last_detected_aruco.items():
                if aruco_id not in correct_id_list and (current_time - last_time) < self.aruco_timeout:
                    self.br.sendTransform(transform)
                    arr1 = [
                        aruco_id,
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z
                    ]
                    tf_array.append(arr1)

            tf_array = Float64MultiArray(data=[item for sublist in tf_array for item in sublist])
            self.tf_pub.publish(tf_array)

        except:
            pass

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
