#!/usr/bin/env python3


import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
from ebot_docking.srv import DockSw
import yaml
from lifecycle_msgs.srv import GetState
import time

pos = []
rack_id = []
i = 2

with open(r'/home/grim/cl_ws/src/ebot_nav2/scripts/config4C.yaml') as file:
    data = yaml.safe_load(file)

    # Extract positions and package IDs
    package_id_order = data['package_id']
    positions = data['position']

rack_names = []
rack_positions = []
rack_orientations = []

for rack_number in package_id_order:
    for rack_data in positions:
        for rack_key, rack_values in rack_data.items():
            if rack_key.endswith(str(rack_number)):
                rack_names.append(rack_key)
                rack_names.append(rack_key)
                rack_positions.append(rack_values)


flat_rack_positions = [item for sublist in rack_positions for item in sublist]
rack_orientations.append(flat_rack_positions[2])     # for rack3
rack_orientations.append(flat_rack_positions[2])     # for rack3 right of the arm

rack_orientations.append(flat_rack_positions[5])     # for rack1
rack_orientations.append(-3.14)                      # for rack1 front of the arm

rack_orientations.append(flat_rack_positions[8])     # for rack2
rack_orientations.append(flat_rack_positions[8])     # for rack2 front of the arm


print("Rack Positions:", flat_rack_positions)
print("Rack Names:", rack_names)
print("Rack orientations:", rack_orientations)


def create_pose_stamped(navigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
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


def dock_service_client(nav, dock_request):
    client = nav.create_client(DockSw, 'dock_control')

    while not client.wait_for_service(timeout_sec=1.0):
        nav.get_logger().info('Service not available, waiting again...')

    future = client.call_async(dock_request)
    rclpy.spin_until_future_complete(nav, future)

    if future.result() is not None:
        nav.get_logger().info('Docking service succeeded with response: %s' % future.result().success)
        nav.get_logger().info('Message: %s' % future.result().message)
        return future.result().success
    else:
        nav.get_logger().error('Service call failed %r' % (future.exception(),))
        return False


def check_amcl_service_availability(nav):
    try:
        client = nav.create_client(GetState, '/amcl/get_state')
        while not client.wait_for_service(timeout_sec=1.0):
            nav.get_logger().info("AMCL service not available, waiting...")
        return True
    
    except Exception as e:
        nav.get_logger().error(f"Error checking AMCL service availability: {e}")
        return False


def main():
    
    global i
    rclpy.init()
    nav = BasicNavigator()

    # Check AMCL state before publsihing initial pose
    if not check_amcl_service_availability(nav):
        return
    
    # set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # wait for nav2 to be active
    nav.waitUntilNav2Active()

    # set NAV2 waypoint follow      
                                                                        
    #goal_pose1 = create_pose_stamped(nav, flat_rack_positions[0] + 0.2, -0.058, flat_rack_positions[2])   # rack3
    #goal_pose2 = create_pose_stamped(nav, flat_rack_positions[0] + 0.2 , -1.75, flat_rack_positions[2])   # rack3 right of arm

    goal_pose1 = create_pose_stamped(nav, 0.338, flat_rack_positions[4] + 0.2, flat_rack_positions[5])  # rack1                    
    goal_pose2 = create_pose_stamped(nav, 0.24, -2.6, 0.0)                                              # rack1 infront of the arm       
    goal_pose3 = create_pose_stamped(nav, flat_rack_positions[6] + 0.25, 1.65, 0.0)                     # rack2    
    goal_pose4 = create_pose_stamped(nav, 1.62, -5.38, flat_rack_positions[8])                           # rack2 left of the arm   
    goal_pose5 = create_pose_stamped(nav, 0.0, 0.0, 0.0)                                                # home
    waypoints = [goal_pose1, goal_pose2, goal_pose3, goal_pose4, goal_pose5]
    #waypoints = [goal_pose1, goal_pose2]

    # Create a DockSw service request

    dock_request = DockSw.Request()
    dock_request.linear_dock = True             # Set to True for linear docking
    dock_request.orientation_dock = True        # Set to True for orientation docking
    #dock_request.distance = 1.0                # Specify the distance parameter


    for waypoint in waypoints:

        nav.goToPose(waypoint)

        while not nav.isTaskComplete():
            rclpy.spin_once(nav)

        dock_request.orientation = rack_orientations[i]      # Specify the orientation parameter
        dock_request.rack_no = str(rack_names[i])
        #print(f"initial value of i: {i}, orient by: {dock_request.orientation}, for {dock_request.rack_no}")

        # Call the DockSw service after reaching the waypoint
        dock_service_client(nav, dock_request)
        i = i+1

        # if dock_service_client(nav, dock_request):
        #     i = i+1
        #     print(f"updated value of i: {i}, orientation for next: {rack_orientations[i]}")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
