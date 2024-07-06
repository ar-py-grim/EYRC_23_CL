#!/usr/bin/env python3

import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations
from ebot_docking.srv import DockSw
import yaml

orient = []
pos = []
rack_id = ''


with open(r'/home/grim/Desktop/config.yaml') as file:
    data = yaml.safe_load(file)
    # Iterate over items in 'position'

#print(f"Package ID: {data['package_id'][0]}")

for item in data['position']:
    rack_name, rack_position = list(item.items())[0]  # Extract rack name and position
    #print(f"{rack_name} Position: {rack_position}")
    result = str(rack_name).find(str(data['package_id'][0]))
    if result!=-1:
        #print(rack_position[0],rack_position[1],rack_position[2])
        pos.append(rack_position[0])
        pos.append(rack_position[1])
        orient.append(rack_position[2])
        orient.append(-3.14)
        rack_id = rack_name
        #print(pos)
        #print(rack_id)

#print(orient)

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

def main():

    rclpy.init()
    nav = BasicNavigator()
    
    # set initial pose
    initial_pose = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    nav.setInitialPose(initial_pose)

    # wait for nav2 to be active
    nav.waitUntilNav2Active()

    # set NAV2 waypoint follow      
    goal_pose1 = create_pose_stamped(nav, pos[0]+0.05, pos[1]+0.5, 0.0)  # reach rack3 position from config.yaml file
    goal_pose2 = create_pose_stamped(nav, 0.1365, -2.52, 2.9147)          # infront of the arm
    goal_pose3 = create_pose_stamped(nav, 0.016, -0.161, 1.748)         # home position
    waypoints = [goal_pose1, goal_pose2,goal_pose3]

    # Create a DockSw service request
    dock_request = DockSw.Request()
    dock_request.linear_dock = True        # Set to True for linear docking
    dock_request.orientation_dock = True   # Set to True for orientation docking
    #dock_request.distance = 1.0           # Specify the distance parameter

    dock_request.orientation = orient.pop(0)      # Specify the orientation parameter
    #print(f" orient by: {dock_request.orientation}")
    #print(f"request_orientation: {dock_request.orientation}")
    #print(f"updated orient: {orient}")
    dock_request.rack_no = str(rack_id)


    for waypoint in waypoints:
        # Navigate to the current waypoint
        nav.followWaypoints([waypoint])
        # Wait for the robot to reach the waypoint
        while not nav.isTaskComplete():
            rclpy.spin_once(nav)

        # Call the DockSw service after reaching the waypoint
        dock_service_client(nav, dock_request)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
