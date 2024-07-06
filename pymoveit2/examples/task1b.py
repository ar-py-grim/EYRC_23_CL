#!/usr/bin/env python3

# Team ID:        CL#1691
# Author List:	  Arpit Gandhi

from threading import Thread
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

# got orientation from task1A 
# points to reach P1 = [0.35, 0.1, 0.68] [0.5, 0.5, 0.5, 0.5]
# D = [-0.37, 0.12, 0.397] [-0.528, -0.493, 0.476, 0.502]
# P2 = [0.194, -0.43, 0.701] [0.707, 0.0, 0.0, 0.707]

def main():

    rclpy.init()

    # Create node for this example
    node = Node("Task1B")

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

                                        # P1
    joint_positions =[[-0.03800758062157334, -1.9107616703217583, 1.6647823094668197, -2.895613292838106, -1.5327887461729115, 3.1415926558663863],
                                        # D
    [-0.03403341798766224, -1.2848632387872256, -1.8567441129914095, -3.185621281551551, -1.545888364367352, 3.1498768354918307],
                                        # P2 
    [-1.3236176592581448, -0.8646291940549192, -0.4890644638726639, -4.930270148334533, 1.8179750937720704, 6.282995771332339],
                                        # D
    [-0.03403341798766224, -1.2848632387872256, -1.8567441129914095, -3.185621281551551, -1.545888364367352, 3.1498768354918307],
                                    # starting position
    [0.0,-2.39110108,2.40855437,-3.14159265,-1.58824962,3.14159265]                                 
    ]

    for joint_position in joint_positions:
        # Move to joint configuration
        node.get_logger().info(f"Moving to {{joint_positions: {list(joint_position)}}}")
        moveit2.move_to_configuration(joint_position)
        moveit2.wait_until_executed()

    rclpy.shutdown()
    exit(0)

if __name__ == "__main__":
    main()