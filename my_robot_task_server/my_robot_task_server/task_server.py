#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import PickPlaceTask
from geometry_msgs.msg import PoseStamped

class MyRobotTaskServerNode(Node):
    def __init__(self):
        super().__init__("my_robot_task_server")
        self.set_pick_place_task_service_ = self.create_service(
            PickPlaceTask, "/my_robot/pick_place_task", self.callback_pick_place_task)
        self.get_logger().info("My Robot Task Server has been started.")

    def go_to_pose_target(self, frame_id, x, y, z, qx, qy, qz, qw):
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = frame_id
        pose_goal.pose.position.x = x
        pose_goal.pose.position.y = y
        pose_goal.pose.position.z = z
        pose_goal.pose.orientation.x = qx
        pose_goal.pose.orientation.y = qy
        pose_goal.pose.orientation.z = qz
        pose_goal.pose.orientation.w = qw
        
        return pose_goal
            
    def callback_pick_place_task(self, request: PickPlaceTask.Request, response: PickPlaceTask.Response):
        self.get_logger().info(f"Received task: {request.instruction}")
        response.pick_pose = self.go_to_pose_target(frame_id="arm_base_link",
                                                    x= 0.5, y= 0.0, z= 0.35,
                                                    qx= 1.0, qy= 0.0, qz= 0.0, qw= 0.0)
        
        response.place_pose = self.go_to_pose_target(frame_id="arm_base_link",
                                                     x= 0.4, y= 0.2, z= 0.35,
                                                     qx= 1.0, qy= 0.0, qz= 0.0, qw= 0.0)
        response.success = True
        response.selected_object_id = "dummy_object"
        response.debug_info = "fallback_hardcoded"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MyRobotTaskServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()