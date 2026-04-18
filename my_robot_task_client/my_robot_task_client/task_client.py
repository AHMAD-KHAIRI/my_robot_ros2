#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.srv import PickPlaceTask
from sensor_msgs.msg import Image

class MyRobotTaskClientNode(Node):
    def __init__(self):
        super().__init__("my_robot_task_client")
        self.set_pick_place_task_client_ = self.create_client(PickPlaceTask, "/my_robot/pick_place_task")
        self.get_logger().info("My Robot Task Client has been started.")

    # - Sends a valid request:
    #   - instruction = anything
    #   - images = dummy or empty (fine for now)
    #   - candidate_object_ids = ["object_1"]
    # - Receives response
    # - Prints:
    #   - pick_pose
    #   - place_pose
    #   - selected_object_id
    #   - debug_info

    def call_pick_place_task_service(self, instruction, rgb_image, depth_image, candidate_object_ids):
        while not self.set_pick_place_task_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for My Robot Task Sever...")

        request = PickPlaceTask.Request()
        request.instruction = instruction
        request.rgb_image = rgb_image
        request.depth_image = depth_image
        request.candidate_object_ids = candidate_object_ids

        future = self.set_pick_place_task_client_.call_async(request)
        future.add_done_callback(self.callback_call_pick_place_task)

    def callback_call_pick_place_task(self, future):
        response: PickPlaceTask.Response = future.result()

        if response.success:
            self.get_logger().info(f"""Service call successful!\n
                                   Selected object: {response.selected_object_id}\n
                                   Pick pose: {response.pick_pose}\n
                                   Place pose: {response.place_pose}\n
                                   Debug info: {response.debug_info}""")
        else:
            self.get_logger().info("Service call failed!")


def main(args=None):
    rclpy.init(args=args)
    node = MyRobotTaskClientNode()
    node.call_pick_place_task_service(instruction="test", rgb_image=Image(), depth_image=Image(), candidate_object_ids=["object_1"])
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()