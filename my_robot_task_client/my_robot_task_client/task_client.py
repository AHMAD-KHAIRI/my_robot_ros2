#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from my_robot_interfaces.srv import PickPlaceTask
from sensor_msgs.msg import Image, JointState
from moveit.planning import MoveItPy
from moveit.planning import PlanningComponent
from moveit_configs_utils import MoveItConfigsBuilder
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory

ROBOT_CONFIG = MoveItConfigsBuilder(robot_name="my_robot", package_name="my_robot_moveit_config")\
                                    .robot_description_semantic("config/my_robot.srdf", {"name": "my_robot"})\
                                    .to_dict()

ROBOT_CONFIG = { 
    **ROBOT_CONFIG,
    "planning_scene_monitor": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": "/joint_states",
            "attached_collision_object_topic": "/moveit_cpp/planning_scene_monitor",
            "publish_planning_scene_topic": "/moveit_cpp/publish_planning_scene",
            "monitored_planning_scene_topic": "/moveit_cpp/monitored_planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
        "planning_pipelines": {
            "pipeline_names": ["ompl"]
        },
        "plan_request_params": {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "max_velocity_scaling_factor": 1.0,
            "max_acceleration_scaling_factor": 1.0
        },
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": ["default_planning_request_adapters/ResolveConstraintFrames",
                            "default_planning_request_adapters/ValidateWorkspaceBounds",
                            "default_planning_request_adapters/CheckStartStateBounds",
                            "default_planning_request_adapters/CheckStartStateCollision"],
            "response_adapters": ["default_planning_response_adapters/AddTimeOptimalParameterization",
                             "default_planning_response_adapters/ValidateSolution",
                             "default_planning_response_adapters/DisplayMotionPath"],
            "start_state_max_bounds_error": 0.1
        }
}

class MyRobotTaskClientNode(Node):
    def __init__(self):
        super().__init__("my_robot_task_client")
        self.robot_ = MoveItPy(node_name="my_robot_task_client_moveit_py", config_dict=ROBOT_CONFIG)
        self.arm_: PlanningComponent = self.robot_.get_planning_component("arm")
        self.get_logger().info("MoveItPy initialized successfully.")
        self.get_logger().info("Arm planning component acquired.")
        self.set_pick_place_task_client_ = self.create_client(PickPlaceTask, "/my_robot/pick_place_task")
        self.get_logger().info("My Robot Task Client has been started.")
        self.joint_state_ready_ = False
        self.task_sent_ = False
        self.qos_ = QoSProfile(depth=10)
        self.qos_.reliability = ReliabilityPolicy.BEST_EFFORT
        self.joint_state_sub_ = self.create_subscription(JointState, "/joint_states", self.callback_joint_state, self.qos_)
        self.startup_timer_ = self.create_timer(0.5, self.callback_startup_check)
        self.arm_action_client = ActionClient(self, FollowJointTrajectory, "/arm_controller/follow_joint_trajectory")
        self.place_pose_ = None
        self.phase_ = None

    def wait_for_arm_action_server(self):
        if not self.arm_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Arm FollowJointTrajectory action server not available.")
            return False

        self.get_logger().info("Arm FollowJointTrajectory action server is available.")
        return True

    def callback_joint_state(self, msg: JointState):
        if len(msg.name) > 0 and len(msg.position) > 0:
            self.joint_state_ready_ = True

    def callback_startup_check(self):
        if self.task_sent_:
            return
        
        if not self.joint_state_ready_:
            self.get_logger().info("Waiting for valid joint states...")
            return
            
        self.get_logger().info("Joint states ready. Sending task request now.")
        self.task_sent_ = True
        self.startup_timer_.cancel()
        self.call_pick_place_task_service(
            instruction="test", rgb_image=Image(), depth_image=Image(), candidate_object_ids=["object_1"]
        )
        
    def call_pick_place_task_service(self, instruction, rgb_image, depth_image, candidate_object_ids):
        while not self.set_pick_place_task_client_.wait_for_service(1.0):
            self.get_logger().warn("Waiting for My Robot Task Server...")

        request = PickPlaceTask.Request()
        request.instruction = instruction
        request.rgb_image = rgb_image
        request.depth_image = depth_image
        request.candidate_object_ids = candidate_object_ids

        future = self.set_pick_place_task_client_.call_async(request)
        future.add_done_callback(self.callback_call_pick_place_task)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Arm trajectory result error_code: {result.error_code}")
        self.get_logger().info(f"Arm trajectory result error_string: {result.error_string}")

        if result.error_code != 0:
            return

        if self.phase_ == "pick":
            self.get_logger().info("Pick done → starting place")

            self.phase_ = "place"

            self.get_logger().info(f"Place pose frame: {self.place_pose_.header.frame_id}")
            self.get_logger().info(
                f"Place pose position: "
                f"x={self.place_pose_.pose.position.x}, "
                f"y={self.place_pose_.pose.position.y}, "
                f"z={self.place_pose_.pose.position.z}"
            )
            self.get_logger().info(
                f"Place pose orientation: "
                f"x={self.place_pose_.pose.orientation.x}, "
                f"y={self.place_pose_.pose.orientation.y}, "
                f"z={self.place_pose_.pose.orientation.z}, "
                f"w={self.place_pose_.pose.orientation.w}"
            )

            self.arm_.set_start_state_to_current_state()
            self.arm_.set_goal_state(
                pose_stamped_msg=self.place_pose_,
                pose_link="tool_link"
            )
            self.plan_and_execute(self.arm_)

        elif self.phase_ == "place":
            self.get_logger().info("Place done → FULL LOOP COMPLETE")

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Arm trajectory goal was rejected.")
            return

        self.get_logger().info("Arm trajectory goal accepted.")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def plan_and_execute(self, interface):
        plan_result = interface.plan()

        if not plan_result:
            self.get_logger().info(f"{self.phase_.upper()} plan failed.")
            return

        self.get_logger().info(f"{self.phase_.upper()} plan succeeded.")
        if not self.wait_for_arm_action_server():
            return
        
        traj_msg = plan_result.trajectory.get_robot_trajectory_msg()
        self.get_logger().info(f"Converted joint names: {traj_msg.joint_trajectory.joint_names}")

        self.get_logger().info(f"Trajectory points: {len(traj_msg.joint_trajectory.points)}")

        if len(traj_msg.joint_trajectory.points) > 0:
            first_point = traj_msg.joint_trajectory.points[0]
            last_point = traj_msg.joint_trajectory.points[-1]
            self.get_logger().info(
                f"First point time_from_start: {first_point.time_from_start.sec}.{first_point.time_from_start.nanosec}"
            )
            self.get_logger().info(
                f"Last point time_from_start: {last_point.time_from_start.sec}.{last_point.time_from_start.nanosec}"
            )

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = traj_msg.joint_trajectory

        self.get_logger().info(f"Built FollowJointTrajectory goal with {len(goal_msg.trajectory.points)} points.")
        self.get_logger().info(f"Goal joint names: {goal_msg.trajectory.joint_names}")
        
        self.get_logger().info("Sending goal to /arm_controller/follow_joint_trajectory ...")
        send_goal_future = self.arm_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        return

    def callback_call_pick_place_task(self, future):
        response: PickPlaceTask.Response = future.result()

        if response.success:
            self.get_logger().info("Service call successful.")
            
            self.place_pose_ = response.place_pose
            self.phase_ = "pick"

            self.get_logger().info(f"Pick pose frame: {response.pick_pose.header.frame_id}")
            self.get_logger().info(
                f"Pick pose position: "
                f"x={response.pick_pose.pose.position.x}, "
                f"y={response.pick_pose.pose.position.y}, "
                f"z={response.pick_pose.pose.position.z}"
            )
            self.get_logger().info(
                f"Pick pose orientation: "
                f"x={response.pick_pose.pose.orientation.x}, "
                f"y={response.pick_pose.pose.orientation.y}, "
                f"z={response.pick_pose.pose.orientation.z}, "
                f"w={response.pick_pose.pose.orientation.w}"
            )
            self.arm_.set_start_state_to_current_state()
            self.arm_.set_goal_state(pose_stamped_msg=response.pick_pose, pose_link="tool_link")
            self.plan_and_execute(self.arm_)

        else:
            self.get_logger().info("Service call failed!")


def main(args=None):
    rclpy.init(args=args)
    node = MyRobotTaskClientNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()