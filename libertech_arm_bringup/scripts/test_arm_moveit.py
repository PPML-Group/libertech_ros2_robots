import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotTrajectory
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.action import ExecuteTrajectory
import action_msgs.msg
import sys

class MoveItNode(Node):
    def __init__(self):
        super().__init__('moveit_node')
        self.motion_plan_client = self.create_client(GetMotionPlan, 'plan_kinematic_path')
        self.execute_traj_client = self.create_client(ExecuteTrajectory, 'execute_trajectory')
        while not self.motion_plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Motion Plan service not available, waiting again...')
        while not self.execute_traj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Execute Trajectory service not available, waiting again...')
        self.get_logger().info('Services are available.')

    def plan_motion(self):
        request = GetMotionPlan.Request()
        # Fill in the request details, e.g., start and goal poses, motion constraints, etc.
        # request.start_state = ...
        # request.goal_constraints = ...
        # request.motion_plan_request = ...
        # Populate the request fields with necessary data for motion planning.
        future = self.motion_plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            motion_plan_response = future.result()
            trajectory = motion_plan_response.motion_plan_response.trajectory
            self.execute_trajectory(trajectory)
        else:
            self.get_logger().error('Failed to get motion plan response.')

    def execute_trajectory(self, trajectory):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        self.execute_traj_client.wait_for_server()
        future = self.execute_traj_client.call_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Trajectory execution completed.')
        else:
            self.get_logger().error('Failed to execute trajectory.')

def main(args=None):
    rclpy.init(args=args)
    moveit_node = MoveItNode()
    moveit_node.plan_motion()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
