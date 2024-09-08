import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Pose2D
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from .proportional_controller import ProportionalController  # Assuming you have this implemented
from action_msgs.msg import GoalStatus
from custom_interfaces.action import ToPose2D  # Import the custom action
from rclpy.action import ActionServer, CancelResponse, GoalResponse
import math


class ControlRobot(Node):
    """
    ROS 2 Action server that controls a robot in closed-loop using odometry and proportional control.
    """

    def __init__(self):
        super().__init__('control_robot_action_server')

        # Publisher for /cmd_vel
        self.velocity_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for /odom
        self.odometry_sub_ = self.create_subscription(
            Odometry, '/odom', self.odometry_callback, 10
        )

        # Action server
        self._action_server = ActionServer(
            self,
            ToPose2D,
            'pose_control',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
        )

        self.proportional_controller_ = ProportionalController()

        # Attributes to store the current goal and feedback
        self._goal_pose = None
        self._current_goal_handle = None
        self._goal_active = False

    def odometry_callback(self, msg: Odometry):
        """
        Callback for /odom topic. Converts Pose to Pose2D and computes velocity.
        """
        if not self._goal_active:
            return

        current_pose = self.pose_to_pose2d(msg.pose.pose)
        distance_to_goal = self.compute_distance(current_pose, self._goal_pose)

        # Publish feedback (distance to the goal)
        feedback_msg = ToPose2D.Feedback()
        feedback_msg.distance_to_goal = distance_to_goal
        self._current_goal_handle.publish_feedback(feedback_msg)

        # If we are close to the goal, complete the goal
        if distance_to_goal < 0.1:  # Threshold for reaching the goal
            result = ToPose2D.Result()
            result.result_pose = current_pose
            self._current_goal_handle.succeed()
            self._goal_active = False
            self.get_logger().info('Goal reached!')
            return

        # Compute and publish velocity towards the goal
        velocity = self.proportional_controller_.compute_velocity_towards_goal(
            current_pose, self._goal_pose
        )
        self.velocity_pub_.publish(velocity)

    def goal_callback(self, goal_request):
        """
        Callback to handle incoming goals.
        """
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        """
        Accept the goal and start processing.
        """
        self._goal_active = True
        self._current_goal_handle = goal_handle
        self._goal_pose = goal_handle.request.target_pose
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """
        Handle cancellation of a goal.
        """
        self.get_logger().info('Received goal cancel request')
        self._goal_active = False
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Execution callback to handle the control logic.
        """
        self.get_logger().info('Executing goal...')
        # Wait for the goal to finish or be canceled
        while self._goal_active:
            rclpy.spin_once(self, timeout_sec=1)

        # Once the goal is complete, return the result
        result = ToPose2D.Result()
        result.target_pose = self._goal_pose
        goal_handle.succeed()
        return result

    def pose_to_pose2d(self, pose: Pose) -> Pose2D:
        """
        Convert Pose to Pose2D.
        """
        pose2d = Pose2D()
        pose2d.x = pose.position.x
        pose2d.y = pose.position.y
        orientation_q = pose.orientation
        _, _, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )
        pose2d.theta = yaw
        return pose2d

    def compute_distance(self, current_pose: Pose2D, goal_pose: Pose2D) -> float:
        """
        Compute Euclidean distance between current pose and goal.
        """
        return math.sqrt(
            (goal_pose.x - current_pose.x)**2 + (goal_pose.y - current_pose.y)**2
        )


def main(args=None):
    rclpy.init(args=args)

    # Start the ControlRobot node
    control_robot = ControlRobot()

    try:
        rclpy.spin(control_robot)
    except KeyboardInterrupt:
        control_robot.get_logger().info('Action server shutting down')
    finally:
        control_robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
