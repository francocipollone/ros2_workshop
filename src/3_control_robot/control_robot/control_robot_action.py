import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Pose2D
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from .proportional_controller import ProportionalController  # Assuming you have this implemented
from action_msgs.msg import GoalStatus
from custom_interfaces.action import ToPose2D  # Import the custom action
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
import math


class ControlRobotAction(Node):
    """
    ROS 2 Action server that controls a robot in closed-loop using odometry and proportional control.
    """

    def __init__(self):
        super().__init__('control_robot_action_server')

        # Publisher for /cmd_vel
        self.velocity_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for /odom
        self.odometry_sub_ = self.create_subscription(Odometry, '/odom',
                                                      self.odometry_callback,
                                                      10)

        # Action server
        self.action_server_ = ActionServer(
            self,
            ToPose2D,
            'pose_control',
            # Callback a ser llamado para procesar el goal despues que es aceptado
            execute_callback=self.execute_action,
            # Callback a ser llamado cuando se recibe un goal.
            goal_callback=self.goal_callback,
            # Callback a ser llamado cuando un goal es aceptado y antes de llamar al execute_callback
            handle_accepted_callback=self.handle_accepted_callback,
            # Callback a ser llamado cuando se recibe un cancel request.
            cancel_callback=self.cancel_callback,
        )

        self.proportional_controller_ = ProportionalController()

        self.robot_odom_ = Odometry()
        # Attributes to store the current goal and feedback
        self.goal_pose_ = None
        self.goal_active_ = False

    def destroy(self):
        self.action_server_.destroy()
        super().destroy_node()

    def odometry_callback(self, msg: Odometry):
        """
        Callback for /odom topic
        """
        self.robot_odom_ = msg

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
        self.goal_active_ = True
        self.goal_pose_ = goal_handle.request.target_pose
        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        """
        Handle cancellation of a goal.
        """
        self.get_logger().info('Received goal cancel request')
        self.goal_active_ = False
        return CancelResponse.ACCEPT

    async def execute_action(self, goal_handle):
        """
        Execution callback to handle the control logic.
        """
        self.get_logger().info('Executing goal...')
        # Wait for the goal to finish or be canceled

        feedback_msg = ToPose2D.Feedback()
        rate = self.create_rate(15)
        while self.goal_active_ and rclpy.ok():
            current_pose = self.pose_to_pose2d(self.robot_odom_.pose.pose)

            # Calcular la velocidad a aplicar al robot para alcanzar el objetivo
            velocity = self.proportional_controller_.compute_velocity_towards_goal(
                current_pose, self.goal_pose_)
            self.velocity_pub_.publish(velocity)

            # Calcular la distancia al objetivo para publicar en el feedback
            distance_to_goal = self.compute_euclidean_distance_to_goal(
                current_pose, self.goal_pose_)
            feedback_msg.distance_to_goal = distance_to_goal
            feedback_msg.current_pose = current_pose
            goal_handle.publish_feedback(feedback_msg)

            # Once the goal is achieved, break the loop
            if self.proportional_controller_.is_at_goal_position(
                    current_pose, self.goal_pose_):
                self.get_logger().info('Goal achieved')
                break

            rate.sleep()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled')
            self.velocity_pub_.publish(Twist())
            return ToPose2D.Result()

        # Once the goal is completed return the result
        result = ToPose2D.Result()
        result.target_pose = self.pose_to_pose2d(self.robot_odom_.pose.pose)
        goal_handle.succeed()
        self.get_logger().info('Goal succeeded')
        return result

    def pose_to_pose2d(self, pose: Pose) -> Pose2D:
        """
        Convertir un mensaje `Pose` a mensaje `Pose2D`.
        """
        pose2d = Pose2D()
        pose2d.x = pose.position.x
        pose2d.y = pose.position.y

        # Extract yaw from quaternion orientation
        orientation_q = pose.orientation
        (_, _, yaw) = euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        pose2d.theta = yaw

        return pose2d

    def compute_euclidean_distance_to_goal(self, current_pose: Pose2D,
                                           goal_pose: Pose2D) -> float:
        """
        Computa la distancia euclideana (en línea recta) hacia el objetivo
        """
        delta_x = goal_pose.x - current_pose.x
        delta_y = goal_pose.y - current_pose.y
        return math.sqrt(delta_x**2 + delta_y**2)


def main(args=None):
    rclpy.init(args=args)

    # Start the ControlRobotAction node
    control_robot = ControlRobotAction()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(control_robot, executor=executor)
    except KeyboardInterrupt:
        control_robot.get_logger().info('Action server shutting down')
    finally:
        control_robot.destroy()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
