import rclpy
import sys
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Pose2D
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from .proportional_controller import ProportionalController

import argparse


class ControlRobot(Node):
    """
    Control a robot in closed-loop using its odometry.
    """

    def __init__(self, goal_pose: Pose2D):
        super().__init__('control_robot')
        self.goal_pose_ = goal_pose

        # Publisher para el tópico /cmd_vel
        self.velocity_pub_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber para el tópico /odom
        self.odometry_sub_ = self.create_subscription(Odometry, '/odom',
                                                      self.odometry_callback,
                                                      10)

        # Proportional controller instance
        self.proportional_controller_ = ProportionalController()

    def odometry_callback(self, msg: Odometry):
        """
        Método de devolución de llamada (callback) del tópico `/odom`.
        """
        current_pose = self.pose_to_pose2d(msg.pose.pose)

        self.get_logger().info(
            f"Posicion actual: ({current_pose.x}; {current_pose.y})")

        # Computa la velocidad a aplicar al robot para alcanzar el objetivo utilizando un control proporcional.
        velocity = self.proportional_controller_.compute_velocity_towards_goal(
            current_pose, self.goal_pose_)
        self.velocity_pub_.publish(velocity)

    def pose_to_pose2d(self, pose: Pose) -> Pose2D:
        """
        Convert a `Pose` message to a `Pose2D` message.
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


def main(argv=sys.argv[1:]):
    # Parseo de argumentos
    non_ros_args = rclpy.utilities.remove_ros_args(argv)
    parser = argparse.ArgumentParser(
        description=
        "Ejemplo de uso:\n\tros2 run control_robot control_robot -x 2.0 -y 2.0\n"
    )
    parser.add_argument('-x',
                        '--goal_x',
                        type=float,
                        default=2.0,
                        help='Goal position: X coordinate')
    parser.add_argument('-y',
                        '--goal_y',
                        type=float,
                        default=2.0,
                        help='Goal position: Y coordinate')
    args = parser.parse_args(args=non_ros_args)

    rclpy.init(args=argv)

    # Define la posición objetivo.
    goal_pose = Pose2D()
    goal_pose.x = args.goal_x
    goal_pose.y = args.goal_y

    # Crea la instancia del nodo ControlRobot
    control_robot = ControlRobot(goal_pose)

    try:
        rclpy.spin(control_robot)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        control_robot.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
