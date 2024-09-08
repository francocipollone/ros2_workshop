import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist


def main(args=None):
    rclpy.init(args=args)
    node = Node('move_robot')
    pub = node.create_publisher(Twist, '/cmd_vel', 1000)
    rate = node.create_rate(10) # 10 Hz

    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = 0.0

    try:
        while(rclpy.ok()):
            # Incrementar la velociidad en 0.01 m/s
            vel_msg.linear.x += 0.01
            node.get_logger().info('Velocidad lineal en X: "{0}"'.format(vel_msg.linear.x))
            pub.publish(vel_msg)
            rclpy.spin_once(node)
            rate.sleep()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
