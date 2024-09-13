import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist


def main(args=None):
    rclpy.init(args=args)
    node = Node('move_robot')
    rate = node.create_rate(10)  # 10 Hz

    # TODO: Crear un publicador para el topico '/cmd_vel' que publique mensajes de tipo Twist
    # Definicion de mensaje Twist: https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html

    try:
        while (rclpy.ok()):
            # TODO: Implementar la logica para avanzar el robot en linea recta incrementando la velocidad lineal en X.








            node.get_logger().info('Velocidad lineal en X: "{0}"'.format(vel_msg.linear.x))
            rclpy.spin_once(node)
            rate.sleep()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
