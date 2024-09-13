import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        # Un subscriber con tipo de Mensaje Twist y nombre de topic '/cmd_vel'
        # Cuando un mensaje es recibido, la función velocity_callback es llamada.
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)

    def velocity_callback(self, msg):
        # Cuando un mensaje es recibido, esta función solamente imprime el mensaje.
        self.get_logger().info('Velocidad lineal en X: "{0}"'.format(msg.linear.x))


def main(args=None):
    rclpy.init(args=args)

    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
