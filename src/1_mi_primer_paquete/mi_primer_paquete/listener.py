import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        # Un subscriber con tipo de Mensaje string y nombre de topic 'mi_primer_topico'
        # Cuando un mensaje es recibido, la función chatter_callback es llamada.
        self.sub = self.create_subscription(String, 'mi_primer_topico', self.chatter_callback, 10)

    def chatter_callback(self, msg):
        # Cuando un mensaje es recibido, esta función solamente imprime el mensaje.
        self.get_logger().info('Escucho: [%s]' % msg.data)


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
