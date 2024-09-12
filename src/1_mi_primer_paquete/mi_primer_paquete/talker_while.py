import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


def main(args=None):
    rclpy.init(args=args)
    node = Node('talker')
    pub = node.create_publisher(String, 'mi_primer_topico', 10)
    rate = node.create_rate(1)
    try:
        while(rclpy.ok()):
            msg = String()
            msg.data = 'Hola Mundo'
            node.get_logger().info('Publicando: "{0}"'.format(msg.data))
            pub.publish(msg)
            rclpy.spin_once(node)
            rate.sleep()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
