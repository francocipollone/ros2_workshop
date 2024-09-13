import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import Twist


# TODO: Implementar una clase Listener que herede de Node y que:
#       - Se subscriba al topico '/cmd_vel' con mensajes de tipo Twist
#       - Imprima en el log de ROS la velocidad lineal en X de los mensajes recibidos

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
