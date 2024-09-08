import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

# Heredo de la clase Node, alternativamente podria usar composition
# y recibir un objeto Node como parametro en el constructor.
class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.i = 0
        # Un publisher con tipo de Mensaje string y nombre de topic 'mi_primer_topico'
        # El tercer parametro es el tamaño de la cola de mensajes que se almacenará.
        self.pub = self.create_publisher(String, 'mi_primer_topico', 10)
        timer_period = 1.0
        # Crear un timer que llamará a la función timer_callback cada 1 segundo.
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hola Mundo: {0}'.format(self.i)
        self.i += 1
        self.get_logger().info('Publicando: "{0}"'.format(msg.data))

        # La función publish() es la forma en que enviás mensajes. El parámetro es
        # el objeto del mensaje. El tipo de este objeto debe coincidir con el tipo
        # dado como un parámetro del template para la llamada de create_publisher
        # como se hizo en el constructor de arriba.
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Talker()

    try:
        # Spin es una función que permite que el nodo siga funcionando:
        # - Chequeo de eventos
        # - Ejecución de callbacks debido a topics, servicios, timers, etc.
        # Este programa se quedara en rclpy.spin(node) hasta que se presione Ctrl+C con
        # eventuales saltos a los callbacks(timer_callback en este caso).
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
