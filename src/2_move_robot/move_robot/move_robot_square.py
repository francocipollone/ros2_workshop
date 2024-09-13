import rclpy
from rclpy.node import Node
from math import pi
from geometry_msgs.msg import Twist


def draw_square(node, pub, largo_lado_cuadrado, velocidad_lineal,
                velocidad_angular):
    """Dibuja un cuadrado con el robot."""
    rate = node.create_rate(10)  # 10 Hz

    for i in range(4):
        move_forward(node, pub, rate, largo_lado_cuadrado, velocidad_lineal)
        rotate(node, pub, rate, pi / 2., velocidad_angular)


def move_forward(node, pub, rate, distancia_lineal, velocidad_lineal):
    node.get_logger().info(
        'Moviendo hacia adelante una distancia de {0}'.format(
            distancia_lineal))

    # TODO: Implementar el movimiento hacia adelante. Inspirarse en el
    # movimiento rotacional. (Metodo rotate)


def rotate(node, pub, rate, distancia_angular, velocidad_angular):
    vel_msg = Twist()
    vel_msg.linear.x = 0.0
    vel_msg.linear.y = 0.0
    vel_msg.linear.z = 0.0
    vel_msg.angular.x = 0.0
    vel_msg.angular.y = 0.0
    vel_msg.angular.z = velocidad_angular

    node.get_logger().info('Rotando el robot')

    angulo_rotado = 0.0
    tiempo_inicio = node.get_clock().now()
    while (rclpy.ok() and angulo_rotado < distancia_angular):
        pub.publish(vel_msg)
        rclpy.spin_once(node)
        rate.sleep()
        # El ángulo se estima a lazo abierto a partir de la velocidad y el
        # tiempo trascurrido (theta(t) = w*t).
        tiempo_fin = node.get_clock().now()
        angulo_rotado = velocidad_angular * (
            tiempo_fin - tiempo_inicio).nanoseconds / 1e9

    # Detener el robot
    node.get_logger().info('Deteniendo el robot')
    vel_msg.angular.z = 0.0
    pub.publish(vel_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Node('move_robot_square')
    pub = node.create_publisher(Twist, '/cmd_vel', 1000)

    largo_lado_cuadrado = 1.0
    velocidad_lineal = 0.5
    velocidad_angular = 0.1

    draw_square(node, pub, largo_lado_cuadrado, velocidad_lineal,
                velocidad_angular)


if __name__ == '__main__':
    main()
