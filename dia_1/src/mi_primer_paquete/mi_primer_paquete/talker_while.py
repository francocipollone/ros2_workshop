# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String


class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.i = 0
        self.pub = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0
        self.tmr = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hola Mundo: {0}'.format(self.i)
        self.i += 1
        self.get_logger().info('Publishing: "{0}"'.format(msg.data))

        # La función publish() es la forma en que enviás mensajes. El parámetro es
        # el objeto del mensaje. El tipo de este objeto debe coincidir con el tipo
        # dado como un parámetro del template para la llamada de create_publisher
        # como se hizo en el constructor de arriba.
        self.pub.publish(msg)


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
