import math
from geometry_msgs.msg import Pose2D, Twist


class ProportionalController:
    """
    Controla un robot a lazo cerrado a partir de su posición actual y la posición
    objetivo, utilizando un controlador proporcional.
    """

    def __init__(self):
        self.k_linear_speed_constant = 0.1
        self.k_angular_speed_constant = 0.2
        self.k_euclidean_distance_to_goal_tolerance = 0.1

    def compute_velocity_towards_goal(self, current_pose: Pose2D,
                                      goal_pose: Pose2D) -> Twist:
        """
        Computa la velocidad a aplicar al robot para alcanzar el objetivo.
        """
        velocity = self._get_zero_velocity()
        if self.is_at_goal_position(current_pose, goal_pose):
            return velocity

        velocity.linear.x = self._compute_linear_speed_to_goal(
            current_pose, goal_pose)
        velocity.angular.z = self._compute_angular_speed_to_look_at_goal(
            current_pose, goal_pose)
        return velocity

    def _get_zero_velocity(self) -> Twist:
        """
        Devuelve un mensaje `geometry_msgs::Twist` con todos sus campos inicializados en cero
        """
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = 0.0
        return velocity

    def is_at_goal_position(self, current_pose: Pose2D,
                             goal_pose: Pose2D) -> bool:
        """
        Devuelve `true` si el robot se encuentra en la posición objetivo.
        """
        return self._compute_euclidean_distance_to_goal(
            current_pose,
            goal_pose) < self.k_euclidean_distance_to_goal_tolerance

    def _compute_linear_speed_to_goal(self, current_pose: Pose2D,
                                      goal_pose: Pose2D) -> float:
        """
        Computa la componente lineal de la velocidad hacia el objetivo.
        """
        return self.k_linear_speed_constant * self._compute_euclidean_distance_to_goal(
            current_pose, goal_pose)

    def _compute_euclidean_distance_to_goal(self, current_pose: Pose2D,
                                           goal_pose: Pose2D) -> float:
        """
        Computa la distancia euclideana (en línea recta) hacia el objetivo
        """
        delta_x = goal_pose.x - current_pose.x
        delta_y = goal_pose.y - current_pose.y
        return math.sqrt(delta_x**2 + delta_y**2)

    def _compute_angular_speed_to_look_at_goal(self, current_pose: Pose2D,
                                               goal_pose: Pose2D) -> float:
        """
        Computa la componente angular de la velocidad hacia el objetivo.
        """
        return self.k_angular_speed_constant * self._compute_angular_distance_to_look_at_goal(
            current_pose, goal_pose)

    def _compute_angular_distance_to_look_at_goal(self, current_pose: Pose2D,
                                                  goal_pose: Pose2D) -> float:
        """
        Computa la distancia angular para ubicar al robot mirando hacia el objetivo.
        """
        steering_angle = math.atan2(goal_pose.y - current_pose.y,
                                    goal_pose.x - current_pose.x)
        return self._normalize_angle(steering_angle - current_pose.theta)

    def _normalize_angle(self, angle: float) -> float:
        """
        Normaliza un ángulo en el rango [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
