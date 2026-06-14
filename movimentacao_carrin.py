import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import math
from transforms3d.euler import quat2euler
from astar import convert_path_to_world_coords_in_expanded_maze

class PathFollower(Node):
    def __init__(self, path):
        super().__init__('path_follower')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.path = path
        self.current_pose = None
        self.current_index = 0

        self.timer = self.create_timer(0.05, self.follow_path)

        # Tolerâncias e ganhos
        self.DIST_TOLERANCE  = 0.2
        self.ANGLE_TOLERANCE = 0.02
        self.MAX_LINEAR      = 0.2
        self.MAX_ANGULAR     = 0.5
        self.KP_LINEAR       = 0.3
        self.KP_ANGULAR      = 1.2

        # Máquina de estados
        self.state = 'ROTATE'

        self.get_logger().info(f"🚀 Iniciando. Total de waypoints: {len(self.path)}")

    def odom_callback(self, msg):
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        )

    def get_yaw_from_quaternion(self, orientation):
        q = orientation
        yaw = quat2euler([q.w, q.x, q.y, q.z])[2]
        return yaw

    def normalize_angle(self, angle):
        """Garante que o ângulo esteja entre -π e π."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def follow_path(self):
        if self.current_pose is None:
            return

        # Para o robô e encerra ao chegar no destino
        if self.current_index >= len(self.path):
            self.publisher.publish(Twist())
            self.get_logger().info("🏁 Destino final alcançado!")
            return

        goal_x, goal_y = self.path[self.current_index]
        x, y, yaw = self.current_pose

        dx = goal_x - x
        dy = goal_y - y
        distance   = math.sqrt(dx**2 + dy**2)
        target_yaw = math.atan2(dy, dx)
        yaw_error  = self.normalize_angle(target_yaw - yaw)

        twist = Twist()

        if self.state == 'ROTATE':
            if abs(yaw_error) > self.ANGLE_TOLERANCE:
                twist.angular.z = max(-self.MAX_ANGULAR,
                                      min(self.MAX_ANGULAR,
                                          self.KP_ANGULAR * yaw_error))
            else:
                self.state = 'MOVE'
                self.get_logger().info(
                    f"➡️  Avançando para waypoint {self.current_index}: "
                    f"({goal_x:.1f}, {goal_y:.1f})"
                )

        elif self.state == 'MOVE':
            if distance > self.DIST_TOLERANCE:
                # Reduz velocidade proporcionalmente à distância
                speed = max(0.05, min(self.MAX_LINEAR,
                                      self.KP_LINEAR * distance))
                # Só corrige ângulo se desvio for grande, senão vai reto
                if abs(yaw_error) > 0.1:
                    twist.linear.x  = speed * 0.5
                    twist.angular.z = max(-self.MAX_ANGULAR,
                                          min(self.MAX_ANGULAR,
                                              self.KP_ANGULAR * yaw_error))
                else:
                    twist.linear.x  = speed
                    twist.angular.z = max(-self.MAX_ANGULAR * 0.3,
                                          min(self.MAX_ANGULAR * 0.3,
                                              self.KP_ANGULAR * yaw_error))
            else:
                self.get_logger().info(f"✅ Waypoint {self.current_index} alcançado!")
                self.current_index += 1
                self.state = 'ROTATE'

        self.publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    caminho = convert_path_to_world_coords_in_expanded_maze()
    print("📍 Caminho convertido:", caminho)
    path_follower = PathFollower(caminho)
    rclpy.spin(path_follower)
    path_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()