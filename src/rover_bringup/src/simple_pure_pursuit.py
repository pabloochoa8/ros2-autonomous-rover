#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from tf2_ros import Buffer, TransformListener
import math

class SimplePurePursuit(Node):
    def __init__(self):
        super().__init__('simple_pure_pursuit')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel_intuitive', 10)
        self.subscription = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        
        # Buffer de TF para saber dónde estamos en el mapa
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.goal = None
        
        # Parámetros del controlador
        self.max_v = 0.3      # Velocidad lineal máxima (m/s)
        self.max_w = 0.8      # Velocidad angular máxima (rad/s)
        self.kp_v = 0.5       # Constante proporcional para avanzar
        self.kp_w = 1.5       # Constante proporcional para girar
        self.dist_tol = 0.15  # Tolerancia de llegada (metros)

        self.get_logger().info("Controlador 'Chapuza' Iniciado. Manda un 2D Goal Pose desde RViz.")

    def goal_callback(self, msg):
        self.goal = msg.pose
        self.get_logger().info(f"Nuevo destino: X={self.goal.position.x:.2f}, Y={self.goal.position.y:.2f}")

    def get_current_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            
            # Cuaternión a Yaw manualmente (sin dependencias extra)
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            
            return x, y, yaw
        except Exception:
            return None

    def control_loop(self):
        if self.goal is None:
            return

        pose = self.get_current_pose()
        if pose is None:
            return
        
        x, y, yaw = pose
        dx = self.goal.position.x - x
        dy = self.goal.position.y - y
        dist = math.hypot(dx, dy)
        
        msg = Twist()
        if dist < self.dist_tol:
            self.get_logger().info("¡Destino alcanzado!")
            self.goal = None
            self.publisher_.publish(msg) # Parada total
            return
            
        target_yaw = math.atan2(dy, dx)
        yaw_error = target_yaw - yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error)) # Normalizar -pi a pi
        
        # Lógica chapuza: Primero me oriento, luego avanzo
        msg.angular.z = max(min(self.kp_w * yaw_error, self.max_w), -self.max_w)
        
        #if abs(yaw_error) < 0.3: # Si estamos mirando casi al objetivo, avanzamos
        #    msg.linear.x = max(min(self.kp_v * dist, self.max_v), -self.max_v)
        #else:
        #    msg.linear.x = 0.0 # Pivotar en el sitio
        # Avanzamos siempre. Si el error de ángulo es grande, reducimos un poco la velocidad 
        # lineal para que el radio de giro sea más cerrado, pero nunca bajamos del 30% de velocidad.
        factor_velocidad = max(0.3, 1.0 - abs(yaw_error) / 1.5)
        msg.linear.x = max(min(self.kp_v * dist, self.max_v), -self.max_v) * factor_velocidad
        
        # LOG DETALLADO DE NAVEGACIÓN (Cada medio segundo para no saturar)
        self.get_logger().info(
            f"[DEBUG] Pose_Actual: (X:{x:.2f}, Y:{y:.2f}, Yaw:{yaw:.2f}) | "
            f"Meta: (X:{self.goal.position.x:.2f}, Y:{self.goal.position.y:.2f}) | "
            f"Distancia: {dist:.2f}m | Error_Giro: {yaw_error:.2f}rad | "
            f"Enviando -> V:{msg.linear.x:.2f} m/s, W:{msg.angular.z:.2f} rad/s",
            throttle_duration_sec=0.5
        )
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePurePursuit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
