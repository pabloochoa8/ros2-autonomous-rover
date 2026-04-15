#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from ros2_aruco_interfaces.msg import ArucoMarkers # CAMBIO: Importamos el mensaje que contiene los IDs
import math

class ArucoProcessor(Node):
    def __init__(self):
        super().__init__('aruco_processor')
        
        # CAMBIO: Nos suscribimos a /aruco_markers para poder leer qué ID (número) estamos viendo
        self.sub = self.create_subscription(ArucoMarkers, '/aruco_markers', self.marker_cb, 10)
        
        self.pub_ekf = self.create_publisher(PoseWithCovarianceStamped, '/aruco_pose', 10)
        self.pub_initial = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.amcl_inicializado = False

        # ==========================================================
        # AQUÍ ESTÁ TU MAPA DE ARUCOS (ID: [X, Y, YAW_GLOBAL])
        # ==========================================================
        self.mapa_arucos = {
            # Pared norte (Y=7), miran hacia el Sur (-pi/2)
            10: (1.5, 7.0, -math.pi/2),
            11: (3.5, 7.0, -math.pi/2),
            12: (5.5, 7.0, -math.pi/2),
            # Pared oeste (X=0), miran hacia el Este (0.0)
             1: (0.0, 1.5, 0.0),
             2: (0.0, 3.5, 0.0),
             3: (0.0, 5.5, 0.0),
            # Pared sur (Y=0), miran hacia el Norte (pi/2)
            30: (1.5, 0.0, math.pi/2),
            31: (3.5, 0.0, math.pi/2),
            32: (5.5, 0.0, math.pi/2),
            # Pared este (X=7), miran hacia el Oeste (pi)
            50: (7.0, 1.5, math.pi),
            51: (7.0, 3.5, math.pi),
            52: (7.0, 5.5, math.pi),
        }
        
        self.get_logger().info("Puente ArUco Real (Modo ABSOLUTO) -> EKF iniciado...")

    def marker_cb(self, msg: ArucoMarkers):
        # Si no hay marcadores o la lista viene vacía, no hacemos nada
        if not msg.marker_ids:
            return

        # Procesamos TODOS los marcadores que la cámara esté viendo en este momento
        for i, marker_id in enumerate(msg.marker_ids):
            # Si el marcador que vemos no está en nuestro mapa, lo ignoramos
            if marker_id not in self.mapa_arucos:
                continue
                
            MARKER_X, MARKER_Y, MARKER_YAW = self.mapa_arucos[marker_id]

            # Tomamos la medición de la cámara para este marcador concreto
            marker_pose = msg.poses[i]
            
            z_c = marker_pose.position.z
            x_c = marker_pose.position.x
            q = marker_pose.orientation

            # Extraemos el vector normal del ArUco desde la vista de la cámara
            N_x = 2.0 * (q.x * q.z + q.w * q.y)
            N_z = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)

            # Calculamos el ángulo relativo del ArUco respecto a la orientación del robot
            alpha_R = math.atan2(-N_x, N_z)

            # Calculamos el Yaw global (brújula) del robot
            yaw_robot = MARKER_YAW - alpha_R
            yaw_robot = math.atan2(math.sin(yaw_robot), math.cos(yaw_robot)) # Normalizar entre -pi y pi

            # Calculamos la posición Global de la cámara
            cam_x = MARKER_X - z_c * math.cos(yaw_robot) - x_c * math.sin(yaw_robot)
            cam_y = MARKER_Y - z_c * math.sin(yaw_robot) + x_c * math.cos(yaw_robot)

            # Desplazamos el centro hacia el base_link (la cámara está 9cm adelante)
            base_x = cam_x - 0.09 * math.cos(yaw_robot)
            base_y = cam_y - 0.09 * math.sin(yaw_robot)
            
            # Creamos el mensaje para el EKF
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            
            pose_msg.pose.pose.position.x = base_x
            pose_msg.pose.pose.position.y = base_y
            pose_msg.pose.pose.position.z = 0.0
            
            # Publicamos el Yaw calculado en formato cuaternión
            pose_msg.pose.pose.orientation.z = math.sin(yaw_robot / 2.0)
            pose_msg.pose.pose.orientation.w = math.cos(yaw_robot / 2.0)

            pose_msg.pose.covariance[0] = 0.05
            pose_msg.pose.covariance[7] = 0.05
            pose_msg.pose.covariance[14] = 9999.0 # Ignorar Z
            pose_msg.pose.covariance[21] = 9999.0 # Ignorar Roll
            pose_msg.pose.covariance[28] = 9999.0 # Ignorar Pitch
            pose_msg.pose.covariance[35] = 0.1    # ¡AHORA SÍ confía en el Yaw de la cámara!

            self.pub_ekf.publish(pose_msg)
            self.get_logger().info(f"[ARUCO {marker_id}] Rover Localizado en -> X:{base_x:.2f}, Y:{base_y:.2f}, Yaw:{yaw_robot:.2f}")
            
            if not self.amcl_inicializado:
                init_msg = PoseWithCovarianceStamped()
                init_msg.header = pose_msg.header
                init_msg.pose.pose = pose_msg.pose.pose
                init_msg.pose.covariance = pose_msg.pose.covariance
                init_msg.pose.covariance[35] = 0.25 # Permitimos dispersión angular en AMCL
                self.pub_initial.publish(init_msg)
                self.amcl_inicializado = True
                self.get_logger().info("Partículas de AMCL inicializadas (Esto solo ocurre la primera vez).")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoProcessor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
