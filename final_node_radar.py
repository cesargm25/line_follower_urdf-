#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError


class LaneTracker:
    def __init__(self):
        self.bridge = CvBridge()

        # Subscripción a la imagen cruda
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)

        # Publicación del comando de velocidad
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.current_speed = 0.0
        self.max_speed = 0.07
        self.speed_increment = 0.01

        # Subscripción al escaneo radar
        self.radar_sub = rospy.Subscriber('/radar_scan', LaserScan, self.radar_callback)

    def image_callback(self, msg):
        try:
            # Convertir el mensaje de ROS Image a una imagen de OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

        # Procesar la imagen para detectar líneas blancas y amarillas
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Detección de líneas blancas
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([255, 50, 255])
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        
        # Detección de líneas amarillas
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Combinar ambas máscaras para detectar ambas líneas simultáneamente
        combined_mask = cv2.bitwise_or(mask_white, mask_yellow)

        blur = cv2.GaussianBlur(combined_mask, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # Transformada de Hough para detectar líneas
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)

        if lines is not None:
            # Variables para almacenar las coordenadas de los puntos medios de las líneas de los carriles
            left_x_total = 0
            right_x_total = 0
            left_line_count = 0
            right_line_count = 0
            mid_y = cv_image.shape[0] // 2  # Usar la mitad inferior de la imagen para mayor precisión

            for line in lines:
                x1, y1, x2, y2 = line[0]
                if y1 > mid_y and y2 > mid_y:  # Filtrar líneas por su posición vertical
                    cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

                    # Diferenciar entre líneas de carril izquierdo y derecho
                    mid_x = (x1 + x2) / 2
                    if mid_x < cv_image.shape[1] / 2:
                        left_x_total += mid_x
                        left_line_count += 1
                    else:
                        right_x_total += mid_x
                        right_line_count += 1

            # Calcula el promedio de las coordenadas de las líneas de carril
            left_x_avg = left_x_total / left_line_count if left_line_count > 0 else 0
            right_x_avg = right_x_total / right_line_count if right_line_count > 0 else cv_image.shape[1]

            # Calcula el centro del carril
            lane_center = (left_x_avg + right_x_avg) / 2

            # Usa la posición del centro del carril para ajustar la velocidad del robot
            err = lane_center - cv_image.shape[1] / 2
            self.current_speed = min(self.current_speed + self.speed_increment, self.max_speed)
            self.twist.linear.x = self.current_speed
            self.twist.angular.z = -float(err) / 100
            self.cmd_vel_pub.publish(self.twist)
        else:
            # Si no se detectan líneas, seguir el patrón de movimiento especificado

            # Definir el tiempo para girar y desplazarse en x
            rotate_time = 0.52  # Tiempo en segundos para girar
            move_time = 0.2  # Tiempo en segundos para desplazarse en x

            # Variable para alternar entre movimientos de giro y desplazamiento
            if not hasattr(self, 'last_movement_time'):
                self.last_movement_time = rospy.get_time()
                self.is_rotating = True  # Iniciar con un movimiento de giro
                self.rotate_direction = 1  # 1 para girar en sentido horario, -1 para girar en sentido antihorario

            # Determinar el tipo de movimiento según el tiempo transcurrido
            current_time = rospy.get_time()
            elapsed_time = current_time - self.last_movement_time

            if self.is_rotating:
                # Girar durante rotate_time segundos en la dirección actual
                self.twist.linear.x = 0.0
                self.twist.angular.z = self.rotate_direction * 0.5  # Velocidad angular para el giro actual
                if elapsed_time >= rotate_time:
                    # Cambiar dirección de giro
                    self.rotate_direction *= -1
                    self.last_movement_time = rospy.get_time()
            else:
                # Desplazarse en x durante move_time segundos (x positivo)
                self.twist.linear.x = 0.2  # Velocidad lineal para desplazarse en x
                self.twist.angular.z = 0.0
                if elapsed_time >= move_time:
                    self.is_rotating = True
                    self.last_movement_time = rospy.get_time()

            # Publicar el comando de velocidad
            self.cmd_vel_pub.publish(self.twist)

        # Mostrar la imagen procesada
        cv2.imshow("Lane Detection", cv_image)
        cv2.waitKey(3)

    def radar_callback(self, msg):
        # Procesar los datos del escaneo radar
        ranges = msg.ranges
        rospy.loginfo("Radar Scan Ranges: {}".format(ranges))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lane_tracker')
    lt = LaneTracker()
    lt.spin()
