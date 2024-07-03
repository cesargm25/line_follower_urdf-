#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RadarBasedNavigator:
    def __init__(self):
        # Publicación del comando de velocidad
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.current_speed = 0.0
        self.max_speed = 0.07
        self.speed_increment = 0.01

        # Subscripción al escaneo radar
        self.radar_sub = rospy.Subscriber('/radar_scan', LaserScan, self.radar_callback)

    def radar_callback(self, msg):
        # Procesar los datos del escaneo radar
        ranges = msg.ranges

        # Verificar si alguna de las lecturas del radar es menor a 0.2 metros
        obstacle_detected = any(distance < 0.2 for distance in ranges)

        if obstacle_detected:
            # Si hay un obstáculo cercano, girar y desplazarse lateralmente
            self.twist.linear.x = 0.02
            self.twist.angular.z = 0.5  # Gira para evitar el obstáculo
        else:
            # Si no hay obstáculos cercanos, moverse hacia adelante
            self.current_speed = min(self.current_speed + self.speed_increment, self.max_speed)
            self.twist.linear.x = self.current_speed
            self.twist.angular.z = 0.0  # Continúa en línea recta

        # Publicar el comando de velocidad
        self.cmd_vel_pub.publish(self.twist)
        rospy.loginfo("Radar Scan Ranges: {}".format(ranges))

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('radar_based_navigator')
    rbn = RadarBasedNavigator()
    rbn.spin()



# import rospy
# import cv2
# import numpy as np
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge, CvBridgeError

# class LaneTracker:
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
#         self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#         self.twist = Twist()
#         self.current_speed = 0.0
#         self.max_speed = 0.07
#         self.speed_increment = 0.01
#         self.yellow_detected = False
#         self.white_detected = False

#     def image_callback(self, msg):
#         try:
#             # Convertir el mensaje de ROS Image a una imagen de OpenCV
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))
#             return
        
#         # Procesar la imagen para detectar líneas blancas y amarillas
#         hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

#         # Detección de líneas blancas
#         lower_white = np.array([0, 0, 200])
#         upper_white = np.array([255, 50, 255])
#         mask_white = cv2.inRange(hsv, lower_white, upper_white)
        
#         # Detección de líneas amarillas
#         lower_yellow = np.array([20, 100, 100])
#         upper_yellow = np.array([30, 255, 255])
#         mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

#         # Aplicar máscaras para detectar líneas específicas
#         white_lines = cv2.HoughLinesP(mask_white, 1, np.pi/180, 50, maxLineGap=50)
#         yellow_lines = cv2.HoughLinesP(mask_yellow, 1, np.pi/180, 50, maxLineGap=50)

#         # Reiniciar las variables de detección en cada iteración
#         self.yellow_detected = False
#         self.white_detected = False

#         if yellow_lines is not None:
#             # Dibujar las líneas amarillas (izquierda) y contarlas
#             for line in yellow_lines:
#                 x1, y1, x2, y2 = line[0]
#                 if y1 > cv_image.shape[0] // 2 and y2 > cv_image.shape[0] // 2:  # Filtrar líneas por su posición vertical
#                     cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 255), 3)  # Amarillo
#                     self.yellow_detected = True

#         if white_lines is not None:
#             # Dibujar las líneas blancas (derecha) y contarlas
#             for line in white_lines:
#                 x1, y1, x2, y2 = line[0]
#                 if y1 > cv_image.shape[0] // 2 and y2 > cv_image.shape[0] // 2:  # Filtrar líneas por su posición vertical
#                     cv2.line(cv_image, (x1, y1), (x2, y2), (255, 255, 255), 3)  # Blanco
#                     self.white_detected = True

#         # Lógica de control de movimiento del robot
#         if self.yellow_detected and not self.white_detected:
#             # Si solo se detecta línea amarilla, girar en sentido horario
#             self.twist.linear.x = 0.0
#             self.twist.angular.z = -0.5  # Girar a la izquierda
#         elif not self.yellow_detected and self.white_detected:
#             # Si solo se detecta línea blanca, girar en sentido antihorario
#             self.twist.linear.x = 0.0
#             self.twist.angular.z = 0.5  # Girar a la derecha
#         elif self.yellow_detected and self.white_detected:
#             # Si se detectan ambas líneas, moverse con normalidad
#             lane_center = self.calculate_lane_center(cv_image, yellow_lines, white_lines)
#             err = lane_center - cv_image.shape[1] / 2
#             self.current_speed = min(self.current_speed + self.speed_increment, self.max_speed)
#             self.twist.linear.x = self.current_speed
#             self.twist.angular.z = -float(err) / 100 * 2.0  # Ajuste proporcional del giro
#         else:
#             # Si no se detecta ninguna línea, girar en sentido contrario a la posición donde debería estar la línea blanca
#             self.twist.linear.x = 0.0
#             expected_lane_position = cv_image.shape[1] / 2
#             if expected_lane_position <= cv_image.shape[1] / 2:
#                 self.twist.angular.z = 0.5  # Girar a la derecha
#             else:
#                 self.twist.angular.z = -0.5  # Girar a la izquierda

#         # Publicar los comandos de velocidad
#         self.cmd_vel_pub.publish(self.twist)

#         # Mostrar la imagen procesada
#         cv2.imshow("Lane Detection", cv_image)
#         cv2.waitKey(3)

#     def calculate_lane_center(self, cv_image, yellow_lines, white_lines):
#         # Función para calcular el centro del carril
#         left_x_total = 0
#         right_x_total = 0
#         left_line_count = 0
#         right_line_count = 0
#         mid_y = cv_image.shape[0] // 2  # Usar la mitad inferior de la imagen para mayor precisión

#         # Procesar líneas amarillas (izquierda)
#         for line in yellow_lines:
#             x1, y1, x2, y2 = line[0]
#             if y1 > mid_y and y2 > mid_y:  # Filtrar líneas por su posición vertical
#                 left_x_total += (x1 + x2) / 2
#                 left_line_count += 1

#         # Procesar líneas blancas (derecha)
#         for line in white_lines:
#             x1, y1, x2, y2 = line[0]
#             if y1 > mid_y and y2 > mid_y:  # Filtrar líneas por su posición vertical
#                 right_x_total += (x1 + x2) / 2
#                 right_line_count += 1

#         # Calcula el promedio de las coordenadas de las líneas de carril
#         left_x_avg = left_x_total / left_line_count if left_line_count > 0 else 0
#         right_x_avg = right_x_total / right_line_count if right_line_count > 0 else cv_image.shape[1]

#         # Calcula el centro del carril
#         lane_center = (left_x_avg + right_x_avg) / 2
#         return lane_center

# if __name__ == '__main__':
#     rospy.init_node('lane_tracker')
#     lt = LaneTracker()
#     rospy.spin()
