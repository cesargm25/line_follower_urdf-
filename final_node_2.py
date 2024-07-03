#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError

class LaneTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.current_speed = 0.0
        self.max_speed = 0.11
        self.speed_increment = 0.01

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

        # Aplicar máscaras para detectar líneas específicas
        white_lines = cv2.HoughLinesP(mask_white, 1, np.pi/180, 50, maxLineGap=50)
        yellow_lines = cv2.HoughLinesP(mask_yellow, 1, np.pi/180, 50, maxLineGap=50)

        if white_lines is not None and yellow_lines is not None:
            # Variables para almacenar las coordenadas de los puntos medios de las líneas de los carriles
            left_x_total = 0
            right_x_total = 0
            left_line_count = 0
            right_line_count = 0
            mid_y = cv_image.shape[0] // 2  # Usar la mitad inferior de la imagen para mayor precisión

            # Filtrar y dibujar las líneas amarillas (izquierda) y blancas (derecha)
            for line in yellow_lines:
                x1, y1, x2, y2 = line[0]
                if y1 > mid_y and y2 > mid_y:  # Filtrar líneas por su posición vertical
                    cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 255), 3)  # Amarillo
                    left_x_total += (x1 + x2) / 2
                    left_line_count += 1

            for line in white_lines:
                x1, y1, x2, y2 = line[0]
                if y1 > mid_y and y2 > mid_y:  # Filtrar líneas por su posición vertical
                    cv2.line(cv_image, (x1, y1), (x2, y2), (255, 255, 255), 3)  # Blanco
                    right_x_total += (x1 + x2) / 2
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

            # Aumentar la velocidad de giro proporcionalmente al error
            self.twist.angular.z = -float(err) / 100   # Multiplicamos por 2.0 para aumentar la velocidad angular
            self.cmd_vel_pub.publish(self.twist)
        else:
            # Si no se detectan líneas, girar en sentido contrario a la posición donde debería estar la línea blanca
            self.twist.linear.x = 0.0

            # Determinar la dirección del giro en base a la posición esperada de la línea blanca
            expected_lane_position = cv_image.shape[1] / 2
            if expected_lane_position <= cv_image.shape[1] / 2:  # Si la línea blanca debería estar a la izquierda
                self.twist.angular.z = 0.5  # Girar a la derecha
            else:
                self.twist.angular.z = -0.5  # Girar a la izquierda

            self.cmd_vel_pub.publish(self.twist)

        # Mostrar la imagen procesada
        cv2.imshow("Lane Detection", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('lane_tracker')
    lt = LaneTracker()
    rospy.spin()
