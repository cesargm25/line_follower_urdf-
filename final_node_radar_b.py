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
        self.max_speed = 0.1
        self.speed_increment = 0.01

        # Subscripción al escaneo radar
        self.radar_sub = rospy.Subscriber('/radar_scan', LaserScan, self.radar_callback)

        # Datos del radar
        self.radar_ranges = []
        self.last_line_detection_time = rospy.get_time()

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
            self.last_line_detection_time = rospy.get_time()

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
        else:
            # Si no se detectan líneas, seguir el patrón de movimiento especificado
            if rospy.get_time() - self.last_line_detection_time > 6:
                self.twist.linear.x = 0.1
                self.twist.angular.z = 0.0
            else:
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

        # Mostrar la imagen procesada
        cv2.imshow("Lane Detection", cv_image)
        cv2.waitKey(3)

        # Publicar el comando de velocidad
        self.cmd_vel_pub.publish(self.twist)

    def radar_callback(self, msg):
        # Procesar los datos del escaneo radar
        self.radar_ranges = msg.ranges
        max_distance = max(self.radar_ranges)
        max_index = self.radar_ranges.index(max_distance)

        rospy.loginfo("Radar Scan Ranges: {}".format(self.radar_ranges))
        rospy.loginfo("Maximum Distance: {:.2f} at index {}".format(max_distance, max_index))

        num_ranges = len(self.radar_ranges)  # Aquí se obtiene el número total de lecturas
        mid_index = num_ranges // 2

        # Calcular el ángulo de la lectura con la distancia máxima
        min_angle = -1.57
        max_angle = 1.57
        angle_increment = (max_angle - min_angle) / (num_ranges - 1)
        max_angle_value = min_angle + max_index * angle_increment

        rospy.loginfo("Maximum Distance Angle: {:.2f} radians".format(max_angle_value))

        if max_index < mid_index:
            # Gira en sentido antihorario si el espacio libre está a la izquierda
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.5
        elif max_index > mid_index:
            # Gira en sentido horario si el espacio libre está a la derecha
            self.twist.linear.x = 0.2
            self.twist.angular.z = -0.5
        else:
            # Si el espacio libre está en el centro, moverse hacia adelante
            self.twist.linear.x = 0.2
            self.twist.angular.z = 0.0

        self.cmd_vel_pub.publish(self.twist)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('lane_tracker')
    lt = LaneTracker()
    lt.spin()


















#---------------------------------------
# import rospy
# import cv2
# import numpy as np
# from sensor_msgs.msg import Image, LaserScan
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge, CvBridgeError

# class LaneTracker:
#     def __init__(self):
#         self.bridge = CvBridge()

#         # Subscripción a la imagen cruda
#         self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)

#         # Publicación del comando de velocidad
#         self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#         self.twist = Twist()
#         self.current_speed = 0.0
#         self.max_speed = 0.1
#         self.speed_increment = 0.01

#         # Subscripción al escaneo radar
#         self.radar_sub = rospy.Subscriber('/radar_scan', LaserScan, self.radar_callback)

#         # Datos del radar
#         self.radar_ranges = []
#         self.last_line_detection_time = rospy.get_time()

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

#         # Combinar ambas máscaras para detectar ambas líneas simultáneamente
#         combined_mask = cv2.bitwise_or(mask_white, mask_yellow)

#         blur = cv2.GaussianBlur(combined_mask, (5, 5), 0)
#         edges = cv2.Canny(blur, 50, 150)

#         # Transformada de Hough para detectar líneas
#         lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)

#         if lines is not None:
#             self.last_line_detection_time = rospy.get_time()

#             # Variables para almacenar las coordenadas de los puntos medios de las líneas de los carriles
#             left_x_total = 0
#             right_x_total = 0
#             left_line_count = 0
#             right_line_count = 0
#             mid_y = cv_image.shape[0] // 2  # Usar la mitad inferior de la imagen para mayor precisión

#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 if y1 > mid_y and y2 > mid_y:  # Filtrar líneas por su posición vertical
#                     cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

#                     # Diferenciar entre líneas de carril izquierdo y derecho
#                     mid_x = (x1 + x2) / 2
#                     if mid_x < cv_image.shape[1] / 2:
#                         left_x_total += mid_x
#                         left_line_count += 1
#                     else:
#                         right_x_total += mid_x
#                         right_line_count += 1

#             # Calcula el promedio de las coordenadas de las líneas de carril
#             left_x_avg = left_x_total / left_line_count if left_line_count > 0 else 0
#             right_x_avg = right_x_total / right_line_count if right_line_count > 0 else cv_image.shape[1]

#             # Calcula el centro del carril
#             lane_center = (left_x_avg + right_x_avg) / 2

#             # Usa la posición del centro del carril para ajustar la velocidad del robot
#             err = lane_center - cv_image.shape[1] / 2
#             self.current_speed = min(self.current_speed + self.speed_increment, self.max_speed)
#             self.twist.linear.x = self.current_speed
#             self.twist.angular.z = -float(err) / 100
#         else:
#             # Si no se detectan líneas, seguir el patrón de movimiento especificado
#             if rospy.get_time() - self.last_line_detection_time > 6:
#                 self.twist.linear.x = 0.1
#                 self.twist.angular.z = 0.0
#             else:
#                 # Definir el tiempo para girar y desplazarse en x
#                 rotate_time = 0.52  # Tiempo en segundos para girar
#                 move_time = 0.2  # Tiempo en segundos para desplazarse en x

#                 # Variable para alternar entre movimientos de giro y desplazamiento
#                 if not hasattr(self, 'last_movement_time'):
#                     self.last_movement_time = rospy.get_time()
#                     self.is_rotating = True  # Iniciar con un movimiento de giro
#                     self.rotate_direction = 1  # 1 para girar en sentido horario, -1 para girar en sentido antihorario

#                 # Determinar el tipo de movimiento según el tiempo transcurrido
#                 current_time = rospy.get_time()
#                 elapsed_time = current_time - self.last_movement_time

#                 if self.is_rotating:
#                     # Girar durante rotate_time segundos en la dirección actual
#                     self.twist.linear.x = 0.0
#                     self.twist.angular.z = self.rotate_direction * 0.5  # Velocidad angular para el giro actual
#                     if elapsed_time >= rotate_time:
#                         # Cambiar dirección de giro
#                         self.rotate_direction *= -1
#                         self.last_movement_time = rospy.get_time()
#                 else:
#                     # Desplazarse en x durante move_time segundos (x positivo)
#                     self.twist.linear.x = 0.2  # Velocidad lineal para desplazarse en x
#                     self.twist.angular.z = 0.0
#                     if elapsed_time >= move_time:
#                         self.is_rotating = True
#                         self.last_movement_time = rospy.get_time()

#         # Mostrar la imagen procesada
#         cv2.imshow("Lane Detection", cv_image)
#         cv2.waitKey(3)

#         # Publicar el comando de velocidad
#         self.cmd_vel_pub.publish(self.twist)

#     def radar_callback(self, msg):
#         # Procesar los datos del escaneo radar
#         self.radar_ranges = msg.ranges
#         min_distance = min(self.radar_ranges)
#         min_index = self.radar_ranges.index(min_distance)

#         rospy.loginfo("Radar Scan Ranges: {}".format(self.radar_ranges))
#         rospy.loginfo("Minimum Distance: {:.2f} at index {}".format(min_distance, min_index))

#         setpoint = 0.22  # Punto de referencia para evitar obstáculos

#         if min_distance < setpoint:
#             num_ranges = len(self.radar_ranges)  # Aquí se obtiene el número total de lecturas
#             mid_index = num_ranges // 2

#             # Calcular el ángulo de la lectura con la distancia mínima
#             min_angle = -1.57
#             max_angle = 1.57
#             angle_increment = (max_angle - min_angle) / (num_ranges - 1)
#             min_angle_value = min_angle + min_index * angle_increment

#             rospy.loginfo("Minimum Distance Angle: {:.2f} radians".format(min_angle_value))

#             if min_index < mid_index:
#                 # Gira en sentido antihorario si el obstáculo está a la izquierda
#                 self.twist.linear.x = 0.09
#                 self.twist.angular.z = 0.5
#             elif min_index > mid_index:
#                 # Gira en sentido horario si el obstáculo está a la derecha
#                 self.twist.linear.x = 0.09
#                 self.twist.angular.z = -0.5
#             else:
#                 # Si el obstáculo está en el centro, moverse hacia el lado con la distancia más lejana
#                 max_distance = max(self.radar_ranges)
#                 max_index = self.radar_ranges.index(max_distance)

#                 if max_index < mid_index:
#                     self.twist.linear.x = 0.2
#                     self.twist.angular.z = 0.5
#                 else:
#                     self.twist.linear.x = 0.2
#                     self.twist.angular.z = -0.5

#             self.cmd_vel_pub.publish(self.twist)

#     def spin(self):
#         rospy.spin()

# if __name__ == '__main__':
#     rospy.init_node('lane_tracker')
#     lt = LaneTracker()
#     lt.spin()












# import rospy
# import cv2
# import numpy as np
# from sensor_msgs.msg import Image, LaserScan
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge, CvBridgeError

# class LaneTracker:
#     def __init__(self):
#         self.bridge = CvBridge()

#         # Subscripción a la imagen cruda
#         self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)

#         # Publicación del comando de velocidad
#         self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#         self.twist = Twist()
#         self.current_speed = 0.0
#         self.max_speed = 0.1
#         self.speed_increment = 0.01

#         # Subscripción al escaneo radar
#         self.radar_sub = rospy.Subscriber('/radar_scan', LaserScan, self.radar_callback)

#         # Datos del radar
#         self.radar_ranges = []

#         # Tiempo para detección de líneas
#         self.last_line_detection_time = rospy.get_time()
#         self.line_detection_timeout = 6.0  # 6 segundos

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

#         # Combinar ambas máscaras para detectar ambas líneas simultáneamente
#         combined_mask = cv2.bitwise_or(mask_white, mask_yellow)

#         blur = cv2.GaussianBlur(combined_mask, (5, 5), 0)
#         edges = cv2.Canny(blur, 50, 150)

#         # Transformada de Hough para detectar líneas
#         lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)

#         if lines is not None:
#             # Reiniciar el temporizador de detección de líneas
#             self.last_line_detection_time = rospy.get_time()

#             # Variables para almacenar las coordenadas de los puntos medios de las líneas de los carriles
#             left_x_total = 0
#             right_x_total = 0
#             left_line_count = 0
#             right_line_count = 0
#             mid_y = cv_image.shape[0] // 2  # Usar la mitad inferior de la imagen para mayor precisión

#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 if y1 > mid_y and y2 > mid_y:  # Filtrar líneas por su posición vertical
#                     cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

#                     # Diferenciar entre líneas de carril izquierdo y derecho
#                     mid_x = (x1 + x2) / 2
#                     if mid_x < cv_image.shape[1] / 2:
#                         left_x_total += mid_x
#                         left_line_count += 1
#                     else:
#                         right_x_total += mid_x
#                         right_line_count += 1

#             # Calcula el promedio de las coordenadas de las líneas de carril
#             left_x_avg = left_x_total / left_line_count if left_line_count > 0 else 0
#             right_x_avg = right_x_total / right_line_count if right_line_count > 0 else cv_image.shape[1]

#             # Calcula el centro del carril
#             lane_center = (left_x_avg + right_x_avg) / 2

#             # Usa la posición del centro del carril para ajustar la velocidad del robot
#             err = lane_center - cv_image.shape[1] / 2
#             self.current_speed = min(self.current_speed + self.speed_increment, self.max_speed)
#             self.twist.linear.x = self.current_speed
#             self.twist.angular.z = -float(err) / 100
#         else:
#             # Verificar si han pasado más de 6 segundos sin detección de líneas
#             current_time = rospy.get_time()
#             if current_time - self.last_line_detection_time > self.line_detection_timeout:
#                 self.twist.linear.x = 0.1
#                 self.twist.angular.z = 0.0
#                 print('-----------------------------------------',current_time,'------',self.last_line_detection_time)
#             else:
#                 # Si no se detectan líneas, seguir el patrón de movimiento especificado

#                 # Definir el tiempo para girar y desplazarse en x
#                 rotate_time = 0.52  # Tiempo en segundos para girar
#                 move_time = 0.2  # Tiempo en segundos para desplazarse en x

#                 # Variable para alternar entre movimientos de giro y desplazamiento
#                 if not hasattr(self, 'last_movement_time'):
#                     self.last_movement_time = rospy.get_time()
#                     self.is_rotating = True  # Iniciar con un movimiento de giro
#                     self.rotate_direction = 1  # 1 para girar en sentido horario, -1 para girar en sentido antihorario

#                 # Determinar el tipo de movimiento según el tiempo transcurrido
#                 current_time = rospy.get_time()
#                 elapsed_time = current_time - self.last_movement_time

#                 if self.is_rotating:
#                     # Girar durante rotate_time segundos en la dirección actual
#                     self.twist.linear.x = 0.0
#                     self.twist.angular.z = self.rotate_direction * 0.5  # Velocidad angular para el giro actual
#                     if elapsed_time >= rotate_time:
#                         # Cambiar dirección de giro
#                         self.rotate_direction *= -1
#                         self.last_movement_time = rospy.get_time()
#                 else:
#                     # Desplazarse en x durante move_time segundos (x positivo)
#                     self.twist.linear.x = 0.2  # Velocidad lineal para desplazarse en x
#                     self.twist.angular.z = 0.0
#                     if elapsed_time >= move_time:
#                         self.is_rotating = True
#                         self.last_movement_time = rospy.get_time()

#         # Mostrar la imagen procesada
#         cv2.imshow("Lane Detection", cv_image)
#         cv2.waitKey(3)

#         # Publicar el comando de velocidad
#         self.cmd_vel_pub.publish(self.twist)

#     def radar_callback(self, msg):
#         # Procesar los datos del escaneo radar
#         self.radar_ranges = msg.ranges
#         min_distance = min(self.radar_ranges)
#         min_index = self.radar_ranges.index(min_distance)

#         rospy.loginfo("Radar Scan Ranges: {}".format(self.radar_ranges))
#         rospy.loginfo("Minimum Distance: {:.2f} at index {}".format(min_distance, min_index))

#         if min_distance < 0.09:
#             num_ranges = len(self.radar_ranges)
#             mid_index = num_ranges // 2

#             if min_index < mid_index:
#                 # Gira en sentido antihorario si el obstáculo está a la izquierda
#                 self.twist.linear.x = 0.09
#                 self.twist.angular.z = 0.5
#             elif min_index > mid_index:
#                 # Gira en sentido horario si el obstáculo está a la derecha
#                 self.twist.linear.x = 0.09
#                 self.twist.angular.z = -0.5
#             else:
#                 # Si el obstáculo está en el centro, moverse hacia el lado con la distancia más lejana
#                 max_distance = max(self.radar_ranges)
#                 max_index = self.radar_ranges.index(max_distance)

#                 if max_index < mid_index:
#                     self.twist.linear.x = 0.2
#                     self.twist.angular.z = 0.5
#                 else:
#                     self.twist.linear.x = 0.2
#                     self.twist.angular.z = -0.5

#             self.cmd_vel_pub.publish(self.twist)

#     def spin(self):
#         rospy.spin()

# if __name__ == '__main__':
#     rospy.init_node('lane_tracker')
#     lt = LaneTracker()
#     lt.spin()



#-------it works------ it has no obstacule avoidence 
# import rospy
# import cv2
# import numpy as np
# from sensor_msgs.msg import Image, LaserScan
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge, CvBridgeError

# class LaneTracker:
#     def __init__(self):
#         self.bridge = CvBridge()

#         # Subscripción a la imagen cruda
#         self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)

#         # Publicación del comando de velocidad
#         self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#         self.twist = Twist()
#         self.current_speed = 0.0
#         self.max_speed = 0.1
#         self.speed_increment = 0.01

#         # Subscripción al escaneo radar
#         self.radar_sub = rospy.Subscriber('/radar_scan', LaserScan, self.radar_callback)

#         # Datos del radar
#         self.radar_ranges = []

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

#         # Combinar ambas máscaras para detectar ambas líneas simultáneamente
#         combined_mask = cv2.bitwise_or(mask_white, mask_yellow)

#         blur = cv2.GaussianBlur(combined_mask, (5, 5), 0)
#         edges = cv2.Canny(blur, 50, 150)

#         # Transformada de Hough para detectar líneas
#         lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)

#         if lines is not None:
#             # Variables para almacenar las coordenadas de los puntos medios de las líneas de los carriles
#             left_x_total = 0
#             right_x_total = 0
#             left_line_count = 0
#             right_line_count = 0
#             mid_y = cv_image.shape[0] // 2  # Usar la mitad inferior de la imagen para mayor precisión

#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 if y1 > mid_y and y2 > mid_y:  # Filtrar líneas por su posición vertical
#                     cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3)

#                     # Diferenciar entre líneas de carril izquierdo y derecho
#                     mid_x = (x1 + x2) / 2
#                     if mid_x < cv_image.shape[1] / 2:
#                         left_x_total += mid_x
#                         left_line_count += 1
#                     else:
#                         right_x_total += mid_x
#                         right_line_count += 1

#             # Calcula el promedio de las coordenadas de las líneas de carril
#             left_x_avg = left_x_total / left_line_count if left_line_count > 0 else 0
#             right_x_avg = right_x_total / right_line_count if right_line_count > 0 else cv_image.shape[1]

#             # Calcula el centro del carril
#             lane_center = (left_x_avg + right_x_avg) / 2

#             # Usa la posición del centro del carril para ajustar la velocidad del robot
#             err = lane_center - cv_image.shape[1] / 2
#             self.current_speed = min(self.current_speed + self.speed_increment, self.max_speed)
#             self.twist.linear.x = self.current_speed
#             self.twist.angular.z = -float(err) / 100
#         else:
#             # Si no se detectan líneas, seguir el patrón de movimiento especificado

#             # Definir el tiempo para girar y desplazarse en x
#             rotate_time = 0.52  # Tiempo en segundos para girar
#             move_time = 0.2  # Tiempo en segundos para desplazarse en x

#             # Variable para alternar entre movimientos de giro y desplazamiento
#             if not hasattr(self, 'last_movement_time'):
#                 self.last_movement_time = rospy.get_time()
#                 self.is_rotating = True  # Iniciar con un movimiento de giro
#                 self.rotate_direction = 1  # 1 para girar en sentido horario, -1 para girar en sentido antihorario

#             # Determinar el tipo de movimiento según el tiempo transcurrido
#             current_time = rospy.get_time()
#             elapsed_time = current_time - self.last_movement_time

#             if self.is_rotating:
#                 # Girar durante rotate_time segundos en la dirección actual
#                 self.twist.linear.x = 0.0
#                 self.twist.angular.z = self.rotate_direction * 0.5  # Velocidad angular para el giro actual
#                 if elapsed_time >= rotate_time:
#                     # Cambiar dirección de giro
#                     self.rotate_direction *= -1
#                     self.last_movement_time = rospy.get_time()
#             else:
#                 # Desplazarse en x durante move_time segundos (x positivo)
#                 self.twist.linear.x = 0.2  # Velocidad lineal para desplazarse en x
#                 self.twist.angular.z = 0.0
#                 if elapsed_time >= move_time:
#                     self.is_rotating = True
#                     self.last_movement_time = rospy.get_time()

#         # Mostrar la imagen procesada
#         cv2.imshow("Lane Detection", cv_image)
#         cv2.waitKey(3)

#         # Publicar el comando de velocidad
#         self.cmd_vel_pub.publish(self.twist)

#     def radar_callback(self, msg):
#         # Procesar los datos del escaneo radar
#         self.radar_ranges = msg.ranges
#         min_distance = min(self.radar_ranges)
#         min_index = self.radar_ranges.index(min_distance)

#         rospy.loginfo("Radar Scan Ranges: {}".format(self.radar_ranges))
#         rospy.loginfo("Minimum Distance: {:.2f} set value to move to the other side {}".format(min_distance, min_index))

#         if min_distance < 0.09:
#             num_ranges = len(self.radar_ranges)
#             mid_index = num_ranges // 2

#             if min_index < mid_index:
#                 # Gira en sentido antihorario si el obstáculo está a la izquierda
#                 self.twist.linear.x = 0.0
#                 self.twist.angular.z = 0.5
#             elif min_index > mid_index:
#                 # Gira en sentido horario si el obstáculo está a la derecha
#                 self.twist.linear.x = 0.0
#                 self.twist.angular.z = -0.5
#             else:
#                 # Si el obstáculo está en el centro, moverse hacia el lado con la distancia más lejana
#                 max_distance = max(self.radar_ranges)
#                 max_index = self.radar_ranges.index(max_distance)

#                 if max_index < mid_index:
#                     self.twist.linear.x = 0.2
#                     self.twist.angular.z = 0.5
#                 else:
#                     self.twist.linear.x = 0.2
#                     self.twist.angular.z = -0.5

#             self.cmd_vel_pub.publish(self.twist)

#     def spin(self):
#         rospy.spin()

# if __name__ == '__main__':
#     rospy.init_node('lane_tracker')
#     lt = LaneTracker()
#     lt.spin()
