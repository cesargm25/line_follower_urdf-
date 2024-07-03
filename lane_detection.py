#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class LaneTracker:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.current_speed = 0.0
        self.max_speed = 0.15
        self.speed_increment = 0.001

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
            # Si no se detectan líneas, detén el robot
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.current_speed = 0.0

        # Mostrar la imagen procesada
        cv2.imshow("Lane Detection", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('lane_tracker')
    lt = LaneTracker()
    rospy.spin()




#__________________No funciona bien ______________
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
#         self.max_speed = 0.1  # Incrementamos la velocidad máxima para que el robot se mueva más notablemente
#         self.last_error = 0.0
#         self.Kp = 0.0025
#         self.Kd = 0.007

#     def image_callback(self, data):
#         try:
#             cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr(e)
#             return

#         # Detección de líneas blancas con un rango más amplio
#         white_lower = np.array([0, 0, 200])
#         white_upper = np.array([180, 80, 255])
#         white_mask = cv2.inRange(cv_image, white_lower, white_upper)
#         white_output = cv2.bitwise_and(cv_image, cv_image, mask=white_mask)

#         # Detección de líneas amarillas
#         hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#         yellow_lower = np.array([20, 100, 100])
#         yellow_upper = np.array([30, 255, 255])
#         yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)
#         yellow_output = cv2.bitwise_and(cv_image, cv_image, mask=yellow_mask)

#         # Combinación de máscaras
#         combined_mask = cv2.bitwise_or(white_mask, yellow_mask)
#         combined_output = cv_image.copy()
#         combined_output[combined_mask != 0] = [0, 255, 0]

#         # Procesamiento de imágenes para encontrar contornos
#         gray = cv2.cvtColor(combined_output, cv2.COLOR_BGR2GRAY)
#         blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#         edges = cv2.Canny(blurred, 50, 150)

#         height, width = edges.shape
#         search_top = 3 * height // 4
#         search_bot = search_top + 20
#         edges[0:search_top, 0:width] = 0
#         edges[search_bot:height, 0:width] = 0

#         # Encontrar contornos
#         contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#         if len(contours) > 0:
#             c = max(contours, key=cv2.contourArea)
#             M = cv2.moments(c)
#             if M["m00"] != 0:
#                 cx = int(M["m10"] / M["m00"])
#                 cy = int(M["m01"] / M["m00"])

#                 error = cx - width // 2
#                 self.twist.linear.x = self.max_speed
#                 self.twist.angular.z = -float(error) * self.Kp - (error - self.last_error) * self.Kd
#                 self.last_error = error

#                 self.cmd_vel_pub.publish(self.twist)
#             else:
#                 self.twist.linear.x = 0
#                 self.twist.angular.z = 0
#                 self.cmd_vel_pub.publish(self.twist)

#         cv2.imshow("Lane Detection", combined_output)
#         cv2.waitKey(3)

# if __name__ == '__main__':
#     rospy.init_node('lane_tracker', anonymous=True)
#     lt = LaneTracker()
#     try:
#         rospy.spin()
#     except KeyboardInterrupt:
#         print("Shutting down")
#     cv2.destroyAllWindows()




#-------------------------------------------first code----------------------------------------------
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
        self.max_speed = 0.15
        self.speed_increment = 0.001

    def image_callback(self, msg):
        try:
            # Convertir el mensaje de ROS Image a una imagen de OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return
        
        # Procesar la imagen para detectar líneas
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([255, 50, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white) # and the yellow ? 
        
        blur = cv2.GaussianBlur(mask, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        # Hough Transform para detectar líneas
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
            # Si no se detectan líneas, detén el robot
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.cmd_vel_pub.publish(self.twist)
            self.current_speed = 0.0

        # Mostrar la imagen procesada
        cv2.imshow("Lane Detection", cv_image)
        cv2.waitKey(3)

if __name__ == '__main__':
    rospy.init_node('lane_tracker')
    lt = LaneTracker()
    rospy.spin()




#__________________________________________________No ha corrido____________________________
# import rospy
# import cv2
# import numpy as np
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge, CvBridgeError

# class LaneTracker:
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
#         self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
#         self.twist = Twist()
#         self.current_speed = 0.0
#         self.max_speed = 0.015
#         self.last_error = 0.0
#         self.Kp = 0.0025
#         self.Kd = 0.007

#         rospy.on_shutdown(self.shutdown)

#     def image_callback(self, msg):
#         try:
#             # Convertir el mensaje de ROS Image a una imagen de OpenCV
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))
#             return
        
#         # Convertir la imagen de BGR a HSV
#         hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
#         # Definir los rangos de color para el amarillo y el blanco en HSV
#         lower_yellow = np.array([20, 100, 100])
#         upper_yellow = np.array([30, 255, 255])
#         lower_white = np.array([0, 0, 200])
#         upper_white = np.array([255, 50, 255])
        
#         # Crear máscaras para segmentar las partes amarillas y blancas de la imagen
#         mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
#         mask_white = cv2.inRange(hsv, lower_white, upper_white)
        
#         # Combinar ambas máscaras para obtener una máscara final que detecte amarillo y blanco
#         mask = cv2.bitwise_or(mask_yellow, mask_white)
        
#         # Aplicar desenfoque y detección de bordes
#         blur = cv2.GaussianBlur(mask, (5, 5), 0)
#         edges = cv2.Canny(blur, 50, 150)
        
#         # Hough Transform para detectar líneas
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

#             # Calcula el error entre el centro del carril y el centro de la imagen
#             error = lane_center - cv_image.shape[1] / 2

#             # Control PID para ajustar la velocidad angular del robot
#             angular_z = self.Kp * error + self.Kd * (error - self.last_error)
#             self.last_error = error

#             # Ajuste de la velocidad lineal basado en el error
#             self.twist.linear.x = min(self.max_speed * ((1 - abs(error) / (cv_image.shape[1] / 2)) ** 2.2), self.max_speed)
#             self.twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
#             self.cmd_vel_pub.publish(self.twist)
#         else:
#             # Si no se detectan líneas, detener el robot
#             self.stop_robot()

#         # Mostrar la imagen procesada
#         cv2.imshow("Lane Detection", cv_image)
#         cv2.waitKey(3)

#     def stop_robot(self):
#         self.twist.linear.x = 0.0
#         self.twist.angular.z = 0.0
#         self.cmd_vel_pub.publish(self.twist)

#     def shutdown(self):
#         rospy.loginfo("Shutting down. cmd_vel will be 0")
#         self.stop_robot()

# if __name__ == '__main__':
#     rospy.init_node('lane_tracker')
#     lt = LaneTracker()
#     rospy.spin()




#_______________________LINE FOLLOWING____________________________
# import rospy
# import cv2
# import numpy as np
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge, CvBridgeError

# class LineFollower:
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
#         self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#         self.twist = Twist()
#         self.current_speed = 0.0
#         self.max_speed = 0.1
#         self.speed_increment = 0.011

#     def image_callback(self, msg):
#         try:
#             # Convertir el mensaje de ROS Image a una imagen de OpenCV
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))
#             return
        
#         # Procesar la imagen para detectar líneas
#         hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#         lower_white = np.array([0, 0, 200])
#         upper_white = np.array([255, 50, 255])
#         mask = cv2.inRange(hsv, lower_white, upper_white)
        
#         blur = cv2.GaussianBlur(mask, (5, 5), 0)
#         edges = cv2.Canny(blur, 50, 150)

#         # Hough Transform para detectar líneas
#         lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)

#         if lines is not None:
#             # Calcula el centro de la línea detectada
#             mid_x_total = 0
#             line_count = 0

#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                
#                 # Calcula el punto medio de la línea
#                 mid_x = (x1 + x2) / 2
#                 mid_x_total += mid_x
#                 line_count += 1

#             # Calcula el promedio del punto medio de todas las líneas detectadas
#             mid_x_avg = mid_x_total / line_count if line_count > 0 else cv_image.shape[1] / 2

#             # Usa la posición del punto medio para ajustar la velocidad del robot
#             err = mid_x_avg - cv_image.shape[1] / 2
#             self.current_speed = min(self.current_speed + self.speed_increment, self.max_speed)
#             self.twist.linear.x = self.current_speed
#             self.twist.angular.z = -float(err) / 100
#             self.cmd_vel_pub.publish(self.twist)
#         else:
#             # Si no se detectan líneas, detén el robot
#             self.twist.linear.x = 0.0
#             self.twist.angular.z = 0.0
#             self.cmd_vel_pub.publish(self.twist)
#             self.current_speed = 0.0

#         # Mostrar la imagen procesada
#         cv2.imshow("Line Detection", cv_image)
#         cv2.waitKey(3)

# if __name__ == '__main__':
#     rospy.init_node('line_follower')
#     lf = LineFollower()
#     rospy.spin()

#_________________________________________sin mask____________________________
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
#         self.max_speed = 0.015
#         self.last_error = 0.0
#         self.Kp = 0.0025
#         self.Kd = 0.007

#         rospy.on_shutdown(self.shutdown)

#     def image_callback(self, msg):
#         try:
#             # Convertir el mensaje de ROS Image a una imagen de OpenCV
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))
#             return
        
#         # Procesar la imagen para detectar líneas
#         hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#         lower_white = np.array([0, 0, 200])
#         upper_white = np.array([255, 50, 255])
#         mask = cv2.inRange(hsv, lower_white, upper_white)
        
#         blur = cv2.GaussianBlur(mask, (5, 5), 0)
#         edges = cv2.Canny(blur, 50, 150)

#         # Hough Transform para detectar líneas
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
#             error = lane_center - cv_image.shape[1] / 2

#             # Control PID
#             angular_z = self.Kp * error + self.Kd * (error - self.last_error)
#             self.last_error = error

#             # Ajuste de la velocidad lineal basado en el error
#             self.twist.linear.x = min(self.max_speed * ((1 - abs(error) / (cv_image.shape[1] / 2)) ** 2.2), self.max_speed)
#             self.twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
#             self.cmd_vel_pub.publish(self.twist)
#         else:
#             # Si no se detectan líneas, detén el robot
#             self.stop_robot()

#         # Mostrar la imagen procesada
#         cv2.imshow("Lane Detection", cv_image)
#         cv2.waitKey(3)

#     def stop_robot(self):
#         self.twist.linear.x = 0.0
#         self.twist.angular.z = 0.0
#         self.cmd_vel_pub.publish(self.twist)

#     def shutdown(self):
#         rospy.loginfo("Shutting down. cmd_vel will be 0")
#         self.stop_robot()

# if __name__ == '__main__':
#     rospy.init_node('lane_tracker')
#     lt = LaneTracker()
#     rospy.spin()



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
#         self.max_speed = 0.015
#         self.speed_increment = 0.001

#     def image_callback(self, msg):
#         try:
#             # Convertir el mensaje de ROS Image a una imagen de OpenCV
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))
#             return
        
#         # Procesar la imagen para detectar líneas
#         hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#         lower_white = np.array([0, 0, 200])
#         upper_white = np.array([255, 50, 255])
#         mask = cv2.inRange(hsv, lower_white, upper_white)
        
#         blur = cv2.GaussianBlur(mask, (5, 5), 0)
#         edges = cv2.Canny(blur, 50, 150)

#         # Hough Transform para detectar líneas
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
#             self.cmd_vel_pub.publish(self.twist)
#         else:
#             # Si no se detectan líneas, detén el robot
#             self.twist.linear.x = 0.0
#             self.twist.angular.z = 0.0
#             self.cmd_vel_pub.publish(self.twist)
#             self.current_speed = 0.0

#         # Mostrar la imagen procesada
#         cv2.imshow("Lane Detection", cv_image)
#         cv2.waitKey(3)

# if __name__ == '__main__':
#     rospy.init_node('lane_tracker')
#     lt = LaneTracker()
#     rospy.spin()




# import rospy
# import cv2
# import numpy as np
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge, CvBridgeError

# class LineFollower:
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.image_sub = rospy.Subscriber('/image_raw', Image, self.image_callback)
#         self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
#         self.twist = Twist()
#         self.current_speed = 0.0
#         self.max_speed = 0.1
#         self.speed_increment = 0.011

#     def image_callback(self, msg):
#         try:
#             # Convertir el mensaje de ROS Image a una imagen de OpenCV
#             cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
#         except CvBridgeError as e:
#             rospy.logerr("CvBridge Error: {0}".format(e))
#             return
        
#         # Procesar la imagen para detectar líneas
#         hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
#         lower_white = np.array([0, 0, 200])
#         upper_white = np.array([255, 50, 255])
#         mask = cv2.inRange(hsv, lower_white, upper_white)
        
#         blur = cv2.GaussianBlur(mask, (5, 5), 0)
#         edges = cv2.Canny(blur, 50, 150)

#         # Hough Transform para detectar líneas
#         lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, maxLineGap=50)

#         if lines is not None:
#             # Calcula el centro de la línea detectada
#             mid_x_total = 0
#             line_count = 0

#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 cv2.line(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 3)
                
#                 # Calcula el punto medio de la línea
#                 mid_x = (x1 + x2) / 2
#                 mid_x_total += mid_x
#                 line_count += 1

#             # Calcula el promedio del punto medio de todas las líneas detectadas
#             mid_x_avg = mid_x_total / line_count if line_count > 0 else cv_image.shape[1] / 2

#             # Usa la posición del punto medio para ajustar la velocidad del robot
#             err = mid_x_avg - cv_image.shape[1] / 2
#             self.current_speed = min(self.current_speed + self.speed_increment, self.max_speed)
#             self.twist.linear.x = self.current_speed
#             self.twist.angular.z = -float(err) / 100
#             self.cmd_vel_pub.publish(self.twist)
#         else:
#             # Si no se detectan líneas, detén el robot
#             self.twist.linear.x = 0.0
#             self.twist.angular.z = 0.0
#             self.cmd_vel_pub.publish(self.twist)
#             self.current_speed = 0.0

#         # Mostrar la imagen procesada
#         cv2.imshow("Line Detection", cv_image)
#         cv2.waitKey(3)

# if __name__ == '__main__':
#     rospy.init_node('line_follower')
#     lf = LineFollower()
#     rospy.spin()




