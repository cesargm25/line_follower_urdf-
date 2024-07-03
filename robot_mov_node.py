#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import sys
import termios
import tty
import select
from sensor_msgs.msg import LaserScan

class RobotController:
    def __init__(self):
        rospy.init_node('robot_controller', anonymous=True)
        
        # Publisher to the new cmd_vel topic
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        # Subscriber to the odom topic to read 
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rad_sub = rospy.Subscriber('/radar_scan', LaserScan, self.radar_callback)
        
        self.rate = rospy.Rate(10)
        self.odom = None
        self.rad = None

    def odom_callback(self, msg):
        self.odom = msg
    
    def radar_callback(self, msg):
        self.rad = msg


    def move_forward(self, speed):
        vel_msg = Twist()
        vel_msg.linear.x = speed

        # Move forward for a fixed duration
        move_duration = 2  # seconds
        end_time = rospy.Time.now() + rospy.Duration(move_duration)
        while rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(vel_msg)
            self.rate.sleep()

        # Stop the robot
        vel_msg.linear.x = 0
        self.cmd_vel_pub.publish(vel_msg)

    def rotate(self, angular_speed):
        vel_msg = Twist()
        vel_msg.angular.z = angular_speed

        # Rotate for a fixed duration
        rotate_duration = 2.0  # seconds
        end_time = rospy.Time.now() + rospy.Duration(rotate_duration)
        while rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(vel_msg)
            self.rate.sleep()

        # Stop the robot
        vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(vel_msg)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print("Use 'w' to move forward , 's' to move backwards, and 'd' and 'a' to rotate. Press 'q' to quit, press 'c' to stop but not kill it ")
        self.settings = termios.tcgetattr(sys.stdin)

        try:
            while not rospy.is_shutdown():
                key = self.get_key()
                if key == 'w':
                    self.move_forward(0.08)
                elif key == 's':
                    self.move_forward(-0.08)
                elif key == 'd':
                    self.rotate(0.5)
                elif key == 'a':
                    self.rotate(-0.5)
                elif key == 'c':
                    self.rotate(0)
                    self.move_forward(0)
                elif key == 'q':
                    break
                self.rate.sleep()
        except Exception as e:
            print(e)
        finally:
            self.stop()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

    def stop(self):
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)

if __name__ == '__main__':
    try:
        controller = RobotController()
        rospy.sleep(2)  # Give some time to initialize everything
        controller.run()
        node_name ="car_controller"
        rospy.init_node(node_name)
    except rospy.ROSInterruptException:
        pass
