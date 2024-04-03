# Student name: AH

import math
import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseStamped, TransformStamped
from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
import matplotlib.pyplot as plt
import time
from tf2_msgs.msg import TFMessage
from copy import copy
from visualization_msgs.msg import Marker
import matplotlib.pyplot as plt
import math
from sklearn.linear_model import RANSACRegressor
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import PolynomialFeatures
from scipy.optimize import curve_fit


# Further info:
# On markers: http://wiki.ros.org/rviz/DisplayTypes/Marker
# Laser Scan message: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html

def line(x, m, b):
        return m * x + b


class CodingExercise3(Node):
   

    
    def __init__(self):
        super().__init__('CodingExercise3')

        self.ranges = [] # lidar measurements
        
        self.point_list = [] # A list of points to draw lines
        self.line = Marker()
        self.line_marker_init(self.line)


        # Ros subscribers and publishers
        self.subscription_ekf = self.create_subscription(Odometry, 'terrasentia/ekf', self.callback_ekf, 10)
        self.subscription_scan = self.create_subscription(LaserScan, 'terrasentia/scan', self.callback_scan, 10)
        self.pub_lines = self.create_publisher(Marker, 'lines', 10)
        self.timer_draw_line_markers= self.create_timer(0.1, self.draw_line_markers)

    
    def callback_ekf(self, msg):
    # Extract translation and rotation from the Odometry message
        translation = msg.pose.pose.position
        rotation = msg.pose.pose.orientation
        print("Translation: x={}, y={}, z={}".format(translation.x, translation.y, translation.z))
        print("Rotation: x={}, y={}, z={}, w={}".format(rotation.x, rotation.y, rotation.z, rotation.w))

   
    def callback_scan(self, msg):
        self.ranges = list(msg.ranges) # Lidar measurements
        print("some-ranges:", self.ranges[0:5])
        print("Number of ranges:", len(self.ranges))

    rho_test = np.array([[10, 11, 11.7, 13, 14, 15, 16, 17, 17, 17, 16.5, 17, 17, 16, 14.5, 14, 13]]).T
    n = rho_test.shape[0]
    theta_test = (np.pi / 180) * np.linspace(0, 85, n).reshape(-1, 1)

    x_test = rho_test * np.cos(theta_test)
    y_test = rho_test * np.sin(theta_test)


    def fit_line(x, y):
        popt, _ = curve_fit(line, x, y)
        return popt

    parameters = fit_line(x_test.flatten(), y_test.flatten())


    best_fit_line = line(x_test.flatten(), *parameters)

    plt.scatter(x_test, y_test)
    plt.plot(x_test.flatten(), best_fit_line, color='red', label='Best Fit Line')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.show()

    

    def draw_line_markers(self, x, y):
 
        self.point_list = []
        
        for i in range(len(x)):
            p = Point()
            p.x = x[i]
            p.y = y[i]
            p.z = 0.0
            self.point_list.append(copy(p))
            
            additional_points = 5
            x_additional = [x[i] + j * 0.1 for j in range(1, additional_points + 1)]
            y_additional = [y[i] + j * 0.1 for j in range(1, additional_points + 1)]
            
            self.point_list.extend([Point(x=a, y=b, z=0.0) for a, b in zip(x_additional, y_additional)])

        self.line.points = self.point_list
        self.pub_lines.publish(self.line)

    def line_marker_init(self, line):
        line.header.frame_id="/odom"
        line.header.stamp=self.get_clock().now().to_msg()

        line.ns = "markers"
        line.id = 0

        line.type=Marker.LINE_LIST
        line.action = Marker.ADD
        line.pose.orientation.w = 1.0

        line.scale.x = 0.05
        line.scale.y= 0.05
        
        line.color.r = 1.0
        line.color.a = 1.0
        #line.lifetime = 0


def main(args=None):
    rclpy.init(args=args)

    cod3_node = CodingExercise3()
    
    rclpy.spin(cod3_node)

    cod3_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
