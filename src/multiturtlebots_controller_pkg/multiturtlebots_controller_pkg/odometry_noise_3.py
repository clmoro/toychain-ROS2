# Author: Angelo Moroncelli
# ROS Version: ROS 2 Foxy Fitzroy

import rclpy
from rclpy.node import Node
from math import *
import math
from nav_msgs.msg import Odometry
import tf_transformations
import tf2_ros
import numpy as np
from random import randrange

last_odom = None


#The cumulative noisy poses
x = 0.0
y = 0.0
z = 0.0
c = 0.0
d = 0.0 

a1 = 0.10
a2 = 0.10
a3 = 0.10
a4 = 0.10

#a1 = 0.0
#a2 = 0.0
#a3 = 0.0
#a4 = 0.0
new_odom_frame = ""
odom_frame = ""

class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('Odom_subscriber')
        self.subscription = self.create_subscription(Odometry, '/bot3/odom', self.get_rotation, 10)
          
        self.publisher_ = self.create_publisher(Odometry, '/bot3/noisy_odom', 10)
        self.subscription

    def publish_value(self):
        msg = Odometry()
                
    def get_rotation(self, msg):
            global last_odom
            global new_odom_frame
            global odom_frame
            global x
            global y
            global z
            global c
            global d
            global a1
            global a2
            global a3
            global a4

            q = [msg.pose.pose.orientation.x,
                 msg.pose.pose.orientation.y,
                 msg.pose.pose.orientation.z,
                 msg.pose.pose.orientation.w]
                 
            (r, p, theta2) = tf_transformations.euler_from_quaternion(q)
            
            #At the initial time step the odometry is without noise, since we need a delta ...
            if(last_odom == None):
                last_odom = msg
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                z = theta2

            else:
                dx = msg.pose.pose.position.x - last_odom.pose.pose.position.x
                dy = msg.pose.pose.position.y - last_odom.pose.pose.position.y
                
                q = [last_odom.pose.pose.orientation.x,
                     last_odom.pose.pose.orientation.y,
                     last_odom.pose.pose.orientation.z,
                     last_odom.pose.pose.orientation.w]
                     
                (r,p, theta1) = tf_transformations.euler_from_quaternion(q)
                rot1 = atan2(dy, dx) - theta1
                rot2 = theta2-theta1-rot1
                
                sd_rot1 = a1*abs(rot1) + a2*sqrt(dx*dx + dy*dy)
                sd_rot2 = a1*abs(rot2) + a2*sqrt(dx*dx + dy*dy)
                sd_dx = a3*dx + a4*(abs(rot1) + abs(rot2))
                sd_dy = a3*dy + a4*(abs(rot1) + abs(rot2))
                
                # dx += np.random.normal(0, sd_dx*sd_dx)
                # dy += np.random.normal(0, sd_dy*sd_dy)
                # rot1 += np.random.normal(0, sd_rot1*sd_rot1)
                # rot2 += np.random.normal(0, sd_rot2*sd_rot2)
                
                x += dx
                x += c
                y += dy
                y += d
                z +=  rot1 + rot2
                c += np.random.normal(0.01, sd_dx*sd_dx)*0.1
                d += np.random.normal(0.01, sd_dy*sd_dy)*0.1
                last_odom = msg

            ## adding position,yaw back 
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            quaternion = tf_transformations.quaternion_from_euler(r,p,z)
            msg.pose.pose.orientation.x = quaternion[0]
            msg.pose.pose.orientation.y = quaternion[1]
            msg.pose.pose.orientation.z = quaternion[2]
            msg.pose.pose.orientation.w = quaternion[3]

            self.publisher_.publish(msg)


                             
def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node('noisy_odometry')

    minimal_subscriber = OdomSubscriber()
    
    # alpha 1 is degree/degree
    if node.has_parameter("~alpha1"):
        a1 = rclpy.get_parameter("~alpha1")
    else:
        node.get_logger().warn("alpha1 is set to default")
        #a1 = 0.15
        a1 = 0.15
    
    # alpha 2 is degree/m  
    if node.has_parameter("~alpha2"):
        a2 = rclpy.get_parameter("~alpha2")
    else:
        #a2 = 10.0*pi/180.0
        a2 = 10.0*pi/180.0
        node.get_logger().warn("alpha2 is set to default")
            
    # alpha 3 is m/meter
    if node.has_parameter("~alpha3"):
        a3 = rclpy.get_parameter("~alpha3")
    else:
        #a3 = 0.15
        a3 = 0.5
        node.get_logger().warn("alpha3 is set to default")

    # alpha 4 is m/degree
    if node.has_parameter("~alpha4"):
        a4 = rclpy.get_parameter("~alpha4")
    else:
        #a4 = 0.10
        a4 = 0.03
        node.get_logger().warn("alpha4 is set to default")         

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
