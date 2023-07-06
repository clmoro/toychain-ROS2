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
from std_msgs.msg import String

import logging
import sys

sys.path.append("/home/angelo/ROS2-WORKSPACES/toychain-ROS2/src/blockchain_controller_pkg/")

from toychain.src.Node import Node as BCNode
from toychain.src.consensus.ProofOfAuth import ProofOfAuthority
from toychain.src.Block import Block, State
from toychain.src.Transaction import Transaction
from toychain.src.constants import LOCALHOST
from toychain.src.utils import gen_enode

class BlockchainSubscriber(Node):

    def __init__(self):

        super().__init__('Blockchain_subscriber')
        self.subscription = self.create_subscription(Odometry, '/bot1/odom', self.odom_callback_1, 10)
        
        self.subscription

    def odom_callback_1(self, msg):

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        d = sqrt(pow((x-0),2)+pow((y-0),2))

        if(d < 4):
            self.get_logger().info('Sending transactions on blockchain')
                             
def main(args=None):

    rclpy.init(args=args)
    node = rclpy.create_node('blockchain_node')

    subscriber = BlockchainSubscriber()        

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
