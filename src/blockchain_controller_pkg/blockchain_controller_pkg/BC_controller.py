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
from numpy import *
from random import randrange
from std_msgs.msg import String

# Enable use of std_msgs/Int64MultiArray message
from std_msgs.msg import Int64MultiArray 

import logging
import sys
import time

sys.path.append("/home/angelo/ROS2-WORKSPACES/toychain-ROS2/src/blockchain_controller_pkg/")

from toychain.src.Node import Node as BCNode
from toychain.src.consensus.ProofOfAuth import ProofOfAuthority
from toychain.src.Block import Block, State
from toychain.src.Transaction import Transaction
from toychain.src.constants import LOCALHOST
from toychain.src.utils import gen_enode

auth_signers = [gen_enode(i) for i in range(1,9)]
initial_state = State()

GENESIS_BLOCK = Block(0, 0000, [], auth_signers, 0, 0, 0, nonce = 1, state = initial_state)
CONSENSUS = ProofOfAuthority(genesis = GENESIS_BLOCK)

# Create the nodes with (id, host, port, consensus_protocol) - BLOCKCHAIN
node1 = BCNode(1, LOCALHOST, 1234, CONSENSUS)
node2 = BCNode(2, LOCALHOST, 1235, CONSENSUS)
node3 = BCNode(3, LOCALHOST, 1236, CONSENSUS)
node4 = BCNode(4, LOCALHOST, 1237, CONSENSUS)
node5 = BCNode(5, LOCALHOST, 1238, CONSENSUS)
node6 = BCNode(6, LOCALHOST, 1239, CONSENSUS)
node7 = BCNode(7, LOCALHOST, 1240, CONSENSUS)
node8 = BCNode(8, LOCALHOST, 1241, CONSENSUS)

# Setup simulation steps - BLOCKCHAIN
max_steps = 150000
curr_step = 0
step = 1

logging.basicConfig(level=logging.INFO)

# Definition of the SCENE MATRIX
S = array([[12,1],[8,9],[8,-7],
           [4,1],[0,9],[0,-7],
           [-4,1],[-8,9],[-8,-7]])

check = [0,0,0,0,0,0,0,0]
scene = [0,0,0,0,0,0,0,0]

class BlockchainSubscriber(Node):

    def __init__(self):

        super().__init__('Blockchain_subscriber')
        self.subscription_1 = self.create_subscription(Odometry, '/bot1/odom', self.odom_callback_1, 10)
        self.subscription_2 = self.create_subscription(Odometry, '/bot2/odom', self.odom_callback_2, 10)
        self.subscription_3 = self.create_subscription(Odometry, '/bot3/odom', self.odom_callback_3, 10)
        self.subscription_4 = self.create_subscription(Odometry, '/bot4/odom', self.odom_callback_4, 10)
        self.subscription_5 = self.create_subscription(Odometry, '/bot5/odom', self.odom_callback_5, 10)
        self.subscription_6 = self.create_subscription(Odometry, '/bot6/odom', self.odom_callback_6, 10)
        self.subscription_7 = self.create_subscription(Odometry, '/bot7/odom', self.odom_callback_7, 10)
        self.subscription_8 = self.create_subscription(Odometry, '/bot8/odom', self.odom_callback_8, 10)
        
        self.subscription_1
        self.subscription_2
        self.subscription_3
        self.subscription_4
        self.subscription_5
        self.subscription_6
        self.subscription_7
        self.subscription_8

        self.publisher = self.create_publisher(Int64MultiArray, '/candidate_information', 10)

        self.init_network()

    # Functions executed every odometry measure from every robot, they calls dist_scene to send the Transaction.
    # They pass an id to identify the robot, I didn't find a way yet to use a more general function, since I need global variables (ROS2 callbacks override local variables).
    def odom_callback_1(self, msg):

        id = 1
        global curr_step

        if curr_step<max_steps:
            print(curr_step)
            node1.step()
            node2.step()
            node3.step()
            node4.step()
            node5.step()
            node6.step()
            node7.step()
            node8.step()
            curr_step += step

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.dist_scene(x, y, id)

    def odom_callback_2(self, msg):

        id = 2
        global curr_step

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.dist_scene(x, y, id)

    def odom_callback_3(self, msg):

        id = 3
        global curr_step

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.dist_scene(x, y, id)

    def odom_callback_4(self, msg):

        id = 4
        global curr_step

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.dist_scene(x, y, id)

    def odom_callback_5(self, msg):

        id = 5
        global curr_step

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.dist_scene(x, y, id)

    def odom_callback_6(self, msg):

        id = 6
        global curr_step

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.dist_scene(x, y, id)

    def odom_callback_7(self, msg):

        id = 7
        global curr_step

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.dist_scene(x, y, id)

    def odom_callback_8(self, msg):

        id = 8
        global curr_step

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.dist_scene(x, y, id)

    # Function that send a Transaction if the robot is near a scene, one time only
    def dist_scene(self, x, y, id):

        global check
        global scene

        if (check[id-1] == 0):
            for i in range(9):
                d = sqrt(pow((x-S[i][0]),2)+pow((y-S[i][1]),2))
                if (d < 2 and check[id-1] == 0):
                    scene[id-1] = i
                    
                    # Scene recognition with real position in scene i
                    print('Scene recognition!')
                    print('Robot ' + str(id))
                    print('In scene ' + str(scene[id-1]))
                    print('At distance ' + str(sqrt(pow((x-S[scene[id-1]][0]),2)+pow((y-S[scene[id-1]][1]),2))))
                    
                    # Create the candidate (1st instance on that scene) or the loop closure 
                    candidate_vector = [id, scene[id-1]]
                    msg = Int64MultiArray()
                    msg.data = candidate_vector
                    self.publisher.publish(msg)

                    check[id-1] = 1

                    # Display the blockchains when something is added
                    print('Node 1')
                    print(node1.display_chain())
                    print('Node 2')
                    print(node2.display_chain())
                    print('Node 3')
                    print(node3.display_chain())
                    print('Node 4')
                    print(node4.display_chain())
                    print('Node 5')
                    print(node5.display_chain())
                    print('Node 6')
                    print(node6.display_chain())
                    print('Node 7')
                    print(node7.display_chain())
                    print('Node 8')
                    print(node8.display_chain())

        if (check[id-1] == 1):
            d_actual = sqrt(pow((x-S[scene[id-1]][0]),2)+pow((y-S[scene[id-1]][1]),2))
            if (d_actual >= 2):
                check[id-1] = 0

    def init_network(self):
    
        # Start the TCP for syncing mempool and blockchain
        node1.start_tcp()
        node2.start_tcp()
        node3.start_tcp()
        node4.start_tcp()
        node5.start_tcp()
        node6.start_tcp()
        node7.start_tcp()
        node8.start_tcp()
    
        # Start the mining process for each node
        node1.start_mining()
        node2.start_mining()
        node3.start_mining()
        node4.start_mining()
        node5.start_mining()
        node6.start_mining()
        node7.start_mining()
        node8.start_mining()

        # Add the peers of each node (now it's GLOBAL, do it based on distances if you want LOCALITY)
        node1.add_peer(node2.enode)
        node1.add_peer(node3.enode)
        node1.add_peer(node4.enode)
        node1.add_peer(node5.enode)
        node1.add_peer(node6.enode)
        node1.add_peer(node7.enode)
        node1.add_peer(node8.enode)
        node2.add_peer(node1.enode)
        node2.add_peer(node3.enode)
        node2.add_peer(node4.enode)
        node2.add_peer(node5.enode)
        node2.add_peer(node6.enode)
        node2.add_peer(node7.enode)
        node2.add_peer(node8.enode)
        node3.add_peer(node1.enode)
        node3.add_peer(node2.enode)
        node3.add_peer(node4.enode)
        node3.add_peer(node5.enode)
        node3.add_peer(node6.enode)
        node3.add_peer(node7.enode)
        node3.add_peer(node8.enode)
        node4.add_peer(node1.enode)
        node4.add_peer(node2.enode)
        node4.add_peer(node3.enode)
        node4.add_peer(node5.enode)
        node4.add_peer(node6.enode)
        node4.add_peer(node7.enode)
        node4.add_peer(node8.enode)
        node5.add_peer(node1.enode)
        node5.add_peer(node2.enode)
        node5.add_peer(node3.enode)
        node5.add_peer(node4.enode)
        node5.add_peer(node6.enode)
        node5.add_peer(node7.enode)
        node5.add_peer(node8.enode)
        node6.add_peer(node1.enode)
        node6.add_peer(node2.enode)
        node6.add_peer(node3.enode)
        node6.add_peer(node4.enode)
        node6.add_peer(node5.enode)
        node6.add_peer(node7.enode)
        node6.add_peer(node8.enode)
        node7.add_peer(node1.enode)
        node7.add_peer(node2.enode)
        node7.add_peer(node3.enode)
        node7.add_peer(node4.enode)
        node7.add_peer(node5.enode)
        node7.add_peer(node6.enode)
        node7.add_peer(node8.enode)
        node8.add_peer(node1.enode)
        node8.add_peer(node2.enode)
        node8.add_peer(node3.enode)
        node8.add_peer(node4.enode)
        node8.add_peer(node5.enode)
        node8.add_peer(node6.enode)
        node8.add_peer(node7.enode)
                             
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







""" BASIC EXAMPLE - BLOCKCHAIN PART
            
            self.init_network()

            # Setup simulation steps
            max_steps = 15000
            curr_step = 0
            step = 1

            while True:
                print(curr_step)
                node1.step()
                node2.step()
                node3.step()
                # time.sleep(0.05), (lower is faster, 0.0 is the fastest possible)
                curr_step += step
                if curr_step>max_steps:
                    break
    
            # Display the final blockchains at the end of the simulation
            print('Node 1')
            print(node1.display_chain())
            print('Node 2')
            print(node2.display_chain())
            print('Node 3')
            print(node3.display_chain())

            self.get_logger().info('Starting the blockchain') 
            
        END BLOCKCHAIN PART """