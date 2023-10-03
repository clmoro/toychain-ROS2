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
# Enable use of std_msgs/Float64MultiArray message
from std_msgs.msg import Float64MultiArray 

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
node1 = BCNode(1, LOCALHOST, 1231, CONSENSUS)
node2 = BCNode(2, LOCALHOST, 1232, CONSENSUS)
node3 = BCNode(3, LOCALHOST, 1233, CONSENSUS)
node4 = BCNode(4, LOCALHOST, 1234, CONSENSUS)
node5 = BCNode(5, LOCALHOST, 1235, CONSENSUS)
node6 = BCNode(6, LOCALHOST, 1236, CONSENSUS)
node7 = BCNode(7, LOCALHOST, 1237, CONSENSUS)
node8 = BCNode(8, LOCALHOST, 1238, CONSENSUS)

# Setup simulation steps - BLOCKCHAIN
max_steps = 150000
curr_step = 0
step = 1

logging.basicConfig(level=logging.DEBUG)

# Definition of the SCENE MATRIX
S = array([[12,1],[8,9],[8,-7],
           [4,1],[0,9],[0,-7],
           [-4,1],[-8,9],[-8,-7]])

adjacency_matrix = np.zeros((8,8), dtype=int)
check = [0,0,0,0,0,0,0,0]
scene = [0,0,0,0,0,0,0,0]
x1 = 0.0
y1 = 0.0
x2 = 0.0
y2 = 0.0
x3 = 0.0
y3 = 0.0
x4 = 0.0
y4 = 0.0
x5 = 0.0
y5 = 0.0
x6 = 0.0
y6 = 0.0
x7 = 0.0
y7 = 0.0
x8 = 0.0
y8 = 0.0

class BlockchainSubscriber(Node):

    def __init__(self):

        self.last_adjacency_matrix_1 = np.zeros((1,8), dtype=int)
        self.last_adjacency_matrix_2 = np.zeros((1,8), dtype=int)
        self.last_adjacency_matrix_3 = np.zeros((1,8), dtype=int)
        self.last_adjacency_matrix_4 = np.zeros((1,8), dtype=int)
        self.last_adjacency_matrix_5 = np.zeros((1,8), dtype=int)
        self.last_adjacency_matrix_6 = np.zeros((1,8), dtype=int)
        self.last_adjacency_matrix_7 = np.zeros((1,8), dtype=int)
        self.last_adjacency_matrix_8 = np.zeros((1,8), dtype=int)

        super().__init__('Blockchain_subscriber')

        # Subscriptions to odometries
        self.subscription_1 = self.create_subscription(Odometry, '/bot1/odom', self.odom_callback_1, 10)
        self.subscription_2 = self.create_subscription(Odometry, '/bot2/odom', self.odom_callback_2, 10)
        self.subscription_3 = self.create_subscription(Odometry, '/bot3/odom', self.odom_callback_3, 10)
        self.subscription_4 = self.create_subscription(Odometry, '/bot4/odom', self.odom_callback_4, 10)
        self.subscription_5 = self.create_subscription(Odometry, '/bot5/odom', self.odom_callback_5, 10)
        self.subscription_6 = self.create_subscription(Odometry, '/bot6/odom', self.odom_callback_6, 10)
        self.subscription_7 = self.create_subscription(Odometry, '/bot7/odom', self.odom_callback_7, 10)
        self.subscription_8 = self.create_subscription(Odometry, '/bot8/odom', self.odom_callback_8, 10)

        # Subscriptions to the "local" database of each robot
        self.subscription_transformation = self.create_subscription(Float64MultiArray, '/blockchain_transformation', self.transformation_callback, 100)

        self.subscription_1
        self.subscription_2
        self.subscription_3
        self.subscription_4
        self.subscription_5
        self.subscription_6
        self.subscription_7
        self.subscription_8
        self.subscription_transformation

        # Publishers
        self.publisher = self.create_publisher(Int64MultiArray, '/candidate_information', 100)
        self.publisher_peers_1 = self.create_publisher(Int64MultiArray, '/peering_1', 10)
        self.publisher_peers_2 = self.create_publisher(Int64MultiArray, '/peering_2', 10)
        self.publisher_peers_3 = self.create_publisher(Int64MultiArray, '/peering_3', 10)
        self.publisher_peers_4 = self.create_publisher(Int64MultiArray, '/peering_4', 10)
        self.publisher_peers_5 = self.create_publisher(Int64MultiArray, '/peering_5', 10)
        self.publisher_peers_6 = self.create_publisher(Int64MultiArray, '/peering_6', 10)
        self.publisher_peers_7 = self.create_publisher(Int64MultiArray, '/peering_7', 10)
        self.publisher_peers_8 = self.create_publisher(Int64MultiArray, '/peering_8', 10)
        self.publisher_approved_transformation = self.create_publisher(Int64MultiArray, '/blockchain_approved_transformation', 100)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.init_network()

    #Callback functions
    # Functions executed every odometry measure from every robot, they calls dist_scene to send the Transaction.
    # They pass an id to identify the robot, I didn't find a way yet to use a more general function, since I need global variables (ROS2 callbacks override local variables).
    def odom_callback_1(self, msg):

        id = 1
        global x1, y1
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

        x1 = msg.pose.pose.position.x
        y1 = msg.pose.pose.position.y
        
        self.dist_scene(x1, y1, id)
        self.check_meeting(x1, y1, id)

    def odom_callback_2(self, msg):

        id = 2
        global x2, y2

        x2 = msg.pose.pose.position.x
        y2 = msg.pose.pose.position.y
        
        self.dist_scene(x2, y2, id)
        self.check_meeting(x2, y2, id)

    def odom_callback_3(self, msg):

        id = 3
        global x3, y3

        x3 = msg.pose.pose.position.x
        y3 = msg.pose.pose.position.y
        
        self.dist_scene(x3, y3, id)
        self.check_meeting(x3, y3, id)

    def odom_callback_4(self, msg):

        id = 4
        global x4, y4

        x4 = msg.pose.pose.position.x
        y4 = msg.pose.pose.position.y
        
        self.dist_scene(x4, y4, id)
        self.check_meeting(x4, y4, id)

    def odom_callback_5(self, msg):

        id = 5
        global x5, y5

        x5 = msg.pose.pose.position.x
        y5 = msg.pose.pose.position.y
        
        self.dist_scene(x5, y5, id)
        self.check_meeting(x5, y5, id)

    def odom_callback_6(self, msg):

        id = 6
        global x6, y6

        x6 = msg.pose.pose.position.x
        y6 = msg.pose.pose.position.y
        
        self.dist_scene(x6, y6, id)
        self.check_meeting(x6, y6, id)

    def odom_callback_7(self, msg):

        id = 7
        global x7, y7

        x7 = msg.pose.pose.position.x
        y7 = msg.pose.pose.position.y
        
        self.dist_scene(x7, y7, id)
        self.check_meeting(x7, y7, id)

    def odom_callback_8(self, msg):

        id = 8
        global x8, y8

        x8 = msg.pose.pose.position.x
        y8 = msg.pose.pose.position.y
        
        self.dist_scene(x8, y8, id)
        self.check_meeting(x8, y8, id)

    # In msg there's a new proposed loop closure to put on the blockchain through a transaction
    def transformation_callback(self, msg):

        curr_transformation = msg.data
        LC_Descriptor_R = int(curr_transformation[0])
        LC_ID_R = int(curr_transformation[1])
        LC_Odomx_R = curr_transformation[2]
        LC_Odomy_R = curr_transformation[3]
        LC_Keyframe_R = int(curr_transformation[4])
        LC_Descriptor_S = int(curr_transformation[5])
        LC_ID_S = int(curr_transformation[6])
        LC_Odomx_S = curr_transformation[7]
        LC_Odomy_S = curr_transformation[8]
        LC_Keyframe_S = int(curr_transformation[9])
        LC_dx = curr_transformation[10]
        LC_dy = curr_transformation[11]
        LC_SCENE = curr_transformation[12]
        
        txdata = {'function': 'apply_validation', 'inputs': [LC_Descriptor_R, LC_ID_R, LC_Odomx_R, LC_Odomy_R, LC_Keyframe_R, LC_Descriptor_S, LC_ID_S, LC_Odomx_S, LC_Odomy_S, LC_Keyframe_S, LC_dx, LC_dy, LC_SCENE]}
        
        # Here put the logic to interact with the blockchain: send transaction, execute smart contract, read an outcome, publish the approved LCs
        ## TO DO: pass the ID, dx, dy as 'inputs', save them in a matrix state variable and get the ID of every line of this matrix
        if(curr_transformation[6] == 1):
            tx = Transaction(sender = 1, receiver = curr_transformation[1], value = 0, data = txdata)
            node1.send_transaction(tx)
            print('tx 1')
        if(curr_transformation[6] == 2):
            tx = Transaction(sender = 2, receiver = curr_transformation[1], value = 0, data = txdata)
            node2.send_transaction(tx)
            print('tx 2')
        if(curr_transformation[6] == 3):
            tx = Transaction(sender = 3, receiver = curr_transformation[1], value = 0, data = txdata)
            node3.send_transaction(tx)
            print('tx 3')
        if(curr_transformation[6] == 4):
            tx = Transaction(sender = 4, receiver = curr_transformation[1], value = 0, data = txdata)
            node4.send_transaction(tx)
            print('tx 4')
        if(curr_transformation[6] == 5):
            tx = Transaction(sender = 5, receiver = curr_transformation[1], value = 0, data = txdata)
            node5.send_transaction(tx)
            print('tx 5')
        if(curr_transformation[6] == 6):
            tx = Transaction(sender = 6, receiver = curr_transformation[1], value = 0, data = txdata)
            node6.send_transaction(tx)
            print('tx 6')
        if(curr_transformation[6] == 7):
            tx = Transaction(sender = 7, receiver = curr_transformation[1], value = 0, data = txdata)
            node7.send_transaction(tx)
            print('tx 7')
        if(curr_transformation[6] == 8):
            tx = Transaction(sender = 8, receiver = curr_transformation[1], value = 0, data = txdata)
            node8.send_transaction(tx)
            print('tx 8')

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

        if (check[id-1] == 1):
            d_actual = sqrt(pow((x-S[scene[id-1]][0]),2)+pow((y-S[scene[id-1]][1]),2))
            if (d_actual >= 2):
                check[id-1] = 0

    def check_meeting(self, x, y, id):

        global adjacency_matrix

        if((sqrt(pow((x-x1),2)+pow((y-y1),2)) < 2) and (id != 1)):
            adjacency_matrix[id-1][0] = 1
        else:
            adjacency_matrix[id-1][0] = 0
        if((sqrt(pow((x-x2),2)+pow((y-y2),2)) < 2) and (id != 2)):
            adjacency_matrix[id-1][1] = 1
        else:
            adjacency_matrix[id-1][1] = 0
        if((sqrt(pow((x-x3),2)+pow((y-y3),2)) < 2) and (id != 3)):
            adjacency_matrix[id-1][2] = 1
        else:
            adjacency_matrix[id-1][2] = 0
        if((sqrt(pow((x-x4),2)+pow((y-y4),2)) < 2) and (id != 4)):
            adjacency_matrix[id-1][3] = 1
        else:
            adjacency_matrix[id-1][3] = 0
        if((sqrt(pow((x-x5),2)+pow((y-y5),2)) < 2) and (id != 5)):
            adjacency_matrix[id-1][4] = 1
        else:
            adjacency_matrix[id-1][4] = 0
        if((sqrt(pow((x-x6),2)+pow((y-y6),2)) < 2) and (id != 6)):
            adjacency_matrix[id-1][5] = 1
        else:
            adjacency_matrix[id-1][5] = 0
        if((sqrt(pow((x-x7),2)+pow((y-y7),2)) < 2) and (id != 7)):
            adjacency_matrix[id-1][6] = 1
        else:
            adjacency_matrix[id-1][6] = 0
        if((sqrt(pow((x-x8),2)+pow((y-y8),2)) < 2) and (id != 8)):
            adjacency_matrix[id-1][7] = 1
        else:
            adjacency_matrix[id-1][7] = 0

    # Function to initialize the network
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

    # Function that publish only the approved loop closures, one at a time in a vector [ID, boolean value]
    def publish_approved_LC(self, Receiver_Descriptor, Sender_Descriptor):
        
        msg = Int64MultiArray()
        msg.data = [Receiver_Descriptor, Sender_Descriptor]
        if (msg.data[0] != 0):
            self.publisher_approved_transformation.publish(msg)

    def timer_callback(self):

        global adjacency_matrix

        # Publish the outcome of the smart contract: the ID of the loop closures that passed the verification through the smart contract
        appr1 = node1.sc.getApprovedLC()
        for i in range(len(appr1['ID_Sender'])):
            if(appr1['ID_Sender'][i] == 1):
                self.publish_approved_LC(appr1['Descriptor_R'][i], appr1['Descriptor_S'][i])

        appr2 = node2.sc.getApprovedLC()
        for i in range(len(appr2['ID_Sender'])):
            if(appr2['ID_Sender'][i] == 2):
                self.publish_approved_LC(appr2['Descriptor_R'][i], appr2['Descriptor_S'][i])

        appr3 = node3.sc.getApprovedLC()
        for i in range(len(appr3['ID_Sender'])):
            if(appr3['ID_Sender'][i] == 3):
                self.publish_approved_LC(appr3['Descriptor_R'][i], appr3['Descriptor_S'][i])

        appr4 = node4.sc.getApprovedLC()
        for i in range(len(appr4['ID_Sender'])):
            if(appr4['ID_Sender'][i] == 4):
                self.publish_approved_LC(appr4['Descriptor_R'][i], appr4['Descriptor_S'][i])

        appr5 = node5.sc.getApprovedLC()
        for i in range(len(appr5['ID_Sender'])):
            if(appr5['ID_Sender'][i] == 5):
                self.publish_approved_LC(appr5['Descriptor_R'][i], appr5['Descriptor_S'][i])

        appr6 = node6.sc.getApprovedLC()
        for i in range(len(appr6['ID_Sender'])):
            if(appr6['ID_Sender'][i] == 6):
                self.publish_approved_LC(appr6['Descriptor_R'][i], appr6['Descriptor_S'][i])

        appr7 = node7.sc.getApprovedLC()
        for i in range(len(appr7['ID_Sender'])):
            if(appr7['ID_Sender'][i] == 7):
                self.publish_approved_LC(appr7['Descriptor_R'][i], appr7['Descriptor_S'][i])

        appr8 = node8.sc.getApprovedLC()
        for i in range(len(appr8['ID_Sender'])):
            if(appr8['ID_Sender'][i] == 8):
                self.publish_approved_LC(appr8['Descriptor_R'][i], appr8['Descriptor_S'][i])

        # Use the adjacency matrix to publish the vector of the new peers of robot msg.data[8], every time a new meeting happens

        # print(self.last_adjacency_matrix_1)
        # print(self.last_adjacency_matrix_2)
        # print(self.last_adjacency_matrix_3)
        # print(self.last_adjacency_matrix_4)
        # print(self.last_adjacency_matrix_5)
        # print(self.last_adjacency_matrix_6)
        # print(self.last_adjacency_matrix_7)
        # print(self.last_adjacency_matrix_8)
        # print('&')
        # print(adjacency_matrix)

        msg = Int64MultiArray()
        if(not(np.allclose(adjacency_matrix[0], self.last_adjacency_matrix_1)) and np.any(adjacency_matrix[0]) and (np.count_nonzero(adjacency_matrix[0] == 1) > np.count_nonzero(self.last_adjacency_matrix_1 == 1))):
            #msg.data = [int(adjacency_matrix[0,0]), int(adjacency_matrix[0,1]), int(adjacency_matrix[0,2]), int(adjacency_matrix[0,3]), int(adjacency_matrix[0,4]), int(adjacency_matrix[0,5]), int(adjacency_matrix[0,6]), int(adjacency_matrix[0,7])]
            comparison_vector = np.equal(adjacency_matrix[0], self.last_adjacency_matrix_1)
            who = np.where(comparison_vector == False)[0]
            msg.data = [-1, -1, -1, -1, -1, -1, -1, -1, 1]
            for i in range(len(who)):
                if (adjacency_matrix[0, who[i]] != 0):
                    msg.data[i] = int(who[i]) + 1
                    print('1 meet someone!')
            self.publisher_peers_1.publish(msg)
        if(not(np.allclose(adjacency_matrix[1], self.last_adjacency_matrix_2)) and np.any(adjacency_matrix[1]) and (np.count_nonzero(adjacency_matrix[1] == 1) > np.count_nonzero(self.last_adjacency_matrix_2 == 1))):
            #msg.data = [int(adjacency_matrix[1,0]), int(adjacency_matrix[1,1]), int(adjacency_matrix[1,2]), int(adjacency_matrix[1,3]), int(adjacency_matrix[1,4]), int(adjacency_matrix[1,5]), int(adjacency_matrix[1,6]), int(adjacency_matrix[1,7])]
            comparison_vector = np.equal(adjacency_matrix[1], self.last_adjacency_matrix_2)
            who = np.where(comparison_vector == False)[0]
            msg.data = [-1, -1, -1, -1, -1, -1, -1, -1, 2]
            for i in range(len(who)):
                if (adjacency_matrix[1, who[i]] != 0):
                    msg.data[i] = int(who[i]) + 1
                    print('2 meet someone!')
            self.publisher_peers_2.publish(msg)
        if(not(np.allclose(adjacency_matrix[2], self.last_adjacency_matrix_3)) and np.any(adjacency_matrix[2]) and (np.count_nonzero(adjacency_matrix[2] == 1) > np.count_nonzero(self.last_adjacency_matrix_3 == 1))):
            #msg.data = [int(adjacency_matrix[2,0]), int(adjacency_matrix[2,1]), int(adjacency_matrix[2,2]), int(adjacency_matrix[2,3]), int(adjacency_matrix[2,4]), int(adjacency_matrix[2,5]), int(adjacency_matrix[2,6]), int(adjacency_matrix[2,7])]
            comparison_vector = np.equal(adjacency_matrix[2], self.last_adjacency_matrix_3)
            who = np.where(comparison_vector == False)[0]
            msg.data = [-1, -1, -1, -1, -1, -1, -1, -1, 3]
            for i in range(len(who)):
                if (adjacency_matrix[2, who[i]] != 0):
                    msg.data[i] = int(who[i]) + 1
                    print('3 meet someone!')
            self.publisher_peers_3.publish(msg)
        if(not(np.allclose(adjacency_matrix[3], self.last_adjacency_matrix_4)) and np.any(adjacency_matrix[3]) and (np.count_nonzero(adjacency_matrix[3] == 1) > np.count_nonzero(self.last_adjacency_matrix_4 == 1))):
            #msg.data = [int(adjacency_matrix[3,0]), int(adjacency_matrix[3,1]), int(adjacency_matrix[3,2]), int(adjacency_matrix[3,3]), int(adjacency_matrix[3,4]), int(adjacency_matrix[3,5]), int(adjacency_matrix[3,6]), int(adjacency_matrix[3,7])]
            comparison_vector = np.equal(adjacency_matrix[3], self.last_adjacency_matrix_4)
            who = np.where(comparison_vector == False)[0]
            msg.data = [-1, -1, -1, -1, -1, -1, -1, -1, 4]
            for i in range(len(who)):
                if (adjacency_matrix[3, who[i]] != 0):
                    msg.data[i] = int(who[i]) + 1
                    print('4 meet someone!')
            self.publisher_peers_4.publish(msg)
        if(not(np.allclose(adjacency_matrix[4], self.last_adjacency_matrix_5)) and np.any(adjacency_matrix[4]) and (np.count_nonzero(adjacency_matrix[4] == 1) > np.count_nonzero(self.last_adjacency_matrix_5 == 1))):
            #msg.data = [int(adjacency_matrix[4,0]), int(adjacency_matrix[4,1]), int(adjacency_matrix[4,2]), int(adjacency_matrix[4,3]), int(adjacency_matrix[4,4]), int(adjacency_matrix[4,5]), int(adjacency_matrix[4,6]), int(adjacency_matrix[4,7])]
            comparison_vector = np.equal(adjacency_matrix[4], self.last_adjacency_matrix_5)
            who = np.where(comparison_vector == False)[0]
            msg.data = [-1, -1, -1, -1, -1, -1, -1, -1, 5]
            for i in range(len(who)):
                if (adjacency_matrix[4, who[i]] != 0):
                    msg.data[i] = int(who[i]) + 1
                    print('5 meet someone!')
            self.publisher_peers_5.publish(msg)
        if(not(np.allclose(adjacency_matrix[5], self.last_adjacency_matrix_6)) and np.any(adjacency_matrix[5]) and (np.count_nonzero(adjacency_matrix[5] == 1) > np.count_nonzero(self.last_adjacency_matrix_6 == 1))):
            #msg.data = [int(adjacency_matrix[5,0]), int(adjacency_matrix[5,1]), int(adjacency_matrix[5,2]), int(adjacency_matrix[5,3]), int(adjacency_matrix[5,4]), int(adjacency_matrix[5,5]), int(adjacency_matrix[5,6]), int(adjacency_matrix[5,7])]
            comparison_vector = np.equal(adjacency_matrix[5], self.last_adjacency_matrix_6)
            who = np.where(comparison_vector == False)[0]
            msg.data = [-1, -1, -1, -1, -1, -1, -1, -1, 6]
            for i in range(len(who)):
                if (adjacency_matrix[5, who[i]] != 0):
                    msg.data[i] = int(who[i]) + 1
                    print('6 meet someone!')
            self.publisher_peers_6.publish(msg)
        if(not(np.allclose(adjacency_matrix[6], self.last_adjacency_matrix_7)) and np.any(adjacency_matrix[6]) and (np.count_nonzero(adjacency_matrix[6] == 1) > np.count_nonzero(self.last_adjacency_matrix_7 == 1))):
            #msg.data = [int(adjacency_matrix[6,0]), int(adjacency_matrix[6,1]), int(adjacency_matrix[6,2]), int(adjacency_matrix[6,3]), int(adjacency_matrix[6,4]), int(adjacency_matrix[6,5]), int(adjacency_matrix[6,6]), int(adjacency_matrix[6,7])]
            comparison_vector = np.equal(adjacency_matrix[6], self.last_adjacency_matrix_7)
            who = np.where(comparison_vector == False)[0]
            msg.data = [-1, -1, -1, -1, -1, -1, -1, -1, 7]
            for i in range(len(who)):
                if (adjacency_matrix[6, who[i]] != 0):
                    msg.data[i] = int(who[i]) + 1
                    print('7 meet someone!')
            self.publisher_peers_7.publish(msg)
        if(not(np.allclose(adjacency_matrix[7], self.last_adjacency_matrix_8)) and np.any(adjacency_matrix[7]) and (np.count_nonzero(adjacency_matrix[7] == 1) > np.count_nonzero(self.last_adjacency_matrix_8 == 1))):
            #msg.data = [int(adjacency_matrix[7,0]), int(adjacency_matrix[7,1]), int(adjacency_matrix[7,2]), int(adjacency_matrix[7,3]), int(adjacency_matrix[7,4]), int(adjacency_matrix[7,5]), int(adjacency_matrix[7,6]), int(adjacency_matrix[7,7])]
            comparison_vector = np.equal(adjacency_matrix[7], self.last_adjacency_matrix_8)
            who = np.where(comparison_vector == False)[0]
            msg.data = [-1, -1, -1, -1, -1, -1, -1, -1, 8]
            for i in range(len(who)):
                if (adjacency_matrix[7, who[i]] != 0):
                    msg.data[i] = int(who[i]) + 1
                    print('8 meet someone!')
            self.publisher_peers_8.publish(msg)

        self.last_adjacency_matrix_1 = np.copy(adjacency_matrix[0])
        self.last_adjacency_matrix_2 = np.copy(adjacency_matrix[1])
        self.last_adjacency_matrix_3 = np.copy(adjacency_matrix[2])
        self.last_adjacency_matrix_4 = np.copy(adjacency_matrix[3])
        self.last_adjacency_matrix_5 = np.copy(adjacency_matrix[4])
        self.last_adjacency_matrix_6 = np.copy(adjacency_matrix[5])
        self.last_adjacency_matrix_7 = np.copy(adjacency_matrix[6])
        self.last_adjacency_matrix_8 = np.copy(adjacency_matrix[7])
                             
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