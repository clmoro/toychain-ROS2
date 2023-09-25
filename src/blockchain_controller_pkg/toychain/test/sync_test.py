import logging
import sys

sys.path.append("/home/angelo/ROS2-WORKSPACES/toychain-ROS2/src/blockchain_controller_pkg/")

from toychain.src.Node import Node
from toychain.src.consensus.ProofOfAuth import ProofOfAuthority
from toychain.src.Block import Block, State
from toychain.src.Transaction import Transaction
from toychain.src.constants import LOCALHOST
from toychain.src.utils import gen_enode


auth_signers = [gen_enode(i) for i in range(1,4)]
initial_state = State()

GENESIS_BLOCK = Block(0, 0000, [], auth_signers, 0, 0, 0, nonce = 1, state = initial_state)
CONSENSUS = ProofOfAuthority(genesis = GENESIS_BLOCK)

# Create the nodes with (id, host, port, consensus_protocol)
node1 = Node(1, LOCALHOST, 1231, CONSENSUS)
node2 = Node(2, LOCALHOST, 1232, CONSENSUS)
node3 = Node(3, LOCALHOST, 1233, CONSENSUS)

logging.basicConfig(level=logging.DEBUG)

def init_network():
    
    # Start the TCP for syncing mempool and blockchain
    node1.start_tcp()
    node2.start_tcp()
    node3.start_tcp()
    
    # Start the mining process for each node
    node1.start_mining()
    node2.start_mining()
    node3.start_mining()

    # Add the peers of each node
    node1.add_peer(node2.enode)
    node1.add_peer(node3.enode)
    node2.add_peer(node1.enode)
    node2.add_peer(node3.enode)
    node3.add_peer(node1.enode)
    node3.add_peer(node2.enode)

import time
if __name__ == '__main__':

    init_network()

    # Setup simulation steps
    max_steps = 1500
    curr_step = 0
    step = 1

    while True:
        print(curr_step)
        node1.step()
        node2.step()
        node3.step()
        time.sleep(0.0001)
        # Transaction 1
        if(curr_step == 500):
            txdata = {'function': 'apply_validation', 'inputs': ['1', '1', '0.11', '0.22']}
            tx = Transaction(sender = 1, receiver = 2, value = 0, data = txdata)
            node1.send_transaction(tx)
        # Transaction 2
        if(curr_step == 700):
            txdata = {'function': 'apply_validation', 'inputs': ['2', '2', '0.01', '0.11']}
            tx = Transaction(sender = 2, receiver = 3, value = 0, data = txdata)
            node2.send_transaction(tx)
        # Transaction 3
        if(curr_step == 900):
            txdata = {'function': 'apply_validation', 'inputs': ['3', '3', '0.1', '0.3']}
            tx = Transaction(sender = 3, receiver = 1, value = 0, data = txdata)
            node3.send_transaction(tx)
        # Transaction 4
        if(curr_step == 1200):
            txdata = {'function': 'apply_validation', 'inputs': ['4', '2', '0.02', '0.11']}
            tx = Transaction(sender = 2, receiver = 3, value = 0, data = txdata)
            node2.send_transaction(tx)
        # Test getApprovedLC
        if(curr_step == 1200):
            print(node1.sc.getApprovedLC())
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

    # Test {ID, Sender} dictionary
    appr1 = node1.sc.getApprovedLC()
    for i in range(len(appr1['Sender'])):
        if(appr1['Sender'][i] == '1'):
            print(appr1['ID'][i])



