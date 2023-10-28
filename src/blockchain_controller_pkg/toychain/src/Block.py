from random import randint
from json import loads as jsload
from copy import copy
from toychain.src.utils import compute_hash, transaction_to_dict
from os import environ
import numpy as np

import logging
logger = logging.getLogger('sc')

class Block:
    """
    Class representing a block of a blockchain containing transactions
    """

    def __init__(self, height, parent_hash, data, miner_id, timestamp, difficulty, total_diff, nonce=None,
                 state_var=None, state = None):
        self.height = height
        self.number = height
        self.parent_hash = parent_hash
        self.data = data
        self.miner_id = miner_id
        self.timestamp = timestamp
        self.difficulty = difficulty
        self.total_difficulty = total_diff + difficulty

        if state:
            self.state = state
        else:
            self.state = State(state_var)

        self.nonce = nonce
        if nonce is None:
            self.nonce = randint(0, 1000)

        self.transactions_root = self.transactions_hash()
        self.hash = self.compute_block_hash()

    def compute_block_hash(self):
        """
        computes the hash of the block header
        :return: hash of the block
        """
        _list = [self.height, self.parent_hash, self.transactions_hash(), self.miner_id, self.timestamp,
                 self.difficulty,
                 self.total_difficulty, self.nonce]

        self.hash = compute_hash(_list)

        return self.hash

    def transactions_hash(self):
        """
        computes the hash of the block transactions
        :return: the hash of the transaction list
        """
        transaction_list = [transaction_to_dict(t) for t in self.data]
        self.transactions_root = compute_hash(transaction_list)
        return self.transactions_root

    def get_header_hash(self):
        header = [self.parent_hash, self.transactions_hash(), self.timestamp, self.difficulty, self.nonce]
        return compute_hash(header)

    def increase_nonce(self):  ###### POW
        self.nonce += 1

    def __repr__(self):
        """
        Translate the block object in a string object
        """
        return f"## H: {self.height}, D: {self.difficulty}, TD: {self.total_difficulty}, P: {self.miner_id}, BH: {self.hash[0:5]}, TS:{self.timestamp}, #T:{len(self.data)}, SH:{self.state.state_hash[0:5]}##"


class StateMixin:
    @property
    def getBalances(self):
        return self.balances
    
    @property
    def getN(self):
        return self.n
        
    @property
    def call(self):
        return None
    
    @property
    def state_variables(self):
        return {k: v for k, v in vars(self).items() if not (k.startswith('_') or k == 'msg' or k == 'block' or k == 'private')}
    
    @property
    def state(self):
        return {k: v for k, v in vars(self).items() if not (k.startswith('_') or k == 'msg' or k == 'block' or k == 'private')}

    @property
    def state_hash(self):
        return compute_hash(self.state.values())
    
    def apply_transaction(self, tx, block):

        self.msg = tx
        self.block = block

        #self.balances.setdefault(tx.sender, 0)
        #self.balances.setdefault(tx.receiver, 0)

        # Check sender funds
        if tx.value and self.balances[tx.sender] < tx.value:
            return
        
        # Apply the transfer of value
        self.balances[tx.sender] -= tx.value
        #self.balances[tx.receiver] += tx.value
        
        # Apply the other functions contained in data
        self.n += 1

        if tx.data and 'function' in tx.data and 'inputs' in tx.data:
            function = getattr(self, tx.data.get("function"))
            inputs   = tx.data.get("inputs")
            try:
                function(*inputs)
            except Exception as e:
                raise e

class State(StateMixin):

    def __init__(self, state_variables = None):

        if state_variables is not None:
            for var, value in state_variables.items(): setattr(self, var, value)     

        else:
            self.private     = {}
            self.n           = 0
            self.balances    = {'1': 300,'2': 300,'3': 300,'4': 300,'5': 300,'6': 300,'7': 300,'8': 300}

            # My custom state variables
            self.candidate_LC = {'LC_Descriptor': [], 'LC_ID_R': [], 'LC_Odomx_R': [], 'LC_Odomy_R': [], 'LC_Keyframe_R': [], 'LC_ID_S': [], 'LC_Odomx_S': [], 'LC_Odomy_S': [], 'LC_Keyframe_S': [], 'LC_dx': [], 'LC_dy': [], 'LC_SCENE': [], 'LC_Security': []}
            self.validated_LC   = {'ID_Sender': [], 'Descriptor': []}
            self.triangles = []
            self.reputation = {'1': 0,'2': 0,'3': 0,'4': 0,'5': 0,'6': 0,'7': 0,'8': 0}

    # Smart contract function to control triangles
    def apply_validation(self, LC_Descriptor, LC_ID_R, LC_Odomx_R, LC_Odomy_R, LC_Keyframe_R, LC_ID_S, LC_Odomx_S, LC_Odomy_S, LC_Keyframe_S, LC_dx, LC_dy, LC_SCENE):

        # Custom constants of the smart contract, security_level = 0, 1, 2, 3 and so on ... 0 -> all, 1 -> all in a triangle, 2 -> all in 2 triangles with no same Senders
        bound = 0.0001
        security_level = 1
        new_triangle_i = True
        new_triangle_j = True
        new_triangle_k = True

        # New Loop Closure registration
        self.candidate_LC['LC_Descriptor'].append(LC_Descriptor)
        self.candidate_LC['LC_ID_R'].append(LC_ID_R)
        self.candidate_LC['LC_Odomx_R'].append(LC_Odomx_R)
        self.candidate_LC['LC_Odomy_R'].append(LC_Odomy_R)
        self.candidate_LC['LC_Keyframe_R'].append(LC_Keyframe_R)
        self.candidate_LC['LC_ID_S'].append(LC_ID_S)
        self.candidate_LC['LC_Odomx_S'].append(LC_Odomx_S)
        self.candidate_LC['LC_Odomy_S'].append(LC_Odomy_S)
        self.candidate_LC['LC_Keyframe_S'].append(LC_Keyframe_S)
        self.candidate_LC['LC_dx'].append(LC_dx)
        self.candidate_LC['LC_dy'].append(LC_dy)
        self.candidate_LC['LC_SCENE'].append(LC_SCENE)
        self.candidate_LC['LC_Security'].append(0)

        # Triangles construction
        for i in range(len(self.candidate_LC['LC_SCENE'])):
            for j in range(len(self.candidate_LC['LC_SCENE'])):
                for k in range(len(self.candidate_LC['LC_SCENE'])):
                    # Check for the scene matching and No mutual edges between two participants (to avoid collusions)
                    if((self.candidate_LC['LC_SCENE'][i] == self.candidate_LC['LC_SCENE'][j] == self.candidate_LC['LC_SCENE'][k] and i != j and i != k and j!= k) and (self.candidate_LC['LC_ID_S'][i] != self.candidate_LC['LC_ID_S'][j] != self.candidate_LC['LC_ID_S'][k]) and (self.candidate_LC['LC_ID_R'][i] != self.candidate_LC['LC_ID_R'][j] != self.candidate_LC['LC_ID_R'][k])):
                        # Check a complete triangle has not already been approved
                        if(not(self.candidate_LC['LC_Descriptor'][i] in self.validated_LC['Descriptor'] and self.candidate_LC['LC_Descriptor'][j] in self.validated_LC['Descriptor'] and self.candidate_LC['LC_Descriptor'][k] in self.validated_LC['Descriptor'])):
                            # This check is for the Head-Tail correspondence in every possible vertice condition of the triangle
                            if(self.candidate_LC['LC_ID_S'][i] == self.candidate_LC['LC_ID_R'][j] and self.candidate_LC['LC_ID_S'][j] == self.candidate_LC['LC_ID_R'][k] and self.candidate_LC['LC_ID_S'][k] == self.candidate_LC['LC_ID_R'][i] and self.candidate_LC['LC_Keyframe_S'][i] == self.candidate_LC['LC_Keyframe_R'][j] and self.candidate_LC['LC_Keyframe_S'][j] == self.candidate_LC['LC_Keyframe_R'][k] and self.candidate_LC['LC_Keyframe_S'][k] == self.candidate_LC['LC_Keyframe_R'][i]):
                                # This check is for every possible combination without taking into account the direction of the trasformation. However, it will result True iff the arrows have coherent circular direction
                                if((abs(self.candidate_LC['LC_dx'][i] + self.candidate_LC['LC_dx'][j] + self.candidate_LC['LC_dx'][k]) < bound) and (abs(self.candidate_LC['LC_dy'][i] + self.candidate_LC['LC_dy'][j] + self.candidate_LC['LC_dy'][k]) < bound)):
                                    # Possibly increase the security parameter of the approved LCs
                                    for z in range(len(self.triangles)):
                                        if(self.candidate_LC['LC_ID_S'][i] in self.triangles[z] and self.candidate_LC['LC_ID_S'][j] in self.triangles[z] and self.candidate_LC['LC_ID_S'][k] in self.triangles[z]):
                                            if(self.candidate_LC['LC_Descriptor'][i] in self.triangles[z]):
                                                new_triangle_i = False
                                            if(self.candidate_LC['LC_Descriptor'][j] in self.triangles[z]):
                                                new_triangle_j = False
                                            if(self.candidate_LC['LC_Descriptor'][k] in self.triangles[z]):
                                                new_triangle_k = False
                                    if(new_triangle_i == True):
                                        self.candidate_LC['LC_Security'][i] += 1
                                    if(new_triangle_j == True):
                                        self.candidate_LC['LC_Security'][j] += 1
                                    if(new_triangle_k == True):
                                        self.candidate_LC['LC_Security'][k] += 1
                                    new_triangle_i = True
                                    new_triangle_j = True
                                    new_triangle_k = True
                                    # Add the new triangle (ID_S + Descriptor of the 3 participants)
                                    self.triangles.append([self.candidate_LC['LC_ID_S'][i],self.candidate_LC['LC_ID_S'][j],self.candidate_LC['LC_ID_S'][k],self.candidate_LC['LC_Descriptor'][i],self.candidate_LC['LC_Descriptor'][j],self.candidate_LC['LC_Descriptor'][k]])
                                    # Send back the validated LCs, if not already published, the field 'LC_Descriptor' univocally defines
                                    if(self.candidate_LC['LC_Security'][i] >= security_level):
                                        self.balances[str(self.candidate_LC['LC_ID_S'][i])] += 2
                                        self.reputation[str(self.candidate_LC['LC_ID_S'][i])] += int(self.candidate_LC['LC_Security'][i])
                                        if(self.candidate_LC['LC_Descriptor'][i] not in self.validated_LC['Descriptor']):
                                            self.validated_LC['ID_Sender'].append(self.candidate_LC['LC_ID_S'][i])
                                            self.validated_LC['Descriptor'].append(self.candidate_LC['LC_Descriptor'][i])
                                    if(self.candidate_LC['LC_Security'][j] >= security_level):
                                        self.balances[str(self.candidate_LC['LC_ID_S'][j])] += 2
                                        self.reputation[str(self.candidate_LC['LC_ID_S'][j])] += int(self.candidate_LC['LC_Security'][j])
                                        if(self.candidate_LC['LC_Descriptor'][j] not in self.validated_LC['Descriptor']):
                                            self.validated_LC['ID_Sender'].append(self.candidate_LC['LC_ID_S'][j])
                                            self.validated_LC['Descriptor'].append(self.candidate_LC['LC_Descriptor'][j])
                                    if(self.candidate_LC['LC_Security'][k] >= security_level):
                                        self.balances[str(self.candidate_LC['LC_ID_S'][k])] += 2
                                        self.reputation[str(self.candidate_LC['LC_ID_S'][k])] += int(self.candidate_LC['LC_Security'][k])
                                        if(self.candidate_LC['LC_Descriptor'][k] not in self.validated_LC['Descriptor']):
                                            self.validated_LC['ID_Sender'].append(self.candidate_LC['LC_ID_S'][k])
                                            self.validated_LC['Descriptor'].append(self.candidate_LC['LC_Descriptor'][k])

    # Read the blockchain state variables containing the validated LCs
    def getApprovedLC(self):

        return self.validated_LC

    # Read the blockchain state variables containing the balances values
    def getReputation(self):

        return self.reputation
    