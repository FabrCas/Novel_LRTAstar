import json
import os
import math
from problems.elements import Node as Node
from problems.elements import Edge as Edge
from problems.elements import NodeNpuzzle as Nodepuzzle
from problems.elements import StateEscape as StateEscape
from problems.elements import NodeEscape as NodeEscape





class problem():
    def __init__(self, name, initial_state):
        self.nodes = []
        self.learned_heuristics = {}
        self.name = name
        self.initial_state = initial_state
        
    def compute_heuristics(self,node):
        keys_list = list(self.learned_heuristics.keys())
        if not(node.name in keys_list):
            node.heuristic = self.initialHeuristic(node)
        else:
            node.heuristic = self.learned_heuristics[node.name]
    
    def memorize_heuristics(self,node,value):
        self.learned_heuristics[node.name] = value
    
    def clean_heuristicsMemorized(self):
        del(self.learned_heuristics)
        self.learned_heuristics = {}
        file = open('./problems/'+ self.name +'H.json', "w")
        json.dump({}, file)
    
    def saveHeuristics(self):
        file = open('./problems/'+ self.name +'H.json', "w")
        json.dump(self.learned_heuristics, file)
        
    def loadHeuristics(self):
        if os.path.exists('./problems/'+ self.name +'H.json'):
            file = open('./problems/'+ self.name +'H.json', "r")
            data = json.load(file)
            self.learned_heuristics = data
            return self.learned_heuristics
        else:
            print("heuristics are not present")
    
    def getLearnedeHeuristics(self):
        return self.learned_heuristics
    
    def getNodes(self):
        return self.nodes
    
    def getInitialState(self):
        return self.initial_state
    
    def getName(self):
        return self.name

# *****************************************************************************

"""
first kind of problem:
    simple path finding
"""

class SimpleTestEnv(problem):
    def __init__(self, name, initial_state):
        super().__init__(name, initial_state)

    
    def initialHeuristic(self,node):
        """ 
        heuristic obtained relaxing the problem:
        identical to the horizonatal distance from the node to the barrier"""
        return node.state
        
    
    def create_env(self):
        nodes = []
        file = open('./problems/'+ self.name +'.json')
        data = json.load(file)[self.name]
        nodes_data = data['nodes']
        edges_data = data['edges']
        # print(nodes_data)
        # print(edges_data)
        # create nodes
        for node_data in nodes_data:
            print("creating the node {} ...".format(node_data['name']))
            tmp_node = Node(node_data['name'], node_data['state'])
            self.compute_heuristics(tmp_node)
            nodes.append(tmp_node)
        # create edges
        for edge_data in edges_data:
            
            from_to = edge_data['name'].split("-")
            name_node_a = from_to[0]
            name_node_b = from_to[1]
            print("creating the edge {}->{} ...".format(name_node_a,name_node_b))
            nodeA = None
            nodeB = None
            
            for node in nodes:
                if node.name == name_node_a:
                    nodeA = node
                elif node.name == name_node_b:
                    nodeB = node
                else:
                    continue
            
            edge = Edge(nodeA, nodeB, edge_data['cost'], edge_data['is_directed'])
            nodeA.addEdge(edge)
            if not (edge.getIsDirected()):
                edge = Edge(nodeB, nodeA, edge_data['cost'], edge_data['is_directed'])
                print("creating the edge {}->{} ...".format(name_node_b,name_node_a))
                nodeB.addEdge(edge)
                
        # return the list of nodes that are been connected       
        self.nodes = nodes
    

# environments to test the correctness of the algoriht 

# optimality case
class BarrierEnv1(SimpleTestEnv):
    
    def __init__(self, initial_state = "a", load_h = False):
        print("\n***************** creating the state space *******************\n") 
        # call superclass
        super().__init__("barrierEnv1", initial_state)
        if load_h: self.loadHeuristics()

# dead-end case    
class BarrierEnv2(SimpleTestEnv):
    
    def __init__(self, initial_state = "a", load_h = False):
        print("\n***************** creating the state space *******************\n") 
        # call superclass
        super().__init__("barrierEnv2", initial_state)
        if load_h: self.loadHeuristics()
        
    
# *****************************************************************************
"""
second kind of problem:
    solve n-puzzle problems 
"""

class Puzzle8(problem):
    # not all the inital state can be solvable, we have to check that the number of
    # invertions it's odd.
    def __init__(self, initial_state = [[1,4,3],[7,None,6],[5,8,2]], load_h = False):
        print("\n**************** initializing the problem ********************\n") 
        # call superclass
        super().__init__("8_puzzle", initial_state)
        self.starting_node = None
        
        # select here other configutions
        self.selectEasyState2()
        if load_h: self.loadHeuristics()
        self._isSolvable(initial_state)
        
        
        
    def selectHardState1(self):
        self.initial_state = [[8,6,7],[2,5,4],[3,None,1]]  # minimum number of moves is 31
        
    def selectHardState2(self):
        self.initial_state = [[6,4,7],[8,5,None],[3,2,1]]  # minimum number of moves is 31
    
    def selectEasyState(self):
        self.initial_state = [[1,3,5],[4,2,None],[7,8,6]]  # minimum number of moves is 5
        
    def selectEasyState2(self):
            self.initial_state = [[1,3,5],[7,4,6],[None,2,8]]  # minimum number of moves is 10

    def selectMidState(self):  # default one
                self.initial_state = [[1,4,3],[7,None,6],[5,8,2]]  # minimum number of moves is 14
                    
    def selectMidState2(self):
            self.initial_state = [[7,1,5],[3,None,6],[2,4,8]]  # minimum number of moves is 16
            
        
    def _getInvertions(self,state):
        inv_count = 0
        empty_value = None
        for i in range(0, 9):
            for j in range(i + 1, 9):
                if state[j] != empty_value and state[i] != empty_value and state[i] > state[j]:
                    inv_count += 1
        return inv_count
        
    def _isSolvable(self, initial_state):
        list_values = [j for sub in initial_state for j in sub]
        inv_count = self._getInvertions(list_values)
        if not (inv_count % 2 == 0):
            raise NameError("The selected state of the 8 puzzle problem it's impossible to solve!")
        else: print("the problem is solvable")
            
        
    def compute_heuristics(self,node):
        keys_list = list(self.learned_heuristics.keys())
        if not(str(node.state) in keys_list):
            node.heuristic = self.initialHeuristic(node)
        else:
            node.heuristic = self.learned_heuristics[str(node.state)]
            
    def memorize_heuristics(self,node,value):
        # print(node.state)
        self.learned_heuristics[str(node.state)] = value

    # goal ->  [[1,2,3],[4,5,6],[7,8,None]]
    def initialHeuristic(self, node):
        state = node.state
        relaxed_h = 8 - ((state[0][0]==1) + (state[0][1]==2) + (state[0][2]==3) + \
        (state[1][0]==4) + (state[1][1]==5) + (state[1][2]==6) + \
        (state[2][0]==7) + (state[2][1]==8) + (state[2][2]=="x"))
        return relaxed_h
       
    def create_env(self): 
        starting_node = Nodepuzzle(self.initial_state)
        self.compute_heuristics(starting_node)
        self.starting_node = starting_node
        
    def getStartingNode(self):
        return self.starting_node
    
# *****************************************************************************
# cost edges -> ceil of the euclidean distance in the cartesian plane * 3 (diagonal) or * 2 (horizontal and vertical)

class Escape(problem):
    def __init__(self, initial_state = (4,5), load_h = False):
        print("\n**************** initializing the problem ********************\n") 
        # call superclass
        super().__init__("escape", initial_state)
        self.state = None
        self.blue_door_pos = None
        self.red_door_pos = None
        self.red_key_pos = None 
        self.blue_key_pos = None 
        if load_h: self.loadHeuristics()
    
    def _manhattanDistance(self, pos1,pos2):
        return abs(pos1[0]- pos2[0]) + abs(pos1[1] - pos2[1])
        
        
    def initialHeuristic(self, state):
        state_info = state.getState()

        pos = (state_info[0],state_info[1])
        has_rk = state_info[2]
        has_bk = state_info[3]
        # print(self.blue_door_pos)
        # print(self.red_door_pos)
        # print(self.blue_key_pos)
        # print(self.red_key_pos)
        # print(pos)
        
        # initialize the 2 distances
        red_distance = self._manhattanDistance(pos, self.red_key_pos) + self._manhattanDistance(self.red_key_pos, self.red_door_pos)
        blue_distance = self._manhattanDistance(pos, self.blue_key_pos) + self._manhattanDistance(self.blue_key_pos, self.blue_door_pos)
        
        
        if has_bk:
            red_distance = self._manhattanDistance(pos, self.red_key_pos) + self._manhattanDistance(self.red_key_pos, self.red_door_pos)
            blue_distance = self._manhattanDistance(pos, self.blue_door_pos)
        elif has_rk:
            red_distance = self._manhattanDistance(pos, self.red_door_pos)
            blue_distance = self._manhattanDistance(pos, self.blue_key_pos) + self._manhattanDistance(self.blue_key_pos, self.blue_door_pos)
        
        
        return min(red_distance,blue_distance)
    
    # this time the heuristic is not pre-evaluated, but directly on the algorithm
    # attention: heuristic from the state and not from the node (more info about key obtained)
    def compute_heuristics(self, state = None):
        if state == None: state = self.state
        
        keys_list = list(self.learned_heuristics.keys())
        if not(str(state.getState()) in keys_list):
            h = self.initialHeuristic(state)
        else:
            h = self.learned_heuristics[str(state.getState())]
        return h
    
    def memorize_heuristics(self,state,value):
        # print(node.state)
        self.learned_heuristics[str(state.getState())] = value
        
    def create_env(self):
        
        nodes = []
        
        starting_node = None
        file = open('./problems/'+ self.name +'.json')
        data = json.load(file)[self.name]
        nodes_data = data['nodes']
        edges_data = data['edges']
        # print(nodes_data)
        # print(edges_data)
        # create nodes
        for node_data in nodes_data:
            print("creating the node {} ...".format(node_data['name']))
            
            tmp_node = NodeEscape(node_data['name'],node_data['state']['posRow'], node_data['state']['posColumn'], node_data['state']['type'])

            # h(s) not pre-computed here
            
            nodes.append(tmp_node)
            
            # save a reference to the actual state for the environmnet problem
            if node_data['state']['posRow'] == self.initial_state[0] and node_data['state']['posColumn'] == self.initial_state[1]:
                starting_node = tmp_node
            
            # save door position
            if node_data['state']['type'] == "blue_door":
                self.blue_door_pos = ( node_data['state']['posRow'], node_data['state']['posColumn'] )
            elif node_data['state']['type'] == "red_door":
                self.red_door_pos = ( node_data['state']['posRow'], node_data['state']['posColumn'] )
            if node_data['state']['type'] == "blue_key":
                self.blue_key_pos = ( node_data['state']['posRow'], node_data['state']['posColumn'] )
            elif node_data['state']['type'] == "red_key":
                self.red_key_pos = ( node_data['state']['posRow'], node_data['state']['posColumn'] )
                
        # create edges
        for edge_data in edges_data:
            
            from_to = edge_data['name'].split("-")
            name_node_a = from_to[0]
            name_node_b = from_to[1]
            print("creating the edge {}->{} ...".format(name_node_a,name_node_b))
            nodeA = None
            nodeB = None
            
            for node in nodes:
                if node.name == name_node_a:
                    nodeA = node
                elif node.name == name_node_b:
                    nodeB = node
                else:
                    continue
            
            edge = Edge(nodeA, nodeB, edge_data['cost'])
            nodeA.addEdge(edge)
            
            # unoriented edges, add the one in the opposite direction

            edge2 = Edge(nodeB, nodeA, edge_data['cost'])
            print("creating the edge {}->{} ...".format(name_node_b,name_node_a))
            nodeB.addEdge(edge2)
                
        # create the actual state for the environment
        
        self.state = StateEscape(starting_node)
        
    def getState(self):
        return self.state
        
    
    
# *****************************************************************************        

def getEnv(problem):
    types = {
        "BarrierEnv1": BarrierEnv1,
        "BarrierEnv2":BarrierEnv2,
        "8_puzzle": Puzzle8,
        "Escape": Escape
        }
    return types[problem]