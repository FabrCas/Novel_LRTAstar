import json
import os
from problems.elements import Node as Node
from problems.elements import Edge as Edge


class SimpleTestEnv():
    def __init__(self, name, initial_state):
        self.nodes = []
        self.learned_heuristics = {}
        self.name = name
        self.initial_state = initial_state
        
    
    
    def compute_heuristics(self,node):
        keys_list = list(self.learned_heuristics.keys())
        if not(node.name in keys_list):
            """ 
            heuristic obtained relaxing the problem:
            identical to the horizonatal distance from the node to the barrier"""
            node.heuristic = node.state
        else:
            node.heuristic = self.learned_heuristics[node.name]
    
    def clean_heuristicsMemorized(self):
        del(self.learned_heuristics)
        self.learned_heuristics = {}
        file = open('./problems/'+ self.name +'H.json', "w")
        json.dump({}, file)
    
    def memorize_heuristics(self,node,value):
        self.learned_heuristics[node.name] = value
    
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
    
    def getLearnedeHeuristics(self):
        return self.learned_heuristics
    
    def getNodes(self):
        return self.nodes
    
    def getInitialState(self):
        return self.initial_state
        



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
        
    


           
            
           


    
    
