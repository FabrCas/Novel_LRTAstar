import json
import os
from envs.elements import Node as Node
from envs.elements import Edge as Edge



learned_heuristics = {}
initial_state = "a"
# identical to the horizonatal distanceat the start
def compute_heuristics(node):
    
    learned_heuristics = loadHeuristics()
    keys_list = list(learned_heuristics.keys())
    if not(node.name in keys_list):
        node.heuristic = node.state
    else:
        node.heuristic = learned_heuristics[node.name]

def clean_heuristicsMemorized():
    del(learned_heuristics)
    learned_heuristics = {}
    
def memorize_heuristics(node,value):
    learned_heuristics[node.name] = value

def saveHeuristics():
    file = open('./envs/barrierEnvH.json', "w")
    json.dump(learned_heuristics, file)
    
def loadHeuristics():
    if os.path.exists('./envs/barrierEnvH.json'):
        file = open('./envs/barrierEnvH.json', "r")
        data = json.load(file)
        learned_heuristics = data
        return learned_heuristics
    else:
        print("heuristics are not present")
    

def create_env(initial_state_name = "a"):
    nodes = []
    file = open('./envs/barrierEnv.json')
    data = json.load(file)["barrier_env"]
    nodes_data = data['nodes']
    edges_data = data['edges']
    # print(nodes_data)
    # print(edges_data)
    # create nodes
    for node_data in nodes_data:
        print("creating the node {} ...".format(node_data['name']))
        tmp_node = Node(node_data['name'], node_data['state'])
        compute_heuristics(tmp_node)
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
    return nodes

           
            
           


    
    
