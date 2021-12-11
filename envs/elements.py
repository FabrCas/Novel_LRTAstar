import os


# states
class NodePos():
    def __init__(self, name, posx, posy):
        self.name = name
        self.posx = posx
        self.posy = posy
        self.edges = []
    
    def getName(self):
        return self.name
    
    def getPosx(self):
        return self.posx
    
    def getPosy(self):
        return self.posy
    
    def getEdges(self):
        return self.edges
    
    def printEdges(self):
        [print(e) for e in self.edges]
    
    def addEdge(self,edge):
        self.edges.append(edge)

class Node():
    def __init__(self, name, state):
        self.name = name
        self.state = state
        self.edges = []
    
    def getName(self):
        return self.name
    
    def getState(self):
        return self.state
    
    def getEdges(self):
        return self.edges

    def printEdges(self):
        [print(e) for e in self.edges]
    
    def addEdge(self,edge):
        if type(edge) == Edge: self.edges.append(edge)
        else: print("you can only add edge object to the list of edges!")
    
    
# actions
class Edge():
    def __init__(self, node_a, node_b,cost, is_ordered):
        
        if not (type(node_a) == Node or type(node_a) == NodePos):
            raise NameError("the node a is not a node object!")
        
        if not (type(node_b) == Node or type(node_b) == NodePos):
            raise NameError("the node b is not a node object!")

        self.node_a = node_a
        self.node_b = node_b
        self.is_ordered = is_ordered
        self.cost = cost
        if self.is_ordered:
            self.name = "e_" + str(self.node_a.getName()) + "->" + str(self.node_b.getName()) 
        else:
            self.name = "e_" + str(self.node_a.getName()) + "<->" + str(self.node_b.getName())
        
    def getName(self):
        return self.name
    
    def getNode_a(self):
        return self.node_a
    
    def getNode_b(self):
        return self.node_b
    
    def getCost(self):
        return self.cost
    
    def getIsOrdered(self):
        return self.is_ordered
    

    
    
    
        