import os
import math
from colorama import init, Fore, Back, Style


def colored(r, g, b, text):
    return "\033[38;2;{};{};{}m{} \033[38;2;255;255;255m".format(r, g, b, text)

def printCorrect(text):
    return colored(0,255,0, text)
    
def printWrong(text):
    return colored(255,0,0,text)

def printFree(text):
    return colored(255,255,255, text)


# states
class Node():
    def __init__(self, name, state):
        self.name = name
        self.state = state
        self.heuristic = None
        self.edges = []
    
    def getName(self):
        return self.name
    
    def getState(self):
        return self.state
    
    def getEdges(self):
        return self.edges
    
    def getHeuristic(self):
        return self.heuristic

    def printEdges(self):
        [print(e) for e in self.edges]
    
    def addEdge(self,edge):
        if type(edge) == Edge: self.edges.append(edge)
        else: print(printWrong("you can only add edge object to the list of edges!"))
        
# actions
class Edge():
    def __init__(self, node_a, node_b,cost, is_directed = False):
        
        if not (type(node_a) == Node or type(node_a) == NodeEscape):
            raise NameError("the node a is not a node object!, but is {}".format(type(node_a)))
        
        if not (type(node_b) == Node or type(node_b) == NodeEscape):
            raise NameError("the node b is not a node object! but is {}".format(type(node_b)))

        self.node_a = node_a
        self.node_b = node_b
        self.is_directed = is_directed
        self.cost = cost

        self.name = "e_" + str(self.node_a.getName()) + "->" + str(self.node_b.getName()) 

        
    def getName(self):
        return self.name
    
    def getNode_a(self):
        return self.node_a
    
    def getNode_b(self):
        return self.node_b
    
    def getCost(self):
        return self.cost
    
    def getIsDirected(self):
        return self.is_directed

class NodeNpuzzle():
    def __init__(self, state, n_puzzle = 9):
        self.state = state #[[None] * int(math.sqrt(n_puzzle))] * int(math.sqrt(n_puzzle))
        self.heuristic = None
        self.adjNodes = []
        self.n_puzzle = n_puzzle
        
    """
    the class has been modelled to have no information about the edges, 'cause possible states
    are estimted in run-time and the cost of the action to move the tile is unitary (c(s,s') = 1)
    for all the possible directions 
    """
    
    def getName(self, goal = [[1,2,3],[4,5,6],[7,8,None]]):   
        name = ""
        for i,rows in enumerate(self.state):
            for j,elem in enumerate(rows):
                if not(elem == None):    
                    if elem == goal[i][j]: name = name + printCorrect("["+ str(elem) + "] ")
                    else:  name = name + printWrong("["+ str(elem) + "] ")
                else:
                    # if elem == goal[i][j]: name = name + printCorrect("[x] ")
                    # else: name = name + printWrong("[x] ")
                    name = name + printFree("[x] ")
            name = name + "\n"
        return name
 
    
    def getState(self):
        return self.state
    
    def getHeuristic(self):
        return self.heuristic
    
    def get_n(self):
        return self.n_puzzle
    
    def getPositionFreeTileCentered(self):
        offset = int(math.sqrt(self.n_puzzle)//2)
        for i,row in enumerate(self.state):
            for j,elem in enumerate(row):
                if elem == None:
                    return (i-offset,j-offset) # centred the index to be 0,0 at the center
                
    def getPositionFreeTile(self):
        for i,row in enumerate(self.state):
            for j,elem in enumerate(row):
                if elem == None:
                    return (i,j) # centred the index to be 0,0 at the center
            
    def getPossibleMoves(self):
        pos = self.getPositionFreeTileCentered()
        
        # (m,x) coordinates of the matrix position, moving vertically i change row
        # while moving horizonatally i change columns
        moves ={"north":(-1,0),"south":(1,0),"west":(0,-1),"east":(0,1)}
        # remove moves if empty tile on the edge of the board
        
        
        if (pos[1] == -1): del moves['west']
        if (pos[1] ==  1): del moves['east']
        if (pos[0] == -1): del moves['north']
        if (pos[0] ==  1): del moves['south']
        
        return moves
    
    def addAdjacentNode(self,node):
        if type(node) == NodeNpuzzle: self.adjNodes.append(node)
        else: print(printWrong("you can only add NodeNpuzzle object to the list of adjacent nodes!"))
        
    def getAdjacentNodes(self):
        return self.adjNodes
    
class NodeEscape():
    def __init__(self, name,posx,posy,typeNode = "free"):
        self.name = name
        self.posx = posx
        self.posy = posy
        self.typeNode = typeNode
        
        self.edges = []
    
    def getName(self):
        return self.name

    
    def getTypeNode(self):
        return self.typeNode
    
    def getPosx(self):
        return self.posx
    
    def getPosy(self):
        return self.posy
    
    def getHeuristic(self):
        return self.heuristic
    
    def printEdges(self):
        [print(e) for e in self.edges]
    
    def getEdges(self):
        return self.edges
    
    def addEdge(self,edge):
        if type(edge) == Edge: self.edges.append(edge)
        else: print(printWrong("you can only add edge object to the list of edges!"))
        
class StateEscape():
    def __init__(self, initialState):
        if type(initialState) == NodeEscape: self.position = initialState
        else: print(printWrong("you can only initilize with NodeEscape object the inital position!"))
        self.hasRedKey = False
        self.hasBlueKey = False
    
    def collectRedKey(self):
        self.hasRedKey = True
        
    def collectBluedKey(self):
        self.hasBlueKey = True
    
    def getState(self):
        return [self.position.posx, self.position.posy, self.hasRedKey, self.hasBlueKey]

    def getNode(self):
        return self.position
    
    def getPossibleMoves(self):
        return self.position.getEdges()
    
    def changePosition(self,state):
        if type(state) == NodeEscape: self.position = state
        else: print(printWrong("you can only initilize with NodeEscape object the inital position!"))
        
    
    
    

    