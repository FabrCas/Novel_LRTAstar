import os
import math
import time
from problems.elements import NodeNpuzzle as Nodepuzzle
from problems.elements import printCorrect, printWrong
from copy import deepcopy

class LRTAStarBarrierEnv():
    
    def __init__(self, env, state_goal = 0, n_simulation = math.inf):
        print("\n********************** starting LSTA* ************************\n")
        self.start = [node for node in env.getNodes() if node.name==env.getInitialState()][0] #this represent the initial state for the algorithm
        self.goal = state_goal
        self.env = env
        self.plan = [] # a list of edges
        # self.h_updated = False
        self.n_simulation = n_simulation
    
    # implement simulation and exectution separetely

    
    def forward(self, save_h = False, verbose = True):
        h_updated = False # initialize the flag for updates to false
        iteration = 0 
        startTime = time.time()
        if not(self.plan == []): self.plan = []
        
        print("- Initial node: {} with state: {}\n- goal state {}".format(self.start.name, self.start.state, self.goal)) if verbose else 0
        total_cost_acc = 0
        actual_state = self.start
        
        while not (actual_state.state == self.goal):
            print("\n-----------[Iteration n°{}]-----------\n".format(iteration +1)) if verbose else 0
            # evaluate next state
            print("- Looking edges of state {}".format(actual_state.name)) if verbose else 0
            # find the lowest estimated cost
            next_node = None
            chosen_edge = None
            lowest_cost_f = math.inf # initialize the smallest cost with infinite
            lowest_cost_c = math.inf 
            for edge in actual_state.edges:
                other_node = edge.getNode_b()
                
                c_i = edge.cost
                h_i = other_node.heuristic 
                f_i = c_i + h_i # estimated cost
                print("- Evaluating motion to {}, estimated cost f= c(s,s') + h(s') = {} + {} = {}".format(other_node.name, c_i,h_i,f_i)) if verbose else 0
                if(f_i < lowest_cost_f):
                    lowest_cost_f = f_i
                    lowest_cost_c = c_i
                    next_node = other_node
                    chosen_edge = edge
            
            print("- All edges checked")if verbose else 0
            
            # we need to update?
            print("- Evaluating the updating of the heuristics... ")if verbose else 0
            if(lowest_cost_f > actual_state.heuristic):
                print("- Updating heuristic of {}, from {} to {}".format(actual_state.name,actual_state.heuristic,lowest_cost_f))if verbose else 0
                actual_state.heuristic = lowest_cost_f
                self.env.memorize_heuristics(actual_state,lowest_cost_f)
                if not(h_updated): h_updated = True
            else:
                print("- No update")if verbose else 0
                    
            # move to next state
            print("- Next state is {}".format(next_node.name) )if verbose else 0
            actual_state = next_node
            self.plan.append(chosen_edge)
            
            total_cost_acc += lowest_cost_c
            iteration+=1 
            print("- Actual cost for the plan {}".format(total_cost_acc)) if verbose else 0
        
        print("\n-----------[Final recap]-----------\n") if verbose else 0
        print("- The goal has been found in the state {}". format(actual_state.name)) 
        print("- Definitive cost for the plan {}".format(total_cost_acc)) 
        print("\n- Plan:") 
        for action in self.plan:
            print("--> move from {} to {}".format(action.getNode_a().name, action.getNode_b().name))
        print("\n- End execution, time: {} [s]".format( (time.time() -startTime) )) if verbose else 0
        print()
        
        if save_h: self.env.saveHeuristics()
        return h_updated
        
    def simulate(self):
        print("\n********************* Simulating LSTA* ***********************\n")
        if self.n_simulation == math.inf:
            convergence = not(self.forward(False,False))
            while(not(convergence)):
                convergence = not(self.forward(False,False))
        
    def execution(self):
        self.simulate()
        print("\n********************* Execution LSTA* ************************\n")
        self.forward(True,True)
            
        
 
class LRTAnPuzzle():       
    
    def __init__(self, env, n_simulation = math.inf, depth_simulations = 500):
        print("\n********************** starting LSTA* ************************\n")
        self.start = env.getStartingNode()
        if (self.start.get_n() == 9): self.goal = [[1,2,3],[4,5,6],[7,8,None]]
        else: self.goal = None 
        
        self.env = env
        self.plan = [self.start.getState()] # a list of edges
        # self.h_updated = False
        self.n_simulation = n_simulation
        self.depth_simulations = depth_simulations
        

    def forward(self, save_h = False, verbose = True):
        h_updated = False # initialize the flag for updates to false
        goal_found = False
        iteration = 0 
        startTime = time.time()
        if not(self.plan == []): self.plan = []
    
        total_cost_acc = 0
        actual_state = self.start
        
        # while not (actual_state.state == self.goal):
        while (iteration < self.depth_simulations) and (not (actual_state.state == self.goal)):   
            print("\n-----------[Iteration n°{}]-----------\n".format(iteration +1)) if verbose else 0
                
            print("- State: \n") if verbose else 0
            print(actual_state.getName()) if verbose else 0
            print() if verbose else 0
                
            # estabilish next possible states
                
            posFree = actual_state.getPositionFreeTile()
            
              
            print("- Possbile moves:") if verbose else 0
            # new possible states
            for k,v in actual_state.getPossibleMoves().items():
                print("--> move to {}".format(k))if verbose else 0
                
                new_pos_free = (posFree[0] + v[0], posFree[1] + v[1])
                value_tile_exchanged = actual_state.state[new_pos_free[0]][new_pos_free[1]]
                new_state = deepcopy(actual_state.state)
                
                # print("moving free tail in the place of {}".format(value_tile_exchanged))
                
                new_state[new_pos_free[0]][new_pos_free[1]] = None
                new_state[posFree[0]][posFree[1]] = value_tile_exchanged
                
                node = Nodepuzzle(new_state)
               
                # print(node.getName())
                self.env.compute_heuristics(node)
                actual_state.addAdjacentNode(node)
                
                # print()
            
            # evaluate next state
            # find the lowest estimated cost
                
            print()
            print("- Analyzing moves:") if verbose else 0
            next_node = None
            chosen_edge = None
            lowest_cost_f = math.inf # initialize the smallest cost with infinite
    
            for idx,node in enumerate(actual_state.getAdjacentNodes()):
                c_i = 1 # unitary cost for each move
                h_i = node.getHeuristic() 
                f_i = c_i + h_i # estimated cost
                print("--> State n°{} with h(s) = {} & f(s) = {}". format(idx, h_i,f_i))if verbose else 0
                print(node.getName())if verbose else 0
                
                if(f_i < lowest_cost_f):
                    lowest_cost_f = f_i
                    next_node = node
            
            print("- All states checked")if verbose else 0
    
            # we need to update?
            print("- Evaluating the updating of the heuristics... ")if verbose else 0
            if(lowest_cost_f > actual_state.heuristic):
                print("- Updating heuristic from {} to {}, for the actual state".format(actual_state.heuristic,lowest_cost_f))if verbose else 0
    
                
                actual_state.heuristic = lowest_cost_f
                self.env.memorize_heuristics(actual_state,lowest_cost_f)
                if not(h_updated): h_updated = True
            else:
                print("- No update")if verbose else 0
                        
            # move to next state
            print("- Next state is:")if verbose else 0
            print(node.getName())if verbose else 0
            actual_state = next_node
            self.plan.append(next_node)
            if actual_state.state == self.goal: goal_found = True
            
            iteration +=1  # in this case the total cost is equal to the number of iteration having the unitary cost 
        
        print("\n-----------[Final recap]-----------\n") if verbose else 0
        print("\n- End execution, time: {} [s]".format( (time.time() -startTime) )) if verbose else 0
        if h_updated:
            print("- The heuristic has been updated")if verbose else 0
        else:
            print("- The heuristic hasn't been updated")if verbose else 0
        
        if goal_found:
            print("- The goal has been found with the cost/a number of iterations equal to {}". format(iteration))
            if (len(self.plan)) > 100:
                print("\n- The Plan is too long to be printed:")
            else:
                print("\n- Plan:") 
                for node in self.plan:
                    print(node.getName())
        else:
            print("- The goal has not been found in {} iterations". format(iteration))
                
        print()if verbose else 0
        
        if save_h: self.env.saveHeuristics()
        return h_updated
        
    def simulate(self, n_simulation = math.inf, depth_simulations = 500 ,save_h = False, verbose = True): # todo
    
        self.n_simulation = n_simulation
        self.depth_simulations = depth_simulations
        print("\n********************* Simulating LSTA* ***********************\n")
        
        if self.n_simulation == math.inf:
            convergence = not(self.forward(False,False))
            while(not(convergence)):
                convergence = not(self.forward(False,False))
        else:
            for i in range(self.n_simulation):
                self.forward(save_h, verbose)
                time.sleep(1)
        
    def execution(self):
        self.simulate()
        print("\n********************* Execution LSTA* ************************\n")
        self.forward(True,True)
        
            
def getLRTA(problem):
    types = {
        "BarrierEnv1": LRTAStarBarrierEnv,
        "BarrierEnv2":LRTAStarBarrierEnv,
        "8_puzzle": LRTAnPuzzle
        }
    return types[problem]
