import os
import math
import time
# from problems.envs import *

class LRTAStar():
    
    def __init__(self, env, state_goal, n_simulation = math.inf):
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
            
        
        
        
            
            
                
                
                

        
    
