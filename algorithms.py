import os
import math
import time
from problems.elements import NodeNpuzzle as Nodepuzzle
from problems.elements import StateEscape as StateEscape
from problems.elements import NodeEscape as NodeEscape
from problems.elements import printCorrect, printWrong
from copy import deepcopy, copy

class LRTAStarBarrierEnv():
    
    def __init__(self, env, state_goal = 0, n_simulation = math.inf, depth_simulations = 8):
        print("\n********************** Starting LSTA* ************************\n")
        self.start = [node for node in env.getNodes() if node.name==env.getInitialState()][0] #this represent the initial state for the algorithm
        self.goal = state_goal
        self.env = env
        self.plan = [] # a list of edges
        # self.h_updated = False
        self.n_simulation = n_simulation
        self.depth_simulations = depth_simulations
    
    # implement simulation and exectution separetely

    
    def forward(self, save_h = False, verbose = True, final_execution= False):
        h_updated = False # initialize the flag for updates to false
        iteration = 0 
        startTime = time.time()
        if not(self.plan == []): self.plan = []
        
        print("- Initial node: {} with state: {}\n- goal state {}".format(self.start.name, self.start.state, self.goal)) if verbose else 0
        total_cost_acc = 0
        actual_state = self.start
        
        while (iteration < self.depth_simulations) and not (actual_state.state == self.goal):
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
        if actual_state.state == self.goal:
            print("- The goal has been found in the state {}". format(actual_state.name)) 
            print("- Definitive cost for the plan {}".format(total_cost_acc)) 
            print("\n- Plan:") 
            for action in self.plan:
                print("--> move from {} to {}".format(action.getNode_a().name, action.getNode_b().name))
        else:
            print("- The goal has not been found") 
            
        print("\n- End execution, time: {} [s]".format( (time.time() -startTime) )) if verbose else 0
        print()
        
        if save_h: self.env.saveHeuristics()
        
        return h_updated or not(actual_state.state == self.goal)
        
    def simulate(self,save_h = False, verbose = False):
        print("\n********************* Simulating LSTA* ***********************\n")
        n_simulations = 1
        if self.n_simulation == math.inf:
            print("\n-----------[Simulation n°1]-----------\n")
            convergence = not(self.forward(save_h,verbose))
            while(not(convergence)):
                n_simulations += 1
                print("\n-----------[Simulation n°{}]-----------\n".format(n_simulations))
                convergence = not(self.forward(save_h,verbose))
        return n_simulations
        
    def execution(self):
        starting_time = time.time()
        n_simulations = self.simulate()
        print("\n********************* Executing LSTA* ************************\n")
        self.forward(False,True,True)
        duration_execution = time.time()-starting_time
        print("\n-----------[Execution: time and simulation iterations]-----------\n")
        print("time = {} [s] | number of simulations = {}".format(duration_execution, n_simulations))
        
        
class LRTAnPuzzle():       
    
    def __init__(self, env, n_simulation = math.inf, depth_simulations = 1000):
        print("\n********************** Starting LSTA* ************************\n")
        self.start = env.getStartingNode()
        if (self.start.get_n() == 9): self.goal = [[1,2,3],[4,5,6],[7,8,None]]
        else: self.goal = None 
        
        self.env = env
        self.plan = [] # a list of edges
        # self.h_updated = False
        self.n_simulation = n_simulation
        self.depth_simulations = depth_simulations
        

    def forward(self, save_h = False, verbose = True, final_execution= False):
        
        h_updated = False # initialize the flag for updates to false
        updates = {}
        goal_found = False
        iteration = 0 
        startTime = time.time()
        if not(self.plan == []): self.plan = []
    
        total_cost_acc = 0
        actual_state = self.start
        self.plan.append(actual_state)
        
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
                
            print()  if verbose else 0
            print("- Analyzing moves:") if verbose else 0
            next_node = None
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
                updates[str(actual_state.state)] = "from: " + str(actual_state.heuristic) + " to: " + str(lowest_cost_f)
                actual_state.heuristic = lowest_cost_f
                self.env.memorize_heuristics(actual_state,lowest_cost_f)
                if not(h_updated): h_updated = True
                
            else:
                print("- No update")if verbose else 0
                        
            # move to next state
            print("- Next state is:")if verbose else 0

            actual_state = next_node
            print(actual_state.getName())if verbose else 0
            self.plan.append(next_node)
            if actual_state.state == self.goal: goal_found = True
            
            iteration +=1  # in this case the total cost is equal to the number of iteration having the unitary cost 
        
        print("\n-----------[Final recap]-----------\n") if verbose else 0
        print("\n- End execution, time: {} [s]".format( (time.time() -startTime) )) if verbose else 0
        
        if h_updated:
            print("- The heuristic has been updated, number of updates = {}". format(len(updates)))
            if verbose:
                for k,v in updates.items():
                    print(k+"  ------>    "+v +"\n")
        else:
            print("- The heuristic hasn't been updated")
        
        if goal_found:
            print("- The goal has been found with the cost/a number of iterations equal to {}". format(iteration))
            if verbose or final_execution:
                if (len(self.plan)) > 35:
                    print("\n- The Plan is too long to be printed:")
                    
                else:
                    print("\n- Plan:") 
                    for node in self.plan:
                        print(node.getName())
        else:
            print("- The goal has not been found in {} iterations". format(iteration))
                
        print()if verbose else 0
        
        if save_h: self.env.saveHeuristics()
        return h_updated or not(goal_found)
        
    def simulate(self, n_simulation = math.inf ,save_h = False, verbose = False): # todo
    
        self.n_simulation = n_simulation
        # self.depth_simulations = depth_simulations
        
        print("\n********************* Simulating LSTA* ***********************\n")
        n_simulations = 1
        if self.n_simulation == math.inf:
            print("\n-----------[Simulation n°1]-----------\n")
            convergence = not(self.forward(save_h,verbose))
            while(not(convergence)):
                n_simulations += 1
                print("\n-----------[Simulation n°{}]-----------\n".format(n_simulations))
                convergence = not(self.forward(save_h,verbose))
        else:
            n_simulation = 0
            for i in range(self.n_simulation):
                n_simulations += 1
                print("\n-----------[Simulation n°{}]-----------\n".format(n_simulations))
                convergence = self.forward(save_h, verbose)
                if convergence:
                    break
                
                # time.sleep(1)
        return n_simulations
        
    def execution(self):
        starting_time = time.time()
        n_simulations = self.simulate()
        print("\n********************* Executing LSTA* ************************\n")
        self.forward(True,False,True)
        duration_execution = time.time()-starting_time
        print("\n-----------[Execution: time and simulation iterations]-----------\n")
        print("time = {} [s] | number of simulations = {}".format(duration_execution, n_simulations))


class LRTAEscape():
    
    def __init__(self, env, n_simulation = math.inf, depth_simulations = 1000):
        print("\n********************** Starting LSTA* ************************\n")
        self.start = deepcopy(env.getState()) #this represent the initial state for the algorithm
        self.env = env
        self.plan = [] # a list of edges
        # self.h_updated = False
        self.n_simulation = n_simulation
        self.depth_simulations = depth_simulations
    
    # implement simulation and exectution separetely
    
    
    def _isGoal(self, state):
        state_info = state.getState()
        
        pos = (state_info[0],state_info[1])
        has_rk = state_info[2]
        has_bk = state_info[3]
        
        return (has_rk and pos == self.env.red_door_pos) or (has_bk and pos == self.env.blue_door_pos)
        

    
    def forward(self, save_h = False, verbose = True, final_execution= False):
        h_updated = False # initialize the flag for updates to false
        iteration = 0 
        startTime = time.time()
        if not(self.plan == []): self.plan = []
        
        print("- Initial node: {} with state: {}\n".format(self.start.getNode().name, str(self.start.getState())) ) if verbose else 0
        total_cost_acc = 0
        actual_state = deepcopy(self.start)  # state object not a node
        
        while (self.depth_simulations > iteration) and not (self._isGoal(actual_state)):
        # for i in range(20):
            
            actual_node = actual_state.getNode()
            
            print("\n-----------[Iteration n°{}]-----------\n".format(iteration +1)) if verbose else 0
            # evaluate next state
            print("- Looking edges of state {}".format(actual_state.getNode().name)) if verbose else 0
            # find the lowest estimated cost
            next_node = None
            chosen_edge = None
            lowest_cost_f = math.inf # initialize the smallest cost with infinite
            lowest_cost_c = math.inf 
            for edge in actual_node.edges:
                other_node = edge.getNode_b()
                
                c_i = edge.cost
                
                # evaluate h(s)
                # create a new state
                if not actual_state.hasRedKey:
                    other_has_rk = (other_node.getPosx() == self.env.red_key_pos[0]) and (other_node.getPosy() == self.env.red_key_pos[1])
                else:
                    other_has_rk = True 
                if not actual_state.hasBlueKey:    
                    other_has_bk = (other_node.getPosx() == self.env.blue_key_pos[0]) and (other_node.getPosy() == self.env.blue_key_pos[1])
                else:
                    other_has_bk = True 
    
                other_state = StateEscape(other_node, other_has_rk, other_has_bk)
                h_i = self.env.compute_heuristics(other_state)
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
            actual_h = self.env.compute_heuristics(actual_state)
            if(lowest_cost_f > actual_h):
                print("- Updating heuristic of {}, from {} to {}".format(actual_state.getNode().name,actual_h,lowest_cost_f))if verbose else 0
                # actual_state.heuristic = lowest_cost_f
                self.env.memorize_heuristics(actual_state,lowest_cost_f)
                if not(h_updated): h_updated = True
            else:
                print("- No update")if verbose else 0
            
            # print(self.env.learned_heuristics)
            
                    
            # move to next state
            
            # check whether in the next state it's possible to find the keys
            if not actual_state.hasRedKey:
                next_has_rk = (next_node.getPosx() == self.env.red_key_pos[0]) and (next_node.getPosy() == self.env.red_key_pos[1])
                if next_has_rk:
                    print("-----------------------------------------------------> collected the red key!!!")if verbose else 0
                    actual_state.collectRedKey()
            
            if not actual_state.hasBlueKey:
                next_has_bk = (next_node.getPosx() == self.env.blue_key_pos[0]) and (next_node.getPosy() == self.env.blue_key_pos[1])
                if next_has_bk:
                    print("-----------------------------------------------------> collected the blue key!!!")if verbose else 0
                    actual_state.collectBluedKey()
                    

            actual_state.changePosition(next_node)
    
            tmp_info = actual_state.getState()
            print("- Next state is: Pos -> {}[x] {}[y], redKey -> {}, blueKey -> {}".format(tmp_info[0],tmp_info[1],tmp_info[2],tmp_info[3]) )if verbose else 0
            
            self.plan.append(chosen_edge)
            
            
            total_cost_acc += lowest_cost_c
            iteration+=1 
            print("- Actual cost for the plan {}".format(total_cost_acc)) if verbose else 0
        
        print("\n-----------[Final recap forward]-----------\n") if verbose else 0
        
        if h_updated:
            print("- The heuristic has been updated")
        else:
            print("- The heuristic has been updated")
            
         
        if self._isGoal(actual_state):
            print("- The goal has been found in the state {}". format(actual_state.getNode().name)) 
            print("- Definitive cost for the plan {}".format(total_cost_acc)) 
            print("- Plan found with {} moves". format(len(self.plan)))
            print("\n- Plan:")  if verbose else 0
            if verbose:
                for action in self.plan:
                    print("--> move from {} to {}".format(action.getNode_a().name, action.getNode_b().name))
                    
        else: print("- Plan not found with {} moves". format(len(self.plan)))
        
        print("\n- End forward, time: {} [s]".format( (time.time() -startTime) )) if verbose else 0
        print()
        
        if save_h: self.env.saveHeuristics()
          
        return h_updated or not(self._isGoal(actual_state))
        
    def simulate(self,save_h = False, verbose = False):
        print("\n********************* Simulating LSTA* ***********************\n")
        n_simulations = 1
        if self.n_simulation == math.inf:
            print("\n-----------[Simulation n°1]-----------\n")
            convergence = not(self.forward(save_h,verbose))
            while(not(convergence)):
                n_simulations += 1 
                print("\n-----------[Simulation n°{}]-----------\n".format(n_simulations))
                convergence = not(self.forward(save_h,verbose))
                
        return n_simulations
        
    def execution(self):
        starting_time = time.time()
        n_simulations= self.simulate()
        print("\n********************* Executing LSTA* ************************\n")
        self.forward(False,True,True)
        duration_execution = time.time()-starting_time
        print("\n-----------[Execution: time and simulation iterations]-----------\n")
        print("time = {} [s] | number of simulations = {}".format(duration_execution, n_simulations))
        
            
def getLRTA(problem):
    types = {
        "BarrierEnv1": LRTAStarBarrierEnv,
        "BarrierEnv2":LRTAStarBarrierEnv,
        "8_puzzle": LRTAnPuzzle,
        "Escape": LRTAEscape
        }
    return types[problem]
