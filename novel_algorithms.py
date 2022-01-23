import math
import time
from problems.elements import NodeNpuzzle as Nodepuzzle
from problems.elements import StateEscape as StateEscape
from problems.elements import NodeEscape as NodeEscape
from problems.elements import printCorrect, printWrong
from copy import deepcopy
import scipy.stats
import random

# random.seed(time.time())
random.seed(22)

""" novelties: 
- disambiguity on f(n) method 
- set h(n) to inf for unreversable dead ends
- dynamic limit of the depth for the search
- restarting based on Gaussian probability distribution
- restarting based on % increment of h(s)
"""

def _getMagnitudeNUmber(n):
    
    def recursive_step(n,order):
        if n >= 0 and n<= 9:
            return order
        else:
            return recursive_step(n/10, order+1)
        
    n = abs(n)
    return recursive_step(n,0)

# disambiguity with tollerance, look appendix report
use_variant_disambiguity = True

class NovelLRTAStarBarrierEnv():
    
    def __init__(self, env, state_goal = 0, n_simulation = math.inf, depth_simulations = 8 ,novelties = []):
        print("\n********************** Starting LSTA* ************************\n")
        self.start = [node for node in env.getNodes() if node.name==env.getInitialState()][0] #this represent the initial state for the algorithm
        self.goal = state_goal
        self.env = env
        self.plan = [] # a list of edges
        # self.h_updated = False
        self.n_simulation = n_simulation
        self.depth_simulations = depth_simulations
        self.novelties = novelties
        self.depth_cap = self.depth_simulations
        if "dynamic_depth_limit" in novelties:  self.depth_scale = 2
        
        if use_variant_disambiguity:
            self.tollerance = 0.4 # value that goes from 0 to 1 and indicates the percentage of tollerence in the selection of states with minimal costs
            self.states_fn = []
           
           
    
    """
    2 simple aspects to decide where move at parity of f(s)
    - the only next state of a possible next state is the current state? avoid it is a dead end (reversable in this case, but still a useless move)
    - at parity of f(s), make a step forward to look at the minimum f(s), the choice is prefer the one with has a min one ahead f(s)
    """
    
    def _minFs(self, state):
        f_min = math.inf
        for edge in state.edges:
            other_node = edge.getNode_b()
            
            c_i = edge.cost
            h_i = other_node.heuristic 
            f_i = c_i + h_i # estimated cost
            
            if(f_i < f_min):
                f_min = f_i
        return f_i
        
    
    def _oneStepCheck(self, actualState, nextStates):
        cost = math.inf
        chosen_state = None
        for nextState in nextStates:
            #  choice based in c(s,s')
            # for edge in actualState.edges:
            #     other_node = edge.getNode_b()
            #     if other_node is nextState:
            #         if edge.cost < cost:
            #             cost = edge.cost
            #             chosen_state = nextState
            
            #
            # print(self._minFs(nextState))
            
            if(self._minFs(nextState) < cost):
                cost = self._minFs(nextState)
                chosen_state = nextState
        return chosen_state
    

        
    def forward(self, save_h = False, verbose = True, final_execution= False):
        
        if final_execution: self.depth_cap = self.depth_simulations
       
        
        h_updated = False # initialize the flag for updates to false
        iteration = 0 
        startTime = time.time()
        if not(self.plan == []): self.plan = []
        
        print("- Initial node: {} with state: {}\n- goal state {}".format(self.start.name, self.start.state, self.goal)) if verbose else 0
        total_cost_acc = 0
        actual_state = self.start
        
        while (iteration < self.depth_cap) and not (actual_state.state == self.goal):
        
            if "gaussian_restart" in self.novelties and not(final_execution):
                scale = 10**_getMagnitudeNUmber(self.depth_simulations)
                mean = int( ((self.depth_simulations/2)/scale) )
                prob = round(scipy.stats.norm(mean,1).pdf(iteration/scale),5) 
                rnd_n = round(random.random(),5)
                # print("g_prob {}, rnd_n {}".format(prob,rnd_n))
                if rnd_n <= prob:
                    print("exiting for gaussian restart")
                    break
                    
            print("\n-----------[Iteration n°{}]-----------\n".format(iteration +1)) if verbose else 0
            # evaluate next state
            print("- Looking edges of state {}".format(actual_state.name)) if verbose else 0
            # find the lowest estimated cost
            next_node = None
            chosen_edge = None
            lowest_cost_f = math.inf # initialize the smallest cost with infinite
            lowest_cost_c = math.inf
            if "f(n)_disambiguity" in self.novelties: next_possible_states = [] # more states can have same f(n) here is introduced a policy to choose 
            if use_variant_disambiguity: self.states_fn = []  
            for edge in actual_state.edges:
                other_node = edge.getNode_b()
                
                
                
                c_i = edge.cost
                h_i = other_node.heuristic 
                f_i = c_i + h_i # estimated cost
                print("- Evaluating motion to {}, estimated cost f= c(s,s') + h(s') = {} + {} = {}".format(other_node.name, c_i,h_i,f_i)) if verbose else 0
                
                if "f(n)_disambiguity" in self.novelties:
                    if not(use_variant_disambiguity):
                        if(f_i < lowest_cost_f):
                            lowest_cost_f = f_i
                            next_possible_states = [other_node]
                        elif(f_i == lowest_cost_f):
                            next_possible_states.append(other_node)
                    else:
                        # 2 ways of acting, using len(next_possible_states) == 0 or at priori using a certain tollerance
                        self.states_fn.append({"state": other_node,"value_f":f_i})
                        if(f_i < lowest_cost_f):
                            lowest_cost_f = f_i
                            next_possible_states = [other_node]
                        elif(f_i == lowest_cost_f):
                            next_possible_states.append(other_node)
                else:
                    if(f_i < lowest_cost_f):
                        lowest_cost_f = f_i
                        lowest_cost_c = c_i
                        next_node = other_node
                        chosen_edge = edge
                

            if "f(n)_disambiguity" in self.novelties: 
                if use_variant_disambiguity and len(next_possible_states) == 1:    #it's possible to have another variant removing the second conditions
                    print("using variant on disambiguity ....\n") if verbose else 0
                    # print("states_fn -> {}".format(len(self.states_fn)))
                    next_possible_states = []
                    
                    for elem in self.states_fn:
                        state = elem["state"]
                        v = elem["value_f"]
                        val_rel = (v/lowest_cost_f)-1  #how much bigger in percentage? 
                        # print("v -> {}".format(v))
                        # print("lowest_cost_f ->  {}".format( lowest_cost_f))
                        # print("val_rel ->  {}".format( val_rel))
                        
                        if val_rel <= self.tollerance:
                            next_possible_states.append(state)
                            
                    print("number of possible states {}".format(len(next_possible_states)))if verbose else 0
                
                # what choose?
                if not len(next_possible_states) == 1: # more choices 
                    next_node = self._oneStepCheck(actual_state, next_possible_states)
                else: 
                    next_node = next_possible_states[0]
                
                # get info for the plan and final cost
                for edge in actual_state.edges:
                    other_node = edge.getNode_b()
                    if other_node is next_node:
                        chosen_edge =  edge
                        break
            
                lowest_cost_c = chosen_edge.cost
            
                # unreversable dead end check
                if len(next_possible_states)==0:    
                    print("found a dead end")
                    actual_state.heuristic = math.inf 
                    self.env.memorize_heuristics(actual_state,math.inf)
                    if not(h_updated): h_updated = True
                    continue 
            
            
            print("- All edges checked")if verbose else 0
            
            # we need to update?
            print("- Evaluating the updating of the heuristics... ")if verbose else 0
            if(lowest_cost_f > actual_state.heuristic):
                print("- Updating heuristic of {}, from {} to {}".format(actual_state.name,actual_state.heuristic,lowest_cost_f))if verbose else 0
                
                old_h = actual_state.heuristic
                actual_state.heuristic = lowest_cost_f
                self.env.memorize_heuristics(actual_state,lowest_cost_f)
                if not(h_updated): h_updated = True
                
                if "increment_restart" in self.novelties:
                    delta = lowest_cost_f - old_h  
                    delta_percentage = round((delta/old_h)*100,4)
                    if delta_percentage>= 75:
                        print("found an update with {}% increase, restarting...".format(delta_percentage))
                        break
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
            if "dynamic_depth_limit" in self.novelties and not(final_execution):
                self.depth_cap = int(math.ceil(iteration/self.depth_scale))
                print("depth limited for next execution at {}".format(self.depth_cap))
        else:
            if "dynamic_depth_limit" in self.novelties: self.depth_cap = self.depth_simulations
            print("- The goal has not been found") 
            
        print("\n- End execution, time: {} [s]".format( (time.time() -startTime) )) if verbose else 0
        print()
        
        if save_h: self.env.saveHeuristics()
        
        # if "dynamic_depth_limit" in self.novelties:
        #     return h_updated or not(actual_state.state == self.goal)
        # else:
        return h_updated or not(actual_state.state == self.goal) # criterion for convergence, False -> convergence, True -> divergence
        
    def simulate(self, save_h = False, verbose = False):
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

       
class NovelLRTAnPuzzle():       
    
    def __init__(self, env, n_simulation = math.inf, depth_simulations = 1000, novelties = []):
        print("\n********************** Starting LSTA* ************************\n")
        self.start = env.getStartingNode()
        if (self.start.get_n() == 9): self.goal = [[1,2,3],[4,5,6],[7,8,None]]
        else: self.goal = None 
        
        self.novelties = novelties
        self.env = env
        self.plan = [] # a list of edges
        # self.h_updated = False
        self.n_simulation = n_simulation
        self.depth_simulations = depth_simulations
        self.depth_cap = self.depth_simulations
        if "dynamic_depth_limit" in novelties:  self.depth_scale = 2
        
        if use_variant_disambiguity:
            self.tollerance = 0.1
            self.states_fn = []
        
    def _minFs(self, state):
        f_min = math.inf
        for k,v in state.getPossibleMoves().items():

            posFree = state.getPositionFreeTile()
            new_pos_free = (posFree[0] + v[0], posFree[1] + v[1])
            value_tile_exchanged = state.state[new_pos_free[0]][new_pos_free[1]]
            new_state = deepcopy(state.state)
            
            # print("moving free tail in the place of {}".format(value_tile_exchanged))
            
            new_state[new_pos_free[0]][new_pos_free[1]] = None
            new_state[posFree[0]][posFree[1]] = value_tile_exchanged
            
            node = Nodepuzzle(new_state)
           
            # print(node.getName())
            self.env.compute_heuristics(node)
            f_i = node.getHeuristic()  + 1 # 1 = c(s,s')
            
            if(f_i < f_min):
                f_min = f_i
        return f_i
        
    
    def _oneStepCheck(self, nextStates):
        cost = math.inf
        chosen_state = None
        # [print(state.name) for state in nextStates]
        for nextState in nextStates:
            #  choice based in c(s,s')
            # for edge in actualState.edges:
            #     other_node = edge.getNode_b()
            #     if other_node is nextState:
            #         if edge.cost < cost:
            #             cost = edge.cost
            #             chosen_state = nextState
            
            #
            # print(self._minFs(nextState))
            
            if(self._minFs(nextState) < cost):
                cost = self._minFs(nextState)
                chosen_state = nextState
        return chosen_state
        

    def forward(self, save_h = False, verbose = True, final_execution=False):
        
        if final_execution: self.depth_cap = self.depth_simulations
        
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
        while (iteration < self.depth_cap) and (not (actual_state.state == self.goal)):
            
            
            if "gaussian_restart" in self.novelties and not(final_execution):
                
                # prob = round(scipy.stats.norm((self.depth_cap/(100)),1).pdf(iteration/100),5)
                scale = 10**(_getMagnitudeNUmber(self.depth_simulations)-1)
                mean = int( (self.depth_simulations/2)/scale ) 
                prob = round(scipy.stats.norm(mean,1).pdf(iteration/scale),5) 
                rnd_n = round(random.random(),5)
                # print("g_prob {}, rnd_n {}".format(prob,rnd_n))
                if rnd_n <= prob:
                    print("exiting for gaussian restart")
                    # print(" g_prob {}, rnd_n {}".format(prob,rnd_n))
                    break
                
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
            
            if "f(n)_disambiguity" in self.novelties: next_possible_states = []
            if use_variant_disambiguity: self.states_fn = [] 
            
            for idx,node in enumerate(actual_state.getAdjacentNodes()):
                c_i = 1 # unitary cost for each move
                h_i = node.getHeuristic() 
                f_i = c_i + h_i # estimated cost
                print("--> State n°{} with h(s) = {} & f(s) = {}". format(idx, h_i,f_i))if verbose else 0
                print(node.getName())if verbose else 0
                

                
                if "f(n)_disambiguity" in self.novelties: 
                    if not(use_variant_disambiguity):
                        if(f_i < lowest_cost_f):
                            lowest_cost_f = f_i
                            next_possible_states = [node]
                        elif(f_i == lowest_cost_f):
                            next_possible_states.append(node)
                    else:     
                        self.states_fn.append({"state": node,"value_f":f_i})
                        if(f_i < lowest_cost_f):
                            lowest_cost_f = f_i
                            next_possible_states = [node]
                        elif(f_i == lowest_cost_f):
                            next_possible_states.append(node)
                else:
                    if(f_i < lowest_cost_f):
                       lowest_cost_f = f_i
                       next_node = node
            
            if "f(n)_disambiguity" in self.novelties:
                
                
                if use_variant_disambiguity and len(next_possible_states) == 1:    #it's possible to have another variant removing the second conditions
                   print("using variant on disambiguity ....\n") if verbose else 0
                   # print("states_fn -> {}".format(len(self.states_fn)))
                   next_possible_states = []
                   
                   for elem in self.states_fn:
                       state = elem["state"]
                       v = elem["value_f"]
                       val_rel = (v/lowest_cost_f)-1  #how much bigger in percentage? 
                       # print("v -> {}".format(v))
                       # print("lowest_cost_f ->  {}".format( lowest_cost_f))
                       # print("val_rel ->  {}".format( val_rel)) if verbose else 0
                       
                       if val_rel <= self.tollerance:
                           next_possible_states.append(state)
                        
                   print("number of possible states {}".format(len(next_possible_states)))if verbose else 0
                           
                if not len(next_possible_states) == 1: # more choices 
                    next_node = self._oneStepCheck(next_possible_states)
                else: 
                    next_node = next_possible_states[0]
                    
                # unreversable dead end check (that's not the case obv)
                if len(next_possible_states)==0:    
                    print("found a dead end")
                    actual_state.heuristic = math.inf 
                    self.env.memorize_heuristics(actual_state,math.inf)
                    if not(h_updated): h_updated = True
                    continue 
            
            
            print("- All states checked")if verbose else 0
    
            # we need to update?
            print("- Evaluating the updating of the heuristics... ")if verbose else 0
            if(lowest_cost_f > actual_state.heuristic):
                print("- Updating heuristic from {} to {}, for the actual state".format(actual_state.heuristic,lowest_cost_f))if verbose else 0
                
                old_h = actual_state.heuristic
                updates[str(actual_state.state)] = "from: " + str(actual_state.heuristic) + " to: " + str(lowest_cost_f)
                actual_state.heuristic = lowest_cost_f
                self.env.memorize_heuristics(actual_state,lowest_cost_f)
                if not(h_updated): h_updated = True
                
                if "increment_restart" in self.novelties:
                    delta = lowest_cost_f - old_h  
                    delta_percentage = round((delta/old_h)*100,4)
                    if delta_percentage>= 75:
                        print("found an update with {}% increase, restarting...".format(delta_percentage))
                        break
                
                
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
                        
            if "dynamic_depth_limit" in self.novelties and not(final_execution):
                self.depth_cap = int(math.ceil(iteration/self.depth_scale))
                print("depth limited for next execution at {}".format(self.depth_cap))
        else:
            if "dynamic_depth_limit" in self.novelties: self.depth_cap = self.depth_simulations
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
                
        return n_simulations
        
    def execution(self):
        starting_time = time.time()
        n_simulations = self.simulate()
        print("\n********************* Executing LSTA* ************************\n")
        self.forward(False,False,True)
        duration_execution = time.time()-starting_time
        print("\n-----------[Execution: time and simulation iterations]-----------\n")
        print("time = {} [s] | number of simulations = {}".format(duration_execution, n_simulations))


class NovelLRTAEscape():
    
    def __init__(self, env, n_simulation = math.inf, depth_simulations = 100, novelties = []):
        print("\n********************** Starting LSTA* ************************\n")
        self.start = deepcopy(env.getState()) #this represent the initial state for the algorithm
        self.env = env
        self.plan = [] # a list of edges
        self.novelties = novelties
        # self.h_updated = False
        self.n_simulation = n_simulation
        self.depth_simulations = depth_simulations
        self.depth_cap = self.depth_simulations
        if "dynamic_depth_limit" in novelties:  self.depth_scale = 2
        
        if use_variant_disambiguity:
            self.tollerance = 0.05
            self.states_fn = []
    
    def _minFs(self, state):
        f_min = math.inf
        node = state.getNode()
        
        for edge in node.edges:
            other_node = edge.getNode_b()
            
            c_i = edge.cost
            
            if not state.hasRedKey:
                other_has_rk = (other_node.getPosx() == self.env.red_key_pos[0]) and (other_node.getPosy() == self.env.red_key_pos[1])
            else:
                other_has_rk = True 
            if not state.hasBlueKey:    
                other_has_bk = (other_node.getPosx() == self.env.blue_key_pos[0]) and (other_node.getPosy() == self.env.blue_key_pos[1])
            else:
                other_has_bk = True 
    
            other_state = StateEscape(other_node, other_has_rk, other_has_bk)
            h_i = self.env.compute_heuristics(other_state)
            f_i = c_i + h_i # estimated cost
          
            if(f_i < f_min):
                f_min = f_i
        return f_i
        
    
    def _oneStepCheck(self, nextStates):
        cost = math.inf
        chosen_state = None
        
        for nextState in nextStates:
            
            if(self._minFs(nextState) < cost):
                cost = self._minFs(nextState)
                chosen_state = nextState
                
        return chosen_state.getNode()
    
    
    def _isGoal(self, state):
        state_info = state.getState()
        
        pos = (state_info[0],state_info[1])
        has_rk = state_info[2]
        has_bk = state_info[3]
        
        return (has_rk and pos == self.env.red_door_pos) or (has_bk and pos == self.env.blue_door_pos)
        

    
    def forward(self, save_h = False, verbose = True, final_execution= False):
        
        if final_execution: self.depth_cap = self.depth_simulations
        h_updated = False # initialize the flag for updates to false
        iteration = 0 
        startTime = time.time()
        if not(self.plan == []): self.plan = []
        
        print("- Initial node: {} with state: {}\n".format(self.start.getNode().name, str(self.start.getState())) ) if verbose else 0
        total_cost_acc = 0
        actual_state = deepcopy(self.start)  # state object not a node
        
        while (iteration < self.depth_cap) and not (self._isGoal(actual_state)):
        # for i in range(20):
            
            actual_node = actual_state.getNode()
            
            if "gaussian_restart" in self.novelties and not(final_execution):
                
                # prob = 0.1
                scale = 10**(_getMagnitudeNUmber(self.depth_simulations)-1)
                mean = int( (self.depth_simulations/2)/scale ) 
                prob = round(scipy.stats.norm(mean,1).pdf(iteration/scale),5) 
                rnd_n = round(random.random(),5)

                if rnd_n <= prob:
                    print("exiting for gaussian restart")
                    # print(" g_prob {}, rnd_n {}".format(prob,rnd_n))
                    break
            
            print("\n-----------[Iteration n°{}]-----------\n".format(iteration +1)) if verbose else 0
            # evaluate next state
            print("- Looking edges of state {}".format(actual_state.getNode().name)) if verbose else 0
            # find the lowest estimated cost
            next_node = None
            chosen_edge = None
            lowest_cost_f = math.inf # initialize the smallest cost with infinite
            lowest_cost_c = math.inf
            
            if "f(n)_disambiguity" in self.novelties: next_possible_states = []
            if use_variant_disambiguity: self.states_fn = []
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
                
                if "f(n)_disambiguity" in self.novelties:
                    if not(use_variant_disambiguity):
                        if(f_i < lowest_cost_f):
                            lowest_cost_f = f_i
                            lowest_cost_c = c_i
                            next_possible_states = [other_state]
                        elif(f_i == lowest_cost_f):
                           next_possible_states.append(other_state)
                    else:
                        self.states_fn.append({"state": other_state,"value_f":f_i})
                        if(f_i < lowest_cost_f):
                            lowest_cost_f = f_i
                            lowest_cost_c = c_i
                            next_possible_states = [other_state]
                        elif(f_i == lowest_cost_f):
                           next_possible_states.append(other_state)
                       
                else:
                    if(f_i < lowest_cost_f):
                       lowest_cost_f = f_i
                       lowest_cost_c = c_i
                       next_node = other_node
                       chosen_edge = edge
                    
            if "f(n)_disambiguity" in self.novelties:
                
                if use_variant_disambiguity and len(next_possible_states) == 1:    #it's possible to have another variant removing the second conditions
                    print("using variant on disambiguity ....\n")  
                    # print("states_fn -> {}".format(len(self.states_fn)))
                    next_possible_states = []
                    
                    for elem in self.states_fn:
                        state = elem["state"]
                        v = elem["value_f"]
                        val_rel = (v/lowest_cost_f)-1  #how much bigger in percentage? 
                        # print("v -> {}".format(v))
                        # print("lowest_cost_f ->  {}".format( lowest_cost_f))
                        # print("val_rel ->  {}".format( val_rel)) 
                        
                        if val_rel <= self.tollerance:
                            next_possible_states.append(state)
                            
                    print("number of possible states {}".format(len(next_possible_states))) if verbose else 0
                
                # what choose?
                if not len(next_possible_states) == 1: # more choices 
                    next_node = self._oneStepCheck(next_possible_states)
                else: 
                    next_node = next_possible_states[0].getNode()
    
                # unreversable dead end check (not the case)
                if len(next_possible_states)==0:    
                    print("found a dead end")
                    self.env.memorize_heuristics(actual_state,math.inf)
                    if not(h_updated): h_updated = True
                    continue
                
                # get info for the plan and final cost
                for edge in actual_node.edges:
                    other_node = edge.getNode_b()
                    if other_node is next_node:
                        chosen_edge =  edge
                        break
               
                lowest_cost_c = chosen_edge.cost
            
            
                
            print("- All edges checked")if verbose else 0
            
            # we need to update?
            print("- Evaluating the updating of the heuristics... ")if verbose else 0
            actual_h = self.env.compute_heuristics(actual_state)
            if(lowest_cost_f > actual_h):
                print("- Updating heuristic of {}, from {} to {}".format(actual_state.getNode().name,actual_h,lowest_cost_f))if verbose else 0
                # actual_state.heuristic = lowest_cost_f
                self.env.memorize_heuristics(actual_state,lowest_cost_f)
                if not(h_updated): h_updated = True
                
                if "increment_restart" in self.novelties:
                    delta = lowest_cost_f - actual_h  
                    delta_percentage = round((delta/actual_h)*100,4)
                    if delta_percentage>= 75:
                        print("found an update with {}% increase, restarting...".format(delta_percentage))
                        break
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
                    
            if "dynamic_depth_limit" in self.novelties and not(final_execution):
                self.depth_cap = int(math.ceil(iteration/self.depth_scale))
                print("depth limited for next execution at {}".format(self.depth_cap))
                    
        else:
            print("- Plan not found with {} moves". format(len(self.plan)))
            if "dynamic_depth_limit" in self.novelties: self.depth_cap = self.depth_simulations
        
        print("\n- End forward, time: {} [s]".format( (time.time() -startTime) )) if verbose else 0
        print()
        
        if save_h: self.env.saveHeuristics()

        return h_updated or not(self._isGoal(actual_state))
        
    def simulate(self, save_h = False, verbose = False):
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
        
            

def getNovelLRTA(problem):
    types = {
        "BarrierEnv1": NovelLRTAStarBarrierEnv,
        "BarrierEnv2":NovelLRTAStarBarrierEnv,
        "8_puzzle": NovelLRTAnPuzzle,
        "Escape": NovelLRTAEscape
        }
    return types[problem]