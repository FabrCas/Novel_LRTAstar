from problems.envs import getEnv
from algorithms import getLRTA
from novel_algorithms import getNovelLRTA
import math


problems = ["BarrierEnv1","BarrierEnv2","Escape","8_puzzle"]
type_algorithms =["classical","novel"]

if __name__ == "__main__":
    
    # choose the problem
    problem = problems[2]
    type_algorithm = type_algorithms[1]
    env = getEnv(problem)(load_h = False)
    
    # create graph
    # env.selectHardState1()
    env.create_env()
    
    
    # LRTA* algorithm definition
     
    lrta_star = getLRTA(problem)(env) if type_algorithm=="classical" else getNovelLRTA(problem)(env)
    
    """ execute(simluation + optimal execution) or just run the algorithm with forward """
    # lrta_star.forward(save_h = False , verbose = True)
    # lrta_star.simulate(save_h = False , verbose = True)
    
    
    """ simulation + final forward """
    lrta_star.execution() 
    
    
    
    
    # time = 32.210394620895386 [s] | number of simulations = 112
    # time = 45.00187587738037 [s] | number of simulations = 110