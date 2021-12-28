from problems.envs import getEnv
from algorithms import getLRTA
import math


problems = ["BarrierEnv1","BarrierEnv2","8_puzzle", "Escape"]

if __name__ == "__main__":
    
    # choose the problem
    problem = problems[3]
    env = getEnv(problem)(load_h = True)
    
    # create graph
    # env.selectHardState1()
    env.create_env()

    
    # LRTA* algorithm definition
    # lrta_star = getLRTA(problem)(env, depth_simulations = 1000)
    
    """ execute(simluation + optimal execution) or just run the algorithm with forward """
    # lrta_star.forward(save_h = True , verbose = False)
    # lrta_star.simulate(n_simulation= math.inf,save_h = True , verbose = False)
    
    
    """ simulation + final forward """
    # lrta_star.execution() 