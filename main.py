from problems.envs import getEnv
from algorithms import getLRTA

problems = ["BarrierEnv1","BarrierEnv2","8_puzzle"]

if __name__ == "__main__":
    
    # choose the problem
    problem = problems[2]
    env = getEnv(problem)(,load_h = True)
    
    # create graph
    env.create_env()
    
    # LRTA* algorithm definition
    lrta_star = getLRTA(problem)(env, depth_simulations = 1000)
    
    # execute(simluation + optimal execution) or just run the algorithm with forward
    
    lrta_star.forward(save_h = True , verbose = True)
    
    # lrta_star.simulate(n_simulation= 100,save_h = True , verbose = False)
    
    # lrta_star.execution()