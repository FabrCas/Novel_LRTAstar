import matplotlib.pyplot as mpl
from problems.envs import BarrierEnv1, BarrierEnv2
from algorithms import LRTAStar


if __name__ == "__main__":
    
    # choose the problem
    env = BarrierEnv2()
    # create graph
    env.create_env()
    # LRTA* algorithm definition
    lrta_star = LRTAStar(env, 0)
    
    # execute(simluation + optimal execution) or just run the algorithm with forward
    # lrta_star.forward(True)
    lrta_star.execution()
    
    
