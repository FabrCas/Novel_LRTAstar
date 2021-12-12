import matplotlib.pyplot as mpl
from problems.envs import barrierEnv1
from algorithms import LRTAStar


if __name__ == "__main__":
    
    # choose the problem
    env = barrierEnv1()
    # create graph
    env.create_env()
    # LRTA* algorithm definition
    lrta_star = LRTAStar(env, 0)
    # lrta_star.forward(True)
    lrta_star.execution()
    
    
