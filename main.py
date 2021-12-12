import os
import time
import matplotlib.pyplot as mpl
from envs.barrierEnv import *
from algorithms import LRTAStar 


if __name__ == "__main__":
    print("\n***************** creating the state space *******************\n")
    env = create_env()
    a =  [node for node in env if node.name==initial_state]
    a = a[0]
    print("\n********************** starting LSTA* ************************\n")
    lrta_star = LRTAStar(a, 0)
    lrta_star.forward(True)
    
    
