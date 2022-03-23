import numpy as np
from random import random

class BicyclePathGoalConfigModule():
    # parameters
    STATE_SIZE = 4
    INPUT_SIZE = 2
    
    EPISODE_NUM = 5
    DT = 0.01
    XY_TOLERANCE = 0.01

    # Input bounds
    INPUT_LOWER_BOUND = np.array([-10, -1.57])
    INPUT_UPPER_BOUND = np.array([10, 1.57])

    # Start, goal pose
    #START_POSES =  np.array([[10, 10, 0], [10, 10, 0], [10, 10, 0], [10, 10, 0], [10, 10, 0]], np.float64) # multiple start pose, size should be same with `EPISODE_NUM`
    #START_POSES =  np.array([10, 10, 0.0]) # single start pose
    START_POSES = "random"
    #GOAL_POSES = np.array([[15, 10, 0], [10, 15, 0], [5, 10, 0], [10, 5, 0], [10, 10, 3.14]], np.float64) # multiple goal pose, size should be same with `EPISODE_NUM`
    #GOAL_POSES = np.array([15, 10, 0.0])  # single goal pose
    GOAL_POSES = "random"

    START_VELOCITY = 0

    INITIAL_STATE = np.ndarray(shape=(STATE_SIZE,), dtype = float)

    def __init__(self):
        # opt configs
        self.opt_config = {
            "PurePursuit": {
                "Kp_rho": 9,
                "Kp_alpha": 15,
                "Kp_beta": -3
            },
            "Stanley": {
                "Kp_rho": 9,
                "Kp_alpha": 15,
                "Kp_beta": -3
            },
            "LQR": {
                "Kp_rho": 9,
                "Kp_alpha": 15,
                "Kp_beta": -3
            },
        }
