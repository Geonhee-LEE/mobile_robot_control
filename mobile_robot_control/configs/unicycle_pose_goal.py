import numpy as np
from random import random

class UnicyclePoseGoalConfigModule():
    # parameters
    STATE_SIZE = 3
    INPUT_SIZE = 2
    
    EPISODE_NUM = 20
    DT = 0.01
    XY_TOLERANCE = 0.01

    # Input bounds
    INPUT_LOWER_BOUND = np.array([-100, -300])
    INPUT_UPPER_BOUND = np.array([100, 300])

    # Start, goal pose
    START_POSES = np.array([10, 10, 0.0]) # random
    #START_POSES = "random"
    GOAL_POSES = np.array([15, 10, 3.14])
    #GOAL_POSES = "random"

    def __init__(self):
        # opt configs
        self.opt_config = {
            "PIDPoseControl": {
                "Kp_rho": 9,
                "Kp_alpha": 15,
                "Kp_beta": -3
            },
        }
