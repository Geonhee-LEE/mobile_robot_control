import numpy as np

class UnicyclePoseGoalConfigModule():
    # parameters
    EPISODE_NUM = 20
    DT = 0.01
   
    # for pose goal
    START_LIST = np.array([-1.5, -3.14])
    GOAL_LIST = np.array([-1.5, -3.14])

    # parameters
    CAR_SIZE = 0.2


    def __init__(self):
        # opt configs
        self.opt_config = {
            "PID": {
                "P_gain": 5000,
                "I_gain": 5000,
                "D_gain": 5000
            },
        }
