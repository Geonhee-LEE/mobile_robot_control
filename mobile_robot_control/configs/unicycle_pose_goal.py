import numpy as np

class UnicyclePoseGoalConfigModule():
    # parameters
    EPISODE_NUM = 20
    DT = 0.01
    XY_TOLERANCE = 0.01

    # Input bounds
    INPUT_LOWER_BOUND = np.array([-1.5, -3.14])
    INPUT_UPPER_BOUND = np.array([0.1, 3.14])

    def __init__(self):
        # opt configs
        self.opt_config = {
            "PID": {
                "P_gain": 5000,
                "I_gain": 5000,
                "D_gain": 5000
            },
        }
