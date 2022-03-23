
import numpy as np
from random import random

class PIDPoseControl(object):
    def __init__(self, config):
        self.Kp_rho = config.opt_config["PIDPoseControl"]["Kp_rho"]
        self.Kp_alpha = config.opt_config["PIDPoseControl"]["Kp_alpha"]
        self.Kp_beta = config.opt_config["PIDPoseControl"]["Kp_beta"]

        # save
        self.history_u = []

    def solve(self, curr_x, g_xs):
        """ calculate the optimal inputs

        Args:
            curr_x (numpy.ndarray): current state, shape(state_size, )
            g_xs (numpy.ndarrya): goal trajectory, shape(plan_len, state_size)
        Returns:
            opt_input (numpy.ndarray): optimal input, shape(input_size, )
        """
        x = curr_x[0]
        y = curr_x[1]
        theta = curr_x[2]
        x_goal, y_goal, theta_goal = g_xs
        
        x_diff = x_goal - x
        y_diff = y_goal - y

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi

        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha + self.Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        self.history_u.append(np.array([v, w]))
        return np.array([v, w])

    def __str__(self):
        return "PID"