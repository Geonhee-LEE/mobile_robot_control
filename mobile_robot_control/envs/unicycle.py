import core

class UnicycleConstEnv(Env):
    """  Unicycle robot with constant goal Env
    """
    def __init__(self):
        """
        """
        self.config = {"state_size": 3,
                       "input_size": 2,
                       "dt": 0.01,
                       "max_step": 500,
                       "input_lower_bound": (-1.5, -3.14),
                       "input_upper_bound": (1.5, 3.14),
                       "car_size": 0.2,
                       "wheel_size": (0.075, 0.015)
                       }

        super(UnicycleConstEnv, self).__init__(self.config)


    def reset(self, init_x=None):
        """ reset state
        Returns:
            init_x (numpy.ndarray): initial state, shape(state_size, )  
            info (dict): information
        """
        self.step_count = 0

        noise = np.clip(np.random.randn(3), -0.1, 0.1)
        noise *= 0.1
        self.curr_x = np.zeros(self.config["state_size"]) + noise

        if init_x is not None:
            self.curr_x = init_x

        # goal
        self.g_x = np.array([2.5, 2.5, 0.])

        # clear memory
        self.history_x = []
        self.history_g_x = []

        return self.curr_x, {"goal_state": self.g_x}

    def step(self, u):
        """ step environments
        Args:
            u (numpy.ndarray) : input, shape(input_size, )
        Returns:
            next_x (numpy.ndarray): next state, shape(state_size, ) 
            cost (float): costs
            done (bool): end the simulation or not
            info (dict): information 
        """
        # clip action
        u = np.clip(u,
                    self.config["input_lower_bound"],
                    self.config["input_upper_bound"])

        # step
        next_x = step_two_wheeled_env(self.curr_x, u, self.config["dt"])

        # TODO: costs
        costs = 0.
        costs += 0.1 * np.sum(u**2)
        costs += np.sum(((self.curr_x - self.g_x)**2) * np.array([5., 5., 1.]))

        # save history
        self.history_x.append(next_x.flatten())
        self.history_g_x.append(self.g_x.flatten())

        # update
        self.curr_x = next_x.flatten()
        # update costs
        self.step_count += 1

        return next_x.flatten(), costs, \
            self.step_count > self.config["max_step"], \
            {"goal_state": self.g_x}
