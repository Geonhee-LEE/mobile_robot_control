import numpy as np

class Planner():
    """
    """
    def __init__(self):
        """
        """
        pass

    def global_plan(self, start, goal):
        """
        global plan from initial state to goal state. It can be called when env's `reset()`

        Args:
            start (numpy.ndarray): start state, shape(state_size)
            goal  (numpy.ndarray): goal state, shape(state_size)
        Returns:
            plan (numpy.ndarrya): global plan's poses, shape(pred_len, state_size)
        """
        raise NotImplementedError("Implement plan func")

    def local_plan(self, curr_x, plan):
        """
        local plan from current state to goal state by pruning. It can be called when agent's `compute_action()`

        Args:
            curr_x (numpy.ndarray): current state, shape(state_size)
            plan (numpy.ndarray): global plan's poses, shape(pred_len, state_size)
        Returns:
            g_xs (numpy.ndarrya): prune plan's poses, shape(prune_path_len, state_size)
        """
        raise NotImplementedError("Implement plan func")