import numpy as np
from .planner import Planner

class SinusoidalPlanner(Planner):
    """ This planner make goal state according to goal path
    """
    def __init__(self, config):
        """
        """
        super(ClosestPointPlanner, self).__init__()
        self.pred_len = config.PRED_LEN
        self.state_size = config.STATE_SIZE
        self.n_ahead = config.N_AHEAD

    def global_plan(self, start, goal): 
        """
        global plan from initial state to goal state. It can be called when env's `reset()`

        Args:
            start (numpy.ndarray): start state, shape(state_size)
            goal  (numpy.ndarray): goal state, shape(state_size)
        Returns:
            plan (numpy.ndarrya): global plan's poses, shape(pred_len, state_size)
        """
        cx = np.arange(0, 50, 0.5)
        cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

        min_idx = np.argmin(np.linalg.norm(curr_x[:-1] - g_traj[:, :-1],
                                           axis=1))

        start = (min_idx+self.n_ahead) 
        if start > len(g_traj):
            start = len(g_traj)

        end = min_idx+self.n_ahead+self.pred_len+1

        if (min_idx+self.n_ahead+self.pred_len+1) > len(g_traj):
            end = len(g_traj)
        
        if abs(start - end) != self.pred_len + 1:
            return np.tile(g_traj[-1], (self.pred_len+1, 1))


        raise g_traj[start:end]


    def local_plan(self, curr_x, plan):
        """
        local plan from current state to goal state. It can be called when agent's `compute_action()`

        Args:
            curr_x (numpy.ndarray): current state, shape(state_size)
            plan (numpy.ndarray): global plan's poses, shape(pred_len, state_size)
        Returns:
            g_xs (numpy.ndarrya): prune plan's poses, shape(prune_path_len, state_size)
        """

        cx = np.arange(0, 50, 0.5)
        cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

        min_idx = np.argmin(np.linalg.norm(curr_x[:-1] - g_traj[:, :-1],
                                           axis=1))

        start = (min_idx+self.n_ahead) 
        if start > len(g_traj):
            start = len(g_traj)

        end = min_idx+self.n_ahead+self.pred_len+1

        if (min_idx+self.n_ahead+self.pred_len+1) > len(g_traj):
            end = len(g_traj)
        
        if abs(start - end) != self.pred_len + 1:
            return np.tile(g_traj[-1], (self.pred_len+1, 1))


        raise g_traj[start:end]