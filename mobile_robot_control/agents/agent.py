import numpy as np
from numpy.linalg import norm

class Agent(object):
    def __init__(self, config, planner, model, controller):
        """
        Base class for agent. Have the physical attributes of an agent.
        """
        self.planner = planner
        self.controller = controller
        self.model = model

        self.px = None
        self.py = None
        self.theta = None
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        self.vx = None
        self.vy = None
        self.time_step = 0.01
        self.xy_tolerance = 0.1 # whether to reach goal within xy_tolerance
        
    def set_pose(self, px, py, theta):
        self.px = px
        self.py = py
        self.theta = theta

    def set_position(self, position):
        self.px = position[0]
        self.py = position[1]

    def set_goal_pose(self, goal_x, goal_y, goal_theta):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_theta = goal_theta

    def set_velocity(self, velocity):
        self.vx = velocity[0]
        self.vy = velocity[1]

    def get_pose(self):
        return self.px, self.py, self.theta

    def get_position(self):
        return self.px, self.py

    def get_goal_pose(self):
        return self.goal_x, self.goal_y, self.goal_theta

    def get_goal_position(self):
        return self.goal_x, self.goal_y

    def get_velocity(self):
        return self.vx, self.vy

    def compute_action(self, observation):
        """
        Compute state using received observation and pass it to policy

        """
        if self.controller is None:
            raise AttributeError('controller attribute has to be set!')

        # plan
        g_xs = []
        if self.planner is not None:
            g_xs = self.planner.plan(observation, self.get_goal_pose())
        else:
            g_xs = self.get_goal_pose()

        # obtain sol
        action = self.controller.calculate_command(observation, g_xs)

        return action

    def predict_model(self, action, time_step):
        """
        Compute state using received observation and pass it to policy

        """
        if self.model.model_type == "HolonomicModel":
            curr_x =  np.array([self.px, self.py]) 
            u =  np.array([action[0], action[1]]) 
            next_x = self.model.predict_next_state(curr_x, u, time_step)
        elif self.model.model_type == "UnicycleKinematicModel":
            curr_x =  np.array([self.px, self.py, self.theta]) 
            u =  np.array([action[0], action[1]]) 
            next_x = self.model.predict_next_state(curr_x, u, time_step)

        return next_x

    def step(self, action):
        """
        Perform an action and update the state
        """
        #self.check_validity(action)
        next_pos = self.predict_model(action, self.time_step)
        self.px, self.py, self.theta = next_pos

        if self.model.model_type == "HolonomicModel":
            self.vx = action[0]
            self.vy = action[1]
        elif self.model.model_type == "UnicycleKinematicModel":
            self.theta = self.theta % (2 * np.pi)
            self.vx = action[0] * np.cos(self.theta)
            self.vy = action[0] * np.sin(self.theta)

    def goal_reached(self):
        return norm(np.array(self.get_position()) - np.array(self.get_goal_position())) < self.xy_tolerance

