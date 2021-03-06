import numpy as np
from numpy.linalg import norm

class Agent(object):
    def __init__(self, config, planner, model, controller):
        """
        Base class for agent. Have the physical attributes of an agent.
        """
        self.config = config
        self.planner = planner
        self.controller = controller
        self.model = model

        self.observation = np.ndarray(shape=(config.STATE_SIZE,), dtype = float)
        
        self.px = None
        self.py = None
        self.theta = None
        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None
        self.vx = None
        self.vy = None
        self.w = None
        self.global_path = None
        self.time_step = config.DT
        self.xy_tolerance = config.XY_TOLERANCE # whether to reach goal within xy_tolerance
    
    def set(self, x_start, y_start, theta_start, x_goal, y_goal, theta_goal, initial_vx, initial_vy):
        self.set_pose(x_start, y_start, theta_start)
        self.set_goal_pose(x_goal, y_goal, theta_goal)
        self.set_velocity(np.array([initial_vx, initial_vy]))

        if self.model.model_type == "HolonomicModel":
            self.observation[0] = x_start # ['x', 'y']
            self.observation[1] = y_start # ['x', 'y']
        elif self.model.model_type == "UnicycleKinematicModel":
            self.observation[0] = x_start # ['x', 'y', 'theta']
            self.observation[1] = y_start # ['x', 'y', 'theta']
            self.observation[2] = theta_start # ['x', 'y', 'theta']
        elif self.model.model_type == "BicycleKinematicModel":
            self.observation[0] = x_start # ['x', 'y', 'yaw', 'v']
            self.observation[1] = y_start # ['x', 'y', 'yaw', 'v']
            self.observation[2] = theta_start # ['x', 'y', 'yaw', 'v']
            self.observation[3] = initial_vx # ['x', 'y', 'yaw', 'v']

        self.global_path = self.planner.global_plan(self.observation, self.get_goal_pose())

    def set_goal_pose(self, goal_x, goal_y, goal_theta):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.goal_theta = goal_theta

    def set_pose(self, px, py, theta):
        self.px = px
        self.py = py
        self.theta = theta
        
        if self.model.model_type == "HolonomicModel": #np.array([self.px, self.py]) 
            self.observation[0] = self.px 
            self.observation[1] = self.py
        elif self.model.model_type == "UnicycleKinematicModel": # np.array([self.px, self.py, self.theta]) #['x', 'y', 'theta']
            self.observation[0] = self.px 
            self.observation[1] = self.py
            self.observation[2] = self.theta
        elif self.model.model_type == "BicycleKinematicModel": #np.array([self.px, self.py, self.theta, self.vx])  # ['x', 'y', 'yaw', 'v']
            self.observation[0] = self.px 
            self.observation[1] = self.py 
            self.observation[2] = self.theta 

    def set_position(self, position):
        self.px = position[0]
        self.py = position[1]
        
        if self.model.model_type == "HolonomicModel":
            self.observation[0] = self.px # ['x', 'y']
            self.observation[1] = self.py # ['x', 'y']
        elif self.model.model_type == "UnicycleKinematicModel":
            self.observation[0] = self.px # ['x', 'y', 'theta']
            self.observation[1] = self.py # ['x', 'y', 'theta']
        elif self.model.model_type == "BicycleKinematicModel":
            self.observation[0] = self.px # ['x', 'y', 'yaw', 'v']
            self.observation[1] = self.py # ['x', 'y', 'yaw', 'v']

    def set_velocity(self, velocity):
        self.vx = velocity[0]
        self.vy = velocity[1]
        
        if self.model.model_type == "BicycleKinematicModel":
            self.observation[3] = self.vx #np.array([self.px, self.py, self.theta, self.vx])  # ['x', 'y', 'yaw', 'v']

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

    def get_obs(self):
        return self.observation 

    # plan and control or optimize
    def compute_action(self, observation):
        """
        Compute state using received observation and pass it to policy

        """
        if self.planner is None:
            raise AttributeError('planner attribute has to be set!')
        if self.controller is None:
            raise AttributeError('controller attribute has to be set!')

        # plan
        g_xs = self.planner.local_plan(observation, self.global_path)

        # obtain sol
        action = self.controller.solve(observation, g_xs)

        return action

    def update(self, action, time_step):
        """
        Compute state using received observation and pass it to policy

        """
        
        if self.model.model_type == "HolonomicModel":
            curr_x =  self.observation #np.array([self.px, self.py]) 
            u =  np.array([action[0], action[1]]) 
            self.observation = self.model.predict_next_state(curr_x, u, time_step)
            self.px, self.py = self.observation[0:2]
        elif self.model.model_type == "UnicycleKinematicModel":
            curr_x =  self.observation # np.array([self.px, self.py, self.theta]) #['x', 'y', 'theta']
            u =  np.array([action[0], action[1]]) # ['v', 'w']
            self.observation = self.model.predict_next_state(curr_x, u, time_step)
            self.px, self.py, self.theta = self.observation[0:3]
        elif self.model.model_type == "BicycleKinematicModel":
            curr_x =  self.observation #np.array([self.px, self.py, self.theta, self.vx])  # ['x', 'y', 'yaw', 'v']
            u =  np.array([action[0], action[1]]) # ['a', 'delta']
            self.observation = self.model.predict_next_state(curr_x, u, time_step)
            self.px, self.py, self.theta = self.observation[0:3]

        #return self.observation

    def step(self, action):
        """
        Perform an action and update the state
        """
        # clip action
        action = np.clip(action,
                    self.config.INPUT_LOWER_BOUND,
                    self.config.INPUT_UPPER_BOUND)
        
        #self.check_validity(action)
        self.update(action, self.time_step)
        

    def goal_reached(self):
        return norm(np.array(self.get_position()) - np.array(self.get_goal_position())) < self.xy_tolerance

