from .core import Env
import matplotlib.pyplot as plt
import numpy as np
from mobile_robot_control.utils.transformation import transformation_matrix
from random import random

class PathGoalEnv(Env):
    """
    Implements a Gym Environment for controller for path  
    """
    def __init__(self, config):
        self.config = config # from configuration file

        #GYM Properties (set in subclasses)

        self.x_traj = []
        self.y_traj = []

        self.x_start = 0
        self.y_start = 0
        self.theta_start = 0
        self.x_goal = 0
        self.y_goal = 0
        self.theta_goal = 0
        
        self.dt = config.DT
        self.episode_num = 0

    def reset(self):
        """
        Reset to initial position
        """
        self.x_traj = []
        self.y_traj = []

        if (isinstance(self.config.START_POSES, str)):
            self.x_start, self.y_start, self.theta_start = np.array([20 * random(), 20 * random(), 2 * np.pi * random() - np.pi]) # random            
        elif len(self.config.START_POSES.shape) == 1: # constant goal
            self.x_start, self.y_start, self.theta_start = self.config.START_POSES
        else: # multiple goals
            self.x_start, self.y_start, self.theta_start = self.config.START_POSES[self.episode_num, :]

        if (isinstance(self.config.GOAL_POSES, str)):
            self.x_goal, self.y_goal, self.theta_goal = np.array([20 * random(), 20 * random(), 2 * np.pi * random() - np.pi]) # random      
        elif len(self.config.GOAL_POSES.shape) == 1: # constant goal
             self.x_goal, self.y_goal, self.theta_goal = self.config.GOAL_POSES 
        else: # multiple goals
            self.x_goal, self.y_goal, self.theta_goal = self.config.GOAL_POSES[self.episode_num, :]

        self.x_traj.append(self.x_start)
        self.y_traj.append(self.y_start)

        self.agent.set_pose(self.x_start, self.y_start, self.theta_start)
        self.agent.set_goal_pose(self.x_goal, self.y_goal, self.theta_goal)

        self.agent.set_velocity(np.array([1,1])) ##### need to set

        self.episode_num = self.episode_num + 1

        return self.agent.get_pose()

    def set_agent(self, agent):
        self.agent = agent
        
    def get_reward(self):
        """
        TODO:Implement reward functionality
        """
        return 0

    def get_obs(self):        
        return self.agent.get_pose()

    def step(self, action):
        """
        Action should be a steer_dict = {"angle":float, "speed":float}
        """

        self.agent.step(action)

        #get reward & check if done & return
        obs = self.get_obs()
        reward = self.get_reward()
        done = self.tooclose()
        info = {}

        self.x_traj.append(obs[0])
        self.y_traj.append(obs[1])
        return obs, reward, done,info
    
    def tooclose(self):
        """
        Uses latest_obs to determine if we are too_close (currently uses LIDAR)
        """
        if self.agent.goal_reached():
            return True

        return False

    def render(self):
        """Renders the environment.
        """
        plt.cla()
        plt.arrow(self.x_start, self.y_start, np.cos(self.theta_start),
                        np.sin(self.theta_start), color='r', width=0.1)
        plt.arrow(self.x_goal, self.y_goal, np.cos(self.theta_goal),
                        np.sin(self.theta_goal), color='g', width=0.1)
        # Corners of triangular vehicle when pointing to the right (0 radians)
        p1_i = np.array([0.5, 0, 1]).T
        p2_i = np.array([-0.5, 0.25, 1]).T
        p3_i = np.array([-0.5, -0.25, 1]).T

        x, y, theta = self.agent.get_pose()
        T = transformation_matrix(x, y, theta)
        p1 = np.matmul(T, p1_i)
        p2 = np.matmul(T, p2_i)
        p3 = np.matmul(T, p3_i)

        plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
        plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
        plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

        plt.plot(self.x_traj, self.y_traj, 'b--')

        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])

        plt.xlim(0, 20)
        plt.ylim(0, 20)

        plt.pause(0.01)
 