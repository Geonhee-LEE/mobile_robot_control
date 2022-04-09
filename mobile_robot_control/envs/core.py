from __future__ import absolute_import, division, print_function, nested_scopes, generators, with_statement, unicode_literals

import abc
import matplotlib.pyplot as plt
import numpy as np
from random import random

__author__ = 'Geonhee-LEE <gunhee6392@gmail.com>'

show_animation = True

class Env(object):
    """
    motivated from OpenAI Gym
    """
    reward_range = (-float('inf'), float('inf'))
    spec = None

    # Set action, observation space
    action_space = None
    observation_space = None

    @abc.abstractmethod
    def step(self, action):
        """Run one timestep of the environment's dynamics. When end of
        episode is reached, you are responsible for calling `reset()`
        to reset this environment's state.
        Accepts an action and returns a tuple (observation, reward, done, info).
        Args:
            action (object): an action provided by the agent
        Returns:
            observation (object): agent's observation of the current environment
            reward (float) : amount of reward returned after previous action
            done (bool): whether the episode has ended, in which case further step() calls will return undefined results
            info (dict): contains auxiliary diagnostic information (helpful for debugging, and sometimes learning)
        """
        raise NotImplementedError

    @abc.abstractmethod
    def reset(self):
        """Resets the state of the environment and returns an initial observation.
        Returns: 
            observation (object): the initial observation.
        """
        raise NotImplementedError

    @abc.abstractmethod
    def render(self):
        """Renders the environment.
        """
        raise NotImplementedError

