"""

Move to specified pose

Author: Daniel Ingram (daniel-s-ingram)
        Atsushi Sakai(@Atsushi_twi)

P. I. Corke, "Robotics, Vision & Control", Springer 2017, ISBN 978-3-319-54413-7

"""

import matplotlib.pyplot as plt
import numpy as np
from random import random

show_animation = True
x_start = 1
y_start = 0
x_goal = 0
y_goal = 0
theta_start = 0
theta_goal = 0
dt = 0.01

def set_init_pose(_x_start, _y_start, _theta_start, _x_goal, _y_goal, _theta_goal):  # pragma: no cover
    global x_start, y_start, theta_start, x_goal, y_goal, theta_goal
    print("set_init_pose %d", x_start)
    x_start = _x_start
    y_start = _y_start
    theta_start = _theta_start
    x_goal = _x_goal
    y_goal = _y_goal
    theta_goal = _theta_goal

def plot_vehicle(x, y, theta, x_traj, y_traj):  # pragma: no cover
    global x_start, y_start, theta_start, x_goal, y_goal, theta_goal
    plt.cla()
    plt.arrow(x_start, y_start, np.cos(theta_start),
                      np.sin(theta_start), color='r', width=0.1)
    plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                      np.sin(theta_goal), color='g', width=0.1)
    # Corners of triangular vehicle when pointing to the right (0 radians)
    p1_i = np.array([0.5, 0, 1]).T
    p2_i = np.array([-0.5, 0.25, 1]).T
    p3_i = np.array([-0.5, -0.25, 1]).T

    T = transformation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    plt.plot(x_traj, y_traj, 'b--')

    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])

    plt.xlim(0, 20)
    plt.ylim(0, 20)

    plt.pause(dt)


def transformation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])


def main():

    for i in range(5):
        x_start = 20 * random()
        y_start = 20 * random()
        theta_start = 2 * np.pi * random() - np.pi
        x_goal = 20 * random()
        y_goal = 20 * random()
        theta_goal = 2 * np.pi * random() - np.pi
        print("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n" %
              (x_start, y_start, theta_start))
        print("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n" %
              (x_goal, y_goal, theta_goal))
        move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)


if __name__ == '__main__':
    main()
