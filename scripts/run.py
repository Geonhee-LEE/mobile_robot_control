import argparse

from mobile_robot_control.envs.load_environment import load_env 
from mobile_robot_control.configs.load_configs import load_config
from mobile_robot_control.agents.load_agents import load_agent

from mobile_robot_control.planners.load_planner import load_planner
from mobile_robot_control.models.load_model import load_model
from mobile_robot_control.controllers.load_controller import load_controller

import numpy as np

def run(args):
    #make_logger(args.result_dir)

    config = load_config(args)
    env = load_env(args, config)
    agent = load_agent(args, config)

    #history_x, history_u, history_g = [], [], []
    step_count = 0
    score = 0.

    env.set_agent(agent) # Set agent in Environment

    for id in range(config.EPISODE_NUM):
        curr_x = env.reset()
        done = False

        while not done:
            if args.visualize:
                env.render()

            u = agent.compute_action(curr_x)

            # step
            next_x, cost, done, info = env.step(u)

            # save
            #history_u.append(u)
            #history_x.append(curr_x)
            #history_g.append(g_xs[0])
            # update
            curr_x = next_x
            score += cost           
            step_count += 1
    #plot_results(np.array(history_x), np.array(history_u), history_g=np.array(history_g), args=args)
    #save_plot_data(np.array(history_x), np.array(history_u), history_g=np.array(history_g), args=args)
    

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--controller_type", type=str, default="PID", choices=['PID', 'PurePursuit']) # choices=['rock', 'paper', 'scissors']
    parser.add_argument("--planner_type", type=str, default="ConstGoal", choices=['ConstGoal', 'SinusoidalPlanner', 'bezier', 'cubic', 'lane']) 
    parser.add_argument("--env", type=str, default="PoseGoal", choices=['PoseGoal', 'PathGoal'])
    parser.add_argument("--model", type=str, default="BicycleKinematicModel", choices=['UnicycleKinematicModel', 'BicycleKinematicModel'])
    parser.add_argument("--config", type=str, default="BicyclePoseGoal", choices=['UnicyclePoseGoal', 'BicyclePoseGoal', 'BicyclePathGoal'])
    parser.add_argument("--save_anim", type=bool, default=False)
    parser.add_argument("--visualize", type=bool, default=True)
    parser.add_argument("--result_dir", type=str, default="./result")

    args = parser.parse_args()

    run(args)


if __name__ == "__main__":
    main()