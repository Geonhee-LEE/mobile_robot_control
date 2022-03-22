from .pose_goal_env import PoseGoalEnv


def load_env(args, config):

    if args.env == "PoseGoal":
        return PoseGoalEnv(config)
    #elif args.env == "PathGoal":
    #    return PathGoalEnv()
    #elif args.env == "TrajectoryGoal":
    #    return TrajectoryGoalEnv()

    raise NotImplementedError("There is not {} Env".format(args.env))