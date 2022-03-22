from .unicycle_pose_goal import UnicyclePoseGoalConfigModule


def load_config(args):
    """
    Returns:
        config (ConfigModule class): configuration for the each env
    """
    if args.config == "UnicyclePoseGoal":
        return UnicyclePoseGoalConfigModule()