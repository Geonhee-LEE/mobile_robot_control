from .unicycle_pose_config import UnicyclePoseGoalConfigModule
from .unicycle_path_config import UnicyclePathGoalConfigModule
from .bicycle_pose_config import BicyclePoseGoalConfigModule

def load_config(args):
    """
    Returns:
        config (ConfigModule class): configuration for the each env
    """
    if args.config == "UnicyclePoseGoal":
        return UnicyclePoseGoalConfigModule()
    elif args.config == "UnicyclePathGoal":
        return UnicyclePathGoalConfigModule()
    elif args.config == "BicyclePoseGoal":
        return BicyclePoseGoalConfigModule()
        