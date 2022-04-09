from .const_planner import ConstantPlanner
from .sinusoidal_planner import SinusoidalPlanner
from .closest_point_planner import ClosestPointPlanner

def load_planner(args, config):

    if args.planner_type == "ConstGoal": # UnicyclePoseGoal, BicyclePoseGoal
        return ConstantPlanner(config)
    elif args.planner_type == "Sinusoidal":
        return SinusoidalPlanner(config)

    raise NotImplementedError(
        "There is not {} Planner".format(args.planner_type))