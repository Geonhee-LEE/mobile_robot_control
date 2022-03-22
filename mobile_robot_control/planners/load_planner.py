from .const_planner import ConstantPlanner
from .closest_point_planner import ClosestPointPlanner


def load_planner(args, config):

    if args.config == "UnicyclePoseGoal":
        return None

    if args.env == "UnicycleConst":
        return ConstantPlanner(config)
    elif args.env == "UnicycleTrack":
        return ClosestPointPlanner(config)

    raise NotImplementedError(
        "There is not {} Planner".format(args.planner_type))