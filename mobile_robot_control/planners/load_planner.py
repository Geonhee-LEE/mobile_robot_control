from .closest_point_planner import ClosestPointPlanner


def load_planner(args, config):

    if args.config == "UnicyclePoseGoal" or args.config == "BicyclePoseGoal" :
        return None

    if args.env == "UnicycleTrack":
        return ClosestPointPlanner(config)

    raise NotImplementedError(
        "There is not {} Planner".format(args.planner_type))