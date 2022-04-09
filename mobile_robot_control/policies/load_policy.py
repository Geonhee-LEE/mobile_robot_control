

def load_planner(args, config):

    if args.env == "UnicycleConst":
        return ConstantPlanner(config)
    elif args.env == "UnicycleTrack":
        return ClosestPointPlanner(config)

    raise NotImplementedError(
        "There is not {} Planner".format(args.planner_type))