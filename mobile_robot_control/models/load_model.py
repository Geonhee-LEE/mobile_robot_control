from .unicycle_kinematic import UnicycleKinematicModel

def load_model(args, config):

    if args.model == "UnicycleKinematicModel":
        return UnicycleKinematicModel(config)

    raise NotImplementedError("There is not {} Model".format(args.env))