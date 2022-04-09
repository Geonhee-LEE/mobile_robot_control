from .unicycle_kinematic import UnicycleKinematicModel
from .bicycle_kinematic import BicycleKinematicModel

def load_model(args, config):

    if args.model == "UnicycleKinematicModel":
        return UnicycleKinematicModel(config)
    elif args.model == "BicycleKinematicModel":
        return BicycleKinematicModel(config)

    raise NotImplementedError("There is not {} Model".format(args.model))