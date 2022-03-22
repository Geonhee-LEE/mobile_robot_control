from .pid_pose_controller import PIDPoseControl
def load_controller(args, config, model):

    if args.controller_type == "PID":
        return PIDPoseControl(config)
    #elif args.controller_type == "CEM":
    #    return CEM(config, model)

    raise ValueError("No controller: {}".format(args.controller_type))