from mobile_robot_control.planners.load_planner import load_planner
from mobile_robot_control.models.load_model import load_model
from mobile_robot_control.controllers.load_controller import load_controller
from .agent import Agent


def load_agent(args, config):
    """
    Returns:
        config (ConfigModule class): configuration for the each env
    """
    planner = load_planner(args, config)
    model = load_model(args, config)
    controller = load_controller(args, config, model)
    
    return Agent(config, planner, model, controller)
