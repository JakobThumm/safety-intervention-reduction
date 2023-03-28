from config_presets import configs

from rlpyt.projects.safe.cppo_pid import CppoPID
from rlpyt.utils.launching.affinity import affinity_from_code
from rlpyt.utils.logging.context import logger_context

from src.agents import CppoAgent, CppoLstmAgent
from src.cpu_sampler import CpuSampler
from src.gym_wrapper import SafetyGymTrajInfo, safety_gym_make
from src.runner import MinibatchRl
from src.env.suite import register_all
from src.activate_shield import config_shield
import os


def make_runner(config):
    """Initialize all elements needed for training."""
    affinity = affinity_from_code(config["slot_affinity_code"])
    sampler = CpuSampler(
        EnvCls=safety_gym_make,
        env_kwargs=config["env"],
        eval_env_kwargs=config["env"],
        TrajInfoCls=SafetyGymTrajInfo,
        **config["sampler"]
    )
    algo = CppoPID(**config["algo"])
    agent = CppoLstmAgent(model_kwargs=config["model"], **config["agent"]) if config["use_lstm"] else CppoAgent(model_kwargs=config["model"], **config["agent"])
    runner = MinibatchRl(
        algo=algo,
        agent=agent,
        sampler=sampler,
        affinity=affinity,
        **config["runner"],
    )
    return runner


def train_regular(config, exp_name, env_name):
    """Train an experiment on multiple seeds."""
    seeds = [0]
    if config["other"]["n_seeds"] > 1:
        seeds = [i*10 for i in range(config["other"]["n_seeds"])]
    run_ID = config["run_ID"]

    for seed in seeds:
        config["runner"]["seed"] = seed
        config["env"]["id"] = env_name
        runner = make_runner(config)
        config_shield.update_config(config)
        with logger_context(f"{env_name.split('-')[1]}", exp_name+run_ID, exp_name, config, snapshot_mode="last"):
            runner.train()

        run_ID = str(int(run_ID) + 1)


def constraint_search(config, exp_name, env_name):
    """Train for multiple cost limit values with a constrained shield."""
    costs = [150, 200, 250, 300, 350, 400]

    run_ID = config["run_ID"]

    for cost in costs:
        config["algo"]["cost_limit"] = cost
        config["env"]["id"] = env_name
        config_shield.update_config(config)
        runner = make_runner(config)
        config_shield.update_config(config)
        with logger_context(f"{env_name.split('-')[1]}_{cost}", exp_name+run_ID, exp_name, config, snapshot_mode="last"):
            runner.train()

        run_ID = str(int(run_ID) + 1)

def reward_search(config, exp_name, env_name):
    """Train for multiple shield penalty values."""
    run_ID = config["run_ID"]

    for rwd in [-1, -0.1, -0.01]:
        config["other"]["reward_penalty"] = rwd
        config["env"]["id"] = env_name
        config_shield.update_config(config)
        runner = make_runner(config)
        config_shield.update_config(config)
        with logger_context(f"{env_name.split('-')[1]}", exp_name+run_ID, exp_name, config, snapshot_mode="last"):
            runner.train()

        run_ID = str(int(run_ID) + 1)

if __name__ == "__main__":
    # Register Safety Gym v1 environments (with shield use)
    register_all()

    # Get env name set in docker-compose file
    exp_name = os.environ["exp_name"]
    env_name = "Safexp-Point" + os.environ["gym_env"]+"-v1"
    print(configs.keys())
    replacement_strat = configs[exp_name]["other"]["replacement_strat"]
    try:
        replacement_strat = os.environ["replacement_strat"]
    except KeyError:
        print(f"No replacement strategy given. Defaulting to {configs[exp_name]['other']['replacement_strat']}.")

    configs[exp_name]["other"]["replacement_strat"] = replacement_strat

    if exp_name in configs.keys():
        train_regular(configs[exp_name], f"{exp_name}_{replacement_strat}", env_name)
    elif exp_name == "Constraint_Search":
        constraint_search(configs["search"][exp_name], f"{exp_name}_{replacement_strat}", env_name)
    elif exp_name == "RewardSearch":
        reward_search(configs["search"][exp_name], f"{exp_name}_{replacement_strat}", env_name)
    else:
        raise NotImplementedError("The required experiment has not been properly defined.")
