import math
import os

import numpy as np
import torch

from config_presets import configs
from rlpyt.projects.safe.cppo_pid import CppoPID
from rlpyt.utils.launching.affinity import affinity_from_code
from rlpyt.utils.logging.context import logger_context
from rlpyt.utils.buffer import torchify_buffer
from rlpyt.agents.base import AgentInputs
from src.activate_shield import config_shield
from src.agents import CppoAgent, CppoLstmAgent
from src.cpu_sampler import CpuSampler
from src.env.suite import register_all
from src.gym_wrapper import SafetyGymTrajInfo, safety_gym_make
from src.runner import MinibatchRl
from tqdm import tqdm

exp_name = "Shielded_PPO" # From config_presets.possible
env_name = "Safexp-PointGoal1-v1"
use_agent = False
VISUALIZE = True
AGENT_PATH = "/home/jakob/Promotion/code/safety-intervention-reduction/data/local/20230331/043419/PointGoal1/run_Shielded_PPO_2/params.pkl"

def make_env_agent(config):
    env = safety_gym_make(**config["env"])
    agent = None

    if use_agent:
        configs[exp_name]["env"]["id"] = env_name
        load_path = AGENT_PATH
        checkpoint = torch.load(load_path)
        initial_model_state_dict = checkpoint["agent_state_dict"]
        agent = CppoAgent(model_kwargs=config["model"], initial_model_state_dict=initial_model_state_dict, **config["agent"])

        agent.initialize(env.spaces)

    return env, agent


def get_agent_action(o, a=[0,0], r=0):
    r = np.asarray(r, dtype="float32")  # Must match torch float dtype here.
    a = np.asarray(a, dtype="float32")  # Must match torch float dtype here.
    agent_inputs = torchify_buffer(AgentInputs(o, a, r))
    a, agent_info = agent.step(*agent_inputs)
    return a

def run_random_angle(env_name, config, act):
    config["env"]["id"] = env_name
    env = safety_gym_make(**config["env"])
    obs = env.reset()
    done = False
    timestep = 0.002
    ep_ret = 0
    ep_cost = 0
    t = 0
    t_max = 49
    mat_rot = env.world.robot_mat()[:2, :2]
    angular_speed = 0
    last_angle = math.atan2(mat_rot[1,0], mat_rot[0,0])
    # print(env.sim.model.body_mass)
    # print(env.sim.get_state())
    first_delta = None
    sum_delta = 0
    steps = 0
    while True:
        if done:
            print('Episode Return: %.3f \t Shield Cost: %.3f'%(ep_ret, ep_cost))
            ep_ret, ep_cost = 0, 0
            obs = env.reset()
            mat_rot = env.world.robot_mat()[:2, :2]
            angular_speed = 0

            last_angle = math.atan2(mat_rot[1,0], mat_rot[0,0])

        # if t<50:
        #     act = [0, 0.1]
        # else:
        #     act = [0, 0]
        # if t == 50:
        #     print("CHANGE")
        last_mat_rot = env.world.robot_mat()[:2, :2]
        obs, reward, done, info = env.step(act)
        if VISUALIZE:
            env.render()

        mat_rot = env.world.robot_mat()[:2, :2]
        angle = math.atan2(mat_rot[1,0], mat_rot[0,0])
        delta = angle - last_angle

        if first_delta is None:
            first_delta = delta

        for i in range(1):
            # angle_sim += 0.002*act[1]
            # compute new rotation matrix
            angular_speed = 0.067*act[1] - 0.0054 # long live regression
            last_mat_rot_sim = last_mat_rot
            mat_rot_dot = np.array(((np.cos(angular_speed), -np.sin(angular_speed)), (np.sin(angular_speed), np.cos(angular_speed))))
            last_mat_rot_sim = np.matmul(last_mat_rot_sim, mat_rot_dot)
        angle_sim = math.atan2(last_mat_rot_sim[1,0], last_mat_rot_sim[0,0])
        maybe_delta = angle_sim - last_angle

        ratio = delta - maybe_delta

        # print((mat_rot - last_mat_rot)/0.002)
        # print(f"Angle : {angle},\t delta = {delta},    \t maybe = {maybe_delta},   \t off by diff = {ratio}")
        sum_delta += delta
        steps += 1
        # if last_angle > 0:
        # print(f"Angle : {angle},\t delta = {delta},    \t first = {first_delta},   \t off by diff   = {first_delta-delta}")

        # Assumption:
        # motor has gear of 0.3

        last_angle = angle
        t+=1
        # if t == 10:
        #     first_delta = None
        #     obs = env.reset()
        #     mat_rot = env.world.robot_mat()[:2, :2]
        #     last_angle = math.atan2(mat_rot[1,0], mat_rot[0,0])
        # env.render()
        if t == t_max:
            break
    print(f"Action : {act[1]}, Result : {sum_delta/steps}")

def run_random_speed(env, config):
    obs = env.reset()
    done = False
    ep_ret = 0
    ep_cost = 0
    t = 0
    t_max = 20
    last_speed = 0
    print(env.sim.model.body_mass)
    print(env.sim.get_state())
    first_delta = None
    while True:
        if done:
            print('Episode Return: %.3f \t Shield Cost: %.3f'%(ep_ret, ep_cost))
            ep_ret, ep_cost = 0, 0
            last_speed = 0
            obs = env.reset()
        if t<10:
            act = [0.01, 0]
        else:
            act = [-0.05, 0]
        obs, reward, done, info = env.step(act)
        if VISUALIZE:
            env.render()
        velocity = env.world.robot_vel()
        speed = math.sqrt(velocity[0]*velocity[0] + velocity[1]*velocity[1])
        delta = speed - last_speed

        if first_delta is None:
            first_delta = speed

        speed_sim = last_speed
        for i in range(10):
            speed_sim += 0.002*abs(act[0])*0.2989/0.00519 - 0.00384*speed_sim
        maybe_delta = speed_sim - last_speed

        ratio = delta/maybe_delta
        print(env.sim.get_state().time)

        print(f"Speed : {speed},\t delta = {delta},\t maybe = {maybe_delta},\t off by factor = {ratio}")
        if last_speed > 0:
            print(f"Speed : {speed},\t delta = {delta},\t first = {first_delta},\t off by diff   = {(first_delta-delta)/last_speed}")
            print(f"alpha = {0.00519*(speed - (1 - 0.00384) * last_speed)/0.002}")
        # Assumption
        # v_t+1 = v_t + dt * alpha - d * v_t

        # for -0.05 < a_0 < 0.05 and a_1 = 0
            # d = 0.00384
            # dt * alpha = first_delta (yay)
            # v_t+1 = v_t + dt * a0 * 0.2989 / m - d * v_t

        last_speed = speed
        t+=1
        if t == 10:
            first_delta = None
            last_speed = 0
            obs = env.reset()
        if t == t_max:
            break

def run_random(env, agent=None):
    env.seed(1)
    obs = env.reset()
    act = [0,0]
    r = 0
    done = False
    ep_ret = 0
    ep_cost = 0
    t = 0
    t_max = 1000

    envs = 100
    # while True:
    for _ in tqdm(range(envs)):
        done = False
        while not done:
            # assert env.observation_space.contains(obs)
            if use_agent:
                act = get_agent_action(obs, act, r)
            else:
                act = [0.001, -0.15]
            obs, r, done, info = env.step(act)
            ep_ret += r
            if VISUALIZE:
                env.render()
            # print(f"Before: {act} \t After: {act_2}")
            # print(info)
            t += 1
            if t >= t_max:
                done = True
                t = 0
            ep_cost += info.shield_active
            if done:
                print('Episode Return: %.3f \t Shield Cost: %.0f \t Cummulative Cost: %.0f'%(ep_ret, ep_cost, info.cum_cost))
                ep_ret, ep_cost = 0, 0
                obs = env.reset()
                act = [0, 0]
                r = 0
        # if info.cost_shield > 0:
        #     if t==1:
        #         done = True
        #         continue
        # env.render()
    print(f"Failsafe_avg: {ep_cost/envs}")

if __name__ == "__main__":
    # Register Safety Gym v1 environments (with shield use)
    register_all()

    config = configs[exp_name]
    config["env"]["id"] = env_name
    # config["other"]["activate_shield"] = False
    config_shield.update_config(config)

    env, agent = make_env_agent(config)
    run_random(env, agent)

    # for act1 in range(1,11):
    #     run_random_angle(env_name, config, [0,act1/10])
