# Towards Provably Safe RL on Mobile Robots: Achieving Zero Collision on the OpenAI Safety Gym

This folder contains the code developped for my master thesis under the supervision of Prof. Matthias Althoff and Jakob Thumm.
Everything needed to reproduce my experiments is present here.

This code is mostly based on [rlpyt](https://github.com/astooke/rlpyt) and [SaRA-shield](https://github.com/JakobThumm/sara-shield).
It is designed to work in a Docker container.

## Installation

The fastest way to use this code is with docker.
It currently uses the `safety_intervention_reduction:1.0` image which can be built with `docker-compose build`.
Start the docker swarm with `docker swarm init`.

## Run an experiment

The experiments can be run simply by running `CURRENT_UID=$(id -u):$(id -g) EXP_NAME=XXX docker stack deploy --compose-file docker-compose.yml STACK_NAME`.
By default, this will start a training on PointGoal1, PointGoal2, PointButton1 and PointButton2 with 3 CPUs each.
Training data will be saved in `./data/local/YYYYMMDD/hhmmss/env_name`. If they don't already exist, you might need to create the `data` and `local` folders yourself. 
**If omitting `CURRENT_UID=$(id -u):$(id -g)` in the launch command, saved data will be owned by the root user.**
The `STACK_NAME` parameter is used to differentiate between different [docker stacks](https://docs.docker.com/engine/reference/commandline/stack_deploy/) and can be set to your convenience (but should not be omitted).

Alternatively to `docker stack`, one can also use `CURRENT_UID=$(id -u):$(id -g) EXP_NAME=XXX docker-compose up`.

To select an experiment, one must modify the exp_name parameter can be chosen among the following:
 - `PPO`: PPO agent. 
    ```[bash]
    CURRENT_UID=$(id -u):$(id -g) EXP_NAME=PPO docker stack deploy --compose-file docker-compose.yml ppo
    ``` 
 - `PID`: PID agent. 
    ```[bash]
    CURRENT_UID=$(id -u):$(id -g) EXP_NAME=PID docker stack deploy --compose-file docker-compose.yml pid
    ``` 
 - `Shielded_PPO`: PPO with provably safe shield that stops the agent from colliding. 
    ```[bash]
    CURRENT_UID=$(id -u):$(id -g) EXP_NAME=Shielded_PPO docker stack deploy --compose-file docker-compose.yml shielded_ppo
    ``` 
 - `Shielded_PPO_Replacement`: PPO with provably safe shield plus a strategy to replace unsafe actions from the agent.
    ```[bash]
    CURRENT_UID=$(id -u):$(id -g) EXP_NAME=Shielded_PPO_Replacement docker stack deploy --compose-file docker-compose.yml shielded_ppo_replacement
    ``` 
 - `Shielded_PPO_Projection`: PPO with provably safe shield plus a strategy to project unsafe actions from the agent towards safe actions.
    ```[bash]
    CURRENT_UID=$(id -u):$(id -g) EXP_NAME=Shielded_PPO_Projection docker stack deploy --compose-file docker-compose.yml shielded_ppo_projection
    ``` 
 - `Shielded_PPO_Reward`: PPO with provably safe shield plus a negative reward when using the shield.
    ```[bash]
    CURRENT_UID=$(id -u):$(id -g) EXP_NAME=Shielded_PPO_Reward docker stack deploy --compose-file docker-compose.yml shielded_ppo_reward
    ``` 
 - `Shielded_PID`: PID Lagrangian to limit the number of shield interventions.
    ```[bash]
    CURRENT_UID=$(id -u):$(id -g) EXP_NAME=Shielded_PID docker stack deploy --compose-file docker-compose.yml shielded_pid
    ``` 
 - `ConstraintSearch`: PID Lagrangian to limit the number of shield interventions with different cost limits.
    ```[bash]
    CURRENT_UID=$(id -u):$(id -g) EXP_NAME=ConstraintSearch docker stack deploy --compose-file docker-compose.yml constraint_search
    ``` 
 - `RewardSearch`: PPO with provably safe shield plus a negative reward when using the shield with different reward punishments.
    ```[bash]
    CURRENT_UID=$(id -u):$(id -g) EXP_NAME=RewardSearch docker stack deploy --compose-file docker-compose.yml reward_search
    ``` 
## Plot results

Training data for my experiments is saved in `plots/data_per_env`.
The csv files are already averaged over multiple seeds, except for the search experiments which used a moving average (window size = 2) instead.
This was done manually, so no code for seed avering is included.

Each folder contains the files generating the plots for the corresponding figure in the report.
The plot style is defined in `plotstyle_pelat.tex`.
Every subsequent .tex file defines one plot.
The `run_fig.sh` file present in each folder creates all plots defined in their folder.

## Baselines

The training data for PID-Lagrangian was obtained directly from [rlpyt](https://github.com/astooke/rlpyt). Similarly, the data for PPO and PPO-Lagrangian was generated with the [OpenAI starter agent kit](https://github.com/openai/safety-starter-agents). Please follow the installation steps from their github repository if you wish to reproduce those results.
