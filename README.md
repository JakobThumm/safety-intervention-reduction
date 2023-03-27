# Towards Provably Safe RL on Mobile Robots: Achieving Zero Collision on the OpenAI Safety Gym

This folder contains the code developped for my master thesis under the supervision of Prof. Matthias Althoff and Jakob Thumm.
Everything needed to reproduce my experiments is present here.

This code is mostly based on [rlpyt](https://github.com/astooke/rlpyt) and [SaRA-shield](https://github.com/JakobThumm/sara-shield).
It is designed to work in a Docker container.

## Installation

The fastest way to use this code is with docker.
It currently uses the safety_shield_pelat:0.3 image which can be built with `docker-compose build`.

## Run an experiment

The experiments can be run simply by running `CURRENT_UID=$(id -u):$(id -g) EXP_NAME=XXX docker stack deploy --compose-file docker-compose.yml STACK_NAME`.
By default, this will start a training on PointGoal1, PointGoal2, PointButton1 and PointButton2 with 3 CPUs each.
Training data will be saved in `./data/local/YYYYMMDD/hhmmss/env_name`. If they don't already exist, you might need to create the `data` and `local` folders yourself. If omitting `CURRENT_UID=$(id -u):$(id -g)` in the launch command, saved data will be owned by the root user.
The `STACK_NAME` parameter is used to differentiate between different [docker stacks](https://docs.docker.com/engine/reference/commandline/stack_deploy/) and can be set to your convenience (but should not be omitted).

To select an experiment, one must modify the exp_name parameter can be chosen among the following:
 - "Shield" : the "basic" shield that stops the agent from colliding. 
`CURRENT_UID=$(id -u):$(id -g) EXP_NAME=Shield docker stack deploy --compose-file docker-compose.yml safetyshield_pointshield` 
 - "Best" : try to find the best action among 20 randomly sampled safe actions, if the agent's action is unsafe.  
`CURRENT_UID=$(id -u):$(id -g) EXP_NAME=Best docker stack deploy --compose-file docker-compose.yml safetyshield_pointbest` 
 - "New" : execute a random safe action, if the agent's action is unsafe.
`CURRENT_UID=$(id -u):$(id -g) EXP_NAME=New docker stack deploy --compose-file docker-compose.yml safetyshield_pointnew` 
 - "Constrained" : constrain the use of the shield with a cost limit set to 450.
`CURRENT_UID=$(id -u):$(id -g) EXP_NAME=Constrained docker stack deploy --compose-file docker-compose.yml safetyshield_pointconstrained` 
 - "ConstrainedNew" : constrain the use of the shield with a cost limit set to 450, and execute a randomly chosen safe action if the agent's action is unsafe.
`CURRENT_UID=$(id -u):$(id -g) EXP_NAME=ConstrainedNew docker stack deploy --compose-file docker-compose.yml safetyshield_pointcstrnew` 
 - "ConstraintSearch" : constrain the use of the shield for different cost limits.
`CURRENT_UID=$(id -u):$(id -g) EXP_NAME=ConstraintSearch docker stack deploy --compose-file docker-compose.yml safetyshield_pointcstrsearch` 
 - "RewardSearch" : penalize the use of the shield with different reward punishments.
`CURRENT_UID=$(id -u):$(id -g) EXP_NAME=RewardSearch docker stack deploy --compose-file docker-compose.yml safetyshield_pointrwdsearch` 
The Search experiments are run on one seed with multiple parameters; all the others use 5.

Additionally, one can define their own experiment configuration in `config_custom.py` and set `EXP_NAME=Custom` to train with their configuration:
`CURRENT_UID=$(id -u):$(id -g) EXP_NAME=Custom docker stack deploy --compose-file docker-compose.yml safetyshield_pointcustom`

## Plot results

Training data for my experiments is saved in `plots/data_per_env`.
The csv files are already averaged over multiple seeds, except for the Search experiments which used a moving average (window size = 2) instead.
This was done manually, so no code for seed avering is included.

Each folder contains the files generating the plots for the corresponding figure in the report.
The plot style is defined in `plotstyle_pelat.tex`.
Every subsequent .tex file defines one plot.
The `run_fig.sh` file present in each folder creates all plots defined in their folder.

## Baselines

The training data for PID-Lagrangian was obtained directly from [rlpyt](https://github.com/astooke/rlpyt). Similarly, the data for PPO and PPO-Lagrangian was generated with the [OpenAI starter agent kit](https://github.com/openai/safety-starter-agents). Please follow the installation steps from their github repository if you wish to reproduce those results.