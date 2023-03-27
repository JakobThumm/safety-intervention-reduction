CURRENT_UID=$(id -u):$(id -g) REPLACE=grid EXP_NAME=S_PPO docker stack deploy --compose-file docker-compose.yml safetyshield_pointshield
CURRENT_UID=$(id -u):$(id -g) REPLACE=grid EXP_NAME=S_PPO_Replacement docker stack deploy --compose-file docker-compose.yml safetyshield_pointnewgrid
# CURRENT_UID=$(id -u):$(id -g) REPLACE=grid EXP_NAME=S_PID docker stack deploy --compose-file docker-compose.yml safetyshield_pointconstrained
# CURRENT_UID=$(id -u):$(id -g) REPLACE=grid EXP_NAME=S_PID_Replacement docker stack deploy --compose-file docker-compose.yml safetyshield_pointcstrnew
CURRENT_UID=$(id -u):$(id -g) REPLACE=random EXP_NAME=S_PPO_Replacement docker stack deploy --compose-file docker-compose.yml safetyshield_pointnewrand
CURRENT_UID=$(id -u):$(id -g) REPLACE=opposite EXP_NAME=S_PPO_Replacement docker stack deploy --compose-file docker-compose.yml safetyshield_pointnewopp
CURRENT_UID=$(id -u):$(id -g) EXP_NAME=Constraint_Search docker stack deploy --compose-file docker-compose.yml safetyshield_pointnewopp
