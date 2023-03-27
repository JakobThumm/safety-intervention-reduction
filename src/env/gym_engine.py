from mujoco_py import MjViewer, MujocoException, const, MjRenderContextOffscreen
from src.activate_shield import config_shield
from safety_gym.envs.engine import Engine, ResamplingError
import numpy as np

from safety_shield_py import SafetyShield, Motion, GymTrajPlanner


# Distinct colors for different types of objects.
# For now this is mostly used for visualization.
# This also affects the vision observation, so if training from pixels.
COLOR_BOX = np.array([1, 1, 0, 1])
COLOR_BUTTON = np.array([1, .5, 0, 1])
COLOR_GOAL = np.array([0, 1, 0, 1])
COLOR_VASE = np.array([0, 1, 1, 1])
COLOR_HAZARD = np.array([0, 0, 1, 1])
COLOR_PILLAR = np.array([.5, .5, 1, 1])
COLOR_WALL = np.array([.5, .5, .5, 1])
COLOR_GREMLIN = np.array([0.5, 0, 1, 1])
COLOR_CIRCLE = np.array([0, 1, 0, 1])
COLOR_RED = np.array([1, 0, 0, 1])
COLOR_SHIELD = np.array([1, 1, 1, 1])
COLOR_TRAJ = np.array([1, .5, 0, 1])

# Groups are a mujoco-specific mechanism for selecting which geom objects to "see"
# We use these for raycasting lidar, where there are different lidar types.
# These work by turning "on" the group to see and "off" all the other groups.
# See obs_lidar_natural() for more.
GROUP_GOAL = 0
GROUP_BOX = 1
GROUP_BUTTON = 1
GROUP_WALL = 2
GROUP_PILLAR = 2
GROUP_HAZARD = 3
GROUP_VASE = 4
GROUP_GREMLIN = 5
GROUP_CIRCLE = 6

# Constant for origin of world
ORIGIN_COORDINATES = np.zeros(3)

# Constant defaults for rendering frames for humans (not used for vision)
DEFAULT_WIDTH = 256
DEFAULT_HEIGHT = 256

RADIUS_HAZARD = 0.45 # 32 for PID
RADIUS_GREMLIN = 0.20
RADIUS_VASE = 0.35
RADIUS_PILLAR = 0.32


class NewEngine(Engine):
    def __init__(self, config={}) -> None:
        super().__init__(config)


        self.robot_type = config["robot_base"].split('/')[-1][:-4]

        self.use_safety_shield = config_shield.VARIABLE_DEGEU_POUR_SHIELD
        self.constrain_shield = config_shield.CONSTRAIN_SHIELD
        self.safety_shield = None
        self.desired_motion = None
        self.traj_plan = None
        self.plan_trajectory = None

        self.gremlins_pos = []
        self.robot_capsules = []
        self.obstacle_capsules = []
        self.failsafe_intervention = False

    def init_shield(self):
        '''Initializes the trajectory planner and the safety shield'''
        if not self.use_safety_shield:
            return

        if self.robot_type == "point":
            sample_time = 0.002
        elif self.robot_type == "car":
            sample_time = 0.004
        else:
            raise ValueError(self.robot_type)

        obstacle_pos = []
        obstacle_radius = []
        vases_pos_ = self.vases_pos
        obstacle_pos.extend(vases_pos_)
        obstacle_radius.extend([RADIUS_VASE for _ in vases_pos_])

        pillars_pos_ = self.pillars_pos
        obstacle_pos.extend(pillars_pos_)
        obstacle_radius.extend([RADIUS_PILLAR for _ in pillars_pos_])

        hazards_pos_ = self.hazards_pos
        obstacle_pos.extend(hazards_pos_)
        obstacle_radius.extend([RADIUS_HAZARD for _ in hazards_pos_])
        obstacle_moves = [False for _ in obstacle_pos]

        self.traj_plan = GymTrajPlanner(300, sample_time, [obstacle[:2] for obstacle in obstacle_pos], obstacle_radius, config_shield.N_TRIES_NEW_TRAJ, config_shield.MAX_TRIES_NEW_TRAJ, 0.25, config_shield.REPLACEMENT_STRAT)
        gremlins_pos_ = self.gremlins_obj_pos

        obstacle_pos.extend(gremlins_pos_)
        obstacle_radius.extend([RADIUS_GREMLIN for _ in gremlins_pos_])
        obstacle_moves.extend([True for _ in gremlins_pos_])

        obstacle_pos = np.array(obstacle_pos)[:, :-1]

        self.safety_shield = SafetyShield(
            activate_shield=True,
            sample_time=sample_time,
            trajectory_config_file="configs/trajectory_parameters_" + self.robot_type + ".yaml",
            robot_config_file="configs/robot_parameters_" + self.robot_type + ".yaml",
            init_x=0,
            init_y=0,
            init_z=0,
            init_roll=0,
            init_pitch=0,
            init_yaw=0,
            init_qpos=[0, 0],
            obstacle_pos=list(obstacle_pos.flatten()),
            obstacle_r=obstacle_radius,
            obstacle_moves=obstacle_moves,
            obstacle_vmax=0.1
        )
        if self.robot_type == "point":
            self.plan_trajectory = self.traj_plan.planner_point
            self.slowdown = self.traj_plan.point_slowdown
        elif self.robot_type == "car":
            self.plan_trajectory = self.traj_plan.planner_car
            self.slowdown = self.traj_plan.car_slowdown
        else:
            raise ValueError(self.robot_type)

    def reset(self):
        '''Resets shield and environment'''
        self.failsafe_intervention = False
        obs = super().reset()
        if self.use_safety_shield:
            self.init_shield()
        return obs

    def set_obstacle_measurement(self):
        '''Set new obstacle position in the safety shield'''
        if not self.use_safety_shield:
            return

        obstacle_pos = []
        vases_pos_ = self.vases_pos
        obstacle_pos.extend(vases_pos_)
        pillars_pos_ = self.pillars_pos
        obstacle_pos.extend(pillars_pos_)
        hazards_pos_ = self.hazards_pos
        obstacle_pos.extend(hazards_pos_)
        gremlins_pos_ = self.gremlins_obj_pos
        obstacle_pos.extend(gremlins_pos_)

        obstacle_pos = np.array(obstacle_pos)[:, :-1]
        self.safety_shield.setObstacleMeasurement(obstacle_pos.flatten(), self.data.time)

    def set_mocaps(self):
        ''' Set mocap object positions before a physics step is executed '''
        gremlins_pos_ = []
        if self.gremlins_num:  # self.constrain_gremlins:
            phase = float(self.data.time)
            for i in range(self.gremlins_num):
                name = f'gremlin{i}'
                target = np.array([np.sin(phase), np.cos(phase)]) * self.gremlins_travel
                pos = np.r_[target, [self.gremlins_size]]
                self.data.set_mocap_pos(name + 'mocap', pos)
                gremlins_pos_.append(np.array([np.sin(phase), np.cos(phase), 0]) * self.gremlins_travel)
        self.gremlins_pos = gremlins_pos_

    def set_action(self, action):
        '''Clip action in the right range'''
        action_range = self.model.actuator_ctrlrange
        # action_scale = action_range[:,1] - action_range[:, 0]
        self.data.ctrl[:] = np.clip(action, action_range[:, 0], action_range[:, 1])  # np.clip(action * 2 / action_scale, -1, 1)
        if self.action_noise:
            self.data.ctrl[:] += self.action_noise * self.rs.randn(self.model.nu)

    def step(self, action):
        ''' Take a step and return observation, reward, done, and info '''
        # action = [0,0]
        action = np.array(action, copy=False)  # Cast to ndarray
        assert not self.done, 'Environment must be reset before stepping'

        info = {}
        self.failsafe_intervention = False

        # check planned trajectory and select new action if needed 
        # don't need to set the trajectory in the shield since it will get checked again later
        if self.use_safety_shield and config_shield.N_TRIES_NEW_TRAJ > 0: 
            self.motion_list = self.plan_trajectory(action, self.world.robot_vel()[:2], self.world.robot_mat()[:2, :2], self.world.body_com("robot")[:2], True)
            action = self.traj_plan.get_action()

        # Set action
        self.set_action(action)

        

        # Simulate physics forward
        exception = False
        failsafe_intervention = False
        for _ in range(self.rs.binomial(self.frameskip_binom_n, self.frameskip_binom_p)):
        # for _ in range(1):
            try:
                if self.use_safety_shield and not self.failsafe_intervention:
                    # plan long term trajectory
                    motion_list = self.plan_trajectory(action, self.world.robot_vel()[:2], self.world.robot_mat()[:2, :2], self.world.body_com("robot")[:2], False)
                    action = self.traj_plan.get_action()
                    self.safety_shield.newLongTermTrajectoryFromMotion(motion_list)

                    # check safety
                    motion = self.safety_shield.step(self.data.time, Motion(self.data.time, self.world.body_com("robot")[:2]))
                    if self.safety_shield.getSafety() is False:
                        self.failsafe_intervention = True
                        # print("Failsafe")
                        info["shield_active"] = 1

                if self.failsafe_intervention:
                    action = self.slowdown(self.world.robot_vel()[:2], action[0], action[1], self.world.robot_mat()[:2, :2])
                
                self.set_action(action)

                self.set_mocaps()
                self.set_obstacle_measurement()
                self.sim.step()  # Physics simulation step

            except MujocoException as me:
                print('MujocoException', me)
                exception = True
                break
        if exception:
            self.done = True
            reward = self.reward_exception
            info['cost_exception'] = 1.0
        else:
            self.sim.forward()  # Needed to get sensor readings correct!

            # Reward processing
            reward = self.reward()
            if config_shield.REWARD_SHIELD and self.failsafe_intervention:
                reward += config_shield.REWARD_PENALTY

            # Constraint violations
            info.update(self.cost())

            # Button timer (used to delay button resampling)
            self.buttons_timer_tick()

            # Goal processing
            if self.goal_met():
                info['goal_met'] = True
                reward += self.reward_goal
                if self.continue_goal:
                    # Update the internal layout so we can correctly resample (given objects have moved)
                    self.update_layout()
                    # Reset the button timer (only used for task='button' environments)
                    self.buttons_timer = self.buttons_resampling_delay
                    # Try to build a new goal, end if we fail
                    if self.terminate_resample_failure:
                        try:
                            self.build_goal()
                        except ResamplingError as e:
                            # Normal end of episode
                            self.done = True
                    else:
                        # Try to make a goal, which could raise a ResamplingError exception
                        self.build_goal()
                else:
                    self.done = True

        # Timeout
        self.steps += 1
        if self.steps >= self.num_steps:
            self.done = True  # Maximum number of steps in an episode reached

        # self.render()
        return self.obs(), reward, self.done, info

    def render_shield(self):
        '''Make obstacle reachable set visible'''
        min_dist = 9999999
        pos_robot = self.world.body_com("robot")[:2]
        for cap in self.safety_shield.getObstacleReachCylinders():
            # min_dist = min(min_dist, np.linalg.norm(pos_robot-cap[:2]))
            self.viewer.add_marker(
                pos=cap[:3],
                size=np.array([1, 1, 0]) * cap[6] + np.array([0, 0, 0.01]),
                type=const.GEOM_CYLINDER,
                rgba=COLOR_SHIELD,
                label=""
            )
        # print(f"Closest obstacle is at {min_dist} meters.")
        # print(len(self.motion_list))
        # if len(self.motion_list) == 300:
        #     for futuremotion in self.motion_list:
        #         vel = futuremotion.getVelocity()
        #         act = futuremotion.getAction()
        #         # print(vel[0]*vel[0] + vel[1]*vel[1])
        #         print(f"{vel[0]}, {vel[1]}, {act[0]}, {act[1]}")
        #     raise KeyboardInterrupt
        for futuremotion in self.motion_list:
            pos = futuremotion.getPos()
            self.viewer.add_marker(
                pos= [pos[0], pos[1], 0],
                size=np.array([1, 1, 0]) * 0.3 + np.array([0, 0, 0.01]),
                type=const.GEOM_CYLINDER,
                rgba=COLOR_TRAJ,
                label=""
            )
            
        

    def render(self,
               mode='human',
               camera_id=None,
               width=DEFAULT_WIDTH,
               height=DEFAULT_HEIGHT
               ):
        ''' Render the environment to the screen '''
        if self.viewer is None or mode != self._old_render_mode:
            # Set camera if specified
            if mode == 'human':
                self.viewer = MjViewer(self.sim)
                self.viewer.cam.fixedcamid = -1
                self.viewer.cam.type = const.CAMERA_FREE
            else:
                self.viewer = MjRenderContextOffscreen(self.sim)
                self.viewer._hide_overlay = True
                self.viewer.cam.fixedcamid = camera_id  # self.model.camera_name2id(mode)
                self.viewer.cam.type = const.CAMERA_FIXED
            self.viewer.render_swap_callback = self.render_swap_callback
            # Turn all the geom groups on
            self.viewer.vopt.geomgroup[:] = 1
            self._old_render_mode = mode
        self.viewer.update_sim(self.sim)

        if camera_id is not None:
            # Update camera if desired
            self.viewer.cam.fixedcamid = camera_id

        # Lidar markers
        if self.render_lidar_markers:
            offset = self.render_lidar_offset_init  # Height offset for successive lidar indicators
            if 'box_lidar' in self.obs_space_dict or 'box_compass' in self.obs_space_dict:
                if 'box_lidar' in self.obs_space_dict:
                    self.render_lidar([self.box_pos], COLOR_BOX, offset, GROUP_BOX)
                if 'box_compass' in self.obs_space_dict:
                    self.render_compass(self.box_pos, COLOR_BOX, offset)
                offset += self.render_lidar_offset_delta
            if 'goal_lidar' in self.obs_space_dict or 'goal_compass' in self.obs_space_dict:
                if 'goal_lidar' in self.obs_space_dict:
                    self.render_lidar([self.goal_pos], COLOR_GOAL, offset, GROUP_GOAL)
                if 'goal_compass' in self.obs_space_dict:
                    self.render_compass(self.goal_pos, COLOR_GOAL, offset)
                offset += self.render_lidar_offset_delta
            if 'buttons_lidar' in self.obs_space_dict:
                self.render_lidar(self.buttons_pos, COLOR_BUTTON, offset, GROUP_BUTTON)
                offset += self.render_lidar_offset_delta
            if 'circle_lidar' in self.obs_space_dict:
                self.render_lidar([ORIGIN_COORDINATES], COLOR_CIRCLE, offset, GROUP_CIRCLE)
                offset += self.render_lidar_offset_delta
            if 'walls_lidar' in self.obs_space_dict:
                self.render_lidar(self.walls_pos, COLOR_WALL, offset, GROUP_WALL)
                offset += self.render_lidar_offset_delta
            if 'hazards_lidar' in self.obs_space_dict:
                self.render_lidar(self.hazards_pos, COLOR_HAZARD, offset, GROUP_HAZARD)
                offset += self.render_lidar_offset_delta
            if 'pillars_lidar' in self.obs_space_dict:
                self.render_lidar(self.pillars_pos, COLOR_PILLAR, offset, GROUP_PILLAR)
                offset += self.render_lidar_offset_delta
            if 'gremlins_lidar' in self.obs_space_dict:
                self.render_lidar(self.gremlins_obj_pos, COLOR_GREMLIN, offset, GROUP_GREMLIN)
                offset += self.render_lidar_offset_delta
            if 'vases_lidar' in self.obs_space_dict:
                self.render_lidar(self.vases_pos, COLOR_VASE, offset, GROUP_VASE)
                offset += self.render_lidar_offset_delta

        if self.use_safety_shield:
            self.render_shield()

        # Add goal marker
        if self.task == 'button':
            self.render_area(self.goal_pos, self.buttons_size * 2, COLOR_BUTTON, 'goal', alpha=0.1)

        # Add indicator for nonzero cost
        if self._cost.get('cost', 0) > 0:
            self.render_sphere(self.world.robot_pos(), 0.25, COLOR_RED, alpha=.5)

        if mode == 'human':
            self.viewer.render()
        elif mode == 'rgb_array':
            self.viewer.render(width, height)
            data = self.viewer.read_pixels(width, height, depth=False)
            self.viewer._markers[:] = []
            self.viewer._overlay.clear()
            return data[::-1, :, :]

    def cost(self):
        ''' Calculate the current costs and return a dict '''
        self.sim.forward()  # Ensure positions and contacts are correct
        cost = {}
        # Conctacts processing
        if self.constrain_shield:
            cost['cost_shield'] = 0
        if self.constrain_vases:
            cost['cost_vases_contact'] = 0
        if self.constrain_pillars:
            cost['cost_pillars'] = 0
        if self.constrain_buttons:
            cost['cost_buttons'] = 0
        if self.constrain_gremlins:
            cost['cost_gremlins'] = 0
        buttons_constraints_active = self.constrain_buttons and (self.buttons_timer == 0)
        for contact in self.data.contact[:self.data.ncon]:
            geom_ids = [contact.geom1, contact.geom2]
            geom_names = sorted([self.model.geom_id2name(g) for g in geom_ids])
            if self.constrain_vases and any(n.startswith('vase') for n in geom_names):
                if any(n in self.robot.geom_names for n in geom_names):
                    cost['cost_vases_contact'] += self.vases_contact_cost
            if self.constrain_pillars and any(n.startswith('pillar') for n in geom_names):
                if any(n in self.robot.geom_names for n in geom_names):
                    cost['cost_pillars'] += self.pillars_cost
            if buttons_constraints_active and any(n.startswith('button') for n in geom_names):
                if any(n in self.robot.geom_names for n in geom_names):
                    if not any(n == f'button{self.goal_button}' for n in geom_names):
                        cost['cost_buttons'] += self.buttons_cost
            if self.constrain_gremlins and any(n.startswith('gremlin') for n in geom_names):
                if any(n in self.robot.geom_names for n in geom_names):
                    cost['cost_gremlins'] += self.gremlins_contact_cost

        # Displacement processing
        if self.constrain_vases and self.vases_displace_cost:
            cost['cost_vases_displace'] = 0
            for i in range(self.vases_num):
                name = f'vase{i}'
                dist = np.sqrt(np.sum(np.square(self.data.get_body_xpos(name)[:2] - self.reset_layout[name])))
                if dist > self.vases_displace_threshold:
                    cost['cost_vases_displace'] += dist * self.vases_displace_cost

        # Velocity processing
        if self.constrain_vases and self.vases_velocity_cost:
            # TODO: penalize rotational velocity too, but requires another cost coefficient
            cost['cost_vases_velocity'] = 0
            for i in range(self.vases_num):
                name = f'vase{i}'
                vel = np.sqrt(np.sum(np.square(self.data.get_body_xvelp(name))))
                if vel >= self.vases_velocity_threshold:
                    cost['cost_vases_velocity'] += vel * self.vases_velocity_cost

        # Calculate constraint violations
        if self.constrain_hazards:
            cost['cost_hazards'] = 0
            for h_pos in self.hazards_pos:
                h_dist = self.dist_xy(h_pos)
                if h_dist <= self.hazards_size:
                    cost['cost_hazards'] += self.hazards_cost * (self.hazards_size - h_dist)

        if self.constrain_shield and self.failsafe_intervention:
            cost['cost_shield'] = 1

        # Sum all costs into single total cost
        cost['cost'] = sum(v for k, v in cost.items() if k.startswith('cost_'))

        # Optionally remove shaping from reward functions.
        if self.constrain_indicator:
            for k in list(cost.keys()):
                cost[k] = float(cost[k] > 0.0)  # Indicator function

        self._cost = cost

        return cost
