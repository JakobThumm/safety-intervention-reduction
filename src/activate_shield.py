from config_presets import config

class ConfigShield:
    '''Singleton class containing the shield configuration for the engine
    Was created to bypass the gym interface that seems to filter config files when creating an environment'''
    __instance = None

    @staticmethod
    def get_instance():
        '''Get the current configuration.
        If there is none, the config is initialized according to the config.py file'''
        if ConfigShield.__instance is None:
            ConfigShield.__instance = ConfigShield()
            ConfigShield.__instance.update_config(config)
        return ConfigShield.__instance


    def update_config(self, cfg):
        '''Update the current configuration with a new config dict'''
        cfg = cfg["other"]
        self.VARIABLE_DEGEU_POUR_SHIELD = cfg["activate_shield"]
        self.REWARD_SHIELD = cfg["reward_shield"]
        self.CONSTRAIN_SHIELD = cfg["constrain_shield"]
        self.REWARD_PENALTY = cfg["reward_penalty"]
        self.N_TRIES_NEW_TRAJ = cfg["n_traj_new_tries"]
        self.MAX_TRIES_NEW_TRAJ = cfg["max_traj_new_tries"]

        if cfg["replacement_strat"] == "grid":
            self.REPLACEMENT_STRAT = 0
        elif cfg["replacement_strat"] == "random":
            self.REPLACEMENT_STRAT = 1
        elif cfg["replacement_strat"] == "opposite":
            self.REPLACEMENT_STRAT = 2

    def activate_shield(self, activate):
        '''Toggle shield activation'''
        self.VARIABLE_DEGEU_POUR_SHIELD = activate

# initialize config when this file is loaded
config_shield = ConfigShield.get_instance()