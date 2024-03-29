config_custom = dict(
    env=dict(
        id="Safexp-PointGoal1-v0", # is overriden in app.py
        obs_prev_cost=True,
        obs_version="default",
    ),
    sampler=dict(
        batch_T=128,
        batch_B=20,  # Might bust memory limits.
        max_decorrelation_steps=1000,
        eval_n_envs=3,
        eval_max_steps=1e4,
        eval_max_trajectories=100,
    ),
    algo=dict(
        discount=0.99,
        learning_rate=1e-4,
        value_loss_coeff=1.,
        entropy_loss_coeff=0,
        clip_grad_norm=1e4,
        gae_lambda=0.97,
        minibatches=1,
        epochs=8,
        ratio_clip=0.1,
        linear_lr_schedule=False,
        normalize_advantage=False,
        cost_discount=None,
        cost_gae_lambda=None,
        cost_value_loss_coeff=0.5,
        ep_cost_ema_alpha=0,  # 0 for hard update.
        objective_penalized=True,
        learn_c_value=True,
        penalty_init=0.,
        cost_limit=25,
        cost_scale=10,  # yes 10.
        normalize_cost_advantage=False,
        pid_Kp=0,
        pid_Ki=1,
        pid_Kd=0,
        pid_d_delay=1,
        pid_delta_p_ema_alpha=0.95,  # 0 for hard update
        pid_delta_d_ema_alpha=0.95,
        sum_norm=True,
        diff_norm=False,
        penalty_max=25,  # only if sum_norm=diff_norm=False
        step_cost_limit_steps=None,
        step_cost_limit_value=None,
        use_beta_kl=False,
        use_beta_grad=False,
        record_beta_kl=False,
        record_beta_grad=False,
        beta_max=10,
        beta_ema_alpha=0.9,
        beta_kl_epochs=1,
        reward_scale=1,
        lagrange_quadratic_penalty=False,
        quadratic_penalty_coeff=1,
    ),
    agent=dict(),
    model=dict(
        hidden_sizes=[256, 256],
        lstm_size=256,
        lstm_skip=True,
        constraint=True,  # must match algo.learn_c_value
        normalize_observation=True,
        var_clip=1e-6,
    ),
    runner=dict(
        n_steps=1e7,
        log_interval_steps=1e5,
        seed=0, # overriden by the n_seeds parameter below
        cost_limit_linear_decrease=False,
        cost_limit_end_value=25,
    ),
    slot_affinity_code="0slt_0gpu_3cpu_3cpr_0saf",
    log_dir="test",
    run_ID="0",
    use_lstm=False,
    other =dict(
        activate_shield = True,
        reward_shield = False,
        reward_penalty = 0.1,
        constrain_shield = False,
        n_traj_new_tries = 0,
        n_seeds = 15,
    ),
)
