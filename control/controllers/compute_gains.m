function gains = compute_gains(plant, params, opts)
%COMPUTE_GAINS Modular controller gain orchestrator for the Furuta pendulum.
%   gains = COMPUTE_GAINS()
%   gains = COMPUTE_GAINS(plant, params)
%   gains = COMPUTE_GAINS(plant, params, Name=Value)
%
%   Delegates synthesis to each controller's dedicated design function:
%     - gains.pid     <- pid_design(plant, params, ...)
%     - gains.pp      <- pole_placement_design(plant, params, ...)
%     - gains.lqr     <- lqr_design(plant, params, ...)
%     - gains.swingup <- swingup_design(plant, params, ...)
%
%   Optional Name-Value cell arrays pass custom overrides to each sub-designer:
%     PIDArgs     - Cell array of Name-Value args for pid_design
%     PPArgs      - Cell array of Name-Value args for pole_placement_design
%     LQRArgs     - Cell array of Name-Value args for lqr_design
%     SwingUpArgs - Cell array of Name-Value args for swingup_design
%
%   Example:
%       gains = compute_gains(plant, params);
%       gains = compute_gains(plant, params, LQRArgs={'ControlMax', 4.0});

    arguments
        plant struct = struct()
        params struct = struct()
        opts.PIDArgs cell = {}
        opts.PPArgs cell = {}
        opts.LQRArgs cell = {}
        opts.SwingUpArgs cell = {}
    end

    if isempty(fieldnames(params))
        params = load_parameters();
    end
    if isempty(fieldnames(plant))
        plant = build_plant(params);
    end

    gains = struct();
    gains.pid     = pid_design(plant, params, opts.PIDArgs{:});
    gains.pp      = pole_placement_design(plant, params, opts.PPArgs{:});
    gains.lqr     = lqr_design(plant, params, opts.LQRArgs{:});
    gains.swingup = swingup_design(plant, params, opts.SwingUpArgs{:});

    % Simscape Multibody coordinate convention (robot_compatible / furuta.slx)
    gains.simscape = struct( ...
        'Theta0Sign',   1.0, ...
        'Theta0Offset', 0.0, ...
        'Theta1Sign',  -1.0, ...
        'Theta1Offset', pi);

    gains.meta = struct( ...
        'state_order',      {{'theta0', 'dtheta0', 'theta1', 'dtheta1'}}, ...
        'Ts',               params.Ts, ...
        'V_max',            params.V_max, ...
        'stabilizer_codes', struct('pid', 1, 'pole_placement', 2, 'lqr', 3));
end
