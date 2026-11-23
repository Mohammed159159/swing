function res = simulate_simscape(params, x0, t_grid, u_spec, opts)
%SIMULATE_SIMSCAPE Non-destructive Simulink.SimulationInput runner for Simscape models.
%   res = SIMULATE_SIMSCAPE(params, x0, t_grid)
%   res = SIMULATE_SIMSCAPE(params, x0, t_grid, u_spec, Name=Value)
%
%   Configures and executes a Simscape Multibody model purely through
%   Simulink.SimulationInput (never modifying the .slx file), injects initial
%   joint targets x0 = [theta0; dtheta0; theta1; dtheta1], optionally passes
%   external open-loop voltage Va(t) via a Dataset OR selects a closed-loop
%   controller mode on furuta_closed_loop.slx, and returns trajectories
%   resampled onto t_grid in the standardized Furuta res struct.
%
%   Name-Value Options:
%     ModelName        - Simulink model name without .slx (default: "robot_compatible")
%     ArmJointBlock    - Revolute Joint relative path (default: "" -> auto-detect Plant/theta_arm or theta_arm)
%     PendJointBlock   - Revolute Joint relative path (default: "" -> auto-detect Plant/theta_pendulum or theta_pendulum)
%     Theta0Offset     - Angle offset [rad]: q_ss = Theta0Sign*theta0 + Theta0Offset (default: [] -> auto)
%     Theta1Offset     - Angle offset [rad]: q_ss = Theta1Sign*theta1 + Theta1Offset (default: [] -> auto)
%     Theta0Sign       - Joint axis polarity (+1 or -1) relative to analytical theta0 (default: [] -> auto)
%     Theta1Sign       - Joint axis polarity (+1 or -1) relative to analytical theta1 (default: [] -> auto)
%     UseExternalInput - Logical override for Dataset external input (default: [] -> auto)
%     OverrideDamping  - Override Revolute Joint DampingCoefficient from params (default: false)
%     Saturate         - Clip open-loop Va to [-params.V_max, +params.V_max] (default: true)
%     FastRestart      - Enable FastRestart in sim() (default: false)
%     Stabilizer       - Closed-loop stabilizer: "none" (default), "pid", "pole_placement", "lqr"
%     ControlMode      - Outer mode: "open_loop" (default), "stabilize", "swingup", "auto_catch"
%     Gains            - Gains struct from compute_gains(plant, params) (default: [] -> auto when closed-loop)
%     Reference        - 4x1 analytical reference x_ref (default: zeros(4,1))
%     UseZOH           - Enable 500 Hz ZOH sensor sampling at params.Ts (default: false)

    arguments
        params (1,1) struct
        x0 (:,1) double
        t_grid (:,1) double
        u_spec = 0
        opts.ModelName (1,1) string = "robot_compatible"
        opts.ArmJointBlock (1,1) string = ""
        opts.PendJointBlock (1,1) string = ""
        opts.Theta0Offset double = []
        opts.Theta1Offset double = []
        opts.Theta0Sign double = []
        opts.Theta1Sign double = []
        opts.UseExternalInput logical = logical.empty()
        opts.OverrideDamping (1,1) logical = false
        opts.Saturate (1,1) logical = true
        opts.FastRestart (1,1) logical = false
        opts.Stabilizer (1,1) string {mustBeMember(opts.Stabilizer, ["none", "pid", "pole_placement", "lqr"])} = "none"
        opts.ControlMode (1,1) string {mustBeMember(opts.ControlMode, ["open_loop", "stabilize", "swingup", "auto_catch"])} = "open_loop"
        opts.Gains = []
        opts.Reference (4,1) double = zeros(4, 1)
        opts.UseZOH (1,1) logical = false
    end

    tStart = tic;
    [x0, t_grid, u_info] = resolve_input_signal(x0, t_grid, u_spec, false);

    % Determine whether running in closed-loop controller mode
    isClosedLoop = (opts.ControlMode ~= "open_loop") || (opts.Stabilizer ~= "none") || ...
                   strcmpi(opts.ModelName, "furuta_closed_loop");

    if isClosedLoop
        if strcmpi(opts.ModelName, "robot_compatible") && (opts.ControlMode ~= "open_loop" || opts.Stabilizer ~= "none")
            opts.ModelName = "furuta_closed_loop";
        end
        if opts.ControlMode == "open_loop"
            opts.ControlMode = "stabilize";
        end
        if opts.Stabilizer == "none"
            opts.Stabilizer = "lqr";
        end
        if isempty(opts.Gains)
            plantTmp = build_plant(params);
            gains    = compute_gains(plantTmp, params);
        else
            gains    = opts.Gains;
            plantTmp = build_plant(params);
        end
    else
        gains    = [];
        plantTmp = [];
    end

    % Resolve coordinate mapping defaults
    if isClosedLoop && isstruct(gains) && isfield(gains, 'simscape')
        defTh0Sign = gains.simscape.Theta0Sign;
        defTh0Off  = gains.simscape.Theta0Offset;
        defTh1Sign = gains.simscape.Theta1Sign;
        defTh1Off  = gains.simscape.Theta1Offset;
    else
        defTh0Sign = 1.0;
        defTh0Off  = 0.0;
        defTh1Sign = 1.0;
        defTh1Off  = 0.0;
    end
    th0Sign = resolveScalarOpt(opts.Theta0Sign, defTh0Sign, [-1, 1]);
    th1Sign = resolveScalarOpt(opts.Theta1Sign, defTh1Sign, [-1, 1]);
    th0Off  = resolveScalarOpt(opts.Theta0Offset, defTh0Off, []);
    th1Off  = resolveScalarOpt(opts.Theta1Offset, defTh1Off, []);

    u_raw = u_info.u_vec;
    if opts.Saturate && isfield(params, 'V_max') && isscalar(params.V_max) && params.V_max > 0
        satMask = abs(u_raw) > params.V_max;
        u_app   = max(-params.V_max, min(params.V_max, u_raw));
        satFrac = mean(satMask);
    else
        u_app   = u_raw;
        satFrac = 0.0;
    end

    if isClosedLoop
        useExtIn = false;
    elseif isempty(opts.UseExternalInput)
        useExtIn = any(u_app ~= 0);
    else
        useExtIn = opts.UseExternalInput;
    end

    modelName = char(opts.ModelName);
    [armRel, pendRel] = resolveJointPaths(modelName, opts.ArmJointBlock, opts.PendJointBlock);
    armPath  = sprintf('%s/%s', modelName, armRel);
    pendPath = sprintf('%s/%s', modelName, pendRel);

    in = Simulink.SimulationInput(modelName);
    in = in.setVariable('params', params);

    if isClosedLoop
        stabCodes = struct('pid', 1, 'pole_placement', 2, 'lqr', 3);
        modeCodes = struct('stabilize', 1, 'swingup', 2, 'auto_catch', 3);
        % Sync coordinate mapping into gains.simscape so model adapter matches x0 injection
        gains.simscape.Theta0Sign   = th0Sign;
        gains.simscape.Theta0Offset = th0Off;
        gains.simscape.Theta1Sign   = th1Sign;
        gains.simscape.Theta1Offset = th1Off;

        in = in.setVariable('plant',           plantTmp);
        in = in.setVariable('gains',           gains);
        in = in.setVariable('stabilizer_mode', stabCodes.(char(opts.Stabilizer)));
        in = in.setVariable('ctrl_mode',       modeCodes.(char(opts.ControlMode)));
        in = in.setVariable('use_zoh',         double(opts.UseZOH));
        in = in.setVariable('x_ref',           opts.Reference(:));
    end

    in = in.setModelParameter( ...
        'StartTime',       num2str(t_grid(1),   '%.12g'), ...
        'StopTime',        num2str(t_grid(end), '%.12g'), ...
        'SimscapeLogType', 'all', ...
        'SimscapeLogName', 'simlog');

    % Inject initial joint positions and velocities (in rad and rad/s)
    % Mapping: q_ss = sign * theta + offset, w_ss = sign * dtheta
    q0_ss = th0Sign * x0(1) + th0Off;
    w0_ss = th0Sign * x0(2);
    q1_ss = th1Sign * x0(3) + th1Off;
    w1_ss = th1Sign * x0(4);
    in = setJointInitialTargets(in, armPath,  q0_ss, w0_ss);
    in = setJointInitialTargets(in, pendPath, q1_ss, w1_ss);

    if opts.OverrideDamping
        in = in.setBlockParameter(armPath,  'DampingCoefficient',      num2str(params.b0, '%.12g'));
        in = in.setBlockParameter(armPath,  'DampingCoefficientUnits', 'N*m/(rad/s)');
        in = in.setBlockParameter(pendPath, 'DampingCoefficient',      num2str(params.b1, '%.12g'));
        in = in.setBlockParameter(pendPath, 'DampingCoefficientUnits', 'N*m/(rad/s)');
    end

    if useExtIn
        ds = Simulink.SimulationData.Dataset;
        ds{1} = timeseries(u_app, t_grid, 'Name', 'Va');
        in = in.setExternalInput(ds);
    end

    if opts.FastRestart
        out = sim(in, 'UseFastRestart', 'on');
    else
        out = sim(in);
    end

    coordCfg = struct('Theta0Sign', th0Sign, 'Theta0Offset', th0Off, ...
                      'Theta1Sign', th1Sign, 'Theta1Offset', th1Off, ...
                      'ArmRel', armRel, 'PendRel', pendRel, ...
                      'IsClosedLoop', isClosedLoop);
    [x_out, u_out, mode_out, logSource] = extractSimscapeStates(out, t_grid, u_app, coordCfg);

    if isClosedLoop && isfield(params, 'V_max') && params.V_max > 0
        satFrac = mean(abs(u_out) >= (params.V_max - 1e-6));
    end

    res = struct();
    res.t = t_grid(:);
    res.x = x_out;
    res.y = x_out(:, [1, 3]);
    res.u = u_out(:);
    if ~isempty(mode_out)
        res.mode = mode_out(:);
    end
    res.meta = struct( ...
        'runner',              'simulate_simscape', ...
        'model',               modelName, ...
        'equilibrium',         'none', ...
        'x0',                  x0, ...
        'input_mode',          u_info.mode, ...
        'used_external_input', useExtIn, ...
        'stabilizer',          char(opts.Stabilizer), ...
        'control_mode',        char(opts.ControlMode), ...
        'use_zoh',             opts.UseZOH, ...
        'log_source',          logSource, ...
        'saturated',           opts.Saturate, ...
        'sat_fraction',        satFrac, ...
        'solver',              'simscape', ...
        'elapsed_s',           toc(tStart));
end

function val = resolveScalarOpt(optVal, defaultVal, allowedMembers)
    if isempty(optVal)
        val = defaultVal;
    else
        if ~isscalar(optVal) || ~isfinite(optVal)
            error('Furuta:InvalidOption', 'Coordinate sign/offset option must be a finite scalar.');
        end
        val = double(optVal);
    end
    if ~isempty(allowedMembers) && ~ismember(val, allowedMembers)
        error('Furuta:InvalidOption', 'Coordinate sign must be +1 or -1.');
    end
end

function [armRel, pendRel] = resolveJointPaths(modelName, armOpt, pendOpt)
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end
    armRel  = resolveSingleJoint(modelName, char(armOpt),  {'Plant/theta_arm', 'theta_arm'});
    pendRel = resolveSingleJoint(modelName, char(pendOpt), {'Plant/theta_pendulum', 'theta_pendulum'});
end

function relPath = resolveSingleJoint(modelName, userPath, defaults)
    if ~isempty(userPath)
        cand = {userPath, ['Plant/' userPath]};
    else
        cand = defaults;
    end
    for k = 1:numel(cand)
        fullP = [modelName '/' cand{k}];
        if getSimulinkBlockHandle(fullP) ~= -1
            relPath = cand{k};
            return;
        end
    end
    % Fallback to first candidate (SimulationInput will report exact path if missing)
    relPath = cand{1};
end

function in = setJointInitialTargets(in, blkPath, q0, w0)
    in = in.setBlockParameter(blkPath, 'PositionTargetSpecify',    'on');
    in = in.setBlockParameter(blkPath, 'PositionTargetPriority',   'High');
    in = in.setBlockParameter(blkPath, 'PositionTargetValue',      num2str(q0, '%.12g'));
    in = in.setBlockParameter(blkPath, 'PositionTargetValueUnits', 'rad');
    in = in.setBlockParameter(blkPath, 'VelocityTargetSpecify',    'on');
    in = in.setBlockParameter(blkPath, 'VelocityTargetPriority',   'High');
    in = in.setBlockParameter(blkPath, 'VelocityTargetValue',      num2str(w0, '%.12g'));
    in = in.setBlockParameter(blkPath, 'VelocityTargetValueUnits', 'rad/s');
end

function [x_grid, u_grid, mode_grid, logSource] = extractSimscapeStates(out, t_grid, u_app, cfg)
    loggedProps = who(out);
    mode_grid   = [];

    % Prefer explicit logsout signals when wired in the Simscape model
    if any(strcmp(loggedProps, 'logsout'))
        names    = out.logsout.getElementNames();
        required = {'theta0', 'dtheta0', 'theta1', 'dtheta1'};
        if all(ismember(required, names))
            % Closed-loop model logs analytical signals directly (already sign/offset converted)
            alreadyAnalytical = cfg.IsClosedLoop || ismember('active_mode', names);
            if alreadyAnalytical
                th0  = resampleSignal(out.logsout.get('theta0').Values,  t_grid);
                dth0 = resampleSignal(out.logsout.get('dtheta0').Values, t_grid);
                th1  = resampleSignal(out.logsout.get('theta1').Values,  t_grid);
                dth1 = resampleSignal(out.logsout.get('dtheta1').Values, t_grid);
            else
                th0  = cfg.Theta0Sign * (resampleSignal(out.logsout.get('theta0').Values,  t_grid) - cfg.Theta0Offset);
                dth0 = cfg.Theta0Sign *  resampleSignal(out.logsout.get('dtheta0').Values, t_grid);
                th1  = cfg.Theta1Sign * (resampleSignal(out.logsout.get('theta1').Values,  t_grid) - cfg.Theta1Offset);
                dth1 = cfg.Theta1Sign *  resampleSignal(out.logsout.get('dtheta1').Values, t_grid);
            end
            x_grid = [th0, dth0, th1, dth1];
            if ismember('Va', names)
                u_grid = resampleSignal(out.logsout.get('Va').Values, t_grid);
            else
                u_grid = u_app;
            end
            if ismember('active_mode', names)
                mode_grid = round(resampleSignalStep(out.logsout.get('active_mode').Values, t_grid));
            end
            logSource = 'logsout';
            return;
        end
    end

    % Fallback to Simscape physical node logging (out.simlog), supporting nested paths
    if ~any(strcmp(loggedProps, 'simlog'))
        error('Furuta:MissingSimscapeLog', ...
            'Neither standard logsout signals nor simlog were found in SimulationOutput.');
    end

    armNode  = traverseSimlogNode(out.simlog, cfg.ArmRel);
    pendNode = traverseSimlogNode(out.simlog, cfg.PendRel);

    t_q0 = armNode.Rz.q.series.time;
    q0   = cfg.Theta0Sign * (armNode.Rz.q.series.values('rad') - cfg.Theta0Offset);
    w0   = cfg.Theta0Sign *  armNode.Rz.w.series.values('rad/s');

    t_q1 = pendNode.Rz.q.series.time;
    q1   = cfg.Theta1Sign * (pendNode.Rz.q.series.values('rad') - cfg.Theta1Offset);
    w1   = cfg.Theta1Sign *  pendNode.Rz.w.series.values('rad/s');

    th0  = resampleSeries(t_q0, q0, t_grid);
    dth0 = resampleSeries(t_q0, w0, t_grid);
    th1  = resampleSeries(t_q1, q1, t_grid);
    dth1 = resampleSeries(t_q1, w1, t_grid);

    x_grid    = [th0, dth0, th1, dth1];
    u_grid    = u_app;
    logSource = 'simlog';
end

function node = traverseSimlogNode(simlogRoot, relPath)
    parts = strsplit(relPath, '/');
    node  = simlogRoot;
    for k = 1:numel(parts)
        node = node.(matlab.lang.makeValidName(parts{k}));
    end
end

function y = resampleSignal(ts, t_grid)
    y = resampleSeries(ts.Time, ts.Data, t_grid);
end

function y = resampleSignalStep(ts, t_grid)
    [t_u, idx] = unique(double(ts.Time(:)), 'last');
    v_u        = double(ts.Data(idx));
    y          = interp1(t_u, v_u, t_grid, 'previous', 'extrap');
    y          = y(:);
end

function y = resampleSeries(t_src, v_src, t_grid)
    [t_u, idx] = unique(double(t_src(:)), 'last');
    v_u        = double(v_src(idx));
    y          = interp1(t_u, v_u, t_grid, 'linear', 'extrap');
    y          = y(:);
end
