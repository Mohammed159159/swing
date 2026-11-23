function res = simulate_simscape(params, x0, t_grid, u_spec, opts)
%SIMULATE_SIMSCAPE Non-destructive Simulink.SimulationInput runner for Simscape plant.
%   res = SIMULATE_SIMSCAPE(params, x0, t_grid)
%   res = SIMULATE_SIMSCAPE(params, x0, t_grid, u_spec, Name=Value)
%
%   Configures and executes a Simscape Multibody model purely through
%   Simulink.SimulationInput (never modifying the .slx file), injects initial
%   joint targets x0 = [theta0; dtheta0; theta1; dtheta1], optionally passes
%   external voltage Va(t) via a Dataset, and returns trajectories resampled
%   onto t_grid in the standardized Furuta res struct.
%
%   Name-Value Options:
%     ModelName        - Simulink model name without .slx (default: "robot_compatible")
%     ArmJointBlock    - Revolute Joint block name for theta0 (default: "theta_arm")
%     PendJointBlock   - Revolute Joint block name for theta1 (default: "theta_pendulum")
%     Theta0Offset     - Angle offset [rad]: q_ss = Theta0Sign*theta0 + Theta0Offset (default: 0)
%     Theta1Offset     - Angle offset [rad]: q_ss = Theta1Sign*theta1 + Theta1Offset (default: 0)
%     Theta0Sign       - Joint axis polarity (+1 or -1) relative to analytical theta0 (default: +1)
%     Theta1Sign       - Joint axis polarity (+1 or -1) relative to analytical theta1 (default: +1)
%     UseExternalInput - Logical override for Dataset external input (default: [] -> auto)
%     OverrideDamping  - Override Revolute Joint DampingCoefficient from params (default: false)
%     Saturate         - Clip Va to [-params.V_max, +params.V_max] (default: true)
%     FastRestart      - Enable FastRestart in sim() (default: false)

    arguments
        params (1,1) struct
        x0 (:,1) double
        t_grid (:,1) double
        u_spec = 0
        opts.ModelName (1,1) string = "robot_compatible"
        opts.ArmJointBlock (1,1) string = "theta_arm"
        opts.PendJointBlock (1,1) string = "theta_pendulum"
        opts.Theta0Offset (1,1) double = 0.0
        opts.Theta1Offset (1,1) double = 0.0
        opts.Theta0Sign (1,1) double {mustBeMember(opts.Theta0Sign, [-1, 1])} = 1.0
        opts.Theta1Sign (1,1) double {mustBeMember(opts.Theta1Sign, [-1, 1])} = 1.0
        opts.UseExternalInput logical = logical.empty()
        opts.OverrideDamping (1,1) logical = false
        opts.Saturate (1,1) logical = true
        opts.FastRestart (1,1) logical = false
    end

    tStart = tic;
    [x0, t_grid, u_info] = resolve_input_signal(x0, t_grid, u_spec, false);

    u_raw = u_info.u_vec;
    if opts.Saturate && isfield(params, 'V_max') && isscalar(params.V_max) && params.V_max > 0
        satMask = abs(u_raw) > params.V_max;
        u_app   = max(-params.V_max, min(params.V_max, u_raw));
        satFrac = mean(satMask);
    else
        u_app   = u_raw;
        satFrac = 0.0;
    end

    if isempty(opts.UseExternalInput)
        useExtIn = any(u_app ~= 0);
    else
        useExtIn = opts.UseExternalInput;
    end

    modelName = char(opts.ModelName);
    armPath   = sprintf('%s/%s', modelName, char(opts.ArmJointBlock));
    pendPath  = sprintf('%s/%s', modelName, char(opts.PendJointBlock));

    in = Simulink.SimulationInput(modelName);
    in = in.setVariable('params', params);
    in = in.setModelParameter( ...
        'StartTime',       num2str(t_grid(1),   '%.12g'), ...
        'StopTime',        num2str(t_grid(end), '%.12g'), ...
        'SimscapeLogType', 'all', ...
        'SimscapeLogName', 'simlog');

    % Inject initial joint positions and velocities (in rad and rad/s)
    % Mapping: q_ss = sign * theta + offset, w_ss = sign * dtheta
    q0_ss = opts.Theta0Sign * x0(1) + opts.Theta0Offset;
    w0_ss = opts.Theta0Sign * x0(2);
    q1_ss = opts.Theta1Sign * x0(3) + opts.Theta1Offset;
    w1_ss = opts.Theta1Sign * x0(4);
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

    [x_out, u_out, logSource] = extractSimscapeStates(out, t_grid, u_app, opts);

    res = struct();
    res.t = t_grid(:);
    res.x = x_out;
    res.y = x_out(:, [1, 3]);
    res.u = u_out(:);
    res.meta = struct( ...
        'runner',             'simulate_simscape', ...
        'model',              modelName, ...
        'equilibrium',        'none', ...
        'x0',                 x0, ...
        'input_mode',         u_info.mode, ...
        'used_external_input', useExtIn, ...
        'log_source',         logSource, ...
        'saturated',          opts.Saturate, ...
        'sat_fraction',       satFrac, ...
        'solver',             'simscape', ...
        'elapsed_s',          toc(tStart));
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

function [x_grid, u_grid, logSource] = extractSimscapeStates(out, t_grid, u_app, opts)
    loggedProps = who(out);

    % Prefer explicit logsout signals when wired in the Simscape model
    if any(strcmp(loggedProps, 'logsout'))
        names = out.logsout.getElementNames();
        required = {'theta0', 'dtheta0', 'theta1', 'dtheta1'};
        if all(ismember(required, names))
            th0  = opts.Theta0Sign * (resampleSignal(out.logsout.get('theta0').Values,  t_grid) - opts.Theta0Offset);
            dth0 = opts.Theta0Sign *  resampleSignal(out.logsout.get('dtheta0').Values, t_grid);
            th1  = opts.Theta1Sign * (resampleSignal(out.logsout.get('theta1').Values,  t_grid) - opts.Theta1Offset);
            dth1 = opts.Theta1Sign *  resampleSignal(out.logsout.get('dtheta1').Values, t_grid);
            x_grid = [th0, dth0, th1, dth1];
            if ismember('Va', names)
                u_grid = resampleSignal(out.logsout.get('Va').Values, t_grid);
            else
                u_grid = u_app;
            end
            logSource = 'logsout';
            return;
        end
    end

    % Fallback to Simscape physical node logging (out.simlog)
    if ~any(strcmp(loggedProps, 'simlog'))
        error('Furuta:MissingSimscapeLog', ...
            'Neither standard logsout signals nor simlog were found in SimulationOutput.');
    end

    armNode  = out.simlog.(matlab.lang.makeValidName(char(opts.ArmJointBlock)));
    pendNode = out.simlog.(matlab.lang.makeValidName(char(opts.PendJointBlock)));

    t_q0 = armNode.Rz.q.series.time;
    q0   = opts.Theta0Sign * (armNode.Rz.q.series.values('rad') - opts.Theta0Offset);
    w0   = opts.Theta0Sign *  armNode.Rz.w.series.values('rad/s');

    t_q1 = pendNode.Rz.q.series.time;
    q1   = opts.Theta1Sign * (pendNode.Rz.q.series.values('rad') - opts.Theta1Offset);
    w1   = opts.Theta1Sign *  pendNode.Rz.w.series.values('rad/s');

    th0  = resampleSeries(t_q0, q0, t_grid);
    dth0 = resampleSeries(t_q0, w0, t_grid);
    th1  = resampleSeries(t_q1, q1, t_grid);
    dth1 = resampleSeries(t_q1, w1, t_grid);

    x_grid    = [th0, dth0, th1, dth1];
    u_grid    = u_app;
    logSource = 'simlog';
end

function y = resampleSignal(ts, t_grid)
    y = resampleSeries(ts.Time, ts.Data, t_grid);
end

function y = resampleSeries(t_src, v_src, t_grid)
    [t_u, idx] = unique(double(t_src(:)), 'last');
    v_u        = double(v_src(idx));
    y          = interp1(t_u, v_u, t_grid, 'linear', 'extrap');
    y          = y(:);
end
