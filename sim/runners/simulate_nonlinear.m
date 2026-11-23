function res = simulate_nonlinear(plant, params, x0, t_grid, u_spec, opts)
%SIMULATE_NONLINEAR Simulate full 3D nonlinear Furuta ODE via MATLAB ODE solvers.
%   res = SIMULATE_NONLINEAR(plant, params, x0, t_grid)
%   res = SIMULATE_NONLINEAR(plant, params, x0, t_grid, u_spec, Name=Value)
%
%   Supports open-loop voltage profiles (empty, scalar, Nx1 vector, @(t)),
%   custom closed-loop handles @(t, x), and built-in controllers selected via
%   Controller="pid"|"pole_placement"|"lqr"|"swingup" with +/- params.V_max
%   actuator saturation.
%
%   Inputs:
%     plant  - Plant struct returned by build_plant(params)
%     params - Parameter struct returned by load_parameters()
%     x0     - 4x1 global initial state [theta0; dtheta0; theta1; dtheta1]
%     t_grid - Nx1 strictly increasing time vector [s]
%     u_spec - Optional voltage input Va [V]: [], scalar, Nx1 vector, @(t), or @(t, x)
%
%   Name-Value Options:
%     Saturate    - Clip Va to [-params.V_max, +params.V_max] (default: true)
%     RelTol      - ODE solver relative tolerance (default: 1e-8)
%     AbsTol      - ODE solver absolute tolerance (default: 1e-10)
%     MaxStep     - Optional max ODE step size [s] (default: [])
%     Solver      - "ode45" (default), "ode15s", or "ode23"
%     InputInterp - Interpolation method for vector u_spec: "previous" (default) or "linear"
%     Controller  - "none" (default), "pid", "pole_placement", "lqr", or "swingup"
%     Gains       - Gains struct from compute_gains(plant, params) (auto if empty)
%     Reference   - 4x1 target state x_ref for closed-loop controllers (default: zeros(4,1))
%
%   Output (Standardized Furuta Trajectory Struct):
%     res.t    - Nx1 time vector [s]
%     res.x    - Nx4 global state trajectory [theta0, dtheta0, theta1, dtheta1]
%     res.y    - Nx2 output trajectory [theta0, theta1]
%     res.u    - Nx1 applied voltage Va [V] (post-saturation if Saturate=true)
%     res.meta - Struct with runner metadata

    arguments
        plant (1,1) struct
        params (1,1) struct
        x0 (:,1) double
        t_grid (:,1) double
        u_spec = 0
        opts.Saturate (1,1) logical = true
        opts.RelTol (1,1) double {mustBePositive} = 1e-8
        opts.AbsTol (1,1) double {mustBePositive} = 1e-10
        opts.MaxStep double = []
        opts.Solver (1,1) string {mustBeMember(opts.Solver, ["ode45", "ode15s", "ode23"])} = "ode45"
        opts.InputInterp (1,1) string {mustBeMember(opts.InputInterp, ["previous", "linear"])} = "previous"
        opts.Controller (1,1) string {mustBeMember(opts.Controller, ["none", "pid", "pole_placement", "lqr", "swingup"])} = "none"
        opts.Gains struct = struct()
        opts.Reference (4,1) double = zeros(4, 1)
    end

    tStart = tic;
    if ~isfield(plant, 'nonlinear') || ~isfield(plant.nonlinear, 'rhs') || ~isfield(plant.nonlinear, 'output')
        error('Furuta:InvalidPlant', 'plant.nonlinear.rhs and plant.nonlinear.output are required.');
    end
    if opts.Saturate && (~isfield(params, 'V_max') || ~isscalar(params.V_max) || ~(params.V_max > 0))
        error('Furuta:InvalidParameters', 'params.V_max must be a positive scalar when Saturate=true.');
    end

    % Resolve built-in closed-loop controller when requested
    if opts.Controller ~= "none"
        gains = opts.Gains;
        if isempty(fieldnames(gains))
            gains = compute_gains(plant, params);
        end
        x_ref = opts.Reference(:);

        if opts.Controller == "pid"
            [x0, t_grid, ~] = resolve_input_signal(x0, t_grid, 0, true);
            res = runNonlinearPID(plant, params, gains.pid, x0, t_grid, x_ref, opts, tStart);
            return;
        elseif opts.Controller == "pole_placement"
            K_pp   = gains.pp.K;
            u_spec = @(~, x) -K_pp * wrapStateErr(x, x_ref);
        elseif opts.Controller == "lqr"
            K_lqr  = gains.lqr.K;
            u_spec = @(~, x) -K_lqr * wrapStateErr(x, x_ref);
        elseif opts.Controller == "swingup"
            u_spec = gains.swingup.u_fcn;
        end
    end

    [x0, t_grid, u_info] = resolve_input_signal(x0, t_grid, u_spec, true);

    % Apply chosen interpolation method when u_spec is a sampled vector
    if strcmp(u_info.mode, 'vector')
        interpMethod = char(opts.InputInterp);
        u_vec_ref = u_info.u_vec;
        rawControl = @(t, ~) interp1(t_grid, u_vec_ref, t, interpMethod, 'extrap');
    else
        rawControl = u_info.u_fcn;
    end

    V_max = params.V_max;
    if opts.Saturate
        appliedControl = @(t, x) max(-V_max, min(V_max, rawControl(t, x)));
    else
        appliedControl = rawControl;
    end

    rhsFcn  = @(t, x) plant.nonlinear.rhs(x, appliedControl(t, x));
    odeOpts = buildOdeOptions(opts);

    solverFcn = str2func(char(opts.Solver));
    [t_out, x_out] = solverFcn(rhsFcn, t_grid, x0, odeOpts);

    nOut   = numel(t_out);
    u_raw  = zeros(nOut, 1);
    u_app  = zeros(nOut, 1);
    y_out  = zeros(nOut, 2);

    for k = 1:nOut
        xk = x_out(k, :).';
        tk = t_out(k);
        ur = rawControl(tk, xk);
        if opts.Saturate
            ua = max(-V_max, min(V_max, ur));
        else
            ua = ur;
        end
        u_raw(k)    = ur;
        u_app(k)    = ua;
        y_out(k, :) = plant.nonlinear.output(xk, ua).';
    end

    if opts.Saturate
        satFrac = mean(abs(u_raw) > V_max);
    else
        satFrac = 0.0;
    end

    res = struct();
    res.t = t_out(:);
    res.x = x_out;
    res.y = y_out;
    res.u = u_app;
    res.meta = struct( ...
        'runner',       'simulate_nonlinear', ...
        'model',        'nonlinear_3d', ...
        'equilibrium',  'none', ...
        'controller',   char(opts.Controller), ...
        'x0',           x0, ...
        'input_mode',   u_info.mode, ...
        'saturated',    opts.Saturate, ...
        'sat_fraction', satFrac, ...
        'solver',       char(opts.Solver), ...
        'elapsed_s',    toc(tStart));
end

function e = wrapStateErr(x, x_ref)
    th1_err = atan2(sin(x(3) - x_ref(3)), cos(x(3) - x_ref(3)));
    e = [x(1) - x_ref(1); x(2) - x_ref(2); th1_err; x(4) - x_ref(4)];
end

function odeOpts = buildOdeOptions(opts)
    odeOptsArgs = {'RelTol', opts.RelTol, 'AbsTol', opts.AbsTol};
    if ~isempty(opts.MaxStep)
        if ~isscalar(opts.MaxStep) || ~(opts.MaxStep > 0)
            error('Furuta:InvalidOption', 'MaxStep must be empty or a positive scalar.');
        end
        odeOptsArgs = [odeOptsArgs, {'MaxStep', opts.MaxStep}];
    end
    odeOpts = odeset(odeOptsArgs{:});
end

function res = runNonlinearPID(plant, params, pid, x0, t_grid, x_ref, opts, tStart)
%RUNNONLINEARPID Coupled 6-state nonlinear RK4 matching Simulink pid_controller.slx.
%   State vector: z = [theta0; dtheta0; theta1; dtheta1; xi; xf]
%   Control law:  e1 = wrapToPi(theta1_ref - theta1),  d_f = Nf*(e1 - xf)
%                 u_raw = -(Kp*e1 + Ki*xi + Kd*d_f),   u = clamp(u_raw, +/-V_max)
%                 Clamping anti-windup freezes integrator when saturated in same direction.
%   Uses sub-stepped 4th-order Runge-Kutta (h <= 0.5 ms) so hard clamping
%   boundaries never trigger adaptive variable-step chattering.
    V_max  = params.V_max;
    doSat  = opts.Saturate;
    th1Ref = x_ref(3);
    nOut   = numel(t_grid);

    x_out = zeros(nOut, 4);
    u_raw = zeros(nOut, 1);
    u_app = zeros(nOut, 1);
    y_out = zeros(nOut, 2);

    z = [x0; 0.0; 0.0]; % Matches Simulink PID Controller default I=0, D_filter=0
    [ua0, ur0]  = evalSingleLoopPID(z(1:4), z(5:6), pid, th1Ref, V_max, doSat);
    x_out(1, :) = z(1:4).';
    u_raw(1)    = ur0;
    u_app(1)    = ua0;
    y_out(1, :) = plant.nonlinear.output(z(1:4), ua0).';

    dt_max = 5e-4; % 0.5 ms RK4 sub-step (well below fast motor pole tau ~ 1.4 ms)
    for k = 2:nOut
        dtSeg = t_grid(k) - t_grid(k - 1);
        nSub  = max(1, ceil(dtSeg / dt_max));
        h     = dtSeg / nSub;
        for s = 1:nSub
            k1 = pidCoupledRhs(z,              plant, pid, th1Ref, V_max, doSat);
            k2 = pidCoupledRhs(z + 0.5*h*k1,   plant, pid, th1Ref, V_max, doSat);
            k3 = pidCoupledRhs(z + 0.5*h*k2,   plant, pid, th1Ref, V_max, doSat);
            k4 = pidCoupledRhs(z + h*k3,       plant, pid, th1Ref, V_max, doSat);
            z  = z + (h / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
        end
        xk = z(1:4);
        [ua, ur]    = evalSingleLoopPID(xk, z(5:6), pid, th1Ref, V_max, doSat);
        x_out(k, :) = xk.';
        u_raw(k)    = ur;
        u_app(k)    = ua;
        y_out(k, :) = plant.nonlinear.output(xk, ua).';
    end

    if doSat
        satFrac = mean(abs(u_raw) > V_max);
    else
        satFrac = 0.0;
    end

    res = struct();
    res.t = t_grid(:);
    res.x = x_out;
    res.y = y_out;
    res.u = u_app;
    res.meta = struct( ...
        'runner',       'simulate_nonlinear', ...
        'model',        'nonlinear_3d', ...
        'equilibrium',  'none', ...
        'controller',   'pid', ...
        'x0',           x0, ...
        'input_mode',   'closed_loop_controller', ...
        'saturated',    doSat, ...
        'sat_fraction', satFrac, ...
        'solver',       'rk4_pid', ...
        'elapsed_s',    toc(tStart));
end

function zdot = pidCoupledRhs(z, plant, pid, th1_ref, V_max, doSat)
    x = z(1:4);
    [u_app, ~, dxi, dxf] = evalSingleLoopPID(x, z(5:6), pid, th1_ref, V_max, doSat);
    xdot = plant.nonlinear.rhs(x, u_app);
    zdot = [xdot; dxi; dxf];
end

function [u_app, u_raw, dxi, dxf] = evalSingleLoopPID(x, z_pid, pid, th1_ref, V_max, doSat)
    e1    = atan2(sin(th1_ref - x(3)), cos(th1_ref - x(3)));
    xi    = z_pid(1);
    xf    = z_pid(2);
    d_f   = pid.Nf * (e1 - xf);
    u_raw = -(pid.Kp * e1 + pid.Ki * xi + pid.Kd * d_f);

    if doSat
        u_app = max(-V_max, min(V_max, u_raw));
        % Clamping anti-windup matching Simulink PID Controller (Clamping):
        % Since u = -PID(e1), saturating at +V_max with e1 < 0 (or -V_max with e1 > 0)
        % corresponds to u_raw * e1 < 0 -> freeze integrator.
        if abs(u_raw) > V_max && (u_raw * e1 < 0)
            dxi = 0.0;
        else
            dxi = e1;
        end
    else
        u_app = u_raw;
        dxi   = e1;
    end
    dxf = d_f;
end
