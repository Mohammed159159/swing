function res = simulate_nonlinear(plant, params, x0, t_grid, u_spec, opts)
%SIMULATE_NONLINEAR Simulate full 3D nonlinear Furuta ODE via MATLAB ODE solvers.
%   res = SIMULATE_NONLINEAR(plant, params, x0, t_grid)
%   res = SIMULATE_NONLINEAR(plant, params, x0, t_grid, u_spec, Name=Value)
%
%   Supports both open-loop voltage profiles (empty, scalar, Nx1 vector, @(t))
%   and closed-loop state-feedback control laws @(t, x) with optional actuator
%   saturation at +/- params.V_max.
%
%   Inputs:
%     plant  - Plant struct returned by build_plant(params)
%     params - Parameter struct returned by load_parameters()
%     x0     - 4x1 global initial state [theta0; dtheta0; theta1; dtheta1]
%     t_grid - Nx1 strictly increasing time vector [s]
%     u_spec - Optional voltage input Va [V]: [], scalar, Nx1 vector, @(t), or @(t, x)
%
%   Name-Value Options:
%     Saturate   - Clip Va to [-params.V_max, +params.V_max] (default: true)
%     RelTol     - ODE solver relative tolerance (default: 1e-8)
%     AbsTol     - ODE solver absolute tolerance (default: 1e-10)
%     MaxStep    - Optional max ODE step size [s] (default: [])
%     Solver     - "ode45" (default), "ode15s", or "ode23"
%     InputInterp - Interpolation method for vector u_spec: "previous" (default) or "linear"
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
    end

    tStart = tic;
    if ~isfield(plant, 'nonlinear') || ~isfield(plant.nonlinear, 'rhs') || ~isfield(plant.nonlinear, 'output')
        error('Furuta:InvalidPlant', 'plant.nonlinear.rhs and plant.nonlinear.output are required.');
    end
    if opts.Saturate && (~isfield(params, 'V_max') || ~isscalar(params.V_max) || ~(params.V_max > 0))
        error('Furuta:InvalidParameters', 'params.V_max must be a positive scalar when Saturate=true.');
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

    rhsFcn = @(t, x) plant.nonlinear.rhs(x, appliedControl(t, x));

    odeOptsArgs = {'RelTol', opts.RelTol, 'AbsTol', opts.AbsTol};
    if ~isempty(opts.MaxStep)
        if ~isscalar(opts.MaxStep) || ~(opts.MaxStep > 0)
            error('Furuta:InvalidOption', 'MaxStep must be empty or a positive scalar.');
        end
        odeOptsArgs = [odeOptsArgs, {'MaxStep', opts.MaxStep}];
    end
    odeOpts = odeset(odeOptsArgs{:});

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
        'x0',           x0, ...
        'input_mode',   u_info.mode, ...
        'saturated',    opts.Saturate, ...
        'sat_fraction', satFrac, ...
        'solver',       char(opts.Solver), ...
        'elapsed_s',    toc(tStart));
end
