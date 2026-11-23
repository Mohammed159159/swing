function res = simulate_linear(plant, x0, t_grid, u_spec, opts)
%SIMULATE_LINEAR Simulate linearized Furuta state-space model via lsim.
%   res = SIMULATE_LINEAR(plant, x0, t_grid)
%   res = SIMULATE_LINEAR(plant, x0, t_grid, u_spec, Name=Value)
%
%   Inputs:
%     plant  - Plant struct returned by build_plant(params)
%     x0     - 4x1 global initial state [theta0; dtheta0; theta1; dtheta1]
%     t_grid - Nx1 strictly increasing time vector [s]
%     u_spec - Optional voltage input Va [V]: [], scalar, Nx1 vector, or @(t)
%
%   Name-Value Options:
%     Equilibrium - "downward" (default, theta1_eq = pi) or "upright" (theta1_eq = 0)
%     Saturate    - Logical flag to clip Va to +/- V_max (default: false)
%     V_max       - Saturation limit [V] when Saturate is true (default: inf)
%
%   Output (Standardized Furuta Trajectory Struct):
%     res.t    - Nx1 time vector [s]
%     res.x    - Nx4 global state trajectory [theta0, dtheta0, theta1, dtheta1]
%     res.y    - Nx2 output trajectory [theta0, theta1]
%     res.u    - Nx1 applied voltage Va [V]
%     res.meta - Struct with runner metadata

    arguments
        plant (1,1) struct
        x0 (:,1) double
        t_grid (:,1) double
        u_spec = 0
        opts.Equilibrium (1,1) string {mustBeMember(opts.Equilibrium, ["downward", "upright"])} = "downward"
        opts.Saturate (1,1) logical = false
        opts.V_max (1,1) double {mustBePositive} = inf
    end

    tStart = tic;
    [x0, t_grid, u_info] = resolve_input_signal(x0, t_grid, u_spec, false);

    if opts.Equilibrium == "upright"
        if ~isfield(plant, 'upright') || ~isfield(plant.upright, 'sys')
            error('Furuta:InvalidPlant', 'plant.upright.sys is required.');
        end
        eq   = plant.upright;
        x_eq = [0; 0; 0; 0];
    else
        if ~isfield(plant, 'downward') || ~isfield(plant.downward, 'sys')
            error('Furuta:InvalidPlant', 'plant.downward.sys is required.');
        end
        eq   = plant.downward;
        x_eq = [0; 0; pi; 0];
    end

    u_raw = u_info.u_vec;
    if opts.Saturate && isfinite(opts.V_max)
        satMask = abs(u_raw) > opts.V_max;
        u_app   = max(-opts.V_max, min(opts.V_max, u_raw));
        satFrac = mean(satMask);
    else
        u_app   = u_raw;
        satFrac = 0.0;
    end

    dx0 = x0 - x_eq;
    if abs(dx0(3)) > pi/2
        warning('Furuta:LargeLinearizationOffset', ...
            'Initial pendulum deviation |theta1(0) - theta1_eq| = %.2f deg exceeds 90 deg for "%s" equilibrium.', ...
            rad2deg(abs(dx0(3))), opts.Equilibrium);
    end

    [~, t_out, dx_out] = lsim(eq.sys, u_app, t_grid, dx0);

    x_out = dx_out + x_eq.';
    y_out = x_out(:, [1, 3]);

    res = struct();
    res.t = t_out(:);
    res.x = x_out;
    res.y = y_out;
    res.u = u_app(:);
    res.meta = struct( ...
        'runner',       'simulate_linear', ...
        'model',        sprintf('linear_%s', char(opts.Equilibrium)), ...
        'equilibrium',  char(opts.Equilibrium), ...
        'x0',           x0, ...
        'x_eq',         x_eq, ...
        'input_mode',   u_info.mode, ...
        'saturated',    opts.Saturate && isfinite(opts.V_max), ...
        'sat_fraction', satFrac, ...
        'solver',       'lsim', ...
        'elapsed_s',    toc(tStart));
end
