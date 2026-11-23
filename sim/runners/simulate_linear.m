function res = simulate_linear(plant, x0, t_grid, u_spec, opts)
%SIMULATE_LINEAR Simulate linearized Furuta state-space model via lsim.
%   res = SIMULATE_LINEAR(plant, x0, t_grid)
%   res = SIMULATE_LINEAR(plant, x0, t_grid, u_spec, Name=Value)
%
%   Supports both open-loop voltage profiles (empty, scalar, Nx1 vector, @(t))
%   and closed-loop upright controllers via Controller="pid"|"pole_placement"|"lqr".
%
%   Inputs:
%     plant  - Plant struct returned by build_plant(params)
%     x0     - 4x1 global initial state [theta0; dtheta0; theta1; dtheta1]
%     t_grid - Nx1 strictly increasing time vector [s]
%     u_spec - Optional open-loop voltage Va [V]: [], scalar, Nx1 vector, or @(t)
%
%   Name-Value Options:
%     Equilibrium - "downward" (default for open-loop) or "upright" (auto for closed-loop)
%     Saturate    - Logical flag to clip Va to +/- V_max (default: false)
%     V_max       - Saturation limit [V] when Saturate is true (default: inf)
%     Controller  - "none" (default), "pid", "pole_placement", or "lqr"
%     Gains       - Gains struct from compute_gains(plant, params) (auto if empty)
%     Reference   - 4x1 target state x_ref for closed-loop mode (default: zeros(4,1))
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
        opts.Controller (1,1) string {mustBeMember(opts.Controller, ["none", "pid", "pole_placement", "lqr"])} = "none"
        opts.Gains struct = struct()
        opts.Reference (4,1) double = zeros(4, 1)
    end

    tStart = tic;
    [x0, t_grid, u_info] = resolve_input_signal(x0, t_grid, u_spec, false);

    if opts.Controller ~= "none"
        res = runClosedLoopLinear(plant, x0, t_grid, opts, tStart);
        return;
    end

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
        'controller',   'none', ...
        'x0',           x0, ...
        'x_eq',         x_eq, ...
        'input_mode',   u_info.mode, ...
        'saturated',    opts.Saturate && isfinite(opts.V_max), ...
        'sat_fraction', satFrac, ...
        'solver',       'lsim', ...
        'elapsed_s',    toc(tStart));
end

function res = runClosedLoopLinear(plant, x0, t_grid, opts, tStart)
    if ~isfield(plant, 'upright') || ~isfield(plant.upright, 'A') || ~isfield(plant.upright, 'B')
        error('Furuta:InvalidPlant', 'plant.upright (A, B) is required for closed-loop linear simulation.');
    end

    gains = opts.Gains;
    if isempty(fieldnames(gains))
        gains = compute_gains(plant, load_parameters());
    end

    A     = plant.upright.A;
    B     = plant.upright.B;
    x_ref = opts.Reference(:);
    nOut  = numel(t_grid);

    switch opts.Controller
        case {"pole_placement", "lqr"}
            if opts.Controller == "pole_placement"
                K = gains.pp.K;
            else
                K = gains.lqr.K;
            end
            A_cl   = A - B * K;
            B_cl   = B * K;
            sys_cl = ss(A_cl, B_cl, eye(4), zeros(4, 4));
            U_ref  = repmat(x_ref.', nOut, 1);
            [x_out, t_out] = lsim(sys_cl, U_ref, t_grid, x0);
            u_raw  = -(x_out - U_ref) * K.';

        case "pid"
            p   = gains.pid;
            Kp  = p.Kp; Ki = p.Ki; Kd = p.Kd; Nf = p.Nf;
            % Exact 6-state linear closed-loop matching pid_controller.slx:
            %   e1 = r3 - x3,  d(xi)/dt = e1,  d(xf)/dt = Nf*(e1 - xf)
            %   u  = -(Kp*e1 + Ki*xi + Kd*Nf*(e1 - xf))
            %      = (Kp + Kd*Nf)*x3 - Ki*xi + Kd*Nf*xf - (Kp + Kd*Nf)*r3
            K_z   = [0, 0, (Kp + Kd * Nf), 0, -Ki, Kd * Nf];
            A_aug = [[A, zeros(4, 2)] + B * K_z; ...
                     0, 0, -1,  0, 0,  0; ...
                     0, 0, -Nf, 0, 0, -Nf];
            B_aug = [-B * (Kp + Kd * Nf); 1; Nf];
            sys_cl = ss(A_aug, B_aug, eye(6), zeros(6, 1));
            r3_vec = repmat(x_ref(3), nOut, 1);
            z0     = [x0; 0.0; 0.0];
            [z_out, t_out] = lsim(sys_cl, r3_vec, t_grid, z0);
            x_out  = z_out(:, 1:4);
            u_raw  = z_out * K_z.' - (Kp + Kd * Nf) * r3_vec;
    end

    if opts.Saturate && isfinite(opts.V_max)
        satMask = abs(u_raw) > opts.V_max;
        u_app   = max(-opts.V_max, min(opts.V_max, u_raw));
        satFrac = mean(satMask);
    else
        u_app   = u_raw;
        satFrac = 0.0;
    end

    res = struct();
    res.t = t_out(:);
    res.x = x_out;
    res.y = x_out(:, [1, 3]);
    res.u = u_app(:);
    res.meta = struct( ...
        'runner',       'simulate_linear', ...
        'model',        'linear_upright', ...
        'equilibrium',  'upright', ...
        'controller',   char(opts.Controller), ...
        'x0',           x0, ...
        'x_eq',         [0; 0; 0; 0], ...
        'x_ref',        x_ref, ...
        'input_mode',   'closed_loop_controller', ...
        'saturated',    opts.Saturate && isfinite(opts.V_max), ...
        'sat_fraction', satFrac, ...
        'solver',       'lsim', ...
        'elapsed_s',    toc(tStart));
end
