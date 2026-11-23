function lqr_gains = lqr_design(plant, params, opts)
%LQR_DESIGN Optimal Linear Quadratic Regulator (LQR) synthesis via Bryson's rule.
%   lqr_gains = LQR_DESIGN(plant, params)
%   lqr_gains = LQR_DESIGN(plant, params, Name=Value)
%
%   Constructs Bryson's rule weighting matrices strictly matching the locked
%   state vector x = [theta0; dtheta0; theta1; dtheta1]:
%       Q = diag(1 ./ StateMax.^2),   R = 1 / ControlMax^2
%   and solves the continuous-time Algebraic Riccati Equation via lqr(A, B, Q, R)
%   for the control law:
%       Va = sat(-K * (x - x_ref), +/- params.V_max)
%          = sat(K * x_ref - K * x, +/- params.V_max)
%
%   Name-Value Options:
%     StateMax   - 4x1 max acceptable deviations [th0; dth0; th1; dth1]
%                  (default: [0.6; 6.0; 0.15; 4.0] rad, rad/s)
%     ControlMax - Effective actuator weighting voltage [V] (default: 0.7 V,
%                  keeps 10 deg recovery unsaturated and avoids high-frequency
%                  chattering against Simscape DC motor inductance/dynamics)

    arguments
        plant (1,1) struct
        params (1,1) struct
        opts.StateMax (4,1) double {mustBePositive} = [0.6; 6.0; 0.15; 4.0]
        opts.ControlMax (1,1) double {mustBePositive} = 0.7
    end

    A = plant.A_full;
    B = plant.B_full;

    if rank(ctrb(A, B)) < size(A, 1)
        error('Furuta:UncontrollablePlant', ...
            'Upright linearized plant (A_full, B_full) is not controllable.');
    end

    q_max     = opts.StateMax(:);
    u_max_eff = opts.ControlMax;
    Q         = diag(1 ./ (q_max .^ 2));
    R         = 1 / (u_max_eff ^ 2);

    [K, P, ~] = lqr(A, B, Q, R);

    % Ready-to-use stateless feedback handle for simulate_nonlinear(plant, params, x0, t, lqr.u_fcn)
    u_fcn = @(~, x) -K * [x(1); x(2); atan2(sin(x(3)), cos(x(3))); x(4)];

    lqr_gains = struct( ...
        'K',         K, ...
        'Nbar',      K(1), ...
        'Q',         Q, ...
        'R',         R, ...
        'P',         P, ...
        'q_max',     q_max, ...
        'u_max_eff', u_max_eff, ...
        'V_max',     params.V_max, ...
        'u_fcn',     u_fcn);
end
