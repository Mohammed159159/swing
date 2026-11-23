function pp_gains = pole_placement_design(plant, params, opts)
%POLE_PLACEMENT_DESIGN Full-state feedback pole-placement synthesis (place / acker).
%   pp_gains = POLE_PLACEMENT_DESIGN(plant, params)
%   pp_gains = POLE_PLACEMENT_DESIGN(plant, params, Name=Value)
%
%   Computes the 1x4 state-feedback gain vector K aligned with the locked state
%   order x = [theta0; dtheta0; theta1; dtheta1] for the control law:
%       Va = sat(-K * (x - x_ref), +/- params.V_max)
%          = sat(K * x_ref - K * x, +/- params.V_max)
%
%   Pole Selection Strategy:
%     - Dominant pendulum pair: p_{1,2} = -zeta*wn +/- j*wn*sqrt(1 - zeta^2)
%       (default zeta = 0.707, wn = 10 rad/s -> Ts ~ 0.57 s <= 0.8 s).
%     - Arm-centering pole:     p_3 = -3.5 rad/s.
%     - Fast motor-damping pole: p_4 defaults to min(-4*zeta*wn, 0.95*lambda_fast)
%       so the controller preserves the natural back-EMF damped motor mode
%       rather than fighting it.
%
%   Name-Value Options:
%     Zeta      - Dominant damping ratio    (default: 0.707)
%     Wn        - Dominant natural freq     (default: 10.0 rad/s)
%     ArmPole   - Real arm-centering pole   (default: -3.5 rad/s)
%     MotorPole - Optional override for 4th pole (default: [] -> preserve natural)

    arguments
        plant (1,1) struct
        params (1,1) struct
        opts.Zeta (1,1) double {mustBePositive} = 0.707
        opts.Wn (1,1) double {mustBePositive} = 10.0
        opts.ArmPole (1,1) double {mustBeNegative} = -3.5
        opts.MotorPole double = []
    end

    A = plant.A_full;
    B = plant.B_full;

    if rank(ctrb(A, B)) < size(A, 1)
        error('Furuta:UncontrollablePlant', ...
            'Upright linearized plant (A_full, B_full) is not controllable.');
    end

    zeta  = opts.Zeta;
    wn    = opts.Wn;
    sigma = zeta * wn;
    if zeta < 1.0
        wd    = wn * sqrt(1 - zeta^2);
        p_dom = [-sigma + 1j * wd, -sigma - 1j * wd];
    else
        rt    = wn * sqrt(zeta^2 - 1);
        p_dom = [-sigma + rt, -sigma - rt];
    end

    p_arm = opts.ArmPole;
    if isempty(opts.MotorPole)
        p_ol_fast = min(real(plant.eig_upright));
        p_mot     = min(-4.0 * sigma, 0.95 * p_ol_fast);
    else
        p_mot = double(opts.MotorPole);
        if ~isscalar(p_mot) || ~isfinite(p_mot) || ~(p_mot < 0)
            error('Furuta:InvalidOption', 'MotorPole must be a finite negative scalar.');
        end
    end

    p_des   = [p_dom, p_arm, p_mot];
    K       = place(A, B, p_des);
    K_acker = acker(A, B, p_des);

    % Ready-to-use stateless feedback handle for simulate_nonlinear(plant, params, x0, t, pp.u_fcn)
    u_fcn = @(~, x) -K * [x(1); x(2); atan2(sin(x(3)), cos(x(3))); x(4)];

    pp_gains = struct( ...
        'K',             K, ...
        'K_acker',       K_acker, ...
        'Nbar',          K(1), ...
        'desired_poles', p_des(:), ...
        'zeta',          zeta, ...
        'wn',            wn, ...
        'p_arm',         p_arm, ...
        'p_motor',       p_mot, ...
        'V_max',         params.V_max, ...
        'u_fcn',         u_fcn);
end
