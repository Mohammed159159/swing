function su_gains = swingup_design(plant, params, opts)
%SWINGUP_DESIGN Smooth Astrom-Furuta energy-based swing-up parameter synthesis.
%   su_gains = SWINGUP_DESIGN(plant, params)
%   su_gains = SWINGUP_DESIGN(plant, params, Name=Value)
%
%   Computes gearbox-aware parameters for the smooth Astrom-Furuta energy pump:
%       E(theta1, dtheta1) = 0.5*J1_hat*dtheta1^2 + m1*g*l1*(cos(theta1) - 1)
%       ddth0_cmd = clamp(k_E * E * tanh((dtheta1*cos(theta1)) / eps_s), +/- a_max)
%       Va = sat(kick(t) + V_scale*ddth0_cmd + k_emf*dtheta0 - kp0*theta0 - kd0*dtheta0, +/- V_max)
%
%   Name-Value Options:
%     KEVolt           - Equivalent pump voltage at E = E_rest [V] (default: 15.0)
%     VPumpMax         - Clamped pump voltage contribution [V]     (default: 6.0)
%     SmoothEps        - Tanh smoothing width [rad/s]              (default: 0.20)
%     KickVolt         - Smooth Gaussian start pulse [V]           (default: 0.8)
%     ArmKp            - Soft arm centering spring [V/rad]         (default: 2.5)
%     ArmKd            - Soft arm centering damper [V/(rad/s)]     (default: 0.15)
%     CaptureAngleRad  - Capture region |theta1| limit [rad]       (default: 15 deg)
%     CaptureVelRadS   - Capture region |dtheta1| limit [rad/s]    (default: 2.5)
%     FallbackAngleRad - Hysteresis fallback |theta1| limit [rad]  (default: 25 deg)

    arguments
        plant (1,1) struct
        params (1,1) struct
        opts.KEVolt (1,1) double {mustBePositive} = 15.0
        opts.VPumpMax (1,1) double {mustBePositive} = 6.0
        opts.SmoothEps (1,1) double {mustBePositive} = 0.20
        opts.KickVolt (1,1) double {mustBeNonnegative} = 0.8
        opts.ArmKp (1,1) double {mustBeNonnegative} = 2.5
        opts.ArmKd (1,1) double {mustBeNonnegative} = 0.15
        opts.CaptureAngleRad (1,1) double {mustBePositive} = deg2rad(15)
        opts.CaptureVelRadS (1,1) double {mustBePositive} = 2.5
        opts.FallbackAngleRad (1,1) double {mustBePositive} = deg2rad(25)
    end

    if ~(opts.FallbackAngleRad > opts.CaptureAngleRad)
        error('Furuta:InvalidHysteresis', ...
            'FallbackAngleRad must be strictly greater than CaptureAngleRad.');
    end

    J1_hat  = plant.derived.J1_hat;
    I0_hat  = plant.derived.I0_hat;
    V_scale = (params.R_m * I0_hat) / (params.N * params.k_t);
    k_emf   = params.N * params.k_b;
    E_mag   = 2.0 * params.m1 * params.g * params.l1;

    k_E   = opts.KEVolt / (V_scale * E_mag);
    a_max = opts.VPumpMax / V_scale;

    su_gains = struct( ...
        'J1_hat',             J1_hat, ...
        'I0_hat',             I0_hat, ...
        'm1',                 params.m1, ...
        'g',                  params.g, ...
        'l1',                 params.l1, ...
        'V_scale',            V_scale, ...
        'k_emf',              k_emf, ...
        'E_rest',             -E_mag, ...
        'k_E',                k_E, ...
        'k_E_volt',           opts.KEVolt, ...
        'a_max',              a_max, ...
        'V_pump_max',         opts.VPumpMax, ...
        'eps_s',              opts.SmoothEps, ...
        'kick_V',             opts.KickVolt, ...
        'kick_t0',            0.05, ...
        'kick_sigma',         0.04, ...
        'kp0',                opts.ArmKp, ...
        'kd0',                opts.ArmKd, ...
        'capture_angle_rad',  opts.CaptureAngleRad, ...
        'capture_vel_rad_s',  opts.CaptureVelRadS, ...
        'fallback_angle_rad', opts.FallbackAngleRad, ...
        'V_max',              params.V_max);

    su_gains.u_fcn = @(t, x) evalSwingUp(t, x, su_gains);
end

function u = evalSwingUp(t, x, su)
    th0  = x(1);
    dth0 = x(2);
    th1  = atan2(sin(x(3)), cos(x(3)));
    dth1 = x(4);

    E         = 0.5 * su.J1_hat * dth1^2 + su.m1 * su.g * su.l1 * (cos(th1) - 1.0);
    kick      = su.kick_V * exp(-((t - su.kick_t0) / su.kick_sigma)^2);
    ddth0_raw = su.k_E * E * tanh((dth1 * cos(th1)) / su.eps_s);
    ddth0_cmd = max(-su.a_max, min(su.a_max, ddth0_raw));

    u = kick + su.V_scale * ddth0_cmd + su.k_emf * dth0 - su.kp0 * th0 - su.kd0 * dth0;
end
