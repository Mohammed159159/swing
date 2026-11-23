function pid_gains = pid_design(plant, params, opts)
%PID_DESIGN Single-loop minimal filtered PID synthesis for pendulum angle theta1.
%   pid_gains = PID_DESIGN(plant, params)
%   pid_gains = PID_DESIGN(plant, params, Name=Value)
%
%   Designs a single-loop SISO filtered PID controller on theta1 alone:
%       e(t) = theta1_ref(t) - theta1(t)
%       Va(t) = -sat(Kp*e + Ki*int(e) + Kd * [Nf*s/(s + Nf)] * e, +/- V_max)
%   (equivalently +sat(...) when error is formed as theta1 - theta1_ref, since
%   G_theta1(s) = Theta1(s)/Va(s) has negative high-frequency gain B_4 < 0).
%
%   Key Routh-Hurwitz Stabilizability Condition:
%       Because G_theta1(s) = (b1*s) / (s^3 + a2*s^2 + a1*s + a0) has a zero
%       at s = 0 (with b1 < 0 and a0 < 0), canceling s/s in the closed-loop
%       characteristic equation gives constant term (|b1|*Ki + a0) > 0, so
%       single-loop theta1 stabilization strictly requires:
%           Ki > Ki_min = -a0 / |b1|.
%
%   Name-Value Options:
%     Kp       - Proportional gain [V/rad]     (default: [] -> adaptive 0.38 * Ki_min)
%     Ki       - Integral gain     [V/(rad*s)] (default: [] -> adaptive 2.55 * Ki_min)
%     Kd       - Derivative gain   [V*s/rad]   (default: [] -> adaptive 0.023 * Ki_min)
%     Nf       - Derivative filter [rad/s]     (default: 60.0)
%     KiFactor - Multiplier above Ki_min when Ki is auto-sized (default: 2.55)

    arguments
        plant (1,1) struct
        params (1,1) struct
        opts.Kp double = []
        opts.Ki double = []
        opts.Kd double = []
        opts.Nf (1,1) double {mustBePositive} = 60.0
        opts.KiFactor (1,1) double {mustBeGreaterThan(opts.KiFactor, 1.0)} = 2.55
    end

    if ~isfield(plant, 'sys_upright')
        error('Furuta:InvalidPlant', 'plant.sys_upright is required by pid_design.');
    end

    % Extract SISO transfer function from Va -> theta1 (output 2, input 1)
    G_theta1 = minreal(tf(plant.sys_upright(2, 1)), 1e-6);
    [num, den] = tfdata(G_theta1, 'v');

    % G_theta1(s) = (b1 * s) / (s^3 + a2*s^2 + a1*s + a0)
    b1 = num(end - 1);
    a0 = den(end);
    if ~(b1 < 0 && a0 < 0)
        error('Furuta:UnexpectedPlantSign', ...
            'Expected upright G_theta1(s) with b1 < 0 and a0 < 0; got b1=%.3g, a0=%.3g.', b1, a0);
    end

    % Exact Routh-Hurwitz minimum integral gain needed to overcome motor back-EMF
    Ki_min = -a0 / abs(b1);

    if isempty(opts.Ki)
        Ki = opts.KiFactor * Ki_min;
    else
        Ki = double(opts.Ki);
    end
    if isempty(opts.Kp)
        Kp = 0.38 * Ki_min;
    else
        Kp = double(opts.Kp);
    end
    if isempty(opts.Kd)
        Kd = 0.023 * Ki_min;
    else
        Kd = double(opts.Kd);
    end
    Nf = opts.Nf;

    if ~(Kp > 0 && Ki > Ki_min && Kd >= 0)
        warning('Furuta:PIDBelowRouthThreshold', ...
            'Single-loop PID gains (Kp=%.2f, Ki=%.2f, Kd=%.2f) may violate Routh-Hurwitz threshold Ki > %.2f.', ...
            Kp, Ki, Kd, Ki_min);
    end

    C_pid = pid(Kp, Ki, Kd, 1 / Nf);

    pid_gains = struct( ...
        'Kp',        Kp, ...
        'Ki',        Ki, ...
        'Kd',        Kd, ...
        'Nf',        Nf, ...
        'Ki_min',    Ki_min, ...
        'V_max',     params.V_max, ...
        'G_theta1',  G_theta1, ...
        'C_pid',     C_pid, ...
        'structure', 'single_loop_theta1', ...
        'note',      'Positive gains for Va = -C_pid*(theta1_ref - theta1) = +C_pid*(theta1 - theta1_ref).');
end
