function m = compute_performance_metrics(res, x_ref, params, opts)
%COMPUTE_PERFORMANCE_METRICS Compute Section 4.7 closed-loop benchmark metrics.
%   m = COMPUTE_PERFORMANCE_METRICS(res)
%   m = COMPUTE_PERFORMANCE_METRICS(res, x_ref, params, Name=Value)
%
%   Evaluates a standardized Furuta closed-loop trajectory struct
%   res.{t, x, y, u, meta} (with x = [theta0, dtheta0, theta1, dtheta1]) and
%   returns transient response, steady-state error, error-integral indices
%   (IAE, ISE, ITAE), and actuator effort/saturation metrics.
%
%   Name-Value Options:
%     Mode             - "disturbance" (default), "impulse", or "arm_step"
%     SettlingFrac     - Relative settling threshold fraction (default: 0.02 -> 2%)
%     SettlingFloorDeg - Absolute settling band floor [deg]   (default: 0.25 deg)
%     SatMarginFrac    - Fraction of V_max counted as saturated (default: 0.995)

    arguments
        res (1,1) struct
        x_ref (4,1) double = zeros(4, 1)
        params struct = struct()
        opts.Mode (1,1) string {mustBeMember(opts.Mode, ["disturbance", "impulse", "arm_step"])} = "disturbance"
        opts.SettlingFrac (1,1) double {mustBePositive} = 0.02
        opts.SettlingFloorDeg (1,1) double {mustBePositive} = 0.25
        opts.SatMarginFrac (1,1) double {mustBePositive} = 0.995
    end

    if ~isfield(res, 't') || ~isfield(res, 'x') || ~isfield(res, 'u') || size(res.x, 2) ~= 4
        error('Furuta:InvalidResultStruct', 'res must contain t (Nx1), x (Nx4), and u (Nx1).');
    end

    t  = double(res.t(:));
    T  = t(end) - t(1);
    if ~(T > 0)
        error('Furuta:InvalidTimeGrid', 'res.t must span a positive duration.');
    end

    th0  = double(res.x(:, 1));
    dth0 = double(res.x(:, 2));
    th1  = double(res.x(:, 3));
    u    = double(res.u(:));

    % Wrapped pendulum error e1 [rad] and arm error e0 [rad]
    e1 = atan2(sin(th1 - x_ref(3)), cos(th1 - x_ref(3)));
    e0 = th0 - x_ref(1);

    % --- 1. Pendulum (theta1) transient & steady-state metrics ---
    [e1PeakRad, iPeak1] = max(abs(e1));
    e1PeakDeg  = rad2deg(e1PeakRad);
    tPeak1     = t(iPeak1) - t(1);
    band1Rad   = max(opts.SettlingFrac * e1PeakRad, deg2rad(opts.SettlingFloorDeg));
    ts1        = computeSettlingTime(t, e1, band1Rad);

    % Steady-state error over final 10% of horizon
    ssMask = t >= (t(1) + 0.90 * T);
    ess1Deg = rad2deg(mean(abs(e1(ssMask))));

    % Rise / initial recovery time (90% -> 10% of initial deflection for disturbance)
    tr1 = NaN;
    os1Pct = 0.0;
    if opts.Mode == "disturbance" && abs(e1(1)) > 1e-6
        s0   = sign(e1(1));
        e1_0 = abs(e1(1));
        i90  = find(s0 * e1 <= 0.90 * e1_0, 1, 'first');
        i10  = find(s0 * e1 <= 0.10 * e1_0, 1, 'first');
        if ~isempty(i90) && ~isempty(i10) && i10 >= i90
            tr1 = t(i10) - t(i90);
        end
        % Opposite-sign overshoot relative to initial deflection
        oppPeak = max(0.0, max(-s0 * e1));
        os1Pct  = 100.0 * oppPeak / e1_0;
    elseif opts.Mode == "impulse" && e1PeakRad > 1e-6
        sPk     = sign(e1(iPeak1));
        oppPeak = max(0.0, max(-sPk * e1(iPeak1:end)));
        os1Pct  = 100.0 * oppPeak / e1PeakRad;
    end

    % --- 2. Arm (theta0) transient & steady-state metrics ---
    e0PeakDeg = rad2deg(max(abs(e0)));
    ess0Deg   = rad2deg(mean(abs(e0(ssMask))));
    band0Rad  = max(opts.SettlingFrac * max(abs(e0)), deg2rad(0.5));
    ts0       = computeSettlingTime(t, e0, band0Rad);
    tr0       = NaN;
    os0Pct    = 0.0;
    if opts.Mode == "arm_step" && abs(x_ref(1) - th0(1)) > 1e-6
        stepMag = x_ref(1) - th0(1);
        prog    = (th0 - th0(1)) / stepMag;
        i10     = find(prog >= 0.10, 1, 'first');
        i90     = find(prog >= 0.90, 1, 'first');
        if ~isempty(i10) && ~isempty(i90) && i90 >= i10
            tr0 = t(i90) - t(i10);
        end
        os0Pct  = 100.0 * max(0.0, max(prog) - 1.0);
    end

    % --- 3. Error-integral indices via trapz ---
    tRel   = t - t(1);
    iae1   = trapz(tRel, abs(e1));
    ise1   = trapz(tRel, e1 .^ 2);
    itae1  = trapz(tRel, tRel .* abs(e1));

    iae0   = trapz(tRel, abs(e0));
    ise0   = trapz(tRel, e0 .^ 2);
    itae0  = trapz(tRel, tRel .* abs(e0));

    % --- 4. Actuator effort & saturation ---
    uPeak   = max(abs(u));
    uEnergy = trapz(tRel, u .^ 2);
    uRms    = sqrt(uEnergy / T);

    if isfield(params, 'V_max') && isscalar(params.V_max) && params.V_max > 0
        satPct = 100.0 * mean(abs(u) >= opts.SatMarginFrac * params.V_max);
    elseif isfield(res, 'meta') && isfield(res.meta, 'sat_fraction')
        satPct = 100.0 * res.meta.sat_fraction;
    else
        satPct = NaN;
    end

    m = struct();
    m.mode                   = opts.Mode;
    m.theta1_peak_deg        = e1PeakDeg;
    m.theta1_peak_time_s     = tPeak1;
    m.theta1_settling_time_s = ts1;
    m.theta1_rise_time_s     = tr1;
    m.theta1_overshoot_pct   = os1Pct;
    m.theta1_ess_deg         = ess1Deg;
    m.theta1_IAE             = iae1;
    m.theta1_ISE             = ise1;
    m.theta1_ITAE            = itae1;

    m.theta0_peak_deg        = e0PeakDeg;
    m.theta0_settling_time_s = ts0;
    m.theta0_rise_time_s     = tr0;
    m.theta0_overshoot_pct   = os0Pct;
    m.theta0_ess_deg         = ess0Deg;
    m.dtheta0_end_rad_s      = abs(dth0(end));
    m.theta0_IAE             = iae0;
    m.theta0_ISE             = ise0;
    m.theta0_ITAE            = itae0;

    m.u_peak_V               = uPeak;
    m.u_rms_V                = uRms;
    m.u_energy_V2s           = uEnergy;
    m.sat_pct                = satPct;
    m.stabilized_theta1      = (ess1Deg < 2.0);
end

function ts = computeSettlingTime(t, err, band)
    idxOut = find(abs(err) > band, 1, 'last');
    if isempty(idxOut)
        ts = 0.0;
    elseif idxOut == numel(t)
        ts = NaN; % Did not settle within simulation window
    else
        ts = t(idxOut + 1) - t(1);
    end
end
