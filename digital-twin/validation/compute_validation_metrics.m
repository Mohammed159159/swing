function m = compute_validation_metrics(res_ref, res_cmp, opts)
%COMPUTE_VALIDATION_METRICS Compute trajectory agreement & dynamic mode metrics.
%   m = COMPUTE_VALIDATION_METRICS(res_ref, res_cmp)
%   m = COMPUTE_VALIDATION_METRICS(res_ref, res_cmp, Name=Value)
%
%   Compares a candidate trajectory `res_cmp` against a reference trajectory
%   `res_ref` (both following the standardized Furuta res.{t, x, y, u, meta}
%   contract with x = [theta0, dtheta0, theta1, dtheta1]).
%
%   Name-Value Options:
%     NrmseGatePct       - Pass/fail NRMSE threshold [%] on theta1 (default: 3.0)
%     EstimateDamping    - Fit logarithmic decrement on downward free decay (default: false)
%     EstimateGrowthRate - Fit exponential topple growth rate on upright fall (default: false)
%     Params             - Optional params struct for uncoupled b1_hat = 2*sigma*J1_hat
%
%   Output struct `m`:
%     m.channels.(theta0|dtheta0|theta1|dtheta1):
%         .rmse, .nrmse_pct, .max_abs_err, .max_abs_err_deg (for angles), .ref_range
%     m.theta0_nrmse_pct, m.theta1_nrmse_pct, m.theta1_max_err_deg
%     m.gate_pct, m.pass_gate
%     m.damping (when EstimateDamping=true)
%     m.growth  (when EstimateGrowthRate=true)

    arguments
        res_ref (1,1) struct
        res_cmp (1,1) struct
        opts.NrmseGatePct (1,1) double = 3.0
        opts.EstimateDamping (1,1) logical = false
        opts.EstimateGrowthRate (1,1) logical = false
        opts.Params struct = struct()
    end

    validateResultStruct(res_ref, 'res_ref');
    validateResultStruct(res_cmp, 'res_cmp');

    t_ref = res_ref.t(:);
    x_ref = res_ref.x;

    if numel(res_cmp.t) == numel(t_ref) && max(abs(res_cmp.t(:) - t_ref)) < 1e-12
        x_cmp = res_cmp.x;
    else
        x_cmp = interp1(res_cmp.t(:), res_cmp.x, t_ref, 'linear', 'extrap');
    end

    chanNames = {'theta0', 'dtheta0', 'theta1', 'dtheta1'};
    isAngle   = [true, false, true, false];

    channels = struct();
    for k = 1:4
        r = x_ref(:, k);
        c = x_cmp(:, k);
        err = c - r;

        rmseVal   = sqrt(mean(err.^2));
        maxAbsVal = max(abs(err));
        refRange  = max(r) - min(r);
        if refRange > 1e-9
            nrmsePct = 100.0 * rmseVal / refRange;
        else
            nrmsePct = NaN;
        end

        ch = struct( ...
            'rmse',        rmseVal, ...
            'nrmse_pct',   nrmsePct, ...
            'max_abs_err', maxAbsVal, ...
            'ref_range',   refRange);
        if isAngle(k)
            ch.max_abs_err_deg = rad2deg(maxAbsVal);
            ch.rmse_deg        = rad2deg(rmseVal);
        end
        channels.(chanNames{k}) = ch;
    end

    th1Nrmse = channels.theta1.nrmse_pct;
    if isnan(opts.NrmseGatePct)
        passGate = true;
    elseif isnan(th1Nrmse)
        passGate = channels.theta1.max_abs_err < 1e-6;
    else
        passGate = th1Nrmse <= opts.NrmseGatePct;
    end

    m = struct();
    m.channels           = channels;
    m.theta0_nrmse_pct   = channels.theta0.nrmse_pct;
    m.theta1_nrmse_pct   = th1Nrmse;
    m.theta0_max_err_deg = channels.theta0.max_abs_err_deg;
    m.theta1_max_err_deg = channels.theta1.max_abs_err_deg;
    m.gate_pct           = opts.NrmseGatePct;
    m.pass_gate          = passGate;

    if opts.EstimateDamping
        m.damping = estimateLogDecrement(t_ref, x_ref(:, 3) - pi, opts.Params);
    end
    if opts.EstimateGrowthRate
        m.growth = estimateToppleGrowthRate(t_ref, x_ref(:, 3));
    end
end

function validateResultStruct(res, label)
    if ~isfield(res, 't') || ~isfield(res, 'x') || size(res.x, 2) ~= 4 || numel(res.t) ~= size(res.x, 1)
        error('Furuta:InvalidResultStruct', ...
            '%s must contain matching t (Nx1) and x (Nx4) fields.', label);
    end
end

function damp = estimateLogDecrement(t, y, params)
%ESTIMATELOGDECREMENT Estimate damping from consecutive positive peaks of y(t).
    y = y(:);
    t = t(:);

    % Find strict local positive maxima above 5% of initial peak amplitude
    peakFloor = 0.05 * max(abs(y));
    peakIdx   = [];
    for k = 2:numel(y) - 1
        if y(k) > y(k - 1) && y(k) >= y(k + 1) && y(k) > peakFloor
            peakIdx(end + 1) = k; %#ok<AGROW>
        end
    end
    % Include t = 0 if signal starts at a clean release peak (y(1) > y(2))
    if numel(y) >= 2 && y(1) > y(2) && y(1) > peakFloor
        peakIdx = [1, peakIdx];
    end

    if numel(peakIdx) < 2
        error('Furuta:InsufficientOscillationPeaks', ...
            'Need at least 2 positive peaks to compute logarithmic decrement (found %d).', ...
            numel(peakIdx));
    end

    % Parabolic sub-sample refinement for interior peaks to reduce Ts quantization
    nPeaks = numel(peakIdx);
    tPeaks = zeros(nPeaks, 1);
    aPeaks = zeros(nPeaks, 1);
    for i = 1:nPeaks
        idx = peakIdx(i);
        if idx > 1 && idx < numel(y)
            yL = y(idx - 1); yC = y(idx); yR = y(idx + 1);
            denom = (yL - 2*yC + yR);
            if abs(denom) > 1e-15
                deltaIdx = 0.5 * (yL - yR) / denom;
                dtLocal  = t(idx + 1) - t(idx);
                tPeaks(i) = t(idx) + deltaIdx * dtLocal;
                aPeaks(i) = yC - 0.25 * (yL - yR) * deltaIdx;
            else
                tPeaks(i) = t(idx);
                aPeaks(i) = yC;
            end
        else
            tPeaks(i) = t(idx);
            aPeaks(i) = y(idx);
        end
    end

    % Fit log(A_k) = c0 - delta * k and t_k = t0 + T_d * k across all detected cycles
    kIdx    = (0:nPeaks - 1).';
    pLog    = polyfit(kIdx, log(aPeaks), 1);
    delta   = -pLog(1);
    pTime   = polyfit(kIdx, tPeaks, 1);
    Td      = pTime(1);
    omega_d = 2 * pi / Td;

    zeta    = delta / sqrt(4 * pi^2 + delta^2);
    omega_n = omega_d / sqrt(max(1 - zeta^2, eps));
    sigma   = zeta * omega_n;

    damp = struct( ...
        'num_peaks',   nPeaks, ...
        't_peaks',     tPeaks, ...
        'amp_peaks',   aPeaks, ...
        'Td',          Td, ...
        'omega_d',     omega_d, ...
        'delta',       delta, ...
        'zeta',        zeta, ...
        'omega_n',     omega_n, ...
        'sigma',       sigma, ...
        'b1_uncoupled', NaN);

    if isfield(params, 'J1') && isfield(params, 'm1') && isfield(params, 'l1')
        J1_hat = params.J1 + params.m1 * params.l1^2;
        damp.b1_uncoupled = 2 * sigma * J1_hat;
    end
end

function growth = estimateToppleGrowthRate(t, theta1)
%ESTIMATETOPPLEGROWTHRATE Fit unstable upright mode rate lambda from ln|theta1(t)|.
%   Uses the second half of the short small-angle topple window (where the stable
%   decaying mode exp(-lambda_s * t) has already faded) while restricting to
%   |theta1| <= 15 deg so small-angle exponential growth dominates.
    t = t(:);
    a = abs(theta1(:));

    tMid = 0.5 * (t(1) + t(end));
    mask = (t >= tMid) & (a > 1e-6) & (a <= deg2rad(20));
    if nnz(mask) < 5
        mask = (t >= tMid) & (a > 1e-6);
    end
    if nnz(mask) < 3
        error('Furuta:InsufficientGrowthSamples', ...
            'Need at least 3 valid samples to fit upright topple growth rate.');
    end

    tFit = t(mask);
    yFit = log(a(mask));
    p    = polyfit(tFit, yFit, 1);
    yHat = polyval(p, tFit);

    ssRes = sum((yFit - yHat).^2);
    ssTot = sum((yFit - mean(yFit)).^2);
    r2    = 1 - ssRes / max(ssTot, eps);

    growth = struct( ...
        'lambda_hat', p(1), ...
        'r_squared',  r2, ...
        't_window',   [tFit(1), tFit(end)], ...
        'num_points', numel(tFit));
end
