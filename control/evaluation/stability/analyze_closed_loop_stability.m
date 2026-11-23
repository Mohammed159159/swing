function rep = analyze_closed_loop_stability(plant, params, gains)
%ANALYZE_CLOSED_LOOP_STABILITY Closed-loop eigenvalue, zero & Bode GM/PM analysis.
%   rep = ANALYZE_CLOSED_LOOP_STABILITY(plant, params, gains) evaluates the
%   closed-loop linear stability of all three stabilizing controllers (PID,
%   Pole Placement, and LQR) about the upright equilibrium (theta1 = 0).
%
%   Mirrors analyze_open_loop_stability(plant, params) and adds loop-transfer
%   Gain Margin (lower & upper, in dB) and Phase Margin (deg) via allmargin().
%
%   Outputs:
%     rep.pid, rep.pp, rep.lqr — Per-controller structs containing:
%       .name, .poles, .mode_labels, .natural_freqs, .damping_ratios,
%       .time_constants_s, .is_stable, .stability_class
%       .zeros_theta0, .zeros_theta1
%       .loop_tf (open-loop return ratio L(s) used for Bode & margins)
%       .margins (lower_gm_dB, upper_gm_dB, pm_deg, w_cg_rad_s, w_cp_rad_s, raw)
%     rep.summary_table — Consolidated MATLAB table across PID, PP, and LQR.

    if nargin < 2 || isempty(params)
        params = load_parameters();
    end
    if nargin < 1 || isempty(plant)
        plant = build_plant(params);
    end
    if nargin < 3 || isempty(gains)
        gains = compute_gains(plant, params);
    end

    A = plant.upright.A;
    B = plant.upright.B;
    C = plant.upright.C;

    rep = struct();
    rep.pid = analyzePID(A, B, gains.pid);
    rep.pp  = analyzeStateFeedback('Pole Placement', A, B, C, gains.pp.K, gains.pp, 'pp');
    rep.lqr = analyzeStateFeedback('LQR',            A, B, C, gains.lqr.K, gains.lqr, 'lqr');
    rep.summary_table = buildSummaryTable(rep);
end

function r = analyzePID(A, B, pidG)
    Kp = pidG.Kp; Ki = pidG.Ki; Kd = pidG.Kd; Nf = pidG.Nf;

    % 1. Exact 6-state augmented closed-loop matrix matching pid_controller.slx
    %    z = [theta0; dtheta0; theta1; dtheta1; xi; xf]
    %    e1 = -theta1, d(xi)/dt = e1, d(xf)/dt = Nf*(e1 - xf)
    %    u  = -(Kp*e1 + Ki*xi + Kd*Nf*(e1 - xf))
    K_z   = [0, 0, (Kp + Kd * Nf), 0, -Ki, Kd * Nf];
    A_aug = [[A, zeros(4, 2)] + B * K_z; ...
             0, 0, -1,  0, 0,  0; ...
             0, 0, -Nf, 0, 0, -Nf];
    B_aug = [-B * (Kp + Kd * Nf); 1; Nf];

    % Separate controlled 5-state subsystem [dtheta0; theta1; dtheta1; xi; xf]
    % from the uncontrolled cyclic arm position pole at s = 0
    A_ctrl   = A_aug(2:6, 2:6);
    B_ctrl   = B_aug(2:6);
    p_ctrl   = sortPolesByReal(eig(A_ctrl));
    p_all    = [0; p_ctrl];

    % Drop the exact s = 0 cancellation inside A_ctrl (theta1-channel zero cancels xi)
    % Wait: in A_ctrl, check how many poles are strictly LHP vs exact 0
    strictLhpMask = real(p_ctrl) < -1e-6;
    nZeroCtrl     = sum(abs(p_ctrl) <= 1e-6);
    isStable      = all(real(p_ctrl) <= 1e-6) && all(strictLhpMask | (abs(p_ctrl) <= 1e-6)) && (Ki > pidG.Ki_min);

    if isStable
        stabClass = "Stable in theta1 (cyclic theta0 uncontrolled)";
    else
        stabClass = "Unstable";
    end

    [wn, zeta, tau_s] = computeModalStats(p_all);
    labels = strings(numel(p_all), 1);
    labels(1) = "Uncontrolled Cyclic Arm Angle (theta0)";
    for k = 2:numel(p_all)
        if abs(p_all(k)) <= 1e-6
            labels(k) = "Exact Origin Cancellation (s = 0)";
        elseif abs(real(p_all(k))) > 100
            labels(k) = "Fast Motor / Derivative-Filter Mode";
        else
            labels(k) = "Controlled Pendulum Mode";
        end
    end

    % 2. Closed-loop zeros of theta1_ref -> theta1 (5-state controlled subsystem)
    sys_cl_th1 = minreal(ss(A_ctrl, B_ctrl, [0, 1, 0, 0, 0], 0), 1e-6, false);
    z_th1      = tzero(sys_cl_th1);

    % 3. Exact minimal SISO return ratio L_pid(s) = -C_pid(s) * G_theta1(s)
    %    Clean exact s=0 factor in G_theta1(s) so s/s cancels analytically
    [numG, denG] = tfdata(pidG.G_theta1, 'v');
    b1 = numG(end - 1);
    G_exact = tf([b1, 0], denG);
    % C_pid(s) = Kp + Ki/s + Kd*Nf*s/(s + Nf)
    %          = ((Kp + Kd*Nf)*s^2 + (Kp*Nf + Ki)*s + Ki*Nf) / (s*(s + Nf))
    % After exact s/s cancellation with G_exact(s) = b1*s / denG(s):
    numL = (-b1) * [Kp + Kd * Nf, Kp * Nf + Ki, Ki * Nf];
    denL = conv(denG, [1, Nf]);
    L_pid = tf(numL, denL);

    margins = extractLoopMargins(L_pid);

    r = struct();
    r.name             = "PID (Single-Loop theta1)";
    r.short_name       = "PID";
    r.poles            = p_all;
    r.controlled_poles = p_ctrl(strictLhpMask);
    r.n_origin_cancel  = nZeroCtrl;
    r.mode_labels      = labels;
    r.natural_freqs    = wn;
    r.damping_ratios   = zeta;
    r.time_constants_s = tau_s;
    r.is_stable        = isStable;
    r.stability_class  = stabClass;
    r.zeros_theta0     = [];
    r.zeros_theta1     = z_th1(:);
    r.loop_tf          = L_pid;
    r.margins          = margins;
    r.Ki_min           = pidG.Ki_min;
    r.Ki_ratio         = Ki / pidG.Ki_min;
end

function r = analyzeStateFeedback(label, A, B, C, K, gStruct, tag)
    A_cl   = A - B * K;
    B_cl   = B * K(1); % Scalar arm reference r = theta0_ref with Nbar = K(1)
    sys_cl = ss(A_cl, B_cl, C, zeros(2, 1));

    poles = sortPolesByReal(eig(A_cl));
    [wn, zeta, tau_s] = computeModalStats(poles);
    isStable = all(real(poles) < -1e-6);

    if isStable
        stabClass = "Asymptotically Stable (all 4 poles LHP)";
    else
        stabClass = "Unstable";
    end

    labels = strings(numel(poles), 1);
    for k = 1:numel(poles)
        if abs(real(poles(k))) > 100
            labels(k) = "Fast Motor-Arm Damping Mode";
        elseif abs(imag(poles(k))) > 1e-4
            labels(k) = "Dominant Pendulum Oscillatory Mode";
        else
            labels(k) = "Arm-Centering / Real Pendulum Mode";
        end
    end

    % Closed-loop zeros from theta0_ref -> theta0 and theta0_ref -> theta1
    z_th0 = tzero(sys_cl(1, 1));
    z_th1 = tzero(minreal(sys_cl(2, 1), 1e-6, false));

    % Input-channel loop transfer function L(s) = K * (sI - A)^(-1) * B
    L_sf    = ss(A, B, K, 0);
    margins = extractLoopMargins(L_sf);

    r = struct();
    r.name             = string(label);
    r.short_name       = string(label);
    r.poles            = poles;
    r.controlled_poles = poles;
    r.mode_labels      = labels;
    r.natural_freqs    = wn;
    r.damping_ratios   = zeta;
    r.time_constants_s = tau_s;
    r.is_stable        = isStable;
    r.stability_class  = stabClass;
    r.zeros_theta0     = z_th0(:);
    r.zeros_theta1     = z_th1(:);
    r.loop_tf          = L_sf;
    r.margins          = margins;

    if strcmp(tag, 'pp')
        r.desired_poles = sortPolesByReal(gStruct.desired_poles(:));
        r.max_pole_err  = max(abs(r.poles - r.desired_poles));
        r.acker_diff    = norm(gStruct.K - gStruct.K_acker);
    elseif strcmp(tag, 'lqr')
        P = gStruct.P; Q = gStruct.Q; R = gStruct.R;
        r.are_residual = norm(A.' * P + P * A - P * B * (R \ (B.' * P)) + Q, 'fro');
        r.min_eig_P    = min(real(eig(P)));
    end
end

function m = extractLoopMargins(L)
    am = allmargin(L);

    % Separate lower gain margin (GM < 1 => negative dB) and upper gain margin (GM > 1 => positive dB)
    gmVals = am.GainMargin(:);
    gmFreq = am.GMFrequency(:);
    valid  = isfinite(gmVals) & (gmVals > 0);
    gmVals = gmVals(valid);
    gmFreq = gmFreq(valid);

    lowerMask = gmVals < 1.0;
    upperMask = gmVals > 1.0;

    if any(lowerMask)
        [gmLow, iL] = max(gmVals(lowerMask)); % closest below 1 (strictest lower GM)
        lowFreqs    = gmFreq(lowerMask);
        lowGmDb     = 20 * log10(gmLow);
        wCgLow      = lowFreqs(iL);
    else
        lowGmDb = -Inf;
        wCgLow  = NaN;
    end

    if any(upperMask)
        [gmUp, iU] = min(gmVals(upperMask)); % closest above 1 (strictest upper GM)
        upFreqs    = gmFreq(upperMask);
        upGmDb     = 20 * log10(gmUp);
        wCgUp      = upFreqs(iU);
    else
        upGmDb = Inf;
        wCgUp  = NaN;
    end

    pmVals = am.PhaseMargin(:);
    pmFreq = am.PMFrequency(:);
    validP = isfinite(pmVals);
    pmVals = pmVals(validP);
    pmFreq = pmFreq(validP);

    if ~isempty(pmVals)
        [pmMin, iP] = min(abs(pmVals));
        pmDeg       = pmVals(iP);
        wCp         = pmFreq(iP);
        if pmMin == 0, pmDeg = 0; end
    else
        pmDeg = Inf;
        wCp   = NaN;
    end

    m = struct( ...
        'lower_gm_dB',    lowGmDb, ...
        'upper_gm_dB',    upGmDb, ...
        'w_cg_low_rad_s', wCgLow, ...
        'w_cg_up_rad_s',  wCgUp, ...
        'pm_deg',         pmDeg, ...
        'w_cp_rad_s',     wCp, ...
        'allmargin',      am);
end

function pSorted = sortPolesByReal(p)
    % Sort from slowest (largest real part) to fastest (most negative real part)
    [~, idx] = sortrows([-real(p(:)), -imag(p(:))]);
    pSorted  = p(idx);
end

function [wn, zeta, tau_s] = computeModalStats(poles)
    n     = numel(poles);
    wn    = abs(poles);
    zeta  = nan(n, 1);
    tau_s = nan(n, 1);
    for k = 1:n
        if wn(k) > 1e-9
            zeta(k) = -real(poles(k)) / wn(k);
        end
        if abs(real(poles(k))) > 1e-9
            tau_s(k) = 1.0 / abs(real(poles(k)));
        end
    end
end

function tbl = buildSummaryTable(rep)
    list = {rep.pid, rep.pp, rep.lqr};
    n    = numel(list);
    Ctrl     = strings(n, 1);
    DomPole  = strings(n, 1);
    MinZeta  = nan(n, 1);
    LowGM    = nan(n, 1);
    UpGM     = nan(n, 1);
    PM       = nan(n, 1);
    Wcp      = nan(n, 1);
    Status   = strings(n, 1);

    for k = 1:n
        r       = list{k};
        Ctrl(k) = r.name;
        cp      = r.controlled_poles;
        [~, iD] = max(real(cp));
        dp      = cp(iD);
        if abs(imag(dp)) < 1e-6
            DomPole(k) = sprintf('%.3f', real(dp));
        else
            DomPole(k) = sprintf('%.3f %+.3fj', real(dp), imag(dp));
        end
        zCtrl      = -real(cp) ./ max(abs(cp), eps);
        MinZeta(k) = min(zCtrl);
        LowGM(k)   = r.margins.lower_gm_dB;
        UpGM(k)    = r.margins.upper_gm_dB;
        PM(k)      = r.margins.pm_deg;
        Wcp(k)     = r.margins.w_cp_rad_s;
        if r.is_stable
            Status(k) = "STABLE";
        else
            Status(k) = "UNSTABLE";
        end
    end

    tbl = table(Ctrl, DomPole, MinZeta, LowGM, UpGM, PM, Wcp, Status, ...
        'VariableNames', {'Controller', 'Dominant_Pole_rad_s', 'Min_Damping_Zeta', ...
        'Lower_GM_dB', 'Upper_GM_dB', 'Phase_Margin_deg', 'Wcp_rad_s', 'Verdict'});
end
