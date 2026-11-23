function rep = analyze_open_loop_stability(plant, params)
%ANALYZE_OPEN_LOOP_STABILITY Open-loop eigenvalue, zero, controllability & observability analysis.
%   rep = ANALYZE_OPEN_LOOP_STABILITY(plant, params) evaluates both the upright
%   (theta1_eq = 0) and downward (theta1_eq = pi) linearized Furuta state-space
%   models returned by build_plant(params).
%
%   For each equilibrium (rep.upright and rep.downward), computes:
%     - Eigenvalues (poles), natural frequencies, damping ratios, time constants,
%       and open-loop stability classification.
%     - MIMO transmission zeros tzero(sys), SISO Va -> theta0 zeros tzero(sys(1,1))
%       (exposing the upright non-minimum phase RHP zero), and minimal-realization
%       SISO Va -> theta1 zeros tzero(minreal(sys(2,1))) plus B4 input polarity.
%     - Kalman controllability matrix ctrb(A, B) and rank.
%     - Kalman observability matrix obsv(A, C) and rank for both encoders,
%       theta0-only, and theta1-only sensor configurations.

    if nargin < 2 || isempty(params)
        params = load_parameters();
    end
    if nargin < 1 || isempty(plant)
        plant = build_plant(params);
    end
    if ~isstruct(plant) || ~isfield(plant, 'upright') || ~isfield(plant, 'downward')
        error('Furuta:InvalidPlant', 'plant must contain .upright and .downward structs from build_plant().');
    end

    rep = struct();
    rep.upright  = analyzeEquilibrium(plant.upright,  'upright',  params);
    rep.downward = analyzeEquilibrium(plant.downward, 'downward', params);
end

function eqRep = analyzeEquilibrium(eq, eqName, params)
    A   = eq.A;
    B   = eq.B;
    C   = eq.C;
    sys = eq.sys;
    n   = size(A, 1);

    % 1. Eigenvalues (open-loop poles)
    rawPoles = eig(A);
    orderIdx = sortPoleIndices(rawPoles);
    poles    = rawPoles(orderIdx);

    modeLabels = classifyModes(poles, eqName);
    wn         = abs(poles);
    zeta       = nan(n, 1);
    tau_s      = nan(n, 1);
    for k = 1:n
        if wn(k) > 1e-9
            zeta(k) = -real(poles(k)) / wn(k);
        end
        if abs(real(poles(k))) > 1e-9
            tau_s(k) = 1.0 / abs(real(poles(k)));
        end
    end

    rhpPoles = poles(real(poles) > 1e-6);
    if ~isempty(rhpPoles)
        stabClass = "Unstable (1 RHP pole)";
        pRhp      = max(real(rhpPoles));
        toppleTau = 1.0 / pRhp;
        bwFloor   = 2.0 * pRhp;
    else
        stabClass = "Marginally Stable (cyclic theta0 + 3 LHP)";
        pRhp      = NaN;
        toppleTau = NaN;
        bwFloor   = NaN;
    end

    % 2. Open-Loop Zeros & Non-Minimum Phase Check
    z_mimo      = tzero(sys);
    z_th0       = tzero(sys(1, 1));
    sys_th1_min = minreal(sys(2, 1), 1e-9, false);
    z_th1       = tzero(sys_th1_min);

    rhpZerosTh0 = z_th0(real(z_th0) > 1e-6);
    isNmpTh0    = ~isempty(rhpZerosTh0);
    if isNmpTh0
        zRhpTh0   = max(real(rhpZerosTh0));
        bwCeilTh0 = 0.5 * zRhpTh0;
    else
        zRhpTh0   = NaN;
        bwCeilTh0 = NaN;
    end

    % 3. Controllability (Kalman rank test)
    Cmat  = ctrb(A, B);
    cRank = rank(Cmat);

    % 4. Observability (Kalman rank test across encoder subsets)
    Omat   = obsv(A, C);
    oRank  = rank(Omat);
    oRank0 = rank(obsv(A, [1, 0, 0, 0]));
    oRank1 = rank(obsv(A, [0, 0, 1, 0]));

    eqRep = struct();
    eqRep.equilibrium             = string(eqName);
    eqRep.poles                   = poles;
    eqRep.mode_labels             = modeLabels;
    eqRep.natural_freqs           = wn;
    eqRep.damping_ratios          = zeta;
    eqRep.time_constants_s        = tau_s;
    eqRep.stability_class         = stabClass;
    eqRep.rhp_pole                = pRhp;
    eqRep.topple_time_const_s     = toppleTau;
    eqRep.sample_ratio_tau_to_Ts  = toppleTau / params.Ts;

    eqRep.zeros_mimo              = z_mimo(:);
    eqRep.zeros_theta0            = z_th0(:);
    eqRep.zeros_theta1            = z_th1(:);
    eqRep.is_nmp_theta0           = isNmpTh0;
    eqRep.rhp_zero_theta0         = zRhpTh0;
    eqRep.b4_gain                 = B(4);
    eqRep.b4_sign                 = sign(B(4));
    eqRep.bw_floor_rad_s          = bwFloor;
    eqRep.bw_ceiling_theta0_rad_s = bwCeilTh0;

    eqRep.ctrb = struct( ...
        'matrix',          Cmat, ...
        'rank',            cRank, ...
        'is_controllable', cRank == n);

    eqRep.obsv = struct( ...
        'matrix',           Omat, ...
        'rank',             oRank, ...
        'is_observable',    oRank == n, ...
        'rank_theta0_only', oRank0, ...
        'rank_theta1_only', oRank1);
end

function idx = sortPoleIndices(p)
    n    = numel(p);
    used = false(n, 1);
    idx  = zeros(n, 1);

    % 1. Cyclic zero pole (smallest magnitude)
    [~, iZero] = min(abs(p));
    idx(1) = iZero; used(iZero) = true;

    % 2. Fast real LHP motor-arm pole (most negative real part among remaining)
    remIdx = find(~used);
    [~, relFast] = min(real(p(remIdx)));
    iFast = remIdx(relFast);
    idx(2) = iFast; used(iFast) = true;

    % 3 & 4. Remaining pendulum pair sorted by ascending real part (then descending imag)
    remIdx = find(~used);
    remP   = p(remIdx);
    [~, sortRem] = sortrows([real(remP), -imag(remP)]);
    idx(3:end) = remIdx(sortRem);
end

function labels = classifyModes(poles, eqName)
    n      = numel(poles);
    labels = strings(n, 1);
    labels(1) = "Cyclic Arm Rotation (theta0)";
    labels(2) = "Fast Motor-Arm Back-EMF Decay";
    if strcmpi(eqName, 'upright')
        labels(3) = "Inverted Pendulum Stable Convergence";
        labels(4) = "Inverted Pendulum Unstable Topple (RHP)";
    else
        labels(3) = "Hanging Pendulum Oscillation (+j)";
        labels(4) = "Hanging Pendulum Oscillation (-j)";
    end
end
