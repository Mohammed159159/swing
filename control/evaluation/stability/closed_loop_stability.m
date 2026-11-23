%% Furuta Pendulum: Closed-Loop Stability, Pole-Zero & Gain/Phase Margin Analysis
% This Live Script analyzes the closed-loop linear stability of all three
% stabilizing controllers — *PID* (Single-Loop $\theta_1$), *Pole Placement*,
% and *LQR* — about the upright inverted apex ($\theta_1 = 0$), mirroring
% |open_loop_stability.mlx| and adding *Gain Margin* and *Phase Margin* checks
% (Section 4.2.3 of the project specification):
%%
% # *Closed-Loop Eigenvalues:* Pole locations $\lambda_{\text{cl}}$, damping ratios $\zeta$, and modal stability verdicts.
% # *Closed-Loop Zeros & Pole–Zero Map:* Dominant closed-loop pole migration vs. open-loop and non-minimum phase zero check.
% # *Closed-Loop Linear Response:* Disturbance recovery ($\theta_1(0) = 10^\circ$) and arm setpoint step ($\theta_{0,\text{ref}} = 30^\circ$).
% # *Return-Ratio Bode Plots + Gain & Phase Margins:* Magnitude/phase of $L_{\text{PID}}(j\omega)$, $L_{\text{PP}}(j\omega)$, and $L_{\text{LQR}}(j\omega)$ with lower/upper GM [dB] and PM [deg].
%
% *Symbol Glossary:*
%%
% * $A_{\text{cl}} = A_{\text{up}} - B_{\text{up}} K$: Closed-loop state matrix for full-state feedback (Pole Placement and LQR).
% * $L_{\text{PID}}(s) = -C_{\text{PID}}(s)\,G_{\theta_1}(s)$ and $L_{\text{SF}}(s) = K(sI - A_{\text{up}})^{-1}B_{\text{up}}$: Loop return-ratio transfer functions.
% * $\mathrm{GM}_{\text{low}},\;\mathrm{GM}_{\text{up}}$ [dB]: Lower and upper gain margins (open-loop unstable plants have a strict lower GM bound where reducing loop gain destabilizes the inverted mode).
% * $\mathrm{PM}$ [deg], $\omega_{cp}$ [rad/s]: Phase margin and gain-crossover frequency.
%% 0. Setup, Plant & Synthesized Gains
% Load SSOT parameters from |load_parameters()|, build the analytical plant with
% |build_plant()|, synthesize all controller gains via |compute_gains()|, and run
% |analyze_closed_loop_stability(plant, params, gains)|.

ensureProjectPaths();

params = load_parameters();
plant  = build_plant(params);
gains  = compute_gains(plant, params);
rep    = analyze_closed_loop_stability(plant, params, gains);
st     = closedLoopStyle();

fprintf('Loaded plant & gains (N = %g, V_max = %.1f V, Ts = %.1f ms)\n', ...
    params.N, params.V_max, params.Ts * 1e3);
fprintf('  PID (single-loop theta1): Kp = %.2f, Ki = %.2f (Ki_min = %.2f), Kd = %.2f, Nf = %.1f rad/s\n', ...
    gains.pid.Kp, gains.pid.Ki, gains.pid.Ki_min, gains.pid.Kd, gains.pid.Nf);
fprintf('  Pole Placement:           K_pp  = [%+.4f, %+.4f, %+.4f, %+.4f]\n', gains.pp.K);
fprintf('  LQR (Bryson''s Rule):      K_lqr = [%+.4f, %+.4f, %+.4f, %+.4f]\n', gains.lqr.K);
%% 1. Closed-Loop Eigenvalues & Modal Properties
% Comparing the closed-loop eigenvalues against the open-loop upright unstable
% pole ($p_{\text{RHP}} = +10.77\text{ rad/s}$) verifies that all three
% controllers relocate the unstable topple mode strictly into the open left
% half-plane:
%%
% * *PID* (Single-Loop $\theta_1$): Relocates all controlled pendulum/motor/filter poles strictly into the LHP once $K_i > K_{i,\min} = 78.62$ (Routh–Hurwitz back-EMF condition), while leaving the cyclic arm position pole $\lambda = 0$ uncontrolled.
% * *Pole Placement:* Places all four closed-loop poles at the exact design targets ($\zeta = 0.707$, $\omega_n = 10\text{ rad/s}$, $p_{\text{arm}} = -3.5\text{ rad/s}$, and preserved fast motor pole), with |place| vs. |acker| error $< 10^{-11}$.
% * *LQR:* Optimal Riccati feedback places all four poles strictly in the LHP with high damping and verifies ARE residual $\approx 0$.

modalTable = buildClosedLoopModalTable(rep);
disp(modalTable);

fprintf('PID Routh-Hurwitz margin: Ki / Ki_min = %.2f (Ki = %.2f > %.2f)\n', ...
    rep.pid.Ki_ratio, gains.pid.Ki, rep.pid.Ki_min);
fprintf('Pole Placement accuracy:  max|eig - p_des| = %.3e rad/s, ||K_place - K_acker|| = %.3e\n', ...
    rep.pp.max_pole_err, rep.pp.acker_diff);
fprintf('LQR Riccati check:        ||ARE residual||_F = %.3e, min eig(P) = %.3e > 0\n', ...
    rep.lqr.are_residual, rep.lqr.min_eig_P);
%% 2. Closed-Loop Zeros & Pole–Zero Map
% Inspecting the closed-loop poles and transmission zeros in the dominant
% region highlights two structural insights:
%%
% * Full-state feedback moves all four poles to the LHP but *cannot move open-loop transmission zeros* — hence the upright non-minimum phase RHP zero $z_{\text{RHP}} > 0$ in $\theta_{0,\text{ref}} \to \theta_0$ remains unchanged for both Pole Placement and LQR.
% * In single-loop PID, the controller introduces two LHP zeros that shape the transient response of $\theta_1$, while the uncontrolled arm pole remains at the origin.

zeroTable = buildClosedLoopZeroTable(rep);
disp(zeroTable);

plotClosedLoopPoleZeroMap(plant, rep, st);
%% 3. Closed-Loop Linear Time Response
% Calling |simulate_linear()| with |Controller = "pid"|, |"pole_placement"|,
% or |"lqr"| verifies the linear closed-loop behavior directly in the time domain:
%%
% * *Top & Bottom-Left* ($10^\circ$ upright disturbance): All three controllers drive $\theta_1 \to 0^\circ$ smoothly. Pole Placement and LQR also recenter $\theta_0 \to 0^\circ$, whereas single-loop PID leaves $\theta_0$ free to coast.
% * *Bottom-Right* ($30^\circ$ arm setpoint step): Both Pole Placement and LQR track $\theta_{0,\text{ref}} = 30^\circ$ with exact zero steady-state error ($N_{\text{bar}} = K_1$), visibly exhibiting the initial inverse response caused by the upright RHP zero $z_{\text{RHP}}$.

plotClosedLoopLinearResponse(plant, params, gains, st);
%% 4. Loop Return-Ratio Bode Plots & Gain / Phase Margins
% Evaluating the open-loop return ratios $L_{\text{PID}}(s) = -C_{\text{PID}}(s)G_{\theta_1}(s)$,
% $L_{\text{PP}}(s) = K_{\text{PP}}(sI-A)^{-1}B$, and $L_{\text{LQR}}(s) = K_{\text{LQR}}(sI-A)^{-1}B$
% via |allmargin()| quantifies stability robustness:
%%
% * *Lower Gain Margin* ($\mathrm{GM}_{\text{low}} < 0\text{ dB}$): Because the open-loop upright plant has one RHP pole, reducing loop gain too far destabilizes the system. All three controllers provide strong downward gain margin.
% * *LQR Guarantees:* LQR achieves $\mathrm{GM}_{\text{low}} \le -6\text{ dB}$, $\mathrm{GM}_{\text{up}} = +\infty\text{ dB}$, and $\mathrm{PM} \ge 60^\circ$, confirming Kalman's theoretical optimal robustness bounds.
% * *Pole Placement Trade-Off:* Arbitrary pole assignment achieves fast well-damped poles but yields tighter gain and phase margins than LQR.

summaryTable = rep.summary_table;
disp(summaryTable);

plotLoopBodeAndMargins(rep, st);

%% Local Helper Functions (Tables & Figures)

function tbl = buildClosedLoopModalTable(rep)
    list = {rep.pid, rep.pp, rep.lqr};
    nTot = sum(cellfun(@(r) numel(r.poles), list));
    Ctrl = strings(nTot, 1);
    Mode = strings(nTot, 1);
    Pole = strings(nTot, 1);
    Wn   = zeros(nTot, 1);
    Zeta = zeros(nTot, 1);
    Tau  = zeros(nTot, 1);

    row = 0;
    for i = 1:numel(list)
        r = list{i};
        for k = 1:numel(r.poles)
            row = row + 1;
            Ctrl(row) = r.short_name;
            Mode(row) = r.mode_labels(k);
            Pole(row) = formatComplex(r.poles(k));
            Wn(row)   = r.natural_freqs(k);
            Zeta(row) = r.damping_ratios(k);
            Tau(row)  = r.time_constants_s(k);
        end
    end

    tbl = table(Ctrl, Mode, Pole, Wn, Zeta, Tau, ...
        'VariableNames', {'Controller', 'Mode_Description', 'Eigenvalue_rad_s', ...
        'Omega_n_rad_s', 'Damping_Zeta', 'Time_Const_s'});
end

function tbl = buildClosedLoopZeroTable(rep)
    list = {rep.pid, rep.pp, rep.lqr};
    Ctrl = strings(3, 1);
    ZTh0 = strings(3, 1);
    ZTh1 = strings(3, 1);
    Note = strings(3, 1);

    for k = 1:3
        r       = list{k};
        Ctrl(k) = r.name;
        if k == 1
            ZTh0(k) = "N/A (theta1-only)";
            ZTh1(k) = formatComplexList(r.zeros_theta1);
            Note(k) = "Controller LHP zeros shape theta1 recovery";
        else
            ZTh0(k) = formatComplexList(r.zeros_theta0);
            ZTh1(k) = formatComplexList(r.zeros_theta1);
            Note(k) = "Upright NMP RHP zero persists in theta0 channel";
        end
    end

    tbl = table(Ctrl, ZTh0, ZTh1, Note, ...
        'VariableNames', {'Controller', 'Ref_to_theta0_Zeros', 'Ref_to_theta1_Zeros', 'Physical_Interpretation'});
end

function plotClosedLoopPoleZeroMap(plant, rep, st)
    fig = newStyledFigure('Figure 1: Closed-Loop Pole-Zero Map', [80, 80, 960, 420]);
    tl  = tiledlayout(fig, 1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, 'Figure 1 — Closed-Loop Pole–Zero Maps (Dominant Region vs. Open-Loop RHP Pole)', ...
        'FontSize', st.superTitleFont, 'FontWeight', 'bold', 'Color', st.textColor);

    list   = {rep.pid, rep.pp, rep.lqr};
    colors = {st.amber, st.teal, st.nl};
    olRhp  = max(real(plant.eig_upright));

    xLim = [-45, 16];
    yLim = [-22, 22];

    for k = 1:3
        r  = list{k};
        ax = nexttile(tl, k);

        hRhp = patch(ax, [0, xLim(2), xLim(2), 0], [yLim(1), yLim(1), yLim(2), yLim(2)], ...
            st.lin, 'FaceAlpha', 0.08, 'EdgeColor', 'none'); hold(ax, 'on');
        xline(ax, 0, '-', 'Color', [0.45, 0.45, 0.45], 'LineWidth', 0.9);
        yline(ax, 0, '-', 'Color', [0.45, 0.45, 0.45], 'LineWidth', 0.7);
        % Damping zeta = 0.707 cone (45 deg)
        hCone = plot(ax, [xLim(1), 0, xLim(1)], [xLim(1), 0, -xLim(1)], ':', ...
            'Color', [0.55, 0.58, 0.62], 'LineWidth', 0.9);

        % Open-loop unstable pole reference
        hOL = plot(ax, olRhp, 0, 's', 'Color', st.lin, 'LineWidth', 1.4, 'MarkerSize', 7.5);

        % Closed-loop poles inside dominant window
        inWin = real(r.poles) >= xLim(1) & real(r.poles) <= xLim(2);
        pDom  = r.poles(inWin);
        pFast = r.poles(~inWin);
        hCL   = plot(ax, real(pDom), imag(pDom), 'x', 'Color', colors{k}, ...
            'LineWidth', 2.2, 'MarkerSize', 9.0);

        % Closed-loop zeros (theta0 for state feedback, theta1 for PID)
        if k == 1
            zPlot = r.zeros_theta1;
        else
            zPlot = r.zeros_theta0;
        end
        zPlot = zPlot(real(zPlot) >= xLim(1) & real(zPlot) <= xLim(2));
        if ~isempty(zPlot)
            hZ = plot(ax, real(zPlot), imag(zPlot), 'o', 'Color', st.slate, ...
                'LineWidth', 1.4, 'MarkerSize', 7.0, 'MarkerFaceColor', 'none');
        end

        if ~isempty(pFast)
            fastTxt = sprintf('Fast pole(s) off-scale:\n%s', formatComplexList(pFast));
            text(ax, xLim(1) + 2, yLim(2) - 2.5, fastTxt, 'FontSize', 7.8, ...
                'Color', st.slate, 'VerticalAlignment', 'top', 'BackgroundColor', 'w', 'Margin', 1.5);
        end

        xlim(ax, xLim); ylim(ax, yLim);
        xlabel(ax, '$\mathrm{Re}(s)\;[\mathrm{rad/s}]$', 'Interpreter', 'latex');
        if k == 1
            ylabel(ax, '$\mathrm{Im}(s)\;[\mathrm{rad/s}]$', 'Interpreter', 'latex');
        end
        title(ax, r.name, 'FontSize', st.titleFont);
        styleAxes(ax, st);
    end

    attachSharedLegend([hCL, hOL, hZ, hCone, hRhp], ...
        {'Closed-Loop Poles', 'Open-Loop RHP Pole (+10.8)', 'Closed-Loop Zeros', ...
         '\zeta = 0.707 Cone', 'Unstable RHP'}, 5, st);
end

function plotClosedLoopLinearResponse(plant, params, gains, st)
    t_dist = (0:params.Ts:3.0).';
    x0_10  = [0; 0; deg2rad(10); 0];

    r_pid = simulate_linear(plant, x0_10, t_dist, 0, Equilibrium="upright", Controller="pid",            Gains=gains);
    r_pp  = simulate_linear(plant, x0_10, t_dist, 0, Equilibrium="upright", Controller="pole_placement", Gains=gains);
    r_lqr = simulate_linear(plant, x0_10, t_dist, 0, Equilibrium="upright", Controller="lqr",            Gains=gains);

    t_step = (0:params.Ts:4.0).';
    x_ref  = [deg2rad(30); 0; 0; 0];
    s_pp   = simulate_linear(plant, zeros(4, 1), t_step, 0, Equilibrium="upright", Controller="pole_placement", Gains=gains, Reference=x_ref);
    s_lqr  = simulate_linear(plant, zeros(4, 1), t_step, 0, Equilibrium="upright", Controller="lqr",            Gains=gains, Reference=x_ref);

    fig = newStyledFigure('Figure 2: Closed-Loop Linear Responses', [90, 90, 960, 560]);
    tl  = tiledlayout(fig, 2, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, 'Figure 2 — Closed-Loop Linear Response: 10^\circ Recovery & 30^\circ Arm Tracking', ...
        'FontSize', st.superTitleFont, 'FontWeight', 'bold', 'Color', st.textColor);

    % Top-Left: Pendulum recovery theta1(t)
    ax1 = nexttile(tl, 1);
    hPid = plot(ax1, r_pid.t, rad2deg(r_pid.x(:, 3)), '-.', 'Color', st.amber, 'LineWidth', st.lwMain); hold(ax1, 'on');
    hPP  = plot(ax1, r_pp.t,  rad2deg(r_pp.x(:, 3)),  '--', 'Color', st.teal,  'LineWidth', st.lwMain);
    hLQR = plot(ax1, r_lqr.t, rad2deg(r_lqr.x(:, 3)), '-',  'Color', st.nl,    'LineWidth', st.lwMain);
    yline(ax1, 0, ':', 'Color', [0.55, 0.55, 0.55], 'LineWidth', 0.8);
    ylabel(ax1, '$\theta_1\;[\mathrm{deg}]$', 'Interpreter', 'latex');
    title(ax1, '10^\circ Disturbance: Pendulum Angle \theta_1(t)', 'FontSize', st.titleFont);
    styleAxes(ax1, st);

    % Top-Right: Arm angle theta0(t) during 10 deg recovery
    ax2 = nexttile(tl, 2);
    plot(ax2, r_pid.t, rad2deg(r_pid.x(:, 1)), '-.', 'Color', st.amber, 'LineWidth', st.lwMain); hold(ax2, 'on');
    plot(ax2, r_pp.t,  rad2deg(r_pp.x(:, 1)),  '--', 'Color', st.teal,  'LineWidth', st.lwMain);
    plot(ax2, r_lqr.t, rad2deg(r_lqr.x(:, 1)), '-',  'Color', st.nl,    'LineWidth', st.lwMain);
    yline(ax2, 0, ':', 'Color', [0.55, 0.55, 0.55], 'LineWidth', 0.8);
    ylabel(ax2, '$\theta_0\;[\mathrm{deg}]$', 'Interpreter', 'latex');
    title(ax2, '10^\circ Disturbance: Arm Angle \theta_0(t)', 'FontSize', st.titleFont);
    styleAxes(ax2, st);

    % Bottom-Left: Linear control voltage Va(t)
    ax3 = nexttile(tl, 3);
    plot(ax3, r_pid.t, r_pid.u, '-.', 'Color', st.amber, 'LineWidth', st.lwMain); hold(ax3, 'on');
    plot(ax3, r_pp.t,  r_pp.u,  '--', 'Color', st.teal,  'LineWidth', st.lwMain);
    plot(ax3, r_lqr.t, r_lqr.u, '-',  'Color', st.nl,    'LineWidth', st.lwMain);
    yline(ax3,  params.V_max, ':', 'Color', st.lin, 'LineWidth', 1.0);
    yline(ax3, -params.V_max, ':', 'Color', st.lin, 'LineWidth', 1.0);
    xlabel(ax3, 'Time [s]');
    ylabel(ax3, '$V_a\;[\mathrm{V}]$', 'Interpreter', 'latex');
    ylim(ax3, [-16, 26]);
    title(ax3, '10^\circ Disturbance: Linear Voltage V_a(t)', 'FontSize', st.titleFont);
    styleAxes(ax3, st);

    % Bottom-Right: 30 deg arm step response (PP vs LQR)
    ax4 = nexttile(tl, 4);
    plot(ax4, s_pp.t,  rad2deg(s_pp.x(:, 1)),  '--', 'Color', st.teal, 'LineWidth', st.lwMain); hold(ax4, 'on');
    plot(ax4, s_lqr.t, rad2deg(s_lqr.x(:, 1)), '-',  'Color', st.nl,   'LineWidth', st.lwMain);
    yline(ax4, 30, ':', 'Color', st.slate, 'LineWidth', 1.0);
    xlabel(ax4, 'Time [s]');
    ylabel(ax4, '$\theta_0\;[\mathrm{deg}]$', 'Interpreter', 'latex');
    title(ax4, '30^\circ Arm Step (PP vs. LQR — NMP Inverse Kick)', 'FontSize', st.titleFont);
    styleAxes(ax4, st);

    attachSharedLegend([hPid, hPP, hLQR], {'PID (theta1-only)', 'Pole Placement', 'LQR'}, 3, st);
end

function plotLoopBodeAndMargins(rep, st)
    wGrid = logspace(-1, 4, 900).';
    list   = {rep.pid, rep.pp, rep.lqr};
    colors = {st.amber, st.teal, st.nl};
    bodeTicks = [1e-1, 1e0, 1e1, 1e2, 1e3, 1e4];

    fig = newStyledFigure('Figure 3: Loop Transfer Function Bode Plots & Margins', [100, 100, 1080, 560]);
    tl  = tiledlayout(fig, 2, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    title(tl, 'Figure 3 — Open-Loop Return Ratio Bode Diagrams & Stability Margins (GM / PM)', ...
        'FontSize', st.superTitleFont, 'FontWeight', 'bold', 'Color', st.textColor);

    for k = 1:3
        r = list{k};
        m = r.margins;
        [magDb, phDeg] = evalBodeDeg(r.loop_tf, wGrid);
        % Wrap phase cleanly around -180 deg for readability
        phPlot = unwrapPhaseNearMinus180(phDeg);

        % Magnitude row
        axM = nexttile(tl, k);
        semilogx(axM, wGrid, magDb, '-', 'Color', colors{k}, 'LineWidth', st.lwMain); hold(axM, 'on');
        yline(axM, 0, ':', 'Color', st.slate, 'LineWidth', 1.0);
        if isfinite(m.w_cp_rad_s)
            xline(axM, m.w_cp_rad_s, '--', 'Color', st.slate, 'LineWidth', 0.9);
            plot(axM, m.w_cp_rad_s, 0, 'o', 'Color', colors{k}, 'MarkerFaceColor', 'w', 'MarkerSize', 5.5);
        end
        if isfinite(m.w_cg_low_rad_s)
            plot(axM, m.w_cg_low_rad_s, -m.lower_gm_dB, 'v', 'Color', st.lin, 'MarkerFaceColor', st.lin, 'MarkerSize', 5.5);
        end
        if isfinite(m.w_cg_up_rad_s)
            plot(axM, m.w_cg_up_rad_s, -m.upper_gm_dB, '^', 'Color', st.lin, 'MarkerFaceColor', st.lin, 'MarkerSize', 5.5);
        end
        xlim(axM, [wGrid(1), wGrid(end)]); xticks(axM, bodeTicks);
        if k == 1, ylabel(axM, 'Magnitude [dB]'); end
        title(axM, sprintf('%s\nGM = [%s, %s] dB', r.short_name, fmtMargin(m.lower_gm_dB), fmtMargin(m.upper_gm_dB)), ...
            'FontSize', st.titleFont);
        styleAxes(axM, st);

        % Phase row
        axP = nexttile(tl, k + 3);
        semilogx(axP, wGrid, phPlot, '-', 'Color', colors{k}, 'LineWidth', st.lwMain); hold(axP, 'on');
        yline(axP, -180, ':', 'Color', st.lin, 'LineWidth', 1.0);
        if isfinite(m.w_cp_rad_s)
            xline(axP, m.w_cp_rad_s, '--', 'Color', st.slate, 'LineWidth', 0.9);
            phAtWcp = interp1(log10(wGrid), phPlot, log10(m.w_cp_rad_s));
            plot(axP, [m.w_cp_rad_s, m.w_cp_rad_s], [-180, phAtWcp], '-', 'Color', st.lin, 'LineWidth', 2.0);
            plot(axP, m.w_cp_rad_s, phAtWcp, 'o', 'Color', colors{k}, 'MarkerFaceColor', 'w', 'MarkerSize', 5.5);
        end
        xlim(axP, [wGrid(1), wGrid(end)]); xticks(axP, bodeTicks);
        xlabel(axP, '\omega [rad/s]');
        if k == 1, ylabel(axP, 'Phase [deg]'); end
        title(axP, sprintf('PM = %.1f^\\circ @ \\omega_{cp} = %.1f rad/s', m.pm_deg, m.w_cp_rad_s), ...
            'FontSize', st.titleFont);
        styleAxes(axP, st);
    end
end

function [magDb, phaseDeg] = evalBodeDeg(sysSiso, wGrid)
    [mag, phase] = bode(sysSiso, wGrid);
    magDb    = 20 * log10(max(squeeze(mag), 1e-16));
    phaseDeg = squeeze(phase);
end

function phOut = unwrapPhaseNearMinus180(phDeg)
    % Shift by multiples of 360 so the median phase sits in [-270, +90] deg
    medPh = median(phDeg);
    shift = 360 * round((medPh + 90) / 360);
    phOut = phDeg - shift;
end

function s = fmtMargin(val)
    if isinf(val)
        if val > 0, s = '+Inf'; else, s = '-Inf'; end
    else
        s = sprintf('%+.1f', val);
    end
end

function st = closedLoopStyle()
    st = struct();
    st.nl             = [0.00, 0.32, 0.61];  % Deep navy (LQR)
    st.lin            = [0.84, 0.25, 0.12];  % Vermilion (RHP / Limits / Margins)
    st.teal           = [0.00, 0.58, 0.45];  % Teal (Pole Placement)
    st.amber          = [0.90, 0.60, 0.00];  % Warm amber (PID)
    st.slate          = [0.22, 0.26, 0.31];  % Dark slate (Annotations)
    st.textColor      = [0.12, 0.14, 0.17];
    st.lwMain         = 1.65;
    st.lwSec          = 1.35;
    st.axisFont       = 9.0;
    st.titleFont      = 9.5;
    st.superTitleFont = 11.0;
end

function fig = newStyledFigure(figName, pos)
    fig = figure('Name', figName, 'Color', 'w', 'Position', pos);
end

function styleAxes(ax, st)
    box(ax, 'off');
    grid(ax, 'on');
    ax.GridAlpha  = 0.18;
    ax.FontSize   = st.axisFont;
    ax.LineWidth  = 0.85;
    ax.TickDir    = 'out';
    ax.XColor     = st.textColor;
    ax.YColor     = st.textColor;
end

function attachSharedLegend(handles, labels, nCols, st)
    lgd = legend(handles, labels, 'NumColumns', nCols, ...
        'Box', 'off', 'FontSize', st.axisFont - 0.5);
    lgd.Layout.Tile = 'north';
end

function s = formatComplex(z)
    if abs(imag(z)) < 1e-6
        s = sprintf('%.3f', real(z));
    else
        s = sprintf('%.3f %+.3fj', real(z), imag(z));
    end
end

function s = formatComplexList(v)
    if isempty(v)
        s = "None ([])";
        return;
    end
    parts = strings(numel(v), 1);
    for k = 1:numel(v)
        if abs(v(k)) < 1e-9
            parts(k) = "0";
        else
            parts(k) = formatComplex(v(k));
        end
    end
    s = strjoin(parts, ', ');
end

function ensureProjectPaths()
    if exist('load_parameters', 'file') && exist('compute_gains', 'file') ...
            && exist('analyze_closed_loop_stability', 'file') && exist('simulate_linear', 'file')
        return;
    end
    cur = pwd;
    for k = 1:6
        if exist(fullfile(cur, 'startup.m'), 'file')
            run(fullfile(cur, 'startup.m'));
            return;
        end
        parent = fileparts(cur);
        if strcmp(parent, cur), break; end
        cur = parent;
    end
    error('Furuta:RepoRootNotFound', 'Could not locate repository root containing startup.m.');
end
