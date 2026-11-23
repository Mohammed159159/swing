classdef test_phase2_evaluation < matlab.unittest.TestCase
%TEST_PHASE2_EVALUATION Automated tests for modular controllers, unified
%   closed-loop runners, closed-loop stability analysis, performance metrics,
%   and both Phase 2 Master Live Scripts (closed_loop_stability.mlx &
%   controller_benchmark.mlx).

    properties (Access = private)
        RepoRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.RepoRoot = fileparts(fileparts(mfilename('fullpath')));
            relDirs = { ...
                '', ...
                'config', ...
                fullfile('sim', 'runners'), ...
                fullfile('sim', 'scenarios'), ...
                fullfile('digital-twin', 'validation'), ...
                fullfile('digital-twin', 'models', 'simscape'), ...
                fullfile('digital-twin', 'models', 'simscape', 'stl'), ...
                'control', ...
                fullfile('control', 'controllers'), ...
                fullfile('control', 'controllers', 'classical', 'pid'), ...
                fullfile('control', 'controllers', 'classical', 'pole_placement'), ...
                fullfile('control', 'controllers', 'optimal', 'lqr'), ...
                fullfile('control', 'controllers', 'swingup'), ...
                fullfile('control', 'evaluation', 'stability'), ...
                fullfile('control', 'evaluation', 'performance') ...
            };
            for k = 1:numel(relDirs)
                d = fullfile(testCase.RepoRoot, relDirs{k});
                if exist(d, 'dir')
                    testCase.applyFixture(matlab.unittest.fixtures.PathFixture(d));
                end
            end
        end
    end

    methods (Test)
        function test_modular_controller_designs_and_compute_gains(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            gains  = compute_gains(plant, params);

            % Verify structure and finiteness
            testCase.verifyTrue(isfield(gains, 'pid') && isfield(gains, 'pp') ...
                && isfield(gains, 'lqr') && isfield(gains, 'swingup'));
            testCase.verifyGreaterThan(gains.pid.Ki, gains.pid.Ki_min);
            testCase.verifyEqual(size(gains.pp.K),  [1, 4]);
            testCase.verifyEqual(size(gains.lqr.K), [1, 4]);
            testCase.verifyEqual(gains.pp.K, gains.pp.K_acker, 'AbsTol', 1e-9);
            testCase.verifyTrue(all(isfinite(gains.pp.K)) && all(isfinite(gains.lqr.K)));

            % Verify override isolation (no side effects on default call)
            gains2 = compute_gains(plant, params, LQRArgs={'ControlMax', 1.5});
            testCase.verifyNotEqual(gains2.lqr.K(3), gains.lqr.K(3));
            testCase.verifyEqual(gains.lqr.K, compute_gains(plant, params).lqr.K, 'AbsTol', 1e-15);
        end

        function test_unified_runners_closed_loop_all_controllers(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            gains  = compute_gains(plant, params);
            S      = closed_loop_scenarios(params);
            sc10   = S.upright_disturbance_10deg;

            for cname = ["pid", "pole_placement", "lqr"]
                r_lin = simulate_linear(plant, sc10.x0, sc10.t_grid, 0, ...
                    Equilibrium="upright", Controller=cname, Gains=gains, Reference=sc10.x_ref);
                r_nl  = simulate_nonlinear(plant, params, sc10.x0, sc10.t_grid, 0, ...
                    Controller=cname, Gains=gains, Reference=sc10.x_ref);
                r_ss  = simulate_simscape(params, sc10.x0, sc10.t_grid, 0, ...
                    ModelName="furuta_closed_loop", Controller=cname, Gains=gains, Reference=sc10.x_ref);

                % All 3 runners stabilize theta1 -> 0 deg
                testCase.verifyLessThan(rad2deg(abs(r_lin.x(end, 3))), 0.2);
                testCase.verifyLessThan(rad2deg(abs(r_nl.x(end, 3))),  0.2);
                testCase.verifyLessThan(rad2deg(abs(r_ss.x(end, 3))),  0.5);

                % Saturation & Simscape logsout contract
                testCase.verifyLessThanOrEqual(max(abs(r_nl.u)), params.V_max + 1e-12);
                testCase.verifyEqual(r_ss.meta.log_source, 'logsout');
                testCase.verifyGreaterThan(max(abs(r_ss.u)), 0.5, ...
                    sprintf('Simscape %s must log non-zero control voltage Va.', cname));

                % Full-state controllers also recenter arm theta0 -> 0
                if cname ~= "pid"
                    testCase.verifyLessThan(rad2deg(abs(r_nl.x(end, 1))), 0.2);
                    testCase.verifyLessThan(rad2deg(abs(r_ss.x(end, 1))), 2.0);
                end
            end

            % Verify 30 deg arm setpoint tracking for Pole Placement & LQR
            scArm = S.upright_arm_step;
            for cname = ["pole_placement", "lqr"]
                r_arm = simulate_nonlinear(plant, params, scArm.x0, scArm.t_grid, 0, ...
                    Controller=cname, Gains=gains, Reference=scArm.x_ref);
                testCase.verifyEqual(rad2deg(r_arm.x(end, 1)), 30.0, 'AbsTol', 0.5);
                testCase.verifyLessThan(rad2deg(abs(r_arm.x(end, 3))), 0.2);
            end
        end

        function test_analyze_closed_loop_stability_and_margins(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            gains  = compute_gains(plant, params);
            rep    = analyze_closed_loop_stability(plant, params, gains);

            testCase.verifyTrue(rep.pid.is_stable);
            testCase.verifyTrue(rep.pp.is_stable);
            testCase.verifyTrue(rep.lqr.is_stable);

            % Strict LHP for all controlled poles
            testCase.verifyTrue(all(real(rep.pid.controlled_poles) < 0));
            testCase.verifyTrue(all(real(rep.pp.controlled_poles)  < 0));
            testCase.verifyTrue(all(real(rep.lqr.controlled_poles) < 0));

            % Placement accuracy & LQR ARE / Kalman margin guarantees
            testCase.verifyLessThan(rep.pp.max_pole_err, 1e-9);
            testCase.verifyLessThan(rep.lqr.are_residual, 1e-8);
            testCase.verifyGreaterThan(rep.lqr.min_eig_P, 0);
            testCase.verifyGreaterThanOrEqual(rep.lqr.margins.pm_deg, 60.0);
            testCase.verifyLessThanOrEqual(rep.lqr.margins.lower_gm_dB, -6.0);
            testCase.verifyEqual(rep.lqr.margins.upper_gm_dB, Inf);

            % Summary table contract
            testCase.verifyClass(rep.summary_table, 'table');
            testCase.verifyEqual(height(rep.summary_table), 3);
            testCase.verifyTrue(all(rep.summary_table.Verdict == "STABLE"));
        end

        function test_compute_performance_metrics_exact_synthetic(testCase)
            % Synthetic exponential decay theta1(t) = 0.2 * exp(-2*t), u(t) = 3.0 V on [0, 4] s
            t = linspace(0, 4, 4001).';
            th1 = 0.2 * exp(-2 * t);
            u   = 3.0 * ones(size(t));
            res = struct('t', t, 'x', [zeros(size(t)), zeros(size(t)), th1, -0.4*exp(-2*t)], ...
                         'y', [zeros(size(t)), th1], 'u', u);
            params = load_parameters();
            m = compute_performance_metrics(res, zeros(4, 1), params, Mode="disturbance");

            % Exact integrals on [0, 4]:
            %   IAE  = int_0^4 0.2 e^(-2t) dt = 0.1 * (1 - e^-8)
            %   ISE  = int_0^4 0.04 e^(-4t) dt = 0.01 * (1 - e^-16)
            %   ITAE = int_0^4 0.2 t e^(-2t) dt = 0.2 * (1 - 9 e^-8) / 4
            iae_exact  = 0.1 * (1 - exp(-8));
            ise_exact  = 0.01 * (1 - exp(-16));
            itae_exact = 0.05 * (1 - 9 * exp(-8));

            testCase.verifyEqual(m.theta1_IAE,  iae_exact,  'RelTol', 1e-3);
            testCase.verifyEqual(m.theta1_ISE,  ise_exact,  'RelTol', 1e-3);
            testCase.verifyEqual(m.theta1_ITAE, itae_exact, 'RelTol', 1e-3);
            testCase.verifyEqual(m.u_peak_V, 3.0, 'AbsTol', 1e-12);
            testCase.verifyEqual(m.u_rms_V,  3.0, 'AbsTol', 1e-12);
            testCase.verifyEqual(m.sat_pct,  0.0, 'AbsTol', 1e-12);
            testCase.verifyTrue(m.stabilized_theta1);
        end

        function test_master_live_scripts_run_cleanly(testCase)
            prevVis = get(0, 'DefaultFigureVisible');
            c1 = onCleanup(@() set(0, 'DefaultFigureVisible', prevVis)); %#ok<NASGU>
            set(0, 'DefaultFigureVisible', 'off');
            c2 = onCleanup(@() close('all', 'force')); %#ok<NASGU>

            stabPath = fullfile(testCase.RepoRoot, 'control', 'evaluation', 'stability', 'closed_loop_stability.mlx');
            perfPath = fullfile(testCase.RepoRoot, 'control', 'evaluation', 'performance', 'controller_benchmark.mlx');

            stabTbl = test_phase2_evaluation.runStabilityScript(stabPath);
            testCase.verifyClass(stabTbl, 'table');
            testCase.verifyEqual(height(stabTbl), 3);
            testCase.verifyTrue(all(stabTbl.Verdict == "STABLE"));

            [tbl10, tblImp] = test_phase2_evaluation.runBenchmarkScript(perfPath);
            testCase.verifyClass(tbl10,  'table');
            testCase.verifyClass(tblImp, 'table');
            testCase.verifyGreaterThanOrEqual(height(tbl10),  3);
            testCase.verifyGreaterThanOrEqual(height(tblImp), 3);
        end
    end

    methods (Static, Access = private)
        function summaryTable = runStabilityScript(scriptPath)
            run(scriptPath);
        end

        function [benchTable10, benchTableImp] = runBenchmarkScript(scriptPath)
            run(scriptPath);
        end
    end
end
