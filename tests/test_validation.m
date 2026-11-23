classdef test_validation < matlab.unittest.TestCase
%TEST_VALIDATION Unit tests for open_loop_scenarios, compute_validation_metrics,
%   and the Phase 1 open_loop_validation Live Script (downward + upright mirror).

    properties (Access = private)
        RepoRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.RepoRoot = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(testCase.RepoRoot, 'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(testCase.RepoRoot, 'sim', 'runners')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(testCase.RepoRoot, 'sim', 'scenarios')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(testCase.RepoRoot, 'digital-twin', 'validation')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(testCase.RepoRoot, 'digital-twin', 'models', 'simscape')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(testCase.RepoRoot, 'digital-twin', 'models', 'simscape', 'stl')));
        end
    end

    methods (Test)
        function test_open_loop_scenarios_contract(testCase)
            params = load_parameters();
            S      = open_loop_scenarios(params);

            expectedIds = { ...
                'decay_small', 'decay_large', 'step_small', 'step_1V', 'step_2V', ...
                'upright_fall_small', 'upright_fall_large', 'upright_step_small', 'upright_step_1V'};

            testCase.verifyEqual(numel(fieldnames(S)), numel(expectedIds));
            for k = 1:numel(expectedIds)
                id = expectedIds{k};
                testCase.verifyTrue(isfield(S, id), sprintf('Missing scenario: %s', id));
                sc = S.(id);
                testCase.verifyEqual(size(sc.x0), [4, 1]);
                testCase.verifyGreaterThan(numel(sc.t_grid), 10);
                testCase.verifyEqual(sc.t_grid(1), 0, 'AbsTol', 1e-15);
                testCase.verifyEqual(median(diff(sc.t_grid)), params.Ts, 'AbsTol', 1e-12);
                testCase.verifyTrue(ismember(sc.equilibrium, ["downward", "upright"]));
                testCase.verifyEqual(sc.simscape_gate_pct, 3.0);
            end

            % Verify downward vs. upright equilibrium initial angles
            testCase.verifyGreaterThan(S.decay_small.x0(3), pi);
            testCase.verifyEqual(S.upright_step_small.x0(3), 0, 'AbsTol', 1e-15);
        end

        function test_compute_validation_metrics_exact_and_known_error(testCase)
            t = linspace(0, 1, 101).';
            th0 = sin(2*pi*t);
            th1 = cos(2*pi*t);
            ref = struct('t', t, 'x', [th0, zeros(size(t)), th1, zeros(size(t))]);

            % Identical trajectories -> exact zero error
            m0 = compute_validation_metrics(ref, ref, NrmseGatePct=1.0);
            testCase.verifyEqual(m0.theta0_nrmse_pct, 0, 'AbsTol', 1e-15);
            testCase.verifyEqual(m0.theta1_nrmse_pct, 0, 'AbsTol', 1e-15);
            testCase.verifyTrue(m0.pass_gate);

            % Known constant offset delta = 0.04 on range 2.0 -> NRMSE = 2.0%
            cmp = ref;
            cmp.x(:, 3) = ref.x(:, 3) + 0.04;
            mPass = compute_validation_metrics(ref, cmp, NrmseGatePct=3.0);
            mFail = compute_validation_metrics(ref, cmp, NrmseGatePct=1.0);

            testCase.verifyEqual(mPass.channels.theta1.rmse, 0.04, 'AbsTol', 1e-12);
            testCase.verifyEqual(mPass.theta1_nrmse_pct, 2.0, 'AbsTol', 1e-10);
            testCase.verifyTrue(mPass.pass_gate);
            testCase.verifyFalse(mFail.pass_gate);
        end

        function test_log_decrement_matches_downward_eigenvalues(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            S      = open_loop_scenarios(params);

            r_lin = simulate_linear(plant, S.decay_small.x0, S.decay_small.t_grid, ...
                S.decay_small.u_spec, Equilibrium="downward");
            r_nl  = simulate_nonlinear(plant, params, S.decay_small.x0, S.decay_small.t_grid, ...
                S.decay_small.u_spec);

            m = compute_validation_metrics(r_nl, r_lin, ...
                NrmseGatePct=1.0, EstimateDamping=true, Params=params);

            oscPoles  = plant.eig_downward(abs(imag(plant.eig_downward)) > 1e-6);
            sigma_eig = -real(oscPoles(1));
            wd_eig    = abs(imag(oscPoles(1)));
            zeta_eig  = sigma_eig / abs(oscPoles(1));

            testCase.verifyEqual(m.damping.omega_d, wd_eig,    'RelTol', 0.02);
            testCase.verifyEqual(m.damping.sigma,   sigma_eig, 'RelTol', 0.02);
            testCase.verifyEqual(m.damping.zeta,    zeta_eig,  'RelTol', 0.02);
            testCase.verifyGreaterThan(m.damping.b1_uncoupled, 0);
        end

        function test_upright_topple_rate_and_input_sign_flip(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            S      = open_loop_scenarios(params);

            % Upright free topple growth rate tracks unstable RHP eigenvalue
            r_uf_lin = simulate_linear(plant, S.upright_fall_small.x0, S.upright_fall_small.t_grid, ...
                S.upright_fall_small.u_spec, Equilibrium="upright");
            r_uf_nl  = simulate_nonlinear(plant, params, S.upright_fall_small.x0, S.upright_fall_small.t_grid, ...
                S.upright_fall_small.u_spec);
            m_uf = compute_validation_metrics(r_uf_nl, r_uf_lin, ...
                NrmseGatePct=1.0, EstimateGrowthRate=true);

            lambda_up = max(real(plant.eig_upright));
            testCase.verifyGreaterThan(m_uf.growth.r_squared, 0.99);
            % On [0.125, 0.25] s, the decaying stable mode (-8.97 rad/s) still slightly
            % pulls slope below asymptotic +9.60 rad/s, so allow 12% relative band
            testCase.verifyEqual(m_uf.growth.lambda_hat, lambda_up, 'RelTol', 0.12);

            % Input-coupling sign flip: Va > 0 drives (theta1 - pi) > 0 at downward
            % equilibrium (B4_down > 0), and theta1 < 0 at upright equilibrium (B4_up < 0)
            r_dn = simulate_nonlinear(plant, params, S.step_small.x0, S.step_small.t_grid, S.step_small.u_spec);
            r_up = simulate_nonlinear(plant, params, S.upright_step_small.x0, S.upright_step_small.t_grid, S.upright_step_small.u_spec);

            testCase.verifyGreaterThan(r_dn.x(25, 3) - pi, 0, ...
                'Positive Va at downward equilibrium must kick theta1 - pi > 0 (B4_down > 0).');
            testCase.verifyLessThan(r_up.x(25, 3), 0, ...
                'Positive Va at upright equilibrium must kick theta1 < 0 (B4_up < 0).');
        end

        function test_small_angle_gates_pass_both_equilibria(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            S      = open_loop_scenarios(params);

            smallIds = {'decay_small', 'step_small', 'upright_fall_small', 'upright_step_small'};
            for k = 1:numel(smallIds)
                sc = S.(smallIds{k});
                r_lin = simulate_linear(plant, sc.x0, sc.t_grid, sc.u_spec, Equilibrium=sc.equilibrium);
                r_nl  = simulate_nonlinear(plant, params, sc.x0, sc.t_grid, sc.u_spec);
                m     = compute_validation_metrics(r_nl, r_lin, NrmseGatePct=sc.nrmse_gate_pct);
                testCase.verifyTrue(m.pass_gate, ...
                    sprintf('Small-angle scenario "%s" failed < 1%% gate (NRMSE = %.3f%%).', ...
                    sc.id, m.theta1_nrmse_pct));
            end

            largeIds = {'decay_large', 'step_2V', 'upright_fall_large'};
            for k = 1:numel(largeIds)
                sc = S.(largeIds{k});
                r_lin = simulate_linear(plant, sc.x0, sc.t_grid, sc.u_spec, Equilibrium=sc.equilibrium);
                r_nl  = simulate_nonlinear(plant, params, sc.x0, sc.t_grid, sc.u_spec);
                m     = compute_validation_metrics(r_nl, r_lin, NrmseGatePct=NaN);
                testCase.verifyGreaterThan(m.theta1_nrmse_pct, 3.0, ...
                    sprintf('Large-angle scenario "%s" should exhibit clear nonlinear divergence (> 3%%).', sc.id));
            end
        end

        function test_open_loop_validation_script_runs_cleanly(testCase)
            prevVis = get(0, 'DefaultFigureVisible');
            cleanupVis = onCleanup(@() set(0, 'DefaultFigureVisible', prevVis)); %#ok<NASGU>
            set(0, 'DefaultFigureVisible', 'off');
            cleanupFigs = onCleanup(@() close('all', 'force')); %#ok<NASGU>

            scriptPath = fullfile(testCase.RepoRoot, 'digital-twin', 'validation', 'open_loop_validation.mlx');
            valTable   = test_validation.runScriptIsolated(scriptPath);

            testCase.verifyClass(valTable, 'table');
            testCase.verifyEqual(height(valTable), 9);
            passRows = valTable(valTable.Lin_Status == "PASS", :);
            testCase.verifyEqual(height(passRows), 4, ...
                'All 4 small-angle scenarios (2 downward + 2 upright) must report Lin_Status PASS.');
            testCase.verifyEqual(sum(valTable.Lin_Status == "FAIL"), 0);
            if ismember('SS_Status', valTable.Properties.VariableNames)
                testCase.verifyTrue(all(ismember(valTable.SS_Status, ["PASS", "FAIL"])), ...
                    'SS_Status must strictly report PASS or FAIL on every scenario.');
            end
        end
    end

    methods (Static, Access = private)
        function valTable = runScriptIsolated(scriptPath)
            run(scriptPath);
        end
    end
end
