classdef test_sim_runners < matlab.unittest.TestCase
%TEST_SIM_RUNNERS Unit tests for startup.m and sim/runners/ simulation engines.

    properties (Access = private)
        RepoRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.RepoRoot = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(testCase.RepoRoot));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(testCase.RepoRoot, 'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(testCase.RepoRoot, 'sim', 'runners')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(testCase.RepoRoot, 'digital-twin', 'models', 'simscape')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(testCase.RepoRoot, 'digital-twin', 'models', 'simscape', 'stl')));
        end
    end

    methods (Test)
        function test_startup_paths_and_workspace(testCase)
            % Run startup inside an isolated helper workspace
            [wsVars, paramsOut, plantOut] = test_sim_runners.runStartupIsolated();
            testCase.verifyTrue(ismember('params', wsVars));
            testCase.verifyTrue(ismember('plant',  wsVars));
            testCase.verifyEqual(numel(wsVars), 2, ...
                'startup.m should not leave temporary loop variables in the workspace.');
            testCase.verifyEqual(paramsOut, load_parameters());
            testCase.verifyEqual(plantOut.A_full, build_plant(paramsOut).A_full, 'AbsTol', 1e-15);

            % Verify pre-existing params override is preserved (not overwritten)
            customParams = load_parameters();
            customParams.V_max = 24.0;
            preservedParams = test_sim_runners.runStartupWithExistingParams(customParams);
            testCase.verifyEqual(preservedParams.V_max, 24.0);
        end

        function test_simulate_linear_contract_and_equilibria(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            t_grid = linspace(0, 1.5, 301).';

            % Downward equilibrium: small perturbation stays bounded and returns global coords
            x0_dn  = [0.1; 0; pi + deg2rad(5); 0];
            res_dn = simulate_linear(plant, x0_dn, t_grid, 0, Equilibrium="downward");

            testCase.verifyEqual(size(res_dn.t), [301, 1]);
            testCase.verifyEqual(size(res_dn.x), [301, 4]);
            testCase.verifyEqual(size(res_dn.y), [301, 2]);
            testCase.verifyEqual(size(res_dn.u), [301, 1]);
            testCase.verifyEqual(res_dn.x(1, :).', x0_dn, 'AbsTol', 1e-12);
            testCase.verifyEqual(res_dn.y, res_dn.x(:, [1, 3]), 'AbsTol', 1e-15);
            testCase.verifyEqual(res_dn.meta.runner, 'simulate_linear');
            testCase.verifyEqual(res_dn.meta.equilibrium, 'downward');
            testCase.verifyLessThan(max(abs(res_dn.x(:, 3) - pi)), deg2rad(6));

            % Upright equilibrium: small perturbation diverges exponentially
            x0_up  = [0; 0; deg2rad(1); 0];
            res_up = simulate_linear(plant, x0_up, linspace(0, 0.6, 121).', 0, Equilibrium="upright");
            testCase.verifyEqual(res_up.x(1, :).', x0_up, 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(abs(res_up.x(end, 3)), 5 * abs(x0_up(3)));
        end

        function test_simulate_nonlinear_open_loop_matches_linear_small_angle(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            t_grid = linspace(0, 1.5, 301).';

            % Case 1: Free hanging decay (theta1(0) = pi + 5 deg, Va = 0)
            x0_decay = [0; 0; pi + deg2rad(5); 0];
            res_lin  = simulate_linear(plant, x0_decay, t_grid, 0, Equilibrium="downward");
            res_nl   = simulate_nonlinear(plant, params, x0_decay, t_grid, 0);

            testCase.verifyEqual(res_nl.x(:, 1), res_lin.x(:, 1), 'AbsTol', 1e-3);
            testCase.verifyEqual(res_nl.x(:, 3), res_lin.x(:, 3), 'AbsTol', 2e-3);
            testCase.verifyEqual(res_nl.y, res_nl.x(:, [1, 3]), 'AbsTol', 1e-15);

            % Case 2: Small open-loop voltage step (Va = 0.05 V)
            x0_step  = [0; 0; pi + 0.02; 0];
            res_lin2 = simulate_linear(plant, x0_step, t_grid, 0.05, Equilibrium="downward");
            res_nl2  = simulate_nonlinear(plant, params, x0_step, t_grid, 0.05);

            testCase.verifyEqual(res_nl2.x(:, 1), res_lin2.x(:, 1), 'AbsTol', 1e-3);
            testCase.verifyEqual(res_nl2.x(:, 3), res_lin2.x(:, 3), 'AbsTol', 1e-3);
            testCase.verifyEqual(res_nl2.u, 0.05 * ones(size(t_grid)), 'AbsTol', 1e-15);
        end

        function test_simulate_nonlinear_energy_conservation_undamped(testCase)
            params     = load_parameters();
            params.b0  = 0;
            params.b1  = 0;
            params.k_b = 0;
            plant      = build_plant(params);

            t_grid = linspace(0, 3.0, 301).';
            x0     = [0; 1.5; pi + deg2rad(60); 0.5];
            res    = simulate_nonlinear(plant, params, x0, t_grid, 0, RelTol=1e-10, AbsTol=1e-12);

            I0_hat = plant.derived.I0_hat;
            J1_hat = plant.derived.J1_hat;
            c_coup = params.m1 * params.L0 * params.l1;

            dth0 = res.x(:, 2);
            th1  = res.x(:, 3);
            dth1 = res.x(:, 4);

            T = 0.5 * (I0_hat + J1_hat .* sin(th1).^2) .* dth0.^2 ...
              + 0.5 * J1_hat .* dth1.^2 ...
              + c_coup .* cos(th1) .* dth0 .* dth1;
            V = params.m1 * params.g * params.l1 .* cos(th1);
            E = T + V;

            relDrift = max(abs(E - E(1))) / max(abs(E(1)), 1e-6);
            testCase.verifyLessThan(relDrift, 1e-5, ...
                sprintf('Undamped 3D mechanical energy drift %.3e exceeded tolerance.', relDrift));
        end

        function test_simulate_nonlinear_closed_loop_and_saturation(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            t_grid = linspace(0, 0.5, 101).';
            x0     = [0; 0; pi + 0.1; 0];

            % Closed-loop state feedback saturating above V_max
            K_big  = [0, 0, 500, 50];
            u_cl   = @(t, x) -K_big * (x - [0; 0; pi; 0]);
            res_sat = simulate_nonlinear(plant, params, x0, t_grid, u_cl, Saturate=true);
            testCase.verifyLessThanOrEqual(max(abs(res_sat.u)), params.V_max + 1e-12);
            testCase.verifyGreaterThan(res_sat.meta.sat_fraction, 0);
            testCase.verifyEqual(res_sat.meta.input_mode, 'closed_loop_fcn');

            % Explicit constant command above V_max: saturated vs unsaturated
            u_hi    = 50.0;
            res_c1  = simulate_nonlinear(plant, params, x0, t_grid, u_hi, Saturate=true);
            res_c2  = simulate_nonlinear(plant, params, x0, t_grid, u_hi, Saturate=false);
            testCase.verifyEqual(res_c1.u, params.V_max * ones(size(t_grid)), 'AbsTol', 1e-12);
            testCase.verifyEqual(res_c1.meta.sat_fraction, 1.0, 'AbsTol', 1e-12);
            testCase.verifyEqual(res_c2.u, u_hi * ones(size(t_grid)), 'AbsTol', 1e-12);
            testCase.verifyEqual(res_c2.meta.sat_fraction, 0.0, 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(abs(res_c2.x(end, 1)), abs(res_c1.x(end, 1)));
        end

        function test_simulate_nonlinear_input_types_and_guards(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            t_grid = linspace(0, 0.4, 81).';
            x0     = [0; 0; pi; 0];

            r_scalar = simulate_nonlinear(plant, params, x0, t_grid, 1.0);
            r_vec    = simulate_nonlinear(plant, params, x0, t_grid, ones(size(t_grid)));
            r_fcn    = simulate_nonlinear(plant, params, x0, t_grid, @(t) 1.0 + 0*t);
            r_zero1  = simulate_nonlinear(plant, params, x0, t_grid, []);
            r_zero2  = simulate_nonlinear(plant, params, x0, t_grid, 0);

            testCase.verifyEqual(r_vec.x,   r_scalar.x, 'AbsTol', 1e-9);
            testCase.verifyEqual(r_fcn.x,   r_scalar.x, 'AbsTol', 1e-9);
            testCase.verifyEqual(r_zero1.x, r_zero2.x,  'AbsTol', 1e-15);

            % Open-loop linear runner rejects closed-loop @(t, x)
            testCase.verifyError( ...
                @() simulate_linear(plant, x0, t_grid, @(t, x) -x(1)), ...
                'Furuta:ClosedLoopNotSupported');
            % Invalid x0 length rejected
            testCase.verifyError( ...
                @() simulate_nonlinear(plant, params, [0; 0], t_grid, 0), ...
                'Furuta:InvalidInitialState');
        end

        function test_simulate_simscape_smoke_and_contract(testCase)
            params = load_parameters();
            t_grid = linspace(0, 0.2, 21).';
            x0     = [0; 0; 0; 0];

            res = simulate_simscape(params, x0, t_grid, 0, UseExternalInput=false);

            testCase.verifyEqual(size(res.t), [21, 1]);
            testCase.verifyEqual(size(res.x), [21, 4]);
            testCase.verifyEqual(size(res.y), [21, 2]);
            testCase.verifyEqual(size(res.u), [21, 1]);
            testCase.verifyTrue(all(isfinite(res.x(:))));
            testCase.verifyEqual(res.y, res.x(:, [1, 3]), 'AbsTol', 1e-15);
            testCase.verifyEqual(res.meta.runner, 'simulate_simscape');
            testCase.verifyEqual(res.meta.model, 'robot_compatible');
        end
    end

    methods (Static, Access = private)
        function [wsVars, paramsOut, plantOut] = runStartupIsolated()
            startup;
            wsVars    = who();
            paramsOut = params;
            plantOut  = plant;
        end

        function preservedParams = runStartupWithExistingParams(params)
            startup;
            preservedParams = params;
        end
    end
end
