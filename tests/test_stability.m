classdef test_stability < matlab.unittest.TestCase
%TEST_STABILITY Unit tests for Step 2 open-loop stability, non-minimum phase
%   zero, controllability, and observability analysis (upright & downward).

    properties (Access = private)
        RepoRoot
    end

    methods (TestClassSetup)
        function addProjectPaths(testCase)
            testCase.RepoRoot = fileparts(fileparts(mfilename('fullpath')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(testCase.RepoRoot, 'config')));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(testCase.RepoRoot, 'control', 'evaluation', 'stability')));
        end
    end

    methods (Test)
        function test_eigenvalue_classification_both_equilibria(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            rep    = analyze_open_loop_stability(plant, params);

            % Upright: 1 cyclic zero pole, 1 RHP pole, 2 LHP poles
            pUp = rep.upright.poles;
            testCase.verifyEqual(abs(pUp(1)), 0, 'AbsTol', 1e-10);
            testCase.verifyLessThan(real(pUp(2)), -10);
            testCase.verifyLessThan(real(pUp(3)), 0);
            testCase.verifyGreaterThan(real(pUp(4)), 0);
            testCase.verifyEqual(rep.upright.rhp_pole, real(pUp(4)), 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(rep.upright.sample_ratio_tau_to_Ts, 10);

            % Downward: 1 cyclic zero pole, 3 open-LHP poles (including complex pair)
            pDn = rep.downward.poles;
            testCase.verifyEqual(abs(pDn(1)), 0, 'AbsTol', 1e-10);
            testCase.verifyTrue(all(real(pDn(2:4)) < 0));
            testCase.verifyEqual(sum(abs(imag(pDn)) > 1e-3), 2);
            testCase.verifyTrue(isnan(rep.downward.rhp_pole));
        end

        function test_non_minimum_phase_zeros_upright_vs_downward(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            rep    = analyze_open_loop_stability(plant, params);

            % MIMO tzero is empty at both equilibria
            testCase.verifyEmpty(rep.upright.zeros_mimo);
            testCase.verifyEmpty(rep.downward.zeros_mimo);

            % SISO Va -> theta0: RHP zero at upright, minimum-phase LHP zeros at downward
            testCase.verifyTrue(rep.upright.is_nmp_theta0);
            testCase.verifyGreaterThan(rep.upright.rhp_zero_theta0, 0);
            testCase.verifyEqual(sum(real(rep.upright.zeros_theta0) > 1e-6), 1);

            testCase.verifyFalse(rep.downward.is_nmp_theta0);
            testCase.verifyTrue(all(real(rep.downward.zeros_theta0) < 0));

            % SISO Va -> theta1 minimal realization: single blocking zero at s = 0
            testCase.verifyEqual(numel(rep.upright.zeros_theta1), 1);
            testCase.verifyEqual(abs(rep.upright.zeros_theta1), 0, 'AbsTol', 1e-9);
            testCase.verifyEqual(numel(rep.downward.zeros_theta1), 1);
            testCase.verifyEqual(abs(rep.downward.zeros_theta1), 0, 'AbsTol', 1e-9);

            % Polarity flip in B4 and bandwidth floor > ceiling at upright
            testCase.verifyLessThan(rep.upright.b4_gain, 0);
            testCase.verifyGreaterThan(rep.downward.b4_gain, 0);
            testCase.verifyGreaterThan(rep.upright.bw_floor_rad_s, rep.upright.bw_ceiling_theta0_rad_s);
        end

        function test_controllability_rank_both_equilibria(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            rep    = analyze_open_loop_stability(plant, params);

            for eqName = {'upright', 'downward'}
                r = rep.(eqName{1});
                testCase.verifyTrue(r.ctrb.is_controllable);
                testCase.verifyEqual(r.ctrb.rank, 4);
            end
        end

        function test_observability_sensor_subsets(testCase)
            params = load_parameters();
            plant  = build_plant(params);
            rep    = analyze_open_loop_stability(plant, params);

            for eqName = {'upright', 'downward'}
                r = rep.(eqName{1});
                testCase.verifyTrue(r.obsv.is_observable);
                testCase.verifyEqual(r.obsv.rank, 4);
                testCase.verifyEqual(r.obsv.rank_theta0_only, 4);
                testCase.verifyEqual(r.obsv.rank_theta1_only, 3, ...
                    'Pendulum encoder alone cannot observe cyclic theta0 (rank must be 3).');
            end
        end

        function test_open_loop_stability_mlx_runs_cleanly(testCase)
            prevVis = get(0, 'DefaultFigureVisible');
            cleanupVis = onCleanup(@() set(0, 'DefaultFigureVisible', prevVis)); %#ok<NASGU>
            set(0, 'DefaultFigureVisible', 'off');
            cleanupFigs = onCleanup(@() close('all', 'force')); %#ok<NASGU>

            mlxPath = fullfile(testCase.RepoRoot, 'control', 'evaluation', 'stability', 'open_loop_stability.mlx');
            [modalTable, zeroTable, ctrlObsvTable] = test_stability.runScriptIsolated(mlxPath);

            testCase.verifyClass(modalTable, 'table');
            testCase.verifyEqual(height(modalTable), 8);
            testCase.verifyClass(zeroTable, 'table');
            testCase.verifyEqual(height(zeroTable), 2);
            testCase.verifyClass(ctrlObsvTable, 'table');
            testCase.verifyEqual(height(ctrlObsvTable), 2);
        end
    end

    methods (Static, Access = private)
        function [modalTable, zeroTable, ctrlObsvTable] = runScriptIsolated(scriptPath)
            run(scriptPath);
        end
    end
end
