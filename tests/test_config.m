classdef test_config < matlab.unittest.TestCase
%TEST_CONFIG Unit tests for config/ SSOT parameters, URDF extraction, and plant builder.

    properties (Access = private)
        RepoRoot
    end

    methods (TestClassSetup)
        function addConfigToPath(testCase)
            testCase.RepoRoot = fileparts(fileparts(mfilename('fullpath')));
            cfgDir = fullfile(testCase.RepoRoot, 'config');
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(cfgDir));
        end
    end

    methods (Test)
        function test_load_parameters_contract(testCase)
            params = load_parameters();

            requiredFields = {'g', 'I0', 'L0', 'm1', 'l1', 'J1', 'Jm', ...
                'b0', 'b1', 'N', 'R_m', 'k_t', 'k_b', ...
                'V_max', 'theta0_ppr', 'theta1_ppr', 'Ts'};
            for k = 1:numel(requiredFields)
                testCase.verifyTrue(isfield(params, requiredFields{k}), ...
                    sprintf('Missing required field: %s', requiredFields{k}));
            end

            strictPos = {'g', 'I0', 'L0', 'm1', 'l1', 'J1', 'N', 'R_m', 'k_t', 'V_max', 'Ts'};
            for k = 1:numel(strictPos)
                testCase.verifyGreaterThan(params.(strictPos{k}), 0);
            end

            nonNeg = {'Jm', 'b0', 'b1', 'k_b'};
            for k = 1:numel(nonNeg)
                testCase.verifyGreaterThanOrEqual(params.(nonNeg{k}), 0);
            end

            testCase.verifyGreaterThanOrEqual(params.theta0_ppr, 360);
            testCase.verifyGreaterThanOrEqual(params.theta1_ppr, 360);

            derivedForbidden = {'I0_hat', 'J1_hat', 'D', 'b_eff'};
            for k = 1:numel(derivedForbidden)
                testCase.verifyFalse(isfield(params, derivedForbidden{k}), ...
                    sprintf('Derived field "%s" must not live in load_parameters.', derivedForbidden{k}));
            end
        end

        function test_generated_matches_symbolic_both_equilibria(testCase)
            [A_sym_fn, B_sym_fn] = test_config.buildIndependentSymbolicCheck();

            baseParams = load_parameters();
            cases = {baseParams, setfield(baseParams, 'Jm', 5e-7)}; %#ok<SFLD>

            for idx = 1:numel(cases)
                p = cases{idx};
                plant = build_plant(p);

                for c_eq = [1, -1]
                    A_sym = A_sym_fn(p.I0, p.L0, p.m1, p.l1, p.J1, p.Jm, p.g, ...
                        p.N, p.R_m, p.k_t, p.k_b, p.b0, p.b1, c_eq);
                    B_sym = B_sym_fn(p.I0, p.L0, p.m1, p.l1, p.J1, p.Jm, p.g, ...
                        p.N, p.R_m, p.k_t, p.k_b, p.b0, p.b1, c_eq);

                    if c_eq == 1
                        eq = plant.upright;
                    else
                        eq = plant.downward;
                    end

                    testCase.verifyEqual(eq.A, A_sym, 'AbsTol', 1e-12);
                    testCase.verifyEqual(eq.B, B_sym, 'AbsTol', 1e-12);
                    testCase.verifyEqual(eq.C, [1 0 0 0; 0 0 1 0], 'AbsTol', 1e-15);
                    testCase.verifyEqual(eq.D, [0; 0], 'AbsTol', 1e-15);
                end
            end
        end

        function test_rotor_inertia_n2_scaling(testCase)
            p_zero = load_parameters();
            p_zero.Jm = 0.0;
            plant_zero = build_plant(p_zero);

            Jm_test = 6.5e-7;
            p_rotor = p_zero;
            p_rotor.Jm = Jm_test;
            plant_rotor = build_plant(p_rotor);

            p_lumped = p_zero;
            p_lumped.I0 = p_zero.I0 + p_zero.N^2 * Jm_test;
            plant_lumped = build_plant(p_lumped);

            testCase.verifyEqual(plant_rotor.derived.I0_hat - plant_zero.derived.I0_hat, ...
                p_zero.N^2 * Jm_test, 'AbsTol', 1e-15);
            testCase.verifyEqual(plant_rotor.A_full, plant_lumped.A_full, 'AbsTol', 1e-12);
            testCase.verifyEqual(plant_rotor.B_full, plant_lumped.B_full, 'AbsTol', 1e-12);
            testCase.verifyEqual(plant_rotor.A_down, plant_lumped.A_down, 'AbsTol', 1e-12);
            testCase.verifyEqual(plant_rotor.B_down, plant_lumped.B_down, 'AbsTol', 1e-12);
        end

        function test_eigenvalue_stability_signatures(testCase)
            params = load_parameters();
            plant  = build_plant(params);

            poles_up = plant.eig_upright;
            zeroUp   = poles_up(abs(poles_up) < 1e-12);
            nonZeroUp = poles_up(abs(poles_up) >= 1e-12);
            testCase.verifyEqual(numel(zeroUp), 1, 'Expected 1 cyclic theta0 zero pole (upright).');
            testCase.verifyEqual(sum(real(nonZeroUp) > 0), 1, 'Expected 1 unstable RHP pole (upright).');
            testCase.verifyEqual(sum(real(nonZeroUp) < 0), 2, 'Expected 2 stable LHP poles (upright).');
            testCase.verifyEqual(max(abs(imag(poles_up))), 0, 'AbsTol', 1e-12);

            poles_dn = plant.eig_downward;
            zeroDn    = poles_dn(abs(poles_dn) < 1e-12);
            nonZeroDn = poles_dn(abs(poles_dn) >= 1e-12);
            testCase.verifyEqual(numel(zeroDn), 1, 'Expected 1 cyclic theta0 zero pole (downward).');
            testCase.verifyTrue(all(real(nonZeroDn) < 0), 'All 3 non-cyclic downward poles must lie in open LHP.');
            testCase.verifyEqual(sum(abs(imag(nonZeroDn)) > 1e-6), 2, ...
                'Expected a complex-conjugate oscillatory pair at downward equilibrium.');
        end

        function test_parameter_sweep_isolation(testCase)
            params      = load_parameters();
            paramsOrig  = params;
            plant1      = build_plant(params);

            p2    = params;
            p2.b1 = params.b1 * 10;
            p2.m1 = params.m1 * 1.5;
            plant2 = build_plant(p2);

            testCase.verifyEqual(params, paramsOrig);
            testCase.verifyNotEqual(plant1.A_full, plant2.A_full);
            testCase.verifyNotEqual(plant1.B_down, plant2.B_down);
        end

        function test_invalid_params_rejected(testCase)
            params = load_parameters();

            badMissing = rmfield(params, 'N');
            testCase.verifyError(@() build_plant(badMissing), 'Furuta:InvalidParameters');

            badJm = params; badJm.Jm = -1e-6;
            testCase.verifyError(@() build_plant(badJm), 'Furuta:InvalidParameters');

            badMass = params; badMass.m1 = 0;
            testCase.verifyError(@() build_plant(badMass), 'Furuta:InvalidParameters');

            badRes = params; badRes.R_m = -2.0;
            testCase.verifyError(@() build_plant(badRes), 'Furuta:InvalidParameters');

            badSing = params;
            badSing.I0 = 1e-22;
            badSing.J1 = 1e-22;
            badSing.Jm = 0;
            testCase.verifyError(@() build_plant(badSing), 'Furuta:SingularInertiaMatrix');
        end

        function test_extract_urdf_params(testCase)
            urdfPath = fullfile(testCase.RepoRoot, 'digital-twin', 'simscape', 'robot.urdf');

            cad = testCase.verifyWarning( ...
                @() extract_urdf_params(urdfPath), 'Furuta:SmallArmLength');

            % Hand-computed check against digital-twin/simscape/robot.urdf
            m_arm = 0.053299999999999979339;
            com_arm_xy2 = (-0.012730712527184277066)^2 + (8.0648459397815786313e-05)^2;
            I0_expected = 3.9415181227059578367e-05 + m_arm * com_arm_xy2;
            L0_expected = hypot(0.0039999999999999723277, 0);
            m1_expected = 0.045800000000000000433;
            l1_expected = hypot(-3.9121175585088163197e-05, -0.057178101805359336784);
            J1_expected = 6.9795432247420085854e-05;

            testCase.verifyEqual(cad.I0, I0_expected, 'AbsTol', 1e-12);
            testCase.verifyEqual(cad.L0, L0_expected, 'AbsTol', 1e-12);
            testCase.verifyEqual(cad.m1, m1_expected, 'AbsTol', 1e-12);
            testCase.verifyEqual(cad.l1, l1_expected, 'AbsTol', 1e-12);
            testCase.verifyEqual(cad.J1, J1_expected, 'AbsTol', 1e-12);
        end

        function test_nonlinear_jacobian_matches_linear_equilibria(testCase)
            params = load_parameters();
            params.Jm = 5e-7;
            plant  = build_plant(params);

            h = 1e-6;
            eqStates = {[0; 0; 0; 0], [0; 0; pi; 0]};
            eqModels = {plant.upright, plant.downward};

            for idx = 1:2
                x_eq = eqStates{idx};
                eq   = eqModels{idx};

                testCase.verifyEqual(plant.nonlinear.rhs(x_eq, 0), zeros(4, 1), 'AbsTol', 1e-12);

                A_fd = zeros(4, 4);
                for k = 1:4
                    dx = zeros(4, 1); dx(k) = h;
                    A_fd(:, k) = (plant.nonlinear.rhs(x_eq + dx, 0) ...
                                - plant.nonlinear.rhs(x_eq - dx, 0)) / (2 * h);
                end
                B_fd = (plant.nonlinear.rhs(x_eq,  h) ...
                      - plant.nonlinear.rhs(x_eq, -h)) / (2 * h);

                testCase.verifyEqual(A_fd, eq.A, 'AbsTol', 1e-6);
                testCase.verifyEqual(B_fd, eq.B, 'AbsTol', 1e-6);
            end
        end

        function test_nonlinear_small_angle_matches_linear_lsim(testCase)
            params = load_parameters();
            plant  = build_plant(params);

            tGrid = linspace(0, 1.5, 301).';
            VaVal = 0.05;
            uVec  = VaVal * ones(size(tGrid));
            dx0   = [0; 0; 0.02; 0];
            x0_nl = [0; 0; pi; 0] + dx0;

            [~, X_nl] = ode45(@(t, x) plant.nonlinear.rhs(x, VaVal), tGrid, x0_nl, ...
                odeset('RelTol', 1e-8, 'AbsTol', 1e-10));
            y_lin = lsim(plant.downward.sys, uVec, tGrid, dx0);

            testCase.verifyEqual(X_nl(:, 1),      y_lin(:, 1), 'AbsTol', 1e-3);
            testCase.verifyEqual(X_nl(:, 3) - pi, y_lin(:, 2), 'AbsTol', 1e-3);
        end

        function test_nonlinear_3d_centrifugal_flinging(testCase)
            params    = load_parameters();
            params.b0 = 0;
            params.b1 = 0;
            params.k_b = 0;
            plant     = build_plant(params);

            th1 = pi / 4;
            dth0 = 10.0;
            xdot = plant.nonlinear.rhs([0; dth0; th1; 0], 0);

            % At dth1=0, Va=0, b0=b1=k_b=0:
            %   M11(th1)*ddth0 + M12(th1)*ddth1 = 0
            %   M21(th1)*ddth0 + J1_hat*ddth1   = (1/2)*J1_hat*sin(2*th1)*dth0^2 + m1*g*l1*sin(th1)
            I0_hat = plant.derived.I0_hat;
            J1_hat = plant.derived.J1_hat;
            M11 = I0_hat + J1_hat * sin(th1)^2;
            M12 = params.m1 * params.L0 * params.l1 * cos(th1);
            rhs2 = 0.5 * J1_hat * sin(2 * th1) * dth0^2 + params.m1 * params.g * params.l1 * sin(th1);
            ddth1_expected = (M11 * rhs2) / (M11 * J1_hat - M12^2);

            testCase.verifyEqual(xdot(4), ddth1_expected, 'AbsTol', 1e-10);
            testCase.verifyGreaterThan(xdot(4), 0);
        end

        function test_nonlinear_output_matches_linear_cd(testCase)
            params = load_parameters();
            plant  = build_plant(params);

            x_test  = [0.4; -1.2; 0.85; 2.1];
            Va_test = 3.5;
            y_nl    = plant.nonlinear.output(x_test, Va_test);
            y_exp   = plant.C_full * x_test + plant.D_full * Va_test;

            testCase.verifyEqual(y_nl, y_exp, 'AbsTol', 1e-15);
        end
    end

    methods (Static, Access = private)
        function [A_fn, B_fn] = buildIndependentSymbolicCheck()
            syms I0 L0 m1 l1 J1 Jm g N R_m k_t k_b b0 b1 V_a c_eq real
            syms th0 dth0 ddth0 th1 dth1 ddth1 real

            I0_tot = I0 + N^2 * Jm + m1 * L0^2;
            J1_tot = J1 + m1 * l1^2;

            tau = (N * k_t / R_m) * V_a - (N^2 * k_t * k_b / R_m) * dth0;
            eq0 = I0_tot * ddth0 + c_eq * m1 * L0 * l1 * ddth1 == tau - b0 * dth0;
            eq1 = c_eq * m1 * L0 * l1 * ddth0 + J1_tot * ddth1 - c_eq * m1 * g * l1 * th1 == -b1 * dth1;

            [M_sym, f_sym] = equationsToMatrix([eq0, eq1], [ddth0, ddth1]);
            ddq = simplify(M_sym \ f_sym);
            xdot = [dth0; ddq(1); dth1; ddq(2)];
            x = [th0; dth0; th1; dth1];

            A_sym = simplify(subs(jacobian(xdot, x),   c_eq^2, 1));
            B_sym = simplify(subs(jacobian(xdot, V_a), c_eq^2, 1));

            vars = {I0, L0, m1, l1, J1, Jm, g, N, R_m, k_t, k_b, b0, b1, c_eq};
            A_fn = matlabFunction(A_sym, 'Vars', vars);
            B_fn = matlabFunction(B_sym, 'Vars', vars);
        end
    end
end
