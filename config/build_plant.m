function plant = build_plant(params)
%BUILD_PLANT Analytical linear and 3D nonlinear plant builder for Furuta pendulum.
%   plant = BUILD_PLANT(params) takes the raw hardware constants struct from
%   load_parameters(), computes invariant derived inertia/damping quantities, and
%   builds:
%       - plant.upright   (theta1 = 0,  c_eq = +1)
%       - plant.downward  (theta1 = pi, c_eq = -1)
%       - plant.nonlinear (.rhs(x, Va) for ode45, .output(x, Va) for sensors)
%
%   All linear matrices (A, B, C, D) and nonlinear dynamics (xdot, y) originate
%   from the symbolic-exported generators furuta_linear_generated and
%   furuta_nonlinear_generated.
%
%   Locked State, Input, and Output Convention:
%       x = [theta0; dtheta0; theta1; dtheta1]  (4x1 column vector)
%       u = Va
%       y = [theta0; theta1]                    (2x1 column vector)

    validateParams(params);

    derived.I0_hat = params.I0 + params.N^2 * params.Jm + params.m1 * params.L0^2;
    derived.J1_hat = params.J1 + params.m1 * params.l1^2;
    derived.D      = derived.I0_hat * derived.J1_hat - (params.m1 * params.L0 * params.l1)^2;
    derived.b_eff  = params.b0 + (params.N^2 * params.k_t * params.k_b) / params.R_m;

    if ~(derived.D > 0)
        error('Furuta:SingularInertiaMatrix', ...
            'Determinant D = %.4e must be strictly positive (I0_hat*J1_hat > (m1*L0*l1)^2).', ...
            derived.D);
    end

    upright  = buildEquilibrium(params, derived, +1);
    downward = buildEquilibrium(params, derived, -1);

    nonlinear = struct();
    nonlinear.rhs    = @(x, Va) evalNonlinearRhs(x, Va, params, derived);
    nonlinear.output = @(x, Va) evalNonlinearOutput(x, Va, params, derived);

    plant = struct();
    plant.derived      = derived;
    plant.upright      = upright;
    plant.downward     = downward;
    plant.nonlinear    = nonlinear;

    % Convenience aliases matching Section 5 & Phase 1 validation naming
    plant.A_full       = upright.A;
    plant.B_full       = upright.B;
    plant.C_full       = upright.C;
    plant.D_full       = upright.D;
    plant.A_down       = downward.A;
    plant.B_down       = downward.B;
    plant.sys_upright  = upright.sys;
    plant.sys_downward = downward.sys;
    plant.eig_upright  = upright.eig;
    plant.eig_downward = downward.eig;
end

function eq = buildEquilibrium(params, derived, c_eq)
    [A, B, C, D_ss] = furuta_linear_generated( ...
        derived.I0_hat, derived.J1_hat, params.m1, params.L0, params.l1, ...
        params.g, params.N, params.R_m, params.k_t, params.k_b, ...
        params.b0, params.b1, c_eq);

    sys = ss(A, B, C, D_ss, ...
        'StateName',  {'theta0', 'dtheta0', 'theta1', 'dtheta1'}, ...
        'InputName',  {'Va'}, ...
        'OutputName', {'theta0', 'theta1'});

    eq = struct( ...
        'c_eq', c_eq, ...
        'A',    A, ...
        'B',    B, ...
        'C',    C, ...
        'D',    D_ss, ...
        'sys',  sys, ...
        'eig',  eig(A));
end

function xdot = evalNonlinearRhs(x, Va, params, derived)
    xdot = furuta_nonlinear_generated(x(:), Va, ...
        derived.I0_hat, derived.J1_hat, params.m1, params.L0, params.l1, ...
        params.g, params.N, params.R_m, params.k_t, params.k_b, ...
        params.b0, params.b1);
end

function y = evalNonlinearOutput(x, Va, params, derived)
    [~, y] = furuta_nonlinear_generated(x(:), Va, ...
        derived.I0_hat, derived.J1_hat, params.m1, params.L0, params.l1, ...
        params.g, params.N, params.R_m, params.k_t, params.k_b, ...
        params.b0, params.b1);
end

function validateParams(params)
    if ~isstruct(params) || ~isscalar(params)
        error('Furuta:InvalidParameters', 'params must be a scalar struct.');
    end

    strictlyPositive = {'I0', 'L0', 'm1', 'l1', 'J1', 'g', 'N', 'R_m', 'k_t'};
    nonNegative      = {'Jm', 'k_b', 'b0', 'b1'};

    checkFields(params, strictlyPositive, @(v) v > 0,  '> 0');
    checkFields(params, nonNegative,      @(v) v >= 0, '>= 0');
end

function checkFields(params, fieldList, pred, desc)
    for k = 1:numel(fieldList)
        fname = fieldList{k};
        if ~isfield(params, fname)
            error('Furuta:InvalidParameters', 'Missing required field params.%s.', fname);
        end
        val = params.(fname);
        if ~isnumeric(val) || ~isscalar(val) || ~isreal(val) || ~isfinite(val) || ~pred(val)
            error('Furuta:InvalidParameters', ...
                'params.%s must be a finite real scalar satisfying %s.', fname, desc);
        end
    end
end
