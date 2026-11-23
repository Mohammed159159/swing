function [x0, t_grid, u_info] = resolve_input_signal(x0, t_grid, u_spec, allowClosedLoop)
%RESOLVE_INPUT_SIGNAL Validate x0, t_grid, and normalize u_spec for Furuta runners.
%   [x0, t_grid, u_info] = RESOLVE_INPUT_SIGNAL(x0, t_grid, u_spec, allowClosedLoop)
%
%   Returns:
%     x0     - 4x1 finite real column vector [theta0; dtheta0; theta1; dtheta1]
%     t_grid - Nx1 strictly increasing finite real column vector (N >= 2)
%     u_info - struct with fields:
%                .mode   : 'zero' | 'constant' | 'vector' | 'open_loop_fcn' | 'closed_loop_fcn'
%                .u_vec  : Nx1 double (empty for 'closed_loop_fcn')
%                .u_fcn  : function handle @(t, x) -> scalar Va (raw, unsat)

    if nargin < 3
        u_spec = 0;
    end
    if nargin < 4
        allowClosedLoop = false;
    end

    % Validate initial state x0
    if ~isnumeric(x0) || ~isreal(x0) || numel(x0) ~= 4 || any(~isfinite(x0))
        error('Furuta:InvalidInitialState', ...
            'x0 must be a 4-element finite real vector [theta0; dtheta0; theta1; dtheta1].');
    end
    x0 = double(x0(:));

    % Validate time grid t_grid
    if ~isnumeric(t_grid) || ~isreal(t_grid) || ~isvector(t_grid) || numel(t_grid) < 2 ...
            || any(~isfinite(t_grid)) || any(diff(t_grid) <= 0)
        error('Furuta:InvalidTimeGrid', ...
            't_grid must be a strictly increasing finite real vector with at least 2 points.');
    end
    t_grid = double(t_grid(:));
    nSteps = numel(t_grid);

    % Resolve u_spec
    u_info = struct('mode', '', 'u_vec', [], 'u_fcn', []);

    if isempty(u_spec)
        u_info.mode  = 'zero';
        u_info.u_vec = zeros(nSteps, 1);
        u_info.u_fcn = @(~, ~) 0.0;
        return;
    end

    if isnumeric(u_spec)
        if ~isreal(u_spec) || any(~isfinite(u_spec(:)))
            error('Furuta:InvalidInputSignal', ...
                'Numeric u_spec must be real and finite.');
        end
        if isscalar(u_spec)
            u0 = double(u_spec);
            if u0 == 0
                u_info.mode = 'zero';
            else
                u_info.mode = 'constant';
            end
            u_info.u_vec = u0 * ones(nSteps, 1);
            u_info.u_fcn = @(~, ~) u0;
        elseif isvector(u_spec) && numel(u_spec) == nSteps
            u_vec = double(u_spec(:));
            u_info.mode  = 'vector';
            u_info.u_vec = u_vec;
            u_info.u_fcn = @(t, ~) interp1(t_grid, u_vec, t, 'previous', 'extrap');
        else
            error('Furuta:InvalidInputSignal', ...
                'Numeric u_spec must be empty, a scalar, or a vector matching numel(t_grid) (%d).', ...
                nSteps);
        end
        return;
    end

    if isa(u_spec, 'function_handle')
        nIn = nargin(u_spec);
        if nIn == 1
            u_vec = evalOpenLoopHandle(u_spec, t_grid);
            u_info.mode  = 'open_loop_fcn';
            u_info.u_vec = u_vec;
            u_info.u_fcn = @(t, ~) double(u_spec(t));
        elseif nIn == 2 || nIn < 0
            if ~allowClosedLoop
                error('Furuta:ClosedLoopNotSupported', ...
                    'Closed-loop handle @(t, x) is not supported by this open-loop runner.');
            end
            u_info.mode  = 'closed_loop_fcn';
            u_info.u_vec = [];
            u_info.u_fcn = @(t, x) double(u_spec(t, x));
        else
            error('Furuta:InvalidInputSignal', ...
                'u_spec function handle must accept @(t) or @(t, x); got %d inputs.', nIn);
        end
        return;
    end

    error('Furuta:InvalidInputSignal', ...
        'u_spec must be empty, scalar, length-N numeric vector, @(t), or @(t, x).');
end

function u_vec = evalOpenLoopHandle(u_fcn, t_grid)
    nSteps = numel(t_grid);
    try
        u_try = u_fcn(t_grid);
        if isnumeric(u_try) && isreal(u_try) && numel(u_try) == nSteps && all(isfinite(u_try(:)))
            u_vec = double(u_try(:));
            return;
        end
    catch
    end

    u_vec = zeros(nSteps, 1);
    for k = 1:nSteps
        val = u_fcn(t_grid(k));
        if ~isnumeric(val) || ~isscalar(val) || ~isreal(val) || ~isfinite(val)
            error('Furuta:InvalidInputSignal', ...
                'u_spec(t) must return a finite real scalar at t = %.6g.', t_grid(k));
        end
        u_vec(k) = double(val);
    end
end
