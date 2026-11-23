function params = load_parameters()
%LOAD_PARAMETERS Single Source of Truth (SSOT) for raw Furuta pendulum constants.
%   params = LOAD_PARAMETERS() returns raw mechanical, electrical, actuator,
%   sensor, and timing constants without any derived quantities (such as
%   I0_hat, J1_hat, or determinant D).
%
%   State ordering used by downstream plant builders:
%       x = [theta0; dtheta0; theta1; dtheta1],  u = Va,  y = [theta0; theta1]

    %% Environment
    params.g = 9.81;        % [m/s^2] Gravitational acceleration

    %% Mechanical Geometry & Inertias (CAD / Experimental SysID)
    params.I0 = 4.8053914e-05; % TODO [kg*m^2] Rotary arm + hub CAD inertia about vertical Z-axis (from robot.urdf)
    params.L0 = 0.0475;         % TODO [m]      Rotary arm pivot radius (temporary 4 mm offset in robot.urdf)
    params.m1 = 0.0458;        % TODO [kg]     Pendulum link mass (from robot.urdf)
    params.l1 = 0.057178115;   % TODO [m]      Pivot-to-pendulum center-of-mass distance (from robot.urdf)
    params.J1 = 6.9795432e-05; % TODO [kg*m^2] Pendulum inertia about its COM (from robot.urdf)
    params.Jm = 0.0;        % TODO [kg*m^2] Motor rotor inertia (motor-shaft referred)

    %% Joint Viscous Friction
    params.b0 = 1.0e-4;     % TODO [N*m*s/rad] Base arm pivot viscous damping
    params.b1 = 5.0e-5;     % TODO [N*m*s/rad] Pendulum pivot viscous damping

    %% Gearbox & DC Motor Electromechanical Constants
    params.N   = 19;        % TODO [-]         Gearbox reduction ratio (omega_m = N*dtheta0)
    params.R_m = 2.5;       % TODO [Ohm]       Armature terminal resistance
    params.k_t = 0.02;      % TODO [N*m/A]     Motor torque constant (motor shaft)
    params.k_b = 0.02;      % TODO [V*s/rad]   Motor back-EMF constant (motor shaft)

    %% Physical Limits, Encoders & Controller Sample Time
    params.V_max      = 12.0;   % TODO [V]   Symmetric actuator voltage limit (+/- V_max)
    params.theta0_ppr = 360;    % TODO [PPR] Rotary arm quadrature encoder resolution
    params.theta1_ppr = 600;    % TODO [PPR] Pendulum encoder resolution (>= 360 PPR required)
    params.Ts         = 0.002;  % TODO [s]   Controller sample time (500 Hz)

    %% Metadata (informational only)
    params.meta.source      = 'Baseline estimates (pending final URDF & hardware SysID)';
    params.meta.state_order = {'theta0', 'dtheta0', 'theta1', 'dtheta1'};
    params.meta.input_name  = 'Va';
    params.meta.output_names = {'theta0', 'theta1'};
end
