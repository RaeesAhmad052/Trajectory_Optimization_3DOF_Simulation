function p = getDefaultParameters()
% GETDEFAULTPARAMETERS
% Returns a consistent fixed-parameter structure for simulation and optimization.

    %% Environment / vehicle
    p.g       = 9.81;      % [m/s^2]
    p.Sref    = 0.50;      % [m^2]
    p.Isp     = 2500;      % [s]
    p.massMin = 1000;      % [kg]

    %% Propulsion
    p.Tmax   = 40000;      % [N]
    p.t_rise = 3;          % [s]
    p.t_hold = 29;         % [s]
    p.t_fall = 8;          % [s]
    p.t_burn = p.t_rise + p.t_hold + p.t_fall;

    %% Control schedule defaults
    p.alpha_boost_deg         = 3.0;
    p.alpha_trim_glide_deg    = 1.8;
    p.alpha_terminal_trim_deg = 0.5;

    p.h_glide_cmd            = 18000;   % [m]
    p.gamma_glide_cmd_deg    = -0.3;    % [deg]
    p.gamma_terminal_cmd_deg = -6.0;    % [deg]
    p.t_terminal_start       = 110;     % [s]

    %% Controller gains
    p.Kh      = 1.0e-4;    % [deg/m]
    p.Kg      = 2.0;       % [deg/deg]
    p.Kg_term = 1.5;       % [deg/deg]

    %% Limits
    p.alpha_min_deg = -2.0;
    p.alpha_max_deg =  8.0;
    p.mu_min_deg    = -60.0;
    p.mu_max_deg    =  60.0;

    %% Initial state
    p.X0     = 0;
    p.Y0     = 0;
    p.Z0     = 500;
    p.V0     = 300;
    p.gamma0 = deg2rad(5);
    p.psi0   = deg2rad(0);
    p.m0     = 1400;

    %% Simulation time
    p.t0 = 0;
    p.tf = 300;

    %% Optimization constraints
    p.qbarMax  = 120e3;    % [Pa]
    p.nLoadMax = 6.0;      % [g]

    %% ODE settings
    p.odeRelTol = 1e-6;
    p.odeAbsTol = 1e-8;
end
