function runOptimization()
% RUNOPTIMIZATION
% Main optimization driver for the uploaded 3DOF point-mass model.
%
% This script:
%   1) builds a consistent fixed-parameter structure
%   2) defines optimization variables and bounds
%   3) runs FMINCON
%   4) simulates the optimized trajectory
%   5) plots and saves the final results
%
% Decision vector:
%   d = [ alpha_boost_deg, ...
%         alpha_trim_glide_deg, ...
%         gamma_glide_cmd_deg, ...
%         h_glide_cmd_m, ...
%         t_terminal_start_s, ...
%         gamma_terminal_cmd_deg ]
%
% Objective:
%   Maximize final downrange X_f
%
% Constraints:
%   - maximum dynamic pressure <= qbarMax
%   - maximum normal load factor <= nLoadMax
%   - final altitude Z_f = 0 (ground hit)
%
% Author note:
%   This framework is written to match the structure of your uploaded
%   3DOF simulation, but in optimization-ready MATLAB form.

    clc;
    close all;

    %% Fixed parameters
    p = getDefaultParameters();

    %% Initial guess for optimization variables
    d0 = [ ...
        3.0, ...     % alpha_boost_deg
        1.8, ...     % alpha_trim_glide_deg
       -0.3, ...     % gamma_glide_cmd_deg
        18000, ...   % h_glide_cmd [m]
        110, ...     % t_terminal_start [s]
       -6.0];        % gamma_terminal_cmd_deg

    %% Lower and upper bounds
    lb = [ ...
       -2.0, ...     % alpha_boost_deg
       -1.0, ...     % alpha_trim_glide_deg
       -3.0, ...     % gamma_glide_cmd_deg
        5000, ...    % h_glide_cmd [m]
        40, ...      % t_terminal_start [s]
       -20.0];       % gamma_terminal_cmd_deg

    ub = [ ...
        8.0, ...     % alpha_boost_deg
        8.0, ...     % alpha_trim_glide_deg
        2.0, ...     % gamma_glide_cmd_deg
        30000, ...   % h_glide_cmd [m]
        250, ...     % t_terminal_start [s]
        0.0];        % gamma_terminal_cmd_deg

    %% FMINCON options
    opts = optimoptions('fmincon', ...
        'Display', 'iter', ...
        'Algorithm', 'sqp', ...
        'MaxFunctionEvaluations', 5000, ...
        'MaxIterations', 200, ...
        'ConstraintTolerance', 1e-6, ...
        'OptimalityTolerance', 1e-6, ...
        'StepTolerance', 1e-8);

    %% Create optimization problem
    problem.objective = @(d) objectiveTrajectory(d, p);
    problem.x0        = d0;
    problem.lb        = lb;
    problem.ub        = ub;
    problem.nonlcon   = @(d) nonlinearConstraints(d, p);
    problem.solver    = 'fmincon';
    problem.options   = opts;

    fprintf('\n=====================================================\n');
    fprintf(' Starting 3DOF trajectory optimization\n');
    fprintf(' Objective: maximize final downrange X\n');
    fprintf('=====================================================\n');

    %% Run optimization
    [dOpt, JOpt, exitflag, output] = fmincon(problem); %#ok<ASGLU>

    %% Simulate optimized trajectory
    simOpt = simulateTrajectory(dOpt, p);

    %% Display optimized values
    fprintf('\n================ OPTIMIZATION COMPLETE ================\n');
    fprintf('Exit flag                 = %d\n', exitflag);
    fprintf('Final objective J         = %.6f\n', JOpt);
    fprintf('alpha_boost_deg           = %.4f deg\n', dOpt(1));
    fprintf('alpha_trim_glide_deg      = %.4f deg\n', dOpt(2));
    fprintf('gamma_glide_cmd_deg       = %.4f deg\n', dOpt(3));
    fprintf('h_glide_cmd               = %.4f m\n',   dOpt(4));
    fprintf('t_terminal_start          = %.4f s\n',   dOpt(5));
    fprintf('gamma_terminal_cmd_deg    = %.4f deg\n', dOpt(6));

    fprintf('\n---------------- Final Trajectory Metrics ----------------\n');
    fprintf('Final time                = %.3f s\n', simOpt.t(end));
    fprintf('Final X                   = %.3f m\n', simOpt.X(end));
    fprintf('Final Y                   = %.3f m\n', simOpt.Y(end));
    fprintf('Final Z                   = %.3f m\n', simOpt.Z(end));
    fprintf('Final V                   = %.3f m/s\n', simOpt.V(end));
    fprintf('Final gamma               = %.3f deg\n', rad2deg(simOpt.gamma(end)));
    fprintf('Final psi                 = %.3f deg\n', rad2deg(simOpt.psi(end)));
    fprintf('Final mass                = %.3f kg\n', simOpt.m(end));
    fprintf('Max qbar                  = %.3f Pa\n', simOpt.maxQbar);
    fprintf('Max load factor           = %.3f g\n', simOpt.maxNLoad);

    %% Plot and save
    plotTrajectoryResults(simOpt);
    save('optimization_result.mat', 'dOpt', 'simOpt', 'JOpt', 'output');

    fprintf('\nSaved optimization_result.mat successfully.\n');
end
