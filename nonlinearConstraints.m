function [c, ceq] = nonlinearConstraints(d, p)
% NONLINEARCONSTRAINTS
% Path and terminal constraints for the 3DOF optimization problem.
%
% Inequality constraints:
%   c <= 0
%
% Equality constraints:
%   ceq = 0

    sim = simulateTrajectory(d, p);

    %% Path constraints
    c1 = sim.maxQbar  - p.qbarMax;
    c2 = sim.maxNLoad - p.nLoadMax;

    c3 = 100 - sim.tf;   % enforce sim.tf >= 100 s

    c = [c1; c2; c3];

    %% Terminal constraint
    ceq = sim.Zf;
end
