function J = objectiveTrajectory(d, p)
% OBJECTIVETRAJECTORY
% Scalar objective function for FMINCON.
%
% Current objective:
%   maximize final downrange X_f
%
% Since FMINCON minimizes, we define:
%   J = -X_f + soft penalties

    sim = simulateTrajectory(d, p);

    %% Base objective: maximize final X
    % J = -sim.Xf;
    J = -sim.Xf - 500*sim.tf;
    %% Soft penalty: encourage terminal altitude near zero
    J = J + 1e3 * max(abs(sim.Zf) - 5.0, 0)^2;

    %% Soft penalty: discourage extremely short/failed trajectories
    if sim.tf < 20
        J = J + 1e6;
    end

    %% Soft penalty: discourage unnecessary lateral deviation
    J = J + 1e-4 * sim.Yf^2;
end
