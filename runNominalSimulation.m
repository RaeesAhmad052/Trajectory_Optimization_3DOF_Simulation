function runNominalSimulation()
% RUNNOMINALSIMULATION
% Runs the corrected nominal simulation without optimization.
%
% This is useful for validating the model before running FMINCON.

    clc;
    close all;

    p = getDefaultParameters();

    dNom = [ ...
        p.alpha_boost_deg, ...
        p.alpha_trim_glide_deg, ...
        p.gamma_glide_cmd_deg, ...
        p.h_glide_cmd, ...
        p.t_terminal_start, ...
        p.gamma_terminal_cmd_deg ];

    sim = simulateTrajectory(dNom, p);

    fprintf('\n===== NOMINAL TRAJECTORY =====\n');
    fprintf('Final time         = %.2f s\n', sim.t(end));
    fprintf('Final X position   = %.2f m\n', sim.X(end));
    fprintf('Final Y position   = %.2f m\n', sim.Y(end));
    fprintf('Final altitude Z   = %.2f m\n', sim.Z(end));
    fprintf('Final speed V      = %.2f m/s\n', sim.V(end));
    fprintf('Final gamma        = %.2f deg\n', rad2deg(sim.gamma(end)));
    fprintf('Final psi          = %.2f deg\n', rad2deg(sim.psi(end)));
    fprintf('Final mass         = %.2f kg\n', sim.m(end));
    fprintf('Final Mach         = %.2f\n', sim.Mach(end));

    disp(' ');
    disp('First 10 simulation samples:');
    disp(sim.resultsTable(1:min(10,height(sim.resultsTable)), :));

    plotTrajectoryResults(sim);
    save('nominal_simulation_result.mat', 'sim');

    fprintf('\nSaved nominal_simulation_result.mat successfully.\n');
end
