function plotTrajectoryResults(sim)
% PLOTTRAJECTORYRESULTS
% Plot utility for the corrected 3DOF trajectory framework.

    t = sim.t;

    figure('Name', '3D Trajectory');
    plot3(sim.X, sim.Y, sim.Z, 'LineWidth', 1.5);
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    zlabel('Z [m]');
    title('3D Trajectory');

    figure('Name', 'Altitude vs Downrange');
    plot(sim.X, sim.Z, 'LineWidth', 1.5);
    grid on;
    xlabel('X [m]');
    ylabel('Altitude Z [m]');
    title('Altitude vs Downrange');

    figure('Name', 'Altitude vs Time');
    plot(t, sim.Z, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Altitude Z [m]');
    title('Altitude vs Time');

    figure('Name', 'Speed vs Time');
    plot(t, sim.V, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Speed [m/s]');
    title('Speed vs Time');

    figure('Name', 'Flight-Path Angle vs Time');
    plot(t, rad2deg(sim.gamma), 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('\gamma [deg]');
    title('Flight-Path Angle vs Time');

    figure('Name', 'Heading Angle vs Time');
    plot(t, rad2deg(sim.psi), 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('\psi [deg]');
    title('Heading Angle vs Time');

    figure('Name', 'Mass vs Time');
    plot(t, sim.m, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Mass [kg]');
    title('Mass vs Time');

    figure('Name', 'Thrust vs Time');
    plot(t, sim.T, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Thrust [N]');
    title('Thrust vs Time');

    figure('Name', 'Mach vs Time');
    plot(t, sim.Mach, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Mach Number');
    title('Mach vs Time');

    figure('Name', 'Dynamic Pressure vs Time');
    plot(t, sim.qbar, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('qbar [Pa]');
    title('Dynamic Pressure vs Time');

    figure('Name', 'Lift Drag Thrust');
    plot(t, sim.L, 'LineWidth', 1.5); hold on;
    plot(t, sim.D, 'LineWidth', 1.5);
    plot(t, sim.T, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Force [N]');
    title('Lift, Drag, and Thrust vs Time');
    legend('Lift', 'Drag', 'Thrust', 'Location', 'best');

    figure('Name', 'Control Histories');
    plot(t, sim.alphaDeg, 'LineWidth', 1.5); hold on;
    plot(t, sim.muDeg, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Angle [deg]');
    title('Control Histories');
    legend('\alpha', '\mu', 'Location', 'best');

    figure('Name', 'Load Factor');
    plot(t, sim.nLoad, 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Load factor [g]');
    title('Normal Load Factor vs Time');
end
