function rho = atmosphereDensity_opt(Z)
% ATMOSPHEREDENSITY_OPT
% Simple exponential atmosphere.

    rho0 = 1.225;   % [kg/m^3]
    H    = 8500;    % [m]

    Z = max(Z, 0);
    rho = rho0 * exp(-Z / H);
end
