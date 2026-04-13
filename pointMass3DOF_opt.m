function xdot = pointMass3DOF_opt(t, x, p)
% POINTMASS3DOF_OPT
% Corrected 3DOF point-mass equations for simulation and optimization.
%
% States:
%   x = [X; Y; Z; V; gamma; psi; m]

    Z     = x(3);
    V     = x(4);
    gamma = x(5);
    psi   = x(6);
    m     = x(7);

    %% Numerical protections
    V = max(V, 1e-3);
    m = max(m, p.massMin);

    cosGamma = cos(gamma);
    if abs(cosGamma) < 1e-6
        if cosGamma >= 0
            cosGamma = 1e-6;
        else
            cosGamma = -1e-6;
        end
    end

    %% Control schedule
    [alphaDeg, muDeg] = controlSchedule_opt(t, x, p);
    alpha = deg2rad(alphaDeg);
    mu    = deg2rad(muDeg);

    %% Thrust model
    if m > p.massMin
        T = thrustProfile_opt(t, p);
    else
        T = 0;
    end

    %% Atmospheric and aerodynamic forces
    rho  = atmosphereDensity_opt(Z);
    a    = speedOfSound_opt(Z);
    Mach = V / max(a, 1e-6);

    [CL, CD] = aeroModel_opt(Mach, alpha);

    qbar = 0.5 * rho * V^2;
    L    = qbar * p.Sref * CL;
    D    = qbar * p.Sref * CD;

    %% Gravity
    g = p.g;

    %% Kinematics
    Xdot = V * cos(gamma) * cos(psi);
    Ydot = V * cos(gamma) * sin(psi);
    Zdot = V * sin(gamma);

    %% Dynamics
    Vdot     = (T * cos(alpha) - D) / m - g * sin(gamma);
    gammadot = ((T * sin(alpha) + L) * cos(mu)) / (m * V) - (g * cos(gamma)) / V;
    psidot   = ((T * sin(alpha) + L) * sin(mu)) / (m * V * cosGamma);
    mdot     = -massFlowRate_opt(T, p);

    xdot = [Xdot; Ydot; Zdot; Vdot; gammadot; psidot; mdot];
end
