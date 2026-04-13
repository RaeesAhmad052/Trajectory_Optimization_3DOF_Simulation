function sim = simulateTrajectory(d, p)
% SIMULATETRAJECTORY
% Runs the corrected 3DOF simulation for a given decision vector.
%
% Inputs:
%   d : [1x6] decision vector
%   p : fixed-parameter structure
%
% Outputs:
%   sim : structure with full state histories, control histories,
%         force histories, and performance metrics.

    arguments
        d (1,6) double
        p struct
    end

    %% Map decision vector into parameter structure
    pRun = p;
    pRun.alpha_boost_deg        = d(1);
    pRun.alpha_trim_glide_deg   = d(2);
    pRun.gamma_glide_cmd_deg    = d(3);
    pRun.h_glide_cmd            = d(4);
    pRun.t_terminal_start       = d(5);
    pRun.gamma_terminal_cmd_deg = d(6);

    %% Initial condition
    x0 = [pRun.X0; pRun.Y0; pRun.Z0; pRun.V0; pRun.gamma0; pRun.psi0; pRun.m0];

    %% Integrate equations of motion
    opts = odeset('RelTol', pRun.odeRelTol, ...
                  'AbsTol', pRun.odeAbsTol, ...
                  'Events', @(t, x) groundEvent_opt(t, x));

    [t, x] = ode45(@(t, x) pointMass3DOF_opt(t, x, pRun), [pRun.t0 pRun.tf], x0, opts);

    %% State extraction
    X     = x(:,1);
    Y     = x(:,2);
    Z     = x(:,3);
    V     = x(:,4);
    gamma = x(:,5);
    psi   = x(:,6);
    m     = x(:,7);

    N = numel(t);

    %% Preallocation
    alphaDeg = zeros(N,1);
    muDeg    = zeros(N,1);
    T        = zeros(N,1);
    rho      = zeros(N,1);
    a        = zeros(N,1);
    Mach     = zeros(N,1);
    CL       = zeros(N,1);
    CD       = zeros(N,1);
    qbar     = zeros(N,1);
    L        = zeros(N,1);
    D        = zeros(N,1);
    nLoad    = zeros(N,1);

    %% Recompute histories for post-processing
    for k = 1:N
        [alphaDeg(k), muDeg(k)] = controlSchedule_opt(t(k), x(k,:).', pRun);

        T(k)   = thrustProfile_opt(t(k), pRun);
        rho(k) = atmosphereDensity_opt(Z(k));
        a(k)   = speedOfSound_opt(Z(k));
        Mach(k)= V(k) / max(a(k), 1e-6);

        [CL(k), CD(k)] = aeroModel_opt(Mach(k), deg2rad(alphaDeg(k)));

        qbar(k) = 0.5 * rho(k) * V(k)^2;
        L(k)    = qbar(k) * pRun.Sref * CL(k);
        D(k)    = qbar(k) * pRun.Sref * CD(k);

        nLoad(k)= L(k) / max(m(k) * pRun.g, 1e-6);
    end

    %% Package outputs
    sim.t     = t;
    sim.x     = x;
    sim.X     = X;
    sim.Y     = Y;
    sim.Z     = Z;
    sim.V     = V;
    sim.gamma = gamma;
    sim.psi   = psi;
    sim.m     = m;

    sim.alphaDeg = alphaDeg;
    sim.muDeg    = muDeg;
    sim.T        = T;
    sim.rho      = rho;
    sim.a        = a;
    sim.Mach     = Mach;
    sim.CL       = CL;
    sim.CD       = CD;
    sim.qbar     = qbar;
    sim.L        = L;
    sim.D        = D;
    sim.nLoad    = nLoad;

    sim.Xf       = X(end);
    sim.Yf       = Y(end);
    sim.Zf       = Z(end);
    sim.Vf       = V(end);
    sim.gammaf   = gamma(end);
    sim.psif     = psi(end);
    sim.mf       = m(end);
    sim.tf       = t(end);

    sim.maxQbar  = max(qbar);
    sim.maxNLoad = max(nLoad);

    sim.reachedGround = sim.Zf <= 1.0;

    sim.resultsTable = table(t, X, Y, Z, V, rad2deg(gamma), rad2deg(psi), m, ...
        alphaDeg, muDeg, Mach, qbar, L, D, T, nLoad, ...
        'VariableNames', {'Time_s','X_m','Y_m','Z_m','V_mps','Gamma_deg', ...
        'Psi_deg','Mass_kg','Alpha_deg','Mu_deg','Mach','Qbar_Pa', ...
        'Lift_N','Drag_N','Thrust_N','NLoad_g'});
end
