function mdot = massFlowRate_opt(T, p)
% MASSFLOWRATE_OPT
% Rocket-like mass depletion model.
%
% mdot = T / (Isp * g0)

    if T <= 0
        mdot = 0;
        return;
    end

    g0 = 9.80665;
    mdot = T / (p.Isp * g0);
end
