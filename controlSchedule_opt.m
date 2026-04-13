function [alphaDeg, muDeg] = controlSchedule_opt(t, x, p)
% CONTROLSCHEDULE_OPT
% Phase-based control schedule preserved from the uploaded model.
%
% Phases:
%   1) boost phase
%   2) glide-hold phase
%   3) terminal phase
%
% For the current optimization setup, bank angle is fixed at zero.

    Z     = x(3);
    gamma = x(5);

    %% Bank schedule
    muDeg = 0.0;
    muDeg = min(max(muDeg, p.mu_min_deg), p.mu_max_deg);

    %% Angle-of-attack schedule
    if t <= p.t_burn
        alphaCmdDeg = p.alpha_boost_deg;

    elseif t <= p.t_terminal_start
        hErr     = p.h_glide_cmd - Z;
        gammaErr = p.gamma_glide_cmd_deg - rad2deg(gamma);

        alphaCmdDeg = p.alpha_trim_glide_deg ...
                    + p.Kh * hErr ...
                    + p.Kg * gammaErr;

    else
        gammaErr = p.gamma_terminal_cmd_deg - rad2deg(gamma);

        alphaCmdDeg = p.alpha_terminal_trim_deg ...
                    + p.Kg_term * gammaErr;
    end

    %% Saturation
    alphaDeg = min(max(alphaCmdDeg, p.alpha_min_deg), p.alpha_max_deg);
end
