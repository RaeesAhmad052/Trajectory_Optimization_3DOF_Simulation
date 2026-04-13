function T = thrustProfile_opt(t, p)
% THRUSTPROFILE_OPT
% Piecewise thrust schedule:
%   - ramp up
%   - hold
%   - ramp down
%   - burnout

    if t < 0
        T = 0;
        return;
    end

    t1 = p.t_rise;
    t2 = p.t_rise + p.t_hold;
    t3 = p.t_rise + p.t_hold + p.t_fall;

    if t <= t1
        T = p.Tmax * (t / max(t1, 1e-6));
    elseif t <= t2
        T = p.Tmax;
    elseif t <= t3
        T = p.Tmax * (1 - (t - t2) / max(p.t_fall, 1e-6));
    else
        T = 0;
    end

    T = max(T, 0);
end
