function [CL, CD] = aeroModel_opt(Mach, alpha)
% AEROMODEL_OPT
% Aerodynamic model based on the uploaded file.
%
% Inputs:
%   Mach  : Mach number [-]
%   alpha : angle of attack [rad]
%
% Outputs:
%   CL, CD

    CL_alpha = 2.8;
    CL0      = 0.05;
    CL       = CL0 + CL_alpha * alpha;

    CD0 = 0.045;
    k   = 0.10;
    CD  = CD0 + k * CL^2;

    if Mach > 0.8
        CD = CD * (1 + 0.06 * (Mach - 0.8));
    end
end
