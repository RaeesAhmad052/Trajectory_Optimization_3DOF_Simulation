function a = speedOfSound_opt(Z)
% SPEEDOFSOUND_OPT
% Improved speed-of-sound model replacing the constant approximation.

    gammaAir = 1.4;
    R        = 287.05;

    Z = max(Z, 0);

    if Z <= 11000
        T = 288.15 - 0.0065 * Z;
    else
        T = 216.65;
    end

    a = sqrt(gammaAir * R * T);
end
