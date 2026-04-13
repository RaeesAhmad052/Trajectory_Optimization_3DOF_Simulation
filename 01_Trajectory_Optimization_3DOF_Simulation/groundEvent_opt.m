function [value, isterminal, direction] = groundEvent_opt(~, x)
% GROUNDEVENT_OPT
% Stop integration when altitude reaches ground while descending.

    Z = x(3);

    value      = Z;
    isterminal = 1;
    direction  = -1;
end
