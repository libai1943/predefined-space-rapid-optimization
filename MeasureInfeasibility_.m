% function infeasibility = MeasureInfeasibility(x, y, theta, v, a, phy, w, xr, yr, xf, yf, tf)
function infeasibility = MeasureInfeasibility_(x, y, theta, v, a, phy, w, tf)
global params_
nfe = length(x);
infeasibility = 0;
hi = tf / (nfe - 1);

for ii = 2 : nfe
    infeasibility = infeasibility + (x(ii) - x(ii-1) - hi * v(ii) * cos(theta(ii)))^2;
end

for ii = 2 : nfe
    infeasibility = infeasibility + (y(ii) - y(ii-1) - hi * v(ii) * sin(theta(ii)))^2;
end

for ii = 2 : nfe
    infeasibility = infeasibility + (v(ii) - v(ii-1) - hi * a(ii - 1))^2;
end

for ii = 2 : nfe
    infeasibility = infeasibility + (theta(ii) - theta(ii-1) - hi * tan(phy(ii)) * v(ii) / params_.vehicle.lw)^2;
end

for ii = 2 : nfe
    infeasibility = infeasibility + (phy(ii) - phy(ii-1) - hi * w(ii - 1))^2;
end

% for ii = 1 : nfe
%     infeasibility = infeasibility + (xf(ii) - x(ii) - params.vehicle.f2p * cos(theta(ii)))^2;
% end
% for ii = 1 : nfe
%     infeasibility = infeasibility + (yf(ii) - y(ii) - params.vehicle.f2p * sin(theta(ii)))^2;
% end
% for ii = 1 : nfe
%     infeasibility = infeasibility + (xr(ii) - x(ii) - params.vehicle.r2p * cos(theta(ii)))^2;
% end
% for ii = 1 : nfe
%     infeasibility = infeasibility + (yr(ii) - y(ii) - params.vehicle.r2p * sin(theta(ii)))^2;
% end
end