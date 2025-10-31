function [Mayer, Lagrange] = mycost(XF, TF, X, U, ~)
% cost: operational energy

% state and control variables at terminal time tf
xf = XF(end,1);
vf = XF(end,2);

% Mayer cost

Mayer = [];

% state and control variables at time t
x = X(:,1);
v = X(:,2);
a = U(:,1);

% Lagrange1
% v_a = 0.5*(v(1:end-1)+v(2:end));
% Lagrange = rta_battery(v_a,a)./(v_a + 1e-5) + 0.1 * a.^2;
Lagrange = rta_battery(v(1:end-1),a);

end

