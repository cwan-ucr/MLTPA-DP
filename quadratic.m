function [X]= quadratic(x0, xf, v0, vf, dt, t, t0, tf)


a = 3*(v0+vf)/(tf-t0)^2 - 6*(xf-x0)/(tf-t0)^3;
b = (vf-v0)/(tf-t0) - a*(tf-t0);
u = 2*a*t + b;
v = a*t.^2 + b*t + v0;
x = x0+(1/3)*a*t.^3 + (1/2)*b*t.^2 + v0*t;

X = [x;v;u]';

end