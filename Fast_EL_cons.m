function [c, ceq] = Fast_EL_cons(X,nsIndex,nuIndex,dt,N,ns,nc,t0,tf,X0_con,XF_con,stage)
%LGL_CONS 此处显示有关此函数的摘要
%   此处显示详细说明

stateVec = X(nsIndex);
stateMat = reshape(stateVec, N+1, ns);

controlVec = X(nuIndex);
controlMat = reshape(controlVec, N, nc);

x_stop = X(end);

% Forward Euler
% X_{k+1} = A*X_{k} + B*U_{k+1}
tmp1 = stateMat(2:end,2)-stateMat(1:end-1,2)-controlMat(1:end)*dt;

tmp2 = stateMat(2:end,1)-stateMat(1:end-1,1)-stateMat(2:end,2)*dt;

tmp3 = abs((controlMat(2:end) - controlMat(1:end-1))/dt) - 2;
tmp3 = 0;
tmp4 = 0;
tmp5 = 0;

N_stop = min(N, ceil(N*x_stop));
if stage == 1
    tmp4 = stateMat(N_stop,1) - XF_con(1);
    tmp5 = 5 - (XF_con(1) - X0_con(1)) / (N_stop*dt);
end

X0 = stateMat(1, :);
XF = stateMat(end, :);

TF = tf;
T0 = t0;

tmp6 = myevent(X0, T0, XF, TF, X0_con, XF_con, stage);

% tmp4 = [stateMat(1,1);  stateMat(1,2)];
% tmp5 = [stateMat(end, 1)-1;  stateMat(end, 2)];

ceq = [tmp1; tmp2; tmp6; tmp4];

c = [tmp3;tmp5];

end

