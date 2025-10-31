function ret = Fast_EL_obj(X,nsIndex,nuIndex,N,ns,nc,t0,tf)
%LGL_ 此处显示有关此函数的摘要
%   此处显示详细说明


stateVec = X(nsIndex);
stateMat = reshape(stateVec, N+1, ns);
controlVec = X(nuIndex);
controlMat = reshape(controlVec, N, nc);

% mycost(XF, TF, X, U)
XF = stateMat(end, :);
XX = stateMat;
U = controlMat;

TF = tf;

[Mayer, Lagrange] = mycost(XF, TF, XX, U);

tmp1 = (1/N)*(tf-t0)*sum(Lagrange);
tmp2 = Mayer;

if isempty(tmp2)
    ret = tmp1;
else
    ret = tmp1+tmp2;
end

% ret = ret/20;

end