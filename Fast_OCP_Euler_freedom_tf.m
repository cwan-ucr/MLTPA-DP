function [stateMat, controlMat] = Fast_OCP_Euler_freedom_tf(x0, v0, xf, vf, t0, tf, vl, dt, stage)
% eco-driving method for CAV
% using pesudo  
if ~exist ('v0')
    x0 = 0;
    v0 = 10.00;
    xf = 200;
    vf = 0;
    t0 = 0;
    tf = 18;
    dt = 1;
end

N = round((tf-t0)/dt);

% bound contraints
xMin = [0 0];
xMax = [inf vl];
uMin = -4.0;
uMax = 3.5;

ttmin = ceil((xf - x0) / vl + (vl - v0)^2 / (2*uMax*vl));

N_stop = 1;
if stage
    N_stop = min(1, (ceil((2*ttmin) / (2*dt) + 6)/N));
end

% 构造初始变量
% N+1个位置的变量
ns = 2;
nc = 1;

nVar = (N+1)*(ns+nc);
nsIndex = 1:(N+1)*ns;
nuIndex = (N+1)*ns+1:nVar-1;

% x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub,nonlcon,options)
% x0 = rand(nVar, 1);
% (0,0) -> (1,0)

t = linspace(t0,tf,N+1)';
X0 = quadratic(x0,xf,v0,vf,dt,t,t0,tf);
X0 = [X0(1:end-1), N_stop];
% X0 = [x0 + v0*t;v0*ones(N+1,1);zeros(N,1)];
% X0 = [X0; N_stop];

% boundary conditions
X0_con = [x0;v0;0];
XF_con = [xf;vf;0];
XF_mayer = [0;0;0];

% constraint
statMin = repmat(xMin, N+1, 1);
statMin = reshape(statMin, [], 1);
statMax = repmat(xMax, N+1, 1);
statMax = reshape(statMax, [], 1);

conMin = repmat(uMin, N, 1);
conMin = reshape(conMin, [], 1);
conMax = repmat(uMax, N, 1);
conMax = reshape(conMax, [], 1);

conMin(1) = -1;
conMax(1) = 1;
conMin(end) = -0.001;
conMax(end) = 0.001;


allVarLower = [statMin(:); conMin(:); 0];
allVarUpper = [statMax(:); conMax(:); 1];

optNLP = optimset( 'LargeScale', 'on', 'GradObj','off', 'GradConstr','off',...
    'DerivativeCheck', 'off', 'Display', 'off', 'TolX', 1e-4,...
    'TolFun', 1e-4, 'TolCon', 1e-7, 'MaxFunEvals',5e6,...
    'DiffMinChange',1e-6,'Algorithm','sqp','PlotFcns',[]); %'PlotFcns','optimplotfval'

% profile clear
% profile on

[X,fval,exitflag,output,lambda,grad,hessian] = fmincon(@(X) Fast_EL_obj(X,nsIndex,nuIndex,N,ns,nc,t0,tf), ...
                                                            X0,[], [], [], [], allVarLower, allVarUpper,...
                                                            @(X) Fast_EL_cons(X,nsIndex,nuIndex,dt,N,ns,nc,t0,tf,X0_con,XF_con,stage), optNLP);

% profile viewer
% %%  Costate Map 
% lambda1= lambda.eqnonlin(1:N+1);
% lambda2= lambda.eqnonlin(N+2:nVar);
% 
% omega1 = weight';
% 
% cs1=  lambda1./omega1;
% cs2=  lambda2./omega1;

%% 
stateVec = X(nsIndex);
stateMat = reshape(stateVec, N+1, ns);

controlVec = X(nuIndex);
controlMat = reshape(controlVec, N, nc);

% [~,Lagrange] = mycost(stateMat(end,:), t(end), stateMat, controlMat);
% 
% nodesPlot = linspace(-1, 1, N)';
% statePlot = zeros(N, ns);
% % 拉格朗日插值
% for i=1:ns
%     curState = stateMat(:, i);
%     curStatePlot = LagrangeInterpolation(nodesPlot, npts, curState); 
%     statePlot(:, i) = curStatePlot;
% end
%  controlPlot = LagrangeInterpolation(nodesPlot, npts, controlMat);
% %时间变换
% nodesPlot1 = 0.5*(tf-t0)*nodesPlot+0.5*(tf+t0);
% npts1 =  0.5*(tf-t0)*npts+0.5*(tf+t0);
% 
% figure;
% plot(nodesPlot1, statePlot(:,1), 's-', nodesPlot1, statePlot(:,2), 's-');
% grid on;
% 
% figure;
% plot(npts1, controlMat(:,1), 's-');
% grid on;
% 
% 
% %% 协态绘制
% figure;
% plot(npts1, cs1, 's-');
% hold on;
% plot(npts1, cs2, 's-');
% legend('costate1', 'costate2');
% grid on;
end