clc; clear;
dbstop if error

rng(2025)
t_cyc_b = 30;
d_max_b = 150;
v_max_b = 10;
dt = 0.5;

n_sample = 1000;
X_sample = lhsdesign(n_sample, 7, "criterion", "correlation");

Tc = t_cyc_b + round(X_sample(:, 1) * t_cyc_b * 3); % cycle_length, range [30, 120]s
D = d_max_b + round(X_sample(:, 2) * 400); % road_length, range [150, 650]m;
Vm = v_max_b + round(X_sample(:, 3) * 10, 1); % max speed, range[10, 20] m/s;
V0 = min(3 + X_sample(:, 4).*(Vm-3), Vm); % initial speed, range[3, Vm] m/s;
T0 = round(X_sample(:, 5).*Tc); %  entry cycle time, range[0.0, 1.0]  tcyc, s;
TG = round( (0.2 + 0.6 * X_sample(:, 6) ).*Tc); % green cycle ratio, range [0.2, 0.8] road length, m
W = X_sample(:, 7); % Weight value for travel time to energy consumption


CT = zeros(n_sample,1);

opt_traj_dataset = [];
Driving_dataset = [];

for m = 1:n_sample
    dtot = D(m);
    vmax = Vm(m);
    tcyc = Tc(m);
    entry_speed = V0(m);
    entry_time = T0(m);
    t_gre = TG(m);
    w = W(m);

    tic
    opt_traj = find_opt_traj(m, dtot, vmax, tcyc, t_gre, entry_time, entry_speed, w, dt);
    ct = toc;
    disp(ct)

    CT(m) = ct;

    X = opt_traj.X;
    V = opt_traj.V;
    A = opt_traj.A;
    T = entry_time + (cumsum(ones(size(X))) - 1)*dt;
    T = rem(T(1:end-1), tcyc);
    
    map = pi + pi*T/(t_gre+3);
    map(T>=(t_gre+3)) = pi*(T(T>=(t_gre+3)) - t_gre - 3)/(tcyc - t_gre - 3);

    X = X(1:end-1)/dtot;
    V = V(1:end-1)/vmax;
    T_l = cos(map);
    T_dot = -sin(map);
    A_pre = [0; A(1:end-1)];
    
    Driving_scenario = [m, dtot/d_max_b, tcyc/t_cyc_b, vmax/v_max_b,  entry_time/tcyc, t_gre/tcyc, entry_speed/vmax, w];
    Driving_input = zeros(length(A), 8);
    Driving_input(:, 1:end) = Driving_input + Driving_scenario;
    
    Feature_input = [X, V, A_pre, T_l, T_dot];
    Label_acc = A;
    data_set = [Driving_input, Feature_input, A];
    
    opt_traj_dataset = [opt_traj_dataset; data_set];
    
    if rem(m, 100) == 0
        csvwrite("opt_traj_data_m1.csv", opt_traj_dataset);
    end
            
end
