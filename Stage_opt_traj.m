function [Traj_stage] = Stage_opt_traj(D, t0, vmax, Tc, Tg, v0, vf, dt)
%% stage  trajectory collection
acceleration_range = 100;
Traj_stage = [];
k=0;
amax = 3.5;

t1min = ceil(D / vmax + (vmax - v0)^2 / (2*amax*vmax));
available_time_set = [];
tfmin = t0 + t1min;
if rem(tfmin, Tc) >= Tg + 3
    tfmin = ceil(tfmin / Tc) * Tc;
end
avail_traj = 0;
for tf_k=tfmin:dt:30 + tfmin
    k=k+1;
    t_con = tf_k - t0;
    if rem(tf_k, Tc) >= Tg + 3
        continue;
    end

    [VT_1, A]= Fast_OCP_Euler_freedom_tf(0, v0, D, vf, 0, t_con, vmax, dt, 0);% stage one trajectroy estimation
    if ~isempty(VT_1)
        v0_con = abs(VT_1(1,2) - v0) > 0.01;
        x0_con = abs(VT_1(1,1)) > 0.01;
        vf_con = abs(VT_1(end,2) - vf) > 0.01;
        xf_con = abs(VT_1(end, 1) - D) > 0.01;

        if v0_con || x0_con || vf_con || xf_con
            continue; % Skip to the next iteration if conditions are not met
        end
        available_time_set(end+1) = t_con;
        stop_flag = 0;
        if (nnz(VT_1(:,2) < 3)) > 16
            if length(available_time_set) > 1
                break
            end
            stop_flag = 1;
            [VT_1, A]= Fast_OCP_Euler_freedom_tf(0, v0, D, vf, 0, available_time_set(1), vmax, dt, 1);% stage one trajectroy estimation
        end
        
        tcc = ceil(acceleration_range / vmax + (vmax - VT_1(end, 2))^2 / (2*amax*vmax));
        DV_T = [];
        A_acc =[];
        Xk = [0, VT_1(end, 2)];
        while Xk(1) < acceleration_range
            Ak = min(amax, (vmax - Xk(2)) / dt);
            Xk(2) = min(vmax, Xk(2) + Ak*dt);
            Xk(1) = Xk(1) + Xk(2)*dt;
            DV_T = [DV_T; Xk];
            A_acc(end+1) = Ak;
        end

        VT2 = VT_1(:,1)-D; % adjusted locations   
        VD2 = [VT_1(1:end-1,2)/2+VT_1(2:end,2)/2, A]; % output speed profile
        
        VAD = [VT_1; DV_T];
        AAD = [A; A_acc'];

        EC = rta_battery(VD2(:,1),VD2(:,2)); % energy consumption estimation
        EC_addition = rta_battery(VAD(1:end-1, 2), AAD);
        Traj_stage(k).X=VT_1(:,1);
        Traj_stage(k).V=VT_1(:,2);
        Traj_stage(k).A= A;
        Traj_stage(k).E=EC;
        Traj_stage(k).TT=length(EC) * dt;
        Traj_stage(k).Point = length(EC) + 1;
        Traj_stage(k).EC = sum(EC) * dt;
        Traj_stage(k).tf = tf_k;
        Traj_stage(k).EC_add = sum(EC_addition) * dt;
        Traj_stage(k).TT_add = length(EC_addition) * dt;


        if k > 20 || (tf_k - t0) / (tfmin - t0) >= 1.2 || stop_flag
            break;
        end
        
    else
        Traj_stage(k).X=[];
        Traj_stage(k).V=[];
        Traj_stage(k).A=[];
        Traj_stage(k).E=[];
        Traj_stage(k).EC=[];
        Traj_stage(k).TT=[];
        Traj_stage(k).tf = [];
        Traj_stage(k).EC_add = [];
        Traj_stage(k).TT_add = [];
    end
end

end
