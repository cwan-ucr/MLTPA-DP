%% 根据进入交叉口的时间、�?�度、公交停靠时间�?�终点�?�度得到�?优轨�?
function Traj_opt = find_opt_traj(m, D, vmax, Tc, Tg, t0, v0, w, dt)

vf = vmax;

[Traj_stage] = Stage_opt_traj(D, t0, vmax, Tc, Tg, v0, vf, dt);

Traj_stage = Traj_stage(~cellfun('isempty', {Traj_stage.X}));

EC_V = [Traj_stage.EC];
TT_V = [Traj_stage.TT];

ECmax = max(EC_V);
ECmin = min(EC_V);

TTmax = max(TT_V);
TTmin = min(TT_V);

index = 1;
if length(EC_V) > 1
    EC_ = (EC_V - ECmin) / (ECmax - ECmin);
    TT_ = (TT_V - TTmin) / (TTmax - TTmin);
    Obj = TT_ * w+  EC_ * (1-w);
    index = find(Obj == min(Obj));
end


Traj_opt.X=Traj_stage(index).X;
Traj_opt.V=Traj_stage(index).V;
Traj_opt.A=Traj_stage(index).A;
Traj_opt.E=Traj_stage(index).E;
Traj_opt.EC=Traj_stage(index).EC;
Traj_opt.TT=Traj_stage(index).TT;
Traj_opt.tf = Traj_stage(index).tf;

figure('Visible', 'off', 'Color', 'w');
hold on;

min_TT_index = find(TT_V == min(TT_V));
min_EC_index = find(EC_V == min(EC_V));

if length(min_TT_index) > 1
    min_TT_index = min_TT_index(1); % Select the first index if multiple minimums exist
end

if length(min_TT_index) > 1
    min_EC_index = min_EC_index(end); % Select the first index if multiple minimums exist
end

T_min = t0:dt:Traj_stage(min_TT_index).tf;
T_max = t0:dt:Traj_stage(min_EC_index).tf;
T_opt = t0:dt:Traj_stage(index).tf;

TL = ceil(max(T_max) / Tc);

plot(T_min, Traj_stage(min_TT_index).X, 'r-.', 'DisplayName', 'Min TT Trajectory', 'LineWidth', 2);
plot(T_opt, Traj_opt.X, 'b:', 'DisplayName', 'OPT Trajectory', 'LineWidth', 2);
plot(T_max, Traj_stage(min_EC_index).X, 'g--', 'DisplayName', 'Min EC Trajectory', 'LineWidth', 2);
legend

for i = 0:TL-1
    plot([i*Tc, i*Tc + Tg], [D - 2.5, D - 2.5], 'g', 'LineWidth', 5);
    plot([i*Tc + Tg, i*Tc + Tg + 3], [D - 2.5, D - 2.5], 'y', 'LineWidth', 5);
    plot([i*Tc + Tg + 3, (i+1)*Tc], [D - 2.5, D - 2.5], 'r', 'LineWidth', 5)
end

xlabel('Time (s)');
ylabel('Position (m)');
xlim([t0, TL*Tc]);
ylim([0 D + 25])

title('Optimal Trajectory Comparison');
% Display parameter values in the figure
papameter_text = {['D: ', num2str(D)];
                  ['v0: ', num2str(v0)];
                  ['vmax: ', num2str(vmax)];
                  ['t0: ', num2str(t0)];
                  ['Tc: ', num2str(Tc)];
                  ['Tg: ', num2str(Tg)];
                  ['w: ', num2str(w)]};
textStr = strjoin(papameter_text, newline);
text(t0 + 1, D - 50, textStr, 'FontSize', 10, 'Color', 'k');
legend('Min TT Trajectory', 'OPT Trajectory', 'Min EC Trajectory', 'Location','southeast');

filepath = 'Opt_Train//';
if ~exist(filepath, 'dir')
    mkdir(filepath);
end
filename = 'Optimal_Trajectory_Comparison_Scenario_';
saveas(gcf, [filepath, filename, num2str(m), '.png']);

hold off;

end