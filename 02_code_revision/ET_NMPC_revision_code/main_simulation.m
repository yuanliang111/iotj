% ==============================================================
% main_simulation.m
%
% ET-NMPC 多动态障碍物复杂城市环境避障仿真主程序
%
% 本文件功能：
% 1. 运行本文提出的事件触发双模式 NMPC 避障方法；
% 2. 支持多个动态障碍物；
% 3. 支持不同动态障碍物运动模式，包括 CV、CA、SIN、RAND；
% 4. 支持动态障碍物观测噪声；
% 5. 使用更精确的静态建筑物碰撞检测，避免长方体外接圆过度保守；
% 6. 记录触发率、CPU 时间、控制能耗、爬升高度、最小动态障碍距离；
% 7. 生成三维避障轨迹图和事件触发风险曲线图；
% 8. 增加终点直接引导逻辑；
% 9. 增加终点直线路径可见性检测，避免前方无障碍时仍绕路。
%
% 说明：
% 1. 本文件只用于 Proposed ET-NMPC 主方法的单次演示仿真；
% 2. 3D-APF、RRT* 等对比算法放到 main_comparison.m；
% 3. 如果发生静态碰撞，本版本会输出碰撞建筑物编号和无人机位置。
% ==============================================================

clear; clc; close all;

%% =========================================================
% 1. 参数初始化
% =========================================================

params = init_params();
params = complete_params_for_main_simulation(params);

if isfield(params, 'random_seed') && ~isempty(params.random_seed)
    rng(params.random_seed);
end

p_0 = params.p_0;
v_0 = params.v_0;
x_curr = [p_0; v_0];
p_goal = params.p_goal;

path_dir = (p_goal - p_0) / norm(p_goal - p_0);
path_len = norm(p_goal - p_0);

N_ref = params.N_ref;
Pi_ref = [
    linspace(p_0(1), p_goal(1), N_ref);
    linspace(p_0(2), p_goal(2), N_ref);
    linspace(p_0(3), p_goal(3), N_ref)
];

buildings = generate_city_map();
obs_array = initialize_dynamic_obstacles(params, p_0, p_goal);
num_obs = length(obs_array);

k = 0;
k_last = -params.T_ref;
sigma_k = 0;
sigma_prev = 0;

collision_flag = false;
success_flag = false;
static_hit_id = -1;
static_hit_info = struct();

% 初始化预测轨迹变量，避免动画阶段未定义
pred_x = x_curr(1);
pred_y = x_curr(2);
pred_z = x_curr(3);

%% =========================================================
% 2. 历史数据记录
% =========================================================

history_x = zeros(6, params.k_max);
history_u = zeros(3, params.k_max);
history_R = zeros(1, params.k_max);
history_sigma = zeros(1, params.k_max);
history_t_solve = zeros(1, params.k_max);
history_obs = zeros(3, params.k_max, num_obs);

history_obs_mode = zeros(1, num_obs);
for i = 1:num_obs
    history_obs_mode(i) = obs_array(i).motion_type;
end

%% =========================================================
% 3. 三维场景初始化
% =========================================================

if params.enable_animation

    fig_scene = figure( ...
        'Name', 'ET-NMPC Multi-Obstacle Urban Dynamic Avoidance', ...
        'Position', [100, 100, 950, 720], ...
        'Color', 'w');

    hold on; grid on;

    plot3( ...
        Pi_ref(1, :), Pi_ref(2, :), Pi_ref(3, :), ...
        'k--', 'LineWidth', 1.5, ...
        'DisplayName', 'Nominal Path');

    draw_buildings(buildings);

    plot3( ...
        p_0(1), p_0(2), p_0(3), ...
        'bo', 'MarkerSize', 8, 'MarkerFaceColor', 'b', ...
        'DisplayName', 'Start');

    plot3( ...
        p_goal(1), p_goal(2), p_goal(3), ...
        'g*', 'MarkerSize', 11, 'LineWidth', 2, ...
        'DisplayName', 'Goal');

    h_uav_traj = plot3( ...
        x_curr(1), x_curr(2), x_curr(3), ...
        'b-', 'LineWidth', 2.5, ...
        'DisplayName', 'UAV Trajectory');

    h_uav_pos = plot3( ...
        x_curr(1), x_curr(2), x_curr(3), ...
        'bo', 'MarkerSize', 6, 'MarkerFaceColor', 'b', ...
        'HandleVisibility', 'off');

    h_pred_traj = plot3( ...
        x_curr(1), x_curr(2), x_curr(3), ...
        'g-', 'LineWidth', 2.0, ...
        'DisplayName', 'NMPC Prediction');

    [sph_x, sph_y, sph_z] = sphere(20);

    h_obs_surf = gobjects(1, num_obs);
    h_obs_traj = gobjects(1, num_obs);

    for i = 1:num_obs

        obs_color = get_obstacle_color(i);

        h_obs_surf(i) = surf( ...
            sph_x * obs_array(i).r_i + obs_array(i).p(1), ...
            sph_y * obs_array(i).r_i + obs_array(i).p(2), ...
            sph_z * obs_array(i).r_i + obs_array(i).p(3), ...
            'FaceColor', obs_color, ...
            'EdgeColor', 'none', ...
            'FaceAlpha', 0.55, ...
            'HandleVisibility', 'off');

        h_obs_traj(i) = plot3( ...
            obs_array(i).p(1), obs_array(i).p(2), obs_array(i).p(3), ...
            '-.', 'LineWidth', 2.0, ...
            'Color', obs_color, ...
            'DisplayName', ['Dynamic Obstacle ', num2str(i)]);
    end

    xlabel('X [m]', 'FontSize', 14);
    ylabel('Y [m]', 'FontSize', 14);
    zlabel('Z [m]', 'FontSize', 14);

    set(gca, 'FontSize', 13);

    daspect([1 1 1]);
    xlim(params.x_lim);
    ylim(params.y_lim);
    zlim(params.z_lim);
    view([-30, 45]);

    camlight;
    lighting gouraud;

    legend('show', 'Location', 'northeast', 'FontSize', 11);
end

disp('==============================================================');
disp('开始运行 ET-NMPC 多动态障碍物仿真');
fprintf('动态障碍物数量：%d\n', num_obs);
fprintf('静态建筑物数量：%d\n', length(buildings));
fprintf('事件触发机制：%d\n', params.use_event_trigger);
fprintf('动态预测机制：%d\n', params.use_prediction);
fprintf('异质运动模式：%d\n', params.use_heterogeneous_motion);
fprintf('不可预测运动扰动：%d\n', params.use_unpredictable_motion);
disp('==============================================================');

%% =========================================================
% 4. 主仿真循环
% =========================================================

while norm(x_curr(1:3) - p_goal) > params.eps_tol && k < params.k_max

    k = k + 1;
    current_time = k * params.dt;

    history_x(:, k) = x_curr;

    %% =====================================================
    % 4.1 更新动态障碍物
    % =====================================================

    obs_array = update_dynamic_obstacles(obs_array, params, current_time);

    for i = 1:num_obs
        history_obs(:, k, i) = obs_array(i).p;
    end

    %% =====================================================
    % 4.2 动态碰撞检测
    % =====================================================

    if params.check_dynamic_collision

        for i = 1:num_obs

            dist_to_obs_i = norm(x_curr(1:3) - obs_array(i).p);
            collision_dist_i = params.r_u + obs_array(i).r_i + params.collision_buffer;

            if dist_to_obs_i <= collision_dist_i

                collision_flag = true;

                fprintf('\n发生动态障碍物碰撞！\n');
                fprintf('step = %d, obstacle = %d\n', k, i);
                fprintf('UAV position = [%.3f, %.3f, %.3f]\n', ...
                    x_curr(1), x_curr(2), x_curr(3));
                fprintf('Obstacle position = [%.3f, %.3f, %.3f]\n', ...
                    obs_array(i).p(1), obs_array(i).p(2), obs_array(i).p(3));
                fprintf('distance = %.3f m, threshold = %.3f m\n', ...
                    dist_to_obs_i, collision_dist_i);

                break;
            end
        end
    end

    if collision_flag
        if params.enable_animation
            set(h_collision_marker, ...
                'XData', x_curr(1), ...
                'YData', x_curr(2), ...
                'ZData', x_curr(3));
            drawnow;
        end
        break;
    end

    %% =====================================================
    % 4.3 静态建筑物碰撞检测
    % =====================================================

    if params.check_static_collision

        [static_collision, hit_id, hit_info] = check_static_collision_precise( ...
            x_curr, buildings, params);

        if static_collision

            collision_flag = true;
            static_hit_id = hit_id;
            static_hit_info = hit_info;

            fprintf('\n发生静态建筑物碰撞！\n');
            fprintf('step = %d\n', k);
            fprintf('碰撞建筑物编号：%d\n', hit_id);

            if isfield(buildings(hit_id), 'name')
                fprintf('碰撞建筑物名称：%s\n', buildings(hit_id).name);
            end

            fprintf('建筑物类型：%s\n', building_type_to_string(buildings(hit_id).type));
            fprintf('建筑物中心：[%.3f, %.3f], 高度：%.3f m\n', ...
                buildings(hit_id).x, buildings(hit_id).y, buildings(hit_id).h);
            fprintf('UAV position = [%.3f, %.3f, %.3f]\n', ...
                x_curr(1), x_curr(2), x_curr(3));
            fprintf('水平安全距离指标 = %.3f m\n', hit_info.horizontal_metric);
            fprintf('水平碰撞阈值 = %.3f m\n', hit_info.horizontal_threshold);
            fprintf('高度判定阈值 = %.3f m\n', hit_info.height_threshold);

            if params.enable_animation
                set(h_collision_marker, ...
                    'XData', x_curr(1), ...
                    'YData', x_curr(2), ...
                    'ZData', x_curr(3));
                drawnow;
            end

            break;
        end
    end

    %% =====================================================
    % 4.4 构造局部参考轨迹
    % =====================================================
    % 核心修改：
    % 1. 不再只依赖 goal_direct_radius；
    % 2. 如果无人机到终点的直线路径没有静态/动态障碍物阻挡，
    %    就直接生成指向终点的局部参考轨迹；
    % 3. 这样可以避免绕过障碍物后，在终点附近仍被名义路径拉回去。
    % =====================================================

    dist_to_goal = norm(p_goal - x_curr(1:3));

    goal_direct_radius = params.goal_direct_radius;
    goal_slow_radius = params.goal_slow_radius;

    goal_dir = p_goal - x_curr(1:3);

    if norm(goal_dir) > 1e-6
        goal_dir_unit = goal_dir / norm(goal_dir);
    else
        goal_dir_unit = zeros(3, 1);
    end

    % 判断当前点到终点之间是否存在明显障碍物阻挡
    line_to_goal_clear = check_line_to_goal_clear( ...
        x_curr(1:3), p_goal, buildings, obs_array, params);

    % 当前沿起点-终点方向的投影进度
    % 该进度用于判断 UAV 是否已经完成大部分全局航路。
    % 当已经绕过主要障碍物且到终点直线可行时，应直接转向终点，
    % 而不是继续被名义路径上的局部参考点牵引。
    s_curr_for_goal = dot(x_curr(1:3) - p_0, path_dir);
    s_curr_for_goal = max(0, min(s_curr_for_goal, path_len));
    path_progress = s_curr_for_goal / path_len;

    % 终点直达模式触发条件：
    % 1. 已经进入终点直接引导半径；
    % 2. 距离终点小于直线路径可见性半径，且当前到终点直线路径无障碍；
    % 3. 已经完成大部分路径，且当前到终点直线路径无障碍。
    %
    % 第 3 条是本次修改的关键：
    % 避免 UAV 绕过最后一个障碍物后，仍继续回到名义路径上绕行。
    use_direct_goal_guidance = ...
        (dist_to_goal <= goal_direct_radius) || ...
        (line_to_goal_clear && dist_to_goal <= params.goal_line_of_sight_radius) || ...
        (line_to_goal_clear && path_progress >= 0.65);

    if use_direct_goal_guidance

        x_ref_pos = p_goal;

        if dist_to_goal > goal_slow_radius
            v_ref_nom = min(params.v_max * 0.55, dist_to_goal) * goal_dir_unit;
        else
            v_ref_nom = min(params.v_max * 0.25, dist_to_goal) * goal_dir_unit;
        end

        Pi_ref_local = zeros(3, params.H);

        for h = 1:params.H
            alpha_h = min(1.0, h / params.H);
            Pi_ref_local(:, h) = x_curr(1:3) + alpha_h * (p_goal - x_curr(1:3));
        end

    else

        s_curr = dot(x_curr(1:3) - p_0, path_dir);
        s_curr = max(0, min(s_curr, path_len));

        s_ref = min(s_curr + 2.0, path_len);
        x_ref_pos = p_0 + s_ref * path_dir;

        v_ref_nom = params.v_max * 0.8 * path_dir;

        Pi_ref_local = zeros(3, params.H);

        for h = 1:params.H
            s_h = min(s_curr + h * 1.5 * (params.v_max * 0.8 * params.dt), path_len);
            Pi_ref_local(:, h) = p_0 + s_h * path_dir;
        end
    end

    x_ref = [x_ref_pos; v_ref_nom];

    %% =====================================================
    % 4.5 生成带噪声动态观测
    % =====================================================

    obs_data = struct([]);

    for i = 1:num_obs

        if params.use_measurement_noise
            measured_p = obs_array(i).p + randn(3, 1) * params.noise_pos_std;
            measured_v = obs_array(i).v + randn(3, 1) * params.noise_vel_std;
        else
            measured_p = obs_array(i).p;
            measured_v = obs_array(i).v;
        end

        obs_data(i).state = [measured_p; measured_v];
        obs_data(i).cov = obs_array(i).cov;
        obs_data(i).r_i = obs_array(i).r_i;
        obs_data(i).motion_type = obs_array(i).motion_type;
    end

    %% =====================================================
    % 4.6 卡尔曼预测
    % =====================================================

    obs_pred = kalman_predict(obs_data, x_curr, params);

    %% =====================================================
    % 4.7 预测消融逻辑
    % =====================================================

    if ~params.use_prediction

        for i = 1:length(obs_pred)
            obs_pred(i).pos = repmat(obs_pred(i).pos(:, 1), 1, params.H);
            obs_pred(i).vel = zeros(3, params.H);
            obs_pred(i).inflation = zeros(1, params.H);
        end
    end

    %% =====================================================
    % 4.8 复合风险计算
    % =====================================================

    [R_k, ~, ~, ~] = compute_risk(x_curr, x_ref, obs_pred, buildings, params);

    %% =====================================================
% 4.9 事件触发
% =====================================================

if params.use_event_trigger

    % 优先处理 R_off
    if R_k <= params.R_off
        sigma_k = 0;       % 立即归零
    elseif use_direct_goal_guidance
        sigma_k = 1;       % 终点直接引导阶段强制 NMPC
    elseif (R_k >= params.R_on) && ((k - k_last) >= params.T_ref)
        sigma_k = 1;       % 达到触发阈值且满足最小触发间隔
    else
        sigma_k = sigma_prev; % 否则保持上一步状态
    end

else
    sigma_k = 1; % 非事件触发模式，始终调用 NMPC
end

    %% =====================================================
    % 4.10 计算控制输入
    % =====================================================

    t_start = tic;

    if sigma_k == 1

        [u_cmd, X_opt, ~] = solve_nmpc_casadi( ...
            x_curr, Pi_ref_local, obs_pred, buildings, params);

        k_last = k;

        if ~isempty(X_opt) && size(X_opt, 1) >= 3
            pred_x = X_opt(1, :);
            pred_y = X_opt(2, :);
            pred_z = X_opt(3, :);
        else
            pred_x = Pi_ref_local(1, :);
            pred_y = Pi_ref_local(2, :);
            pred_z = Pi_ref_local(3, :);
        end

    else

        e_p = x_ref(1:3) - x_curr(1:3);
        e_v = x_ref(4:6) - x_curr(4:6);

        if use_direct_goal_guidance
            u_cmd_raw = 2.2 * e_p + 3.2 * e_v;
        elseif dist_to_goal < goal_slow_radius
            u_cmd_raw = 2.0 * e_p + 3.0 * e_v;
        else
            u_cmd_raw = 1.5 * e_p + 2.5 * e_v;
        end

        u_cmd = limit_vector_norm(u_cmd_raw, params.u_max);

        pred_x = Pi_ref_local(1, :);
        pred_y = Pi_ref_local(2, :);
        pred_z = Pi_ref_local(3, :);
    end

    history_t_solve(k) = toc(t_start);

    %% =====================================================
    % 4.11 UAV 状态更新
    % =====================================================

    x_next = zeros(6, 1);

    x_next(1:3) = x_curr(1:3) + x_curr(4:6) * params.dt;
    x_next(4:6) = x_curr(4:6) + u_cmd * params.dt;

    x_next(4:6) = limit_vector_norm(x_next(4:6), params.v_max);
    x_next(3) = max(min(x_next(3), params.z_max), params.z_min);

    history_u(:, k) = u_cmd;
    history_R(k) = R_k;
    history_sigma(k) = sigma_k;

    x_curr = x_next;
    sigma_prev = sigma_k;

    %% =====================================================
    % 4.12 动态更新三维图像
    % =====================================================

    if params.enable_animation

        set(h_uav_traj, ...
            'XData', history_x(1, 1:k), ...
            'YData', history_x(2, 1:k), ...
            'ZData', history_x(3, 1:k));

        set(h_uav_pos, ...
            'XData', x_curr(1), ...
            'YData', x_curr(2), ...
            'ZData', x_curr(3));

        set(h_pred_traj, ...
            'XData', pred_x, ...
            'YData', pred_y, ...
            'ZData', pred_z);

        for i = 1:num_obs

            set(h_obs_traj(i), ...
                'XData', squeeze(history_obs(1, 1:k, i)), ...
                'YData', squeeze(history_obs(2, 1:k, i)), ...
                'ZData', squeeze(history_obs(3, 1:k, i)));

            set(h_obs_surf(i), ...
                'XData', sph_x * obs_array(i).r_i + obs_array(i).p(1), ...
                'YData', sph_y * obs_array(i).r_i + obs_array(i).p(2), ...
                'ZData', sph_z * obs_array(i).r_i + obs_array(i).p(3));
        end

        drawnow;

        if k == 70 && params.save_figures
            snapshot_file = fullfile(params.output_folder, 'Avoidance_Snapshot_MultiObstacle.png');
            exportgraphics(fig_scene, snapshot_file, 'Resolution', params.figure_resolution);
            disp(['核心避障瞬间图已保存：', snapshot_file]);
        end
    end
end

%% =========================================================
% 5. 仿真结束状态判断
% =========================================================

if ~collision_flag && norm(x_curr(1:3) - p_goal) <= params.eps_tol
    success_flag = true;
end

valid_k = max(1, k);

disp(' ');
disp('==============================================================');
disp('仿真结束');
fprintf('结束步数：%d\n', k);
fprintf('是否成功到达目标：%d\n', success_flag);
fprintf('是否发生碰撞：%d\n', collision_flag);

if collision_flag && static_hit_id > 0
    fprintf('静态碰撞建筑物编号：%d\n', static_hit_id);
    fprintf('静态碰撞建筑物类型：%s\n', building_type_to_string(buildings(static_hit_id).type));
    fprintf('碰撞时 UAV 位置：[%.3f, %.3f, %.3f]\n', ...
        x_curr(1), x_curr(2), x_curr(3));
    fprintf('碰撞水平指标：%.3f，阈值：%.3f\n', ...
        static_hit_info.horizontal_metric, static_hit_info.horizontal_threshold);
end

disp('==============================================================');

if params.enable_animation && params.save_figures
    final_file = fullfile(params.output_folder, 'Avoidance_Final_MultiObstacle.png');
    exportgraphics(fig_scene, final_file, 'Resolution', params.figure_resolution);
    disp(['完整避障轨迹图已保存：', final_file]);
end

%% =========================================================
% 6. 自动提取论文核心数据
% =========================================================

trigger_rate = (sum(history_sigma(1:valid_k)) / valid_k) * 100;

avg_t_solve = mean(history_t_solve(1:valid_k)) * 1000;
max_t_solve = max(history_t_solve(1:valid_k)) * 1000;
std_t_solve = std(history_t_solve(1:valid_k)) * 1000;

total_control_effort = sum(sum(history_u(:, 1:valid_k).^2)) * params.dt;

delta_z = diff(history_x(3, 1:valid_k));
total_climb = sum(max(0, delta_z));

[min_dist_dyn, min_dist_each_obs] = compute_min_distance_to_obstacles( ...
    history_x, history_obs, obs_array, valid_k, params);

min_clearance_each_obs = zeros(1, num_obs);

for i = 1:num_obs
    min_clearance_each_obs(i) = min_dist_each_obs(i) - params.r_u - obs_array(i).r_i;
end

min_clearance_dyn = min(min_clearance_each_obs);

disp(' ');
disp('==============================================================');
disp('论文核心数据提取结果');
disp('==============================================================');
fprintf('动态障碍物数量                  : %d\n', num_obs);
fprintf('静态建筑物数量                  : %d\n', length(buildings));
fprintf('事件触发率 / NMPC Duty Cycle    : %.1f %%\n', trigger_rate);
fprintf('平均每步计算时间                : %.2f ms\n', avg_t_solve);
fprintf('最大单步计算时间                : %.2f ms\n', max_t_solve);
fprintf('计算时间标准差                  : %.2f ms\n', std_t_solve);
fprintf('控制输入能耗代理值              : %.2f\n', total_control_effort);
fprintf('总爬升高度                      : %.2f m\n', total_climb);
fprintf('到动态障碍物的最小距离          : %.2f m\n', min_dist_dyn);
fprintf('到动态障碍物的最小净安全间隔    : %.2f m\n', min_clearance_dyn);
fprintf('任务成功标志                    : %d\n', success_flag);
fprintf('碰撞标志                        : %d\n', collision_flag);
disp('==============================================================');

for i = 1:num_obs
    fprintf('Obstacle %d | 运动模式 = %s | 最小距离 = %.2f m | 最小净安全间隔 = %.2f m\n', ...
        i, motion_type_to_string(obs_array(i).motion_type), ...
        min_dist_each_obs(i), min_clearance_each_obs(i));
end

%% =========================================================
% 7. 保存数值结果为 CSV 文件
% =========================================================

if params.save_csv

    if static_hit_id > 0
        static_collision_building = static_hit_id;
    else
        static_collision_building = 0;
    end

    result_table = table( ...
        num_obs, ...
        length(buildings), ...
        success_flag, ...
        collision_flag, ...
        static_collision_building, ...
        trigger_rate, ...
        avg_t_solve, ...
        max_t_solve, ...
        std_t_solve, ...
        total_control_effort, ...
        total_climb, ...
        min_dist_dyn, ...
        min_clearance_dyn, ...
        'VariableNames', { ...
        'NumDynamicObstacles', ...
        'NumStaticBuildings', ...
        'Success', ...
        'Collision', ...
        'StaticCollisionBuildingID', ...
        'TriggerRatePercent', ...
        'AvgCPUms', ...
        'MaxCPUms', ...
        'StdCPUms', ...
        'ControlEffort', ...
        'TotalClimbMeter', ...
        'MinDynamicDistanceMeter', ...
        'MinDynamicClearanceMeter'});

    result_file = fullfile(params.output_folder, 'single_run_multi_obstacle_result.csv');
    writetable(result_table, result_file);
    disp(['单次仿真结果已保存：', result_file]);
end

%% =========================================================
% 8. 生成事件触发风险曲线图
% =========================================================

fig_trigger = figure( ...
    'Name', 'Event-Triggering Logic', ...
    'Position', [200, 120, 700, 520], ...
    'Color', 'w');

% =====================================================
% 统一上下两张图的横坐标范围和刻度
% =====================================================
% 例如 valid_k = 131 时，x_axis_max = 140；
% 这样上下两张图都会显示到 140，刻度也完全对齐。
x_axis_max = ceil((valid_k + 5) / 20) * 20;
x_ticks = 0:20:x_axis_max;

% -------------------- 上图：复合风险曲线 --------------------
subplot(2, 1, 1);

plot(1:valid_k, history_R(1:valid_k), 'LineWidth', 2);
hold on;

yline(params.R_on, 'r--', 'R_{on}', 'LineWidth', 1.5);
yline(params.R_off, 'g--', 'R_{off}', 'LineWidth', 1.5);

title('Composite Risk R_k over Time', 'FontSize', 14);
ylabel('Risk Value', 'FontSize', 14);

xlim([0, x_axis_max]);
xticks(x_ticks);

set(gca, 'FontSize', 13);
grid on;

% -------------------- 下图：事件触发信号 --------------------
subplot(2, 1, 2);

plot_k = 1:valid_k;
plot_sigma = history_sigma(1:valid_k);

% =====================================================
% 为了显示任务结束后 NMPC 关闭，并且让 0 状态延伸到右边界，
% 这里在绘图数据末尾补两个点：
%
% 1. valid_k + 1：让 sigma_k 从 1 掉到 0；
% 2. x_axis_max ：让 sigma_k = 0 一直延伸到横坐标右边界。
%
% 注意：
% 这只影响绘图，不改变控制逻辑，不改变 trigger_rate 统计。
% =====================================================
if success_flag && ~collision_flag
    plot_k = [plot_k, valid_k + 1, x_axis_max];
    plot_sigma = [plot_sigma, 0, 0];
end

stairs(plot_k, plot_sigma, 'LineWidth', 2);

title('Triggering Signal \sigma_k', 'FontSize', 14);
xlabel('Time Step k', 'FontSize', 14);
ylabel('Mode', 'FontSize', 14);

xlim([0, x_axis_max]);
xticks(x_ticks);
ylim([-0.2, 1.2]);

set(gca, 'FontSize', 13);
grid on;

if params.save_figures
    trigger_file = fullfile(params.output_folder, 'Risk_Triggering_MultiObstacle.png');
    exportgraphics(fig_trigger, trigger_file, 'Resolution', params.figure_resolution);
    disp(['事件触发风险曲线图已保存：', trigger_file]);
end

%% =========================================================
% 9. 局部函数
% =========================================================

function params = complete_params_for_main_simulation(params)
    % =====================================================
    % 补充缺失参数，保证旧版本 init_params.m 也可以运行
    % =====================================================

    if ~isfield(params, 'p_0')
        params.p_0 = [0; 0; 3];
    end

    if ~isfield(params, 'v_0')
        params.v_0 = [0; 0; 0];
    end

    if ~isfield(params, 'p_goal')
        params.p_goal = [40; 40; 12];
    end

    if ~isfield(params, 'N_ref')
        params.N_ref = 300;
    end

    if ~isfield(params, 'goal_direct_radius')
        params.goal_direct_radius = 18.0;
    end

    if ~isfield(params, 'goal_slow_radius')
        params.goal_slow_radius = 6.0;
    end

    % 新增：直线路径可见性半径
    % 当 UAV 距离终点小于该值，并且直线路径无障碍时，直接朝终点飞行。
    if ~isfield(params, 'goal_line_of_sight_radius')
        params.goal_line_of_sight_radius = 28.0;
    end

    % 新增：终点直线检查分辨率
    if ~isfield(params, 'goal_line_check_resolution')
        params.goal_line_check_resolution = 1.0;
    end

    % 新增：终点直达时的静态安全缓冲
    if ~isfield(params, 'goal_line_static_margin')
        params.goal_line_static_margin = 0.5;
    end

    % 新增：终点直达时的动态安全缓冲
    if ~isfield(params, 'goal_line_dynamic_margin')
        params.goal_line_dynamic_margin = 0.5;
    end

    if ~isfield(params, 'terminal_pos_weight')
        params.terminal_pos_weight = 120.0;
    end

    if ~isfield(params, 'terminal_vel_weight')
        params.terminal_vel_weight = 20.0;
    end

    % 新增：进入该半径后，solve_nmpc_casadi.m 可直接使用真实终点作为终端参考
    if ~isfield(params, 'terminal_goal_attract_radius')
        params.terminal_goal_attract_radius = 25.0;
    end

    % 新增：预测时域内的渐进终点吸引权重
    if ~isfield(params, 'terminal_path_weight')
        params.terminal_path_weight = 8.0;
    end

    % 新增：终点附近风险代价衰减半径
    if ~isfield(params, 'terminal_risk_decay_radius')
        params.terminal_risk_decay_radius = 12.0;
    end

    % 新增：终点附近风险代价最低保留比例
    if ~isfield(params, 'terminal_risk_min_factor')
        params.terminal_risk_min_factor = 0.45;
    end

    if ~isfield(params, 'use_multi_obstacles')
        params.use_multi_obstacles = true;
    end

    if ~isfield(params, 'num_dynamic_obstacles')
        params.num_dynamic_obstacles = 5;
    end

    if ~isfield(params, 'use_heterogeneous_motion')
        params.use_heterogeneous_motion = true;
    end

    if ~isfield(params, 'use_unpredictable_motion')
        params.use_unpredictable_motion = true;
    end

    if ~isfield(params, 'use_measurement_noise')
        params.use_measurement_noise = true;
    end

    if ~isfield(params, 'noise_pos_std')
        params.noise_pos_std = 0.4;
    end

    if ~isfield(params, 'noise_vel_std')
        params.noise_vel_std = 0.2;
    end

    if ~isfield(params, 'default_obs_radius')
        params.default_obs_radius = 1.5;
    end

    if ~isfield(params, 'large_obs_radius')
        params.large_obs_radius = 2.0;
    end

    if ~isfield(params, 'obs_cov_default')
        params.obs_cov_default = diag([0.1, 0.1, 0.1, 0.05, 0.05, 0.05]);
    end

    if ~isfield(params, 'motion_type_CV')
        params.motion_type_CV = 1;
    end

    if ~isfield(params, 'motion_type_CA')
        params.motion_type_CA = 2;
    end

    if ~isfield(params, 'motion_type_SIN')
        params.motion_type_SIN = 3;
    end

    if ~isfield(params, 'motion_type_RAND')
        params.motion_type_RAND = 4;
    end

    if ~isfield(params, 'heterogeneous_motion_types')
        params.heterogeneous_motion_types = [
            params.motion_type_CV, ...
            params.motion_type_CA, ...
            params.motion_type_SIN, ...
            params.motion_type_RAND
        ];
    end

    if ~isfield(params, 'default_motion_type')
        params.default_motion_type = params.motion_type_CV;
    end

    if ~isfield(params, 'obs_random_acc_max')
        params.obs_random_acc_max = 0.8;
    end

    if ~isfield(params, 'obs_random_acc_std')
        params.obs_random_acc_std = 0.35;
    end

    if ~isfield(params, 'obs_maneuver_prob')
        params.obs_maneuver_prob = 0.05;
    end

    if ~isfield(params, 'obs_velocity_jump_max')
        params.obs_velocity_jump_max = 0.6;
    end

    if ~isfield(params, 'obs_v_max')
        params.obs_v_max = 2.0;
    end

    if ~isfield(params, 'obs_sin_amp')
        params.obs_sin_amp = 2.0;
    end

    if ~isfield(params, 'obs_sin_omega')
        params.obs_sin_omega = 0.8;
    end

    if ~isfield(params, 'obs_ca_acc')
        params.obs_ca_acc = 0.25;
    end

    if ~isfield(params, 'obs_vertical_amp')
        params.obs_vertical_amp = 0.8;
    end

    if ~isfield(params, 'obs_vertical_omega')
        params.obs_vertical_omega = 0.5;
    end

    if ~isfield(params, 'collision_buffer')
        params.collision_buffer = 0.0;
    end

    if ~isfield(params, 'check_static_collision')
        params.check_static_collision = true;
    end

    if ~isfield(params, 'check_dynamic_collision')
        params.check_dynamic_collision = true;
    end

    if ~isfield(params, 'enable_animation')
        params.enable_animation = true;
    end

    if ~isfield(params, 'save_figures')
        params.save_figures = true;
    end

    if ~isfield(params, 'figure_resolution')
        params.figure_resolution = 300;
    end

    if ~isfield(params, 'save_csv')
        params.save_csv = true;
    end

    if ~isfield(params, 'output_folder')
        params.output_folder = 'results_reviewer_response';
    end

    if ~isfield(params, 'x_lim')
        params.x_lim = [-5, 45];
    end

    if ~isfield(params, 'y_lim')
        params.y_lim = [-5, 45];
    end

    if ~isfield(params, 'z_lim')
        params.z_lim = [0, 20];
    end

    if params.goal_direct_radius <= params.goal_slow_radius
        params.goal_direct_radius = params.goal_slow_radius + 3.0;
    end

    if params.goal_line_of_sight_radius <= params.goal_direct_radius
        params.goal_line_of_sight_radius = params.goal_direct_radius + 10.0;
    end

    if params.goal_line_check_resolution <= 0
        params.goal_line_check_resolution = 1.0;
    end

    params.terminal_risk_min_factor = max(0.0, min(1.0, params.terminal_risk_min_factor));

    if params.save_csv || params.save_figures
        if ~exist(params.output_folder, 'dir')
            mkdir(params.output_folder);
        end
    end
end

function obs_array = initialize_dynamic_obstacles(params, p_0, p_goal)
    % =====================================================
    % 初始化多个动态障碍物
    % =====================================================

    if params.use_multi_obstacles
        num_obs = params.num_dynamic_obstacles;
    else
        num_obs = 1;
    end

    obs_array = struct([]);

    path_vec = p_goal - p_0;
    path_dir = path_vec / norm(path_vec);

    vertical_dir = [0; 0; 1];

    lateral_dir_1 = cross(path_dir, vertical_dir);

    if norm(lateral_dir_1) < 1e-6
        lateral_dir_1 = [1; 0; 0];
    else
        lateral_dir_1 = lateral_dir_1 / norm(lateral_dir_1);
    end

    lateral_dir_2 = cross(path_dir, lateral_dir_1);
    lateral_dir_2 = lateral_dir_2 / norm(lateral_dir_2);

    if num_obs == 1
        s_list = 0.35;
    else
        s_list = linspace(0.25, 0.75, num_obs);
    end

    for i = 1:num_obs

        center_on_path = p_0 + s_list(i) * path_vec;
        side_sign = (-1)^i;

        lateral_offset = side_sign * (5.0 + 1.5 * rand) * lateral_dir_1;
        height_offset = (rand - 0.5) * 3.0 * lateral_dir_2;

        p_init = center_on_path + lateral_offset + height_offset;
        p_init(3) = max(min(p_init(3), params.z_max - 3.0), params.z_min + 3.0);

        to_path = center_on_path - p_init;

        if norm(to_path) < 1e-6
            v_dir = -side_sign * lateral_dir_1;
        else
            v_dir = to_path / norm(to_path);
        end

        speed_i = 0.7 + 0.6 * rand;
        v_init = speed_i * v_dir;
        v_init(3) = 0.15 * (rand - 0.5);

        if params.use_heterogeneous_motion
            mode_list = params.heterogeneous_motion_types;
            motion_type = mode_list(mod(i - 1, length(mode_list)) + 1);
        else
            motion_type = params.default_motion_type;
        end

        if i == 1
            p_init = [14; 30; 8];
            v_init = [0.8; -0.8; 0];
            r_i = params.large_obs_radius;
            motion_type = params.motion_type_CV;
        else
            r_i = params.default_obs_radius * (0.85 + 0.3 * rand);
        end

        obs_array(i).p = p_init;
        obs_array(i).p0 = p_init;
        obs_array(i).v = v_init;
        obs_array(i).v0 = v_init;
        obs_array(i).r_i = r_i;
        obs_array(i).cov = params.obs_cov_default;
        obs_array(i).motion_type = motion_type;
        obs_array(i).phase = 2 * pi * rand;
        obs_array(i).lateral_dir = side_sign * lateral_dir_1;

        acc_dir = 0.7 * v_dir + 0.3 * lateral_dir_1 * side_sign;

        if norm(acc_dir) < 1e-6
            acc_dir = v_dir;
        end

        obs_array(i).acc_dir = acc_dir / norm(acc_dir);
    end
end

function obs_array = update_dynamic_obstacles(obs_array, params, current_time)
    % =====================================================
    % 更新多个动态障碍物的真实运动状态
    % =====================================================

    for i = 1:length(obs_array)

        motion_type = obs_array(i).motion_type;

        if motion_type == params.motion_type_CV

            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;

        elseif motion_type == params.motion_type_CA

            acc = params.obs_ca_acc * obs_array(i).acc_dir;

            obs_array(i).v = obs_array(i).v + acc * params.dt;
            obs_array(i).v = limit_vector_norm(obs_array(i).v, params.obs_v_max);
            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;

        elseif motion_type == params.motion_type_SIN

            base_p = obs_array(i).p0 + obs_array(i).v0 * current_time;

            lateral_offset = ...
                params.obs_sin_amp * sin(params.obs_sin_omega * current_time + obs_array(i).phase) ...
                * obs_array(i).lateral_dir;

            vertical_offset = [
                0;
                0;
                params.obs_vertical_amp * sin(params.obs_vertical_omega * current_time + obs_array(i).phase)
            ];

            obs_array(i).p = base_p + lateral_offset + vertical_offset;

            lateral_v = ...
                params.obs_sin_amp * params.obs_sin_omega ...
                * cos(params.obs_sin_omega * current_time + obs_array(i).phase) ...
                * obs_array(i).lateral_dir;

            vertical_v = [
                0;
                0;
                params.obs_vertical_amp * params.obs_vertical_omega ...
                * cos(params.obs_vertical_omega * current_time + obs_array(i).phase)
            ];

            obs_array(i).v = obs_array(i).v0 + lateral_v + vertical_v;
            obs_array(i).v = limit_vector_norm(obs_array(i).v, params.obs_v_max);

        elseif motion_type == params.motion_type_RAND

            if params.use_unpredictable_motion

                acc_random = params.obs_random_acc_std * randn(3, 1);

                if norm(acc_random) > params.obs_random_acc_max
                    acc_random = acc_random / norm(acc_random) * params.obs_random_acc_max;
                end

                obs_array(i).v = obs_array(i).v + acc_random * params.dt;

                if rand < params.obs_maneuver_prob
                    dv = 2 * rand(3, 1) - 1;

                    if norm(dv) > 1e-6
                        dv = dv / norm(dv) * params.obs_velocity_jump_max;
                    end

                    obs_array(i).v = obs_array(i).v + dv;
                end
            end

            obs_array(i).v = limit_vector_norm(obs_array(i).v, params.obs_v_max);
            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;

        else

            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;
        end

        obs_array(i).p(3) = max(min(obs_array(i).p(3), params.z_max - 0.5), params.z_min + 0.5);
    end
end

function v_limited = limit_vector_norm(v, max_norm)
    % =====================================================
    % 向量范数限幅
    % =====================================================

    if norm(v) > max_norm
        v_limited = v / norm(v) * max_norm;
    else
        v_limited = v;
    end
end

function draw_buildings(buildings)
    % =====================================================
    % 绘制城市静态建筑物
    % type = 1：长方体
    % type = 2：圆柱体
    % type = 3：圆锥体
    % =====================================================

    for i = 1:length(buildings)

        b = buildings(i);

        if b.type == 1

            V = [
                b.x - b.w/2, b.y - b.l/2, 0;
                b.x + b.w/2, b.y - b.l/2, 0;
                b.x + b.w/2, b.y + b.l/2, 0;
                b.x - b.w/2, b.y + b.l/2, 0;
                b.x - b.w/2, b.y - b.l/2, b.h;
                b.x + b.w/2, b.y - b.l/2, b.h;
                b.x + b.w/2, b.y + b.l/2, b.h;
                b.x - b.w/2, b.y + b.l/2, b.h
            ];

            F = [
                1 2 6 5;
                2 3 7 6;
                3 4 8 7;
                4 1 5 8;
                5 6 7 8
            ];

            patch( ...
                'Vertices', V, ...
                'Faces', F, ...
                'FaceColor', [0.5 0.6 0.7], ...
                'FaceAlpha', 0.6, ...
                'HandleVisibility', 'off');

        elseif b.type == 2

            [xc, yc, zc] = cylinder(b.r, 30);

            surf( ...
                xc + b.x, yc + b.y, zc * b.h, ...
                'FaceColor', [0.7 0.5 0.5], ...
                'EdgeColor', 'none', ...
                'FaceAlpha', 0.6, ...
                'HandleVisibility', 'off');

        elseif b.type == 3

            [xc, yc, zc] = cylinder([b.r, 0], 30);

            surf( ...
                xc + b.x, yc + b.y, zc * b.h, ...
                'FaceColor', [0.5 0.7 0.5], ...
                'EdgeColor', 'none', ...
                'FaceAlpha', 0.6, ...
                'HandleVisibility', 'off');
        end
    end
end

function flag = check_line_to_goal_clear(p_start, p_goal, buildings, obs_array, params)
    % =====================================================
    % 检查从当前位置到终点的直线路径是否可通行
    %
    % 如果该直线路径上没有明显静态或动态障碍物，
    % main_simulation 就直接生成朝向终点的局部参考轨迹。
    % =====================================================

    flag = true;

    segment_len = norm(p_goal - p_start);

    if segment_len < 1e-6
        return;
    end

    n_check = max(3, ceil(segment_len / params.goal_line_check_resolution));

    for ii = 0:n_check

        alpha = ii / n_check;
        p = (1 - alpha) * p_start + alpha * p_goal;

        if is_point_inside_static_safety_zone(p, buildings, params)
            flag = false;
            return;
        end

        if is_point_inside_dynamic_safety_zone(p, obs_array, params)
            flag = false;
            return;
        end
    end
end

function flag = is_point_inside_static_safety_zone(p, buildings, params)
    % =====================================================
    % 判断某个点是否进入静态建筑物的安全缓冲区
    % 该函数比真实碰撞检测更保守，用于判断是否适合直线飞向终点。
    % =====================================================

    flag = false;

    for i = 1:length(buildings)

        b = buildings(i);

        height_threshold = b.h + params.r_u + params.goal_line_static_margin;

        if p(3) > height_threshold
            continue;
        end

        if b.type == 1

            dx_out = max(abs(p(1) - b.x) - b.w / 2, 0);
            dy_out = max(abs(p(2) - b.y) - b.l / 2, 0);

            dist_to_rect = sqrt(dx_out^2 + dy_out^2);
            threshold = params.r_u + params.goal_line_static_margin;

            if dist_to_rect <= threshold
                flag = true;
                return;
            end

        elseif b.type == 2

            dist_xy = norm(p(1:2) - [b.x; b.y]);
            threshold = b.r + params.r_u + params.goal_line_static_margin;

            if dist_xy <= threshold
                flag = true;
                return;
            end

        elseif b.type == 3

            if p(3) <= b.h + params.r_u + params.goal_line_static_margin

                cone_scale = max(0, 1 - p(3) / max(b.h, 1e-6));
                cone_radius_at_z = b.r * cone_scale;

                dist_xy = norm(p(1:2) - [b.x; b.y]);
                threshold = cone_radius_at_z + params.r_u + params.goal_line_static_margin;

                if dist_xy <= threshold
                    flag = true;
                    return;
                end
            end
        end
    end
end

function flag = is_point_inside_dynamic_safety_zone(p, obs_array, params)
    % =====================================================
    % 判断某个点是否进入动态障碍物当前安全缓冲区
    % 用于判断是否适合直线飞向终点。
    % =====================================================

    flag = false;

    for i = 1:length(obs_array)

        dist_i = norm(p - obs_array(i).p);
        threshold = params.r_u + obs_array(i).r_i + params.goal_line_dynamic_margin;

        if dist_i <= threshold
            flag = true;
            return;
        end
    end
end

function [flag, hit_id, info] = check_static_collision_precise(x_curr, buildings, params)
    % =====================================================
    % 精确静态建筑物碰撞检测
    % =====================================================

    flag = false;
    hit_id = -1;

    info.horizontal_metric = inf;
    info.horizontal_threshold = inf;
    info.height_threshold = inf;

    p = x_curr(1:3);

    for i = 1:length(buildings)

        b = buildings(i);

        height_threshold = b.h + params.r_u + params.collision_buffer;

        if p(3) > height_threshold
            continue;
        end

        if b.type == 1

            dx_out = max(abs(p(1) - b.x) - b.w / 2, 0);
            dy_out = max(abs(p(2) - b.y) - b.l / 2, 0);

            dist_to_rect = sqrt(dx_out^2 + dy_out^2);

            horizontal_metric = dist_to_rect;
            horizontal_threshold = params.r_u + params.collision_buffer;

            if dist_to_rect <= horizontal_threshold
                flag = true;
                hit_id = i;
                info.horizontal_metric = horizontal_metric;
                info.horizontal_threshold = horizontal_threshold;
                info.height_threshold = height_threshold;
                return;
            end

        elseif b.type == 2

            dist_xy = norm(p(1:2) - [b.x; b.y]);

            horizontal_metric = dist_xy;
            horizontal_threshold = b.r + params.r_u + params.collision_buffer;

            if dist_xy <= horizontal_threshold
                flag = true;
                hit_id = i;
                info.horizontal_metric = horizontal_metric;
                info.horizontal_threshold = horizontal_threshold;
                info.height_threshold = height_threshold;
                return;
            end

        elseif b.type == 3

            if p(3) <= b.h + params.r_u

                cone_scale = max(0, 1 - p(3) / max(b.h, 1e-6));
                cone_radius_at_z = b.r * cone_scale;

                dist_xy = norm(p(1:2) - [b.x; b.y]);

                horizontal_metric = dist_xy;
                horizontal_threshold = cone_radius_at_z + params.r_u + params.collision_buffer;

                if dist_xy <= horizontal_threshold
                    flag = true;
                    hit_id = i;
                    info.horizontal_metric = horizontal_metric;
                    info.horizontal_threshold = horizontal_threshold;
                    info.height_threshold = height_threshold;
                    return;
                end
            end
        end
    end
end

function [min_dist_dyn, min_dist_each_obs] = compute_min_distance_to_obstacles( ...
    history_x, history_obs, obs_array, valid_k, params)
    % =====================================================
    % 计算无人机到所有动态障碍物的最小距离
    % =====================================================

    %#ok<INUSD>

    num_obs = length(obs_array);
    min_dist_each_obs = inf(1, num_obs);

    for i = 1:num_obs

        pos_obs_i = squeeze(history_obs(:, 1:valid_k, i));
        pos_uav = history_x(1:3, 1:valid_k);

        dist_i = sqrt(sum((pos_uav - pos_obs_i).^2, 1));

        if isempty(dist_i)
            min_dist_each_obs(i) = inf;
        else
            min_dist_each_obs(i) = min(dist_i);
        end
    end

    min_dist_dyn = min(min_dist_each_obs);

    if isempty(min_dist_dyn) || isinf(min_dist_dyn)
        min_dist_dyn = NaN;
    end
end

function color = get_obstacle_color(index)
    % =====================================================
    % 为不同动态障碍物设置不同颜色
    % =====================================================

    color_list = [
        0.85, 0.10, 0.10;
        0.10, 0.45, 0.85;
        0.10, 0.65, 0.20;
        0.75, 0.35, 0.85;
        0.95, 0.55, 0.10;
        0.20, 0.75, 0.75;
        0.55, 0.30, 0.10;
        0.30, 0.30, 0.30
    ];

    color = color_list(mod(index - 1, size(color_list, 1)) + 1, :);
end

function str = motion_type_to_string(motion_type)
    % =====================================================
    % 将运动模式编号转换为字符串
    % =====================================================

    if motion_type == 1
        str = 'CV';
    elseif motion_type == 2
        str = 'CA';
    elseif motion_type == 3
        str = 'SIN';
    elseif motion_type == 4
        str = 'RAND';
    else
        str = 'UNKNOWN';
    end
end

function str = building_type_to_string(type_id)
    % =====================================================
    % 将建筑物类型编号转换为字符串
    % =====================================================

    if type_id == 1
        str = 'Cuboid';
    elseif type_id == 2
        str = 'Cylinder';
    elseif type_id == 3
        str = 'Cone';
    else
        str = 'Unknown';
    end
end