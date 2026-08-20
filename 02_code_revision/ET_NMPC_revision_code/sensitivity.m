% ==============================================================
% sensitivity.m
%
% 多动态障碍物复杂场景下的事件触发参数敏感性分析脚本
%
% 本文件作用：
% 1. 在复杂城市峡谷和多动态障碍物场景下测试 R_on 的影响；
% 2. 在复杂城市峡谷和多动态障碍物场景下测试 T_ref 的影响；
% 3. 自动输出触发率、CPU 时间、安全距离、成功率、碰撞率等指标；
% 4. 保存 CSV 表格；
% 5. 生成适合论文使用的 PNG 和 PDF 图。
%
% 对应论文内容：
% 用于说明事件触发参数 R_on 和 T_ref 的选择合理性。
%
% 对应审稿意见：
% 1. Reviewer 1 第 4 条：复杂场景与可扩展性；
% 2. Reviewer 2 第 2 条：不可预测动态障碍物下的鲁棒性；
% 3. Reviewer 2 第 5 条：参数选择依据。
% ==============================================================

clear; clc; close all;

%% =========================================================
% 1. 初始化基础参数
% =========================================================

params_base = init_params();
params_base = complete_params_for_sensitivity(params_base);

% 固定随机种子，保证结果可复现
if isfield(params_base, 'random_seed') && ~isempty(params_base.random_seed)
    rng(params_base.random_seed);
end

% 起点、终点和路径方向
p_0 = params_base.p_0;
v_0 = params_base.v_0;
p_goal = params_base.p_goal;

path_dir = (p_goal - p_0) / norm(p_goal - p_0);
path_len = norm(p_goal - p_0);

% 生成静态建筑物
buildings = generate_city_map();

% 输出文件夹
if ~exist(params_base.output_folder, 'dir')
    mkdir(params_base.output_folder);
end

sensitivity_folder = fullfile(params_base.output_folder, 'sensitivity_results');

if ~exist(sensitivity_folder, 'dir')
    mkdir(sensitivity_folder);
end

%% =========================================================
% 2. 敏感性实验参数设置
% =========================================================

% R_on 扫描列表
R_on_list = [0.4, 0.5, 0.6, 0.7, 0.8];

% T_ref 扫描列表
T_ref_list = [1, 5, 10, 15, 20];

% 每个参数重复次数
% 如果你只是先调试，可以改成 3；
% 如果要放论文，建议至少 10 或 20。
num_repeats = params_base.num_sensitivity_repeats;

% 多动态障碍物数量
% 用 5 个障碍物作为复杂场景代表
num_dynamic_obstacles = params_base.num_dynamic_obstacles;

% 固定参数
default_R_on = params_base.R_on;
default_R_off = params_base.R_off;
default_T_ref = params_base.T_ref;

disp(' ');
disp('==============================================================');
disp('开始执行事件触发参数敏感性分析');
fprintf('动态障碍物数量：%d\n', num_dynamic_obstacles);
fprintf('R_on 扫描列表：%s\n', mat2str(R_on_list));
fprintf('T_ref 扫描列表：%s\n', mat2str(T_ref_list));
fprintf('每组重复次数：%d\n', num_repeats);
disp('==============================================================');

%% =========================================================
% 3. 实验 A：扫描 R_on
% =========================================================

data_Ron = zeros(length(R_on_list), 9);

% 列含义：
% 1 TriggerRate
% 2 AvgCPU
% 3 MaxCPU
% 4 StdCPU
% 5 MinDistance
% 6 MinClearance
% 7 SuccessRate
% 8 CollisionRate
% 9 SwitchCount

detail_Ron_rows = [];

disp(' ');
disp('--------------------------------------------------------------');
disp('实验 A：扫描 R_on');
disp('--------------------------------------------------------------');

for i = 1:length(R_on_list)

    R_on_value = R_on_list(i);

    % 为了保持迟滞区间宽度相对一致，R_off 设置为 R_on - 0.3
    % 但不能小于 0.1。
    R_off_value = max(0.1, R_on_value - 0.3);

    temp_results = zeros(num_repeats, 9);

    fprintf('\n当前 R_on = %.2f, R_off = %.2f\n', R_on_value, R_off_value);

    for r = 1:num_repeats

        fprintf('  重复实验 %d / %d\n', r, num_repeats);

        params = params_base;

        % 设置复杂场景
        params.num_dynamic_obstacles = num_dynamic_obstacles;
        params.use_multi_obstacles = true;
        params.use_heterogeneous_motion = true;
        params.use_unpredictable_motion = true;
        params.use_measurement_noise = true;

        % 只测试 Proposed ET-NMPC
        params.use_event_trigger = true;
        params.use_prediction = true;
        params.use_asymmetric_energy = true;

        % 设置当前扫描参数
        params.R_on = R_on_value;
        params.R_off = R_off_value;
        params.T_ref = default_T_ref;

        % 关闭动画，加快敏感性实验速度
        params.enable_animation = false;
        params.save_figures = false;

        % 设置随机种子
        scenario_seed = params_base.random_seed + 1000 * i + r;

        result = run_single_sensitivity_simulation( ...
            params, p_0, v_0, p_goal, path_dir, path_len, buildings, scenario_seed);

        temp_results(r, :) = [
            result.trigger_rate, ...
            result.avg_cpu, ...
            result.max_cpu, ...
            result.std_cpu, ...
            result.min_dist_dyn, ...
            result.min_clearance_dyn, ...
            result.success, ...
            result.collision, ...
            result.switch_count
        ];

        detail_Ron_rows = [detail_Ron_rows; { ...
            'R_on', ...
            R_on_value, ...
            R_off_value, ...
            params.T_ref, ...
            r, ...
            result.trigger_rate, ...
            result.avg_cpu, ...
            result.max_cpu, ...
            result.std_cpu, ...
            result.min_dist_dyn, ...
            result.min_clearance_dyn, ...
            result.success, ...
            result.collision, ...
            result.switch_count, ...
            result.steps ...
        }]; %#ok<AGROW>
    end

    % 统计平均值
    data_Ron(i, 1) = mean(temp_results(:, 1));       % 触发率
    data_Ron(i, 2) = mean(temp_results(:, 2));       % 平均 CPU
    data_Ron(i, 3) = mean(temp_results(:, 3));       % 最大 CPU
    data_Ron(i, 4) = mean(temp_results(:, 4));       % CPU 标准差
    data_Ron(i, 5) = mean(temp_results(:, 5));       % 最小距离
    data_Ron(i, 6) = mean(temp_results(:, 6));       % 最小净安全间隔
    data_Ron(i, 7) = mean(temp_results(:, 7)) * 100; % 成功率
    data_Ron(i, 8) = mean(temp_results(:, 8)) * 100; % 碰撞率
    data_Ron(i, 9) = mean(temp_results(:, 9));       % 切换次数

    fprintf('完成 R_on = %.2f | Trigger = %.1f%% | CPU = %.2f ms | Clearance = %.2f m | Success = %.1f%%\n', ...
        R_on_value, data_Ron(i, 1), data_Ron(i, 2), data_Ron(i, 6), data_Ron(i, 7));
end

%% =========================================================
% 4. 实验 B：扫描 T_ref
% =========================================================

data_Tref = zeros(length(T_ref_list), 9);

detail_Tref_rows = [];

disp(' ');
disp('--------------------------------------------------------------');
disp('实验 B：扫描 T_ref');
disp('--------------------------------------------------------------');

for i = 1:length(T_ref_list)

    T_ref_value = T_ref_list(i);

    temp_results = zeros(num_repeats, 9);

    fprintf('\n当前 T_ref = %d\n', T_ref_value);

    for r = 1:num_repeats

        fprintf('  重复实验 %d / %d\n', r, num_repeats);

        params = params_base;

        % 设置复杂场景
        params.num_dynamic_obstacles = num_dynamic_obstacles;
        params.use_multi_obstacles = true;
        params.use_heterogeneous_motion = true;
        params.use_unpredictable_motion = true;
        params.use_measurement_noise = true;

        % 只测试 Proposed ET-NMPC
        params.use_event_trigger = true;
        params.use_prediction = true;
        params.use_asymmetric_energy = true;

        % 设置当前扫描参数
        params.R_on = default_R_on;
        params.R_off = default_R_off;
        params.T_ref = T_ref_value;

        % 关闭动画，加快实验
        params.enable_animation = false;
        params.save_figures = false;

        % 设置随机种子
        scenario_seed = params_base.random_seed + 10000 + 1000 * i + r;

        result = run_single_sensitivity_simulation( ...
            params, p_0, v_0, p_goal, path_dir, path_len, buildings, scenario_seed);

        temp_results(r, :) = [
            result.trigger_rate, ...
            result.avg_cpu, ...
            result.max_cpu, ...
            result.std_cpu, ...
            result.min_dist_dyn, ...
            result.min_clearance_dyn, ...
            result.success, ...
            result.collision, ...
            result.switch_count
        ];

        detail_Tref_rows = [detail_Tref_rows; { ...
            'T_ref', ...
            params.R_on, ...
            params.R_off, ...
            T_ref_value, ...
            r, ...
            result.trigger_rate, ...
            result.avg_cpu, ...
            result.max_cpu, ...
            result.std_cpu, ...
            result.min_dist_dyn, ...
            result.min_clearance_dyn, ...
            result.success, ...
            result.collision, ...
            result.switch_count, ...
            result.steps ...
        }]; %#ok<AGROW>
    end

    % 统计平均值
    data_Tref(i, 1) = mean(temp_results(:, 1));
    data_Tref(i, 2) = mean(temp_results(:, 2));
    data_Tref(i, 3) = mean(temp_results(:, 3));
    data_Tref(i, 4) = mean(temp_results(:, 4));
    data_Tref(i, 5) = mean(temp_results(:, 5));
    data_Tref(i, 6) = mean(temp_results(:, 6));
    data_Tref(i, 7) = mean(temp_results(:, 7)) * 100;
    data_Tref(i, 8) = mean(temp_results(:, 8)) * 100;
    data_Tref(i, 9) = mean(temp_results(:, 9));

    fprintf('完成 T_ref = %d | Trigger = %.1f%% | CPU = %.2f ms | Clearance = %.2f m | Success = %.1f%%\n', ...
        T_ref_value, data_Tref(i, 1), data_Tref(i, 2), data_Tref(i, 6), data_Tref(i, 7));
end

%% =========================================================
% 5. 打印结果表格
% =========================================================

disp(' ');
disp('==============================================================');
disp('实验 A：R_on 敏感性分析结果');
disp('==============================================================');
fprintf('R_on | Trigger | Avg CPU | Max CPU | Min Dist | Clearance | Success | Collision | Switch\n');
fprintf('-----------------------------------------------------------------------------------------\n');

for i = 1:length(R_on_list)
    fprintf('%.2f | %7.1f | %7.2f | %7.2f | %8.2f | %9.2f | %7.1f | %9.1f | %6.1f\n', ...
        R_on_list(i), ...
        data_Ron(i, 1), ...
        data_Ron(i, 2), ...
        data_Ron(i, 3), ...
        data_Ron(i, 5), ...
        data_Ron(i, 6), ...
        data_Ron(i, 7), ...
        data_Ron(i, 8), ...
        data_Ron(i, 9));
end

disp(' ');
disp('==============================================================');
disp('实验 B：T_ref 敏感性分析结果');
disp('==============================================================');
fprintf('T_ref | Trigger | Avg CPU | Max CPU | Min Dist | Clearance | Success | Collision | Switch\n');
fprintf('-----------------------------------------------------------------------------------------\n');

for i = 1:length(T_ref_list)
    fprintf('%5d | %7.1f | %7.2f | %7.2f | %8.2f | %9.2f | %7.1f | %9.1f | %6.1f\n', ...
        T_ref_list(i), ...
        data_Tref(i, 1), ...
        data_Tref(i, 2), ...
        data_Tref(i, 3), ...
        data_Tref(i, 5), ...
        data_Tref(i, 6), ...
        data_Tref(i, 7), ...
        data_Tref(i, 8), ...
        data_Tref(i, 9));
end

%% =========================================================
% 6. 保存 CSV 文件
% =========================================================

% R_on 汇总表
table_Ron = table( ...
    R_on_list(:), ...
    max(0.1, R_on_list(:) - 0.3), ...
    repmat(default_T_ref, length(R_on_list), 1), ...
    data_Ron(:, 1), ...
    data_Ron(:, 2), ...
    data_Ron(:, 3), ...
    data_Ron(:, 4), ...
    data_Ron(:, 5), ...
    data_Ron(:, 6), ...
    data_Ron(:, 7), ...
    data_Ron(:, 8), ...
    data_Ron(:, 9), ...
    'VariableNames', { ...
    'R_on', ...
    'R_off', ...
    'T_ref', ...
    'TriggerRatePercent', ...
    'AvgCPUms', ...
    'MaxCPUms', ...
    'StdCPUms', ...
    'MinDistanceMeter', ...
    'MinClearanceMeter', ...
    'SuccessRatePercent', ...
    'CollisionRatePercent', ...
    'SwitchCount'});

Ron_file = fullfile(sensitivity_folder, 'sensitivity_Ron_results.csv');
writetable(table_Ron, Ron_file);

% T_ref 汇总表
table_Tref = table( ...
    repmat(default_R_on, length(T_ref_list), 1), ...
    repmat(default_R_off, length(T_ref_list), 1), ...
    T_ref_list(:), ...
    data_Tref(:, 1), ...
    data_Tref(:, 2), ...
    data_Tref(:, 3), ...
    data_Tref(:, 4), ...
    data_Tref(:, 5), ...
    data_Tref(:, 6), ...
    data_Tref(:, 7), ...
    data_Tref(:, 8), ...
    data_Tref(:, 9), ...
    'VariableNames', { ...
    'R_on', ...
    'R_off', ...
    'T_ref', ...
    'TriggerRatePercent', ...
    'AvgCPUms', ...
    'MaxCPUms', ...
    'StdCPUms', ...
    'MinDistanceMeter', ...
    'MinClearanceMeter', ...
    'SuccessRatePercent', ...
    'CollisionRatePercent', ...
    'SwitchCount'});

Tref_file = fullfile(sensitivity_folder, 'sensitivity_Tref_results.csv');
writetable(table_Tref, Tref_file);

% 详细表
detail_all_rows = [detail_Ron_rows; detail_Tref_rows];

detail_table = cell2table(detail_all_rows, ...
    'VariableNames', { ...
    'ExperimentType', ...
    'R_on', ...
    'R_off', ...
    'T_ref', ...
    'RepeatIndex', ...
    'TriggerRatePercent', ...
    'AvgCPUms', ...
    'MaxCPUms', ...
    'StdCPUms', ...
    'MinDistanceMeter', ...
    'MinClearanceMeter', ...
    'Success', ...
    'Collision', ...
    'SwitchCount', ...
    'Steps'});

detail_file = fullfile(sensitivity_folder, 'sensitivity_detail_results.csv');
writetable(detail_table, detail_file);

disp(' ');
disp(['R_on 敏感性结果已保存：', Ron_file]);
disp(['T_ref 敏感性结果已保存：', Tref_file]);
disp(['敏感性详细结果已保存：', detail_file]);

%% =========================================================
% 7. 绘制敏感性分析图
% =========================================================

plot_sensitivity_figures( ...
    R_on_list, T_ref_list, data_Ron, data_Tref, sensitivity_folder, params_base);

disp(' ');
disp('==============================================================');
disp('事件触发参数敏感性分析全部完成');
disp('==============================================================');

%% =========================================================================
% 局部函数 1：单次敏感性仿真
% =========================================================================

function result = run_single_sensitivity_simulation( ...
    params, p_0, v_0, p_goal, path_dir, path_len, buildings, scenario_seed)

    rng(scenario_seed);

    x_curr = [p_0; v_0];

    obs_array = initialize_dynamic_obstacles_sens(params, p_0, p_goal);

    num_obs = length(obs_array);

    k = 0;
    k_last = -params.T_ref;
    sigma_k = 0;
    sigma_prev = 0;

    switch_count = 0;

    history_x = zeros(6, params.k_max);
    history_u = zeros(3, params.k_max);
    history_sigma = zeros(1, params.k_max);
    history_t_solve = zeros(1, params.k_max);
    history_obs = zeros(3, params.k_max, num_obs);

    collision_flag = false;
    success_flag = false;

    while norm(x_curr(1:3) - p_goal) > params.eps_tol && k < params.k_max

        k = k + 1;
        current_time = k * params.dt;

        history_x(:, k) = x_curr;

        % 更新动态障碍物
        obs_array = update_dynamic_obstacles_sens(obs_array, params, current_time);

        for i = 1:num_obs
            history_obs(:, k, i) = obs_array(i).p;
        end

        % 动态碰撞检测
        for i = 1:num_obs
            dist_to_obs = norm(x_curr(1:3) - obs_array(i).p);
            collision_dist = params.r_u + obs_array(i).r_i + params.collision_buffer;

            if dist_to_obs <= collision_dist
                collision_flag = true;
                break;
            end
        end

        if collision_flag
            break;
        end

        % 静态碰撞检测
        if check_static_collision_sens(x_curr, buildings, params)
            collision_flag = true;
            break;
        end

        % 构造当前参考状态
        s_curr = dot(x_curr(1:3) - p_0, path_dir);
        s_curr = max(0, min(s_curr, path_len));

        s_ref = min(s_curr + 2.0, path_len);
        x_ref_pos = p_0 + s_ref * path_dir;

        dist_to_goal = norm(p_goal - x_curr(1:3));

        if dist_to_goal > 2.0
            v_ref_nom = params.v_max * 0.8 * path_dir;
        else
            v_ref_nom = [0; 0; 0];
        end

        x_ref = [x_ref_pos; v_ref_nom];

        % 构造局部参考轨迹
        Pi_ref_local = zeros(3, params.H);

        for h = 1:params.H
            s_h = min(s_curr + h * 1.5 * (params.v_max * 0.8 * params.dt), path_len);
            Pi_ref_local(:, h) = p_0 + s_h * path_dir;
        end

        % 构造动态障碍物观测
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

        obs_pred = kalman_predict(obs_data, x_curr, params);

        % 计算风险
        [R_k, ~, ~, ~] = compute_risk(x_curr, x_ref, obs_pred, buildings, params);

        % 事件触发判断
        if params.use_event_trigger
            if (R_k >= params.R_on) && ((k - k_last) >= params.T_ref)
                sigma_k = 1;
            elseif R_k <= params.R_off
                sigma_k = 0;
            else
                sigma_k = sigma_prev;
            end
        else
            sigma_k = 1;
        end

        % 记录切换次数
        if sigma_k ~= sigma_prev
            switch_count = switch_count + 1;
        end

        % 控制计算
        t_start = tic;

        if sigma_k == 1
            [u_cmd, ~, ~] = solve_nmpc_casadi( ...
                x_curr, Pi_ref_local, obs_pred, buildings, params);
            k_last = k;
        else
            e_p = x_ref(1:3) - x_curr(1:3);
            e_v = x_ref(4:6) - x_curr(4:6);

            if dist_to_goal < 2.0
                u_cmd_raw = 2.0 * e_p + 3.0 * e_v;
            else
                u_cmd_raw = 1.5 * e_p + 2.5 * e_v;
            end

            u_cmd = limit_vector_norm_sens(u_cmd_raw, params.u_max);
        end

        history_t_solve(k) = toc(t_start);

        % 状态更新
        x_next = zeros(6, 1);
        x_next(1:3) = x_curr(1:3) + x_curr(4:6) * params.dt;
        x_next(4:6) = x_curr(4:6) + u_cmd * params.dt;

        x_next(4:6) = limit_vector_norm_sens(x_next(4:6), params.v_max);
        x_next(3) = max(min(x_next(3), params.z_max), params.z_min);

        history_u(:, k) = u_cmd;
        history_sigma(k) = sigma_k;

        x_curr = x_next;
        sigma_prev = sigma_k;
    end

    if ~collision_flag && norm(x_curr(1:3) - p_goal) <= params.eps_tol
        success_flag = true;
    end

    valid_k = max(1, k);

    result.success = success_flag;
    result.collision = collision_flag;
    result.steps = valid_k;
    result.switch_count = switch_count;

    result.trigger_rate = sum(history_sigma(1:valid_k)) / valid_k * 100;
    result.avg_cpu = mean(history_t_solve(1:valid_k)) * 1000;
    result.max_cpu = max(history_t_solve(1:valid_k)) * 1000;
    result.std_cpu = std(history_t_solve(1:valid_k)) * 1000;

    [result.min_dist_dyn, result.min_clearance_dyn] = compute_dynamic_distance_metrics_sens( ...
        history_x, history_obs, obs_array, valid_k, params);
end

%% =========================================================================
% 局部函数 2：初始化动态障碍物
% =========================================================================

function obs_array = initialize_dynamic_obstacles_sens(params, p_0, p_goal)

    num_obs = params.num_dynamic_obstacles;

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
        s_list = linspace(0.25, 0.78, num_obs);
    end

    for i = 1:num_obs

        center_on_path = p_0 + s_list(i) * path_vec;
        side_sign = (-1)^i;

        lateral_offset = side_sign * (5.0 + 2.0 * rand) * lateral_dir_1;
        vertical_offset = (rand - 0.5) * 3.5 * lateral_dir_2;

        p_init = center_on_path + lateral_offset + vertical_offset;
        p_init(3) = max(min(p_init(3), params.z_max - 3.0), params.z_min + 3.0);

        to_path = center_on_path - p_init;

        if norm(to_path) < 1e-6
            v_dir = -side_sign * lateral_dir_1;
        else
            v_dir = to_path / norm(to_path);
        end

        speed_i = 0.7 + 0.7 * rand;
        v_init = speed_i * v_dir;
        v_init(3) = 0.2 * (rand - 0.5);

        if i == 1
            p_init = [14; 30; 8] + [randn * 0.8; randn * 1.5; randn * 0.8];
            v_init = [0.8; -0.8; 0] + [randn * 0.15; randn * 0.15; randn * 0.05];
            r_i = params.large_obs_radius;
        else
            r_i = params.default_obs_radius * (0.85 + 0.3 * rand);
        end

        if params.use_heterogeneous_motion
            mode_list = params.heterogeneous_motion_types;
            motion_type = mode_list(mod(i - 1, length(mode_list)) + 1);
        else
            motion_type = params.default_motion_type;
        end

        if num_obs == 1
            motion_type = params.motion_type_CV;
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

        acc_dir = 0.7 * v_dir + 0.3 * side_sign * lateral_dir_1;

        if norm(acc_dir) < 1e-6
            acc_dir = v_dir;
        end

        obs_array(i).acc_dir = acc_dir / norm(acc_dir);
    end
end

%% =========================================================================
% 局部函数 3：更新动态障碍物
% =========================================================================

function obs_array = update_dynamic_obstacles_sens(obs_array, params, current_time)

    for i = 1:length(obs_array)

        motion_type = obs_array(i).motion_type;

        if motion_type == params.motion_type_CV

            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;

        elseif motion_type == params.motion_type_CA

            acc = params.obs_ca_acc * obs_array(i).acc_dir;

            obs_array(i).v = obs_array(i).v + acc * params.dt;
            obs_array(i).v = limit_vector_norm_sens(obs_array(i).v, params.obs_v_max);
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
            obs_array(i).v = limit_vector_norm_sens(obs_array(i).v, params.obs_v_max);

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

            obs_array(i).v = limit_vector_norm_sens(obs_array(i).v, params.obs_v_max);
            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;

        else

            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;
        end

        obs_array(i).p(3) = max(min(obs_array(i).p(3), params.z_max - 0.5), params.z_min + 0.5);
    end
end

%% =========================================================================
% 局部函数 4：动态距离指标
% =========================================================================

function [min_dist_dyn, min_clearance_dyn] = compute_dynamic_distance_metrics_sens( ...
    history_x, history_obs, obs_array, valid_k, params)

    num_obs = length(obs_array);

    min_dist_each = inf(1, num_obs);
    min_clearance_each = inf(1, num_obs);

    for i = 1:num_obs

        pos_uav = history_x(1:3, 1:valid_k);
        pos_obs = squeeze(history_obs(:, 1:valid_k, i));

        dist_i = sqrt(sum((pos_uav - pos_obs).^2, 1));

        min_dist_each(i) = min(dist_i);
        min_clearance_each(i) = min_dist_each(i) - params.r_u - obs_array(i).r_i;
    end

    min_dist_dyn = min(min_dist_each);
    min_clearance_dyn = min(min_clearance_each);
end

%% =========================================================================
% 局部函数 5：静态碰撞检测
% =========================================================================

function flag = check_static_collision_sens(x_curr, buildings, params)

    flag = false;
    p = x_curr(1:3);

    for i = 1:length(buildings)

        b = buildings(i);

        if b.type == 1
            eff_r = sqrt((b.w / 2)^2 + (b.l / 2)^2);
        elseif b.type == 2
            eff_r = b.r;
        elseif b.type == 3
            eff_r = b.r * max(0, 1 - p(3) / max(b.h, 1e-6));
        else
            eff_r = b.r;
        end

        dist_xy = norm(p(1:2) - [b.x; b.y]);

        if dist_xy <= eff_r + params.r_u && p(3) <= b.h + params.r_u
            flag = true;
            return;
        end
    end
end

%% =========================================================================
% 局部函数 6：绘图
% =========================================================================

function plot_sensitivity_figures(R_on_list, T_ref_list, data_Ron, data_Tref, folder, params)

    font_name = 'Times New Roman';
    font_size = 14;
    line_width = 2.5;
    marker_size = 9;

    if ~exist(folder, 'dir')
        mkdir(folder);
    end

    % =====================================================
    % 图 1：R_on 对 CPU 和安全间隔的影响
    % =====================================================
    fig1 = figure('Name', 'Sensitivity to R_on', 'Color', 'w', 'Position', [100, 100, 560, 430]);

    yyaxis left;
    p1 = plot(R_on_list, data_Ron(:, 2), '-s', ...
        'LineWidth', line_width, ...
        'MarkerSize', marker_size, ...
        'MarkerFaceColor', 'w');
    ylabel('Average CPU Time (ms)', 'FontName', font_name, 'FontSize', font_size);
    ax = gca;
    ax.YColor = 'k';

    yyaxis right;
    p2 = plot(R_on_list, data_Ron(:, 6), '-o', ...
        'LineWidth', line_width, ...
        'MarkerSize', marker_size, ...
        'MarkerFaceColor', 'w');
    ylabel('Minimum Clearance (m)', 'FontName', font_name, 'FontSize', font_size);
    ax = gca;
    ax.YColor = 'k';

    xlabel('Activation Threshold R_{on}', 'FontName', font_name, 'FontSize', font_size);
    title('Sensitivity to R_{on}', 'FontName', font_name, 'FontSize', font_size);

    set(gca, ...
        'FontName', font_name, ...
        'FontSize', font_size, ...
        'XTick', R_on_list, ...
        'XGrid', 'on', ...
        'YGrid', 'on', ...
        'GridAlpha', 0.18);

    legend([p1, p2], {'Avg. CPU Time', 'Min. Clearance'}, ...
        'Location', 'best', ...
        'FontName', font_name, ...
        'FontSize', 12, ...
        'Box', 'off');

    save_figure_sens(fig1, folder, 'Sensitivity_Ron_CPU_Clearance', params.figure_resolution);

    % =====================================================
    % 图 2：R_on 对触发率和成功率的影响
    % =====================================================
    fig2 = figure('Name', 'R_on Trigger Success', 'Color', 'w', 'Position', [700, 100, 560, 430]);

    yyaxis left;
    p1 = plot(R_on_list, data_Ron(:, 1), '-s', ...
        'LineWidth', line_width, ...
        'MarkerSize', marker_size, ...
        'MarkerFaceColor', 'w');
    ylabel('Trigger Rate (%)', 'FontName', font_name, 'FontSize', font_size);
    ax = gca;
    ax.YColor = 'k';

    yyaxis right;
    p2 = plot(R_on_list, data_Ron(:, 7), '-o', ...
        'LineWidth', line_width, ...
        'MarkerSize', marker_size, ...
        'MarkerFaceColor', 'w');
    ylabel('Success Rate (%)', 'FontName', font_name, 'FontSize', font_size);
    ylim([0, 105]);
    ax = gca;
    ax.YColor = 'k';

    xlabel('Activation Threshold R_{on}', 'FontName', font_name, 'FontSize', font_size);
    title('Trigger and Success Sensitivity to R_{on}', 'FontName', font_name, 'FontSize', font_size);

    set(gca, ...
        'FontName', font_name, ...
        'FontSize', font_size, ...
        'XTick', R_on_list, ...
        'XGrid', 'on', ...
        'YGrid', 'on', ...
        'GridAlpha', 0.18);

    legend([p1, p2], {'Trigger Rate', 'Success Rate'}, ...
        'Location', 'best', ...
        'FontName', font_name, ...
        'FontSize', 12, ...
        'Box', 'off');

    save_figure_sens(fig2, folder, 'Sensitivity_Ron_Trigger_Success', params.figure_resolution);

    % =====================================================
    % 图 3：T_ref 对 CPU 和安全间隔的影响
    % =====================================================
    fig3 = figure('Name', 'Sensitivity to T_ref', 'Color', 'w', 'Position', [100, 580, 560, 430]);

    yyaxis left;
    p1 = plot(T_ref_list, data_Tref(:, 2), '-s', ...
        'LineWidth', line_width, ...
        'MarkerSize', marker_size, ...
        'MarkerFaceColor', 'w');
    ylabel('Average CPU Time (ms)', 'FontName', font_name, 'FontSize', font_size);
    ax = gca;
    ax.YColor = 'k';

    yyaxis right;
    p2 = plot(T_ref_list, data_Tref(:, 6), '-o', ...
        'LineWidth', line_width, ...
        'MarkerSize', marker_size, ...
        'MarkerFaceColor', 'w');
    ylabel('Minimum Clearance (m)', 'FontName', font_name, 'FontSize', font_size);
    ax = gca;
    ax.YColor = 'k';

    xlabel('Refractory Interval T_{ref} (steps)', 'FontName', font_name, 'FontSize', font_size);
    title('Sensitivity to T_{ref}', 'FontName', font_name, 'FontSize', font_size);

    set(gca, ...
        'FontName', font_name, ...
        'FontSize', font_size, ...
        'XTick', T_ref_list, ...
        'XGrid', 'on', ...
        'YGrid', 'on', ...
        'GridAlpha', 0.18);

    legend([p1, p2], {'Avg. CPU Time', 'Min. Clearance'}, ...
        'Location', 'best', ...
        'FontName', font_name, ...
        'FontSize', 12, ...
        'Box', 'off');

    save_figure_sens(fig3, folder, 'Sensitivity_Tref_CPU_Clearance', params.figure_resolution);

    % =====================================================
    % 图 4：T_ref 对触发率和切换次数的影响
    % =====================================================
    fig4 = figure('Name', 'T_ref Trigger Switch', 'Color', 'w', 'Position', [700, 580, 560, 430]);

    yyaxis left;
    p1 = plot(T_ref_list, data_Tref(:, 1), '-s', ...
        'LineWidth', line_width, ...
        'MarkerSize', marker_size, ...
        'MarkerFaceColor', 'w');
    ylabel('Trigger Rate (%)', 'FontName', font_name, 'FontSize', font_size);
    ax = gca;
    ax.YColor = 'k';

    yyaxis right;
    p2 = plot(T_ref_list, data_Tref(:, 9), '-o', ...
        'LineWidth', line_width, ...
        'MarkerSize', marker_size, ...
        'MarkerFaceColor', 'w');
    ylabel('Switch Count', 'FontName', font_name, 'FontSize', font_size);
    ax = gca;
    ax.YColor = 'k';

    xlabel('Refractory Interval T_{ref} (steps)', 'FontName', font_name, 'FontSize', font_size);
    title('Trigger and Switching Sensitivity to T_{ref}', 'FontName', font_name, 'FontSize', font_size);

    set(gca, ...
        'FontName', font_name, ...
        'FontSize', font_size, ...
        'XTick', T_ref_list, ...
        'XGrid', 'on', ...
        'YGrid', 'on', ...
        'GridAlpha', 0.18);

    legend([p1, p2], {'Trigger Rate', 'Switch Count'}, ...
        'Location', 'best', ...
        'FontName', font_name, ...
        'FontSize', 12, ...
        'Box', 'off');

    save_figure_sens(fig4, folder, 'Sensitivity_Tref_Trigger_Switch', params.figure_resolution);
end

%% =========================================================================
% 局部函数 7：保存图像
% =========================================================================

function save_figure_sens(fig_handle, folder, file_name, resolution)

    png_file = fullfile(folder, [file_name, '.png']);
    pdf_file = fullfile(folder, [file_name, '.pdf']);

    exportgraphics(fig_handle, png_file, 'Resolution', resolution);
    exportgraphics(fig_handle, pdf_file, 'ContentType', 'vector');

    disp(['已保存图片：', png_file]);
    disp(['已保存矢量图：', pdf_file]);
end

%% =========================================================================
% 局部函数 8：向量范数限幅
% =========================================================================

function v_limited = limit_vector_norm_sens(v, max_norm)

    if norm(v) > max_norm
        v_limited = v / norm(v) * max_norm;
    else
        v_limited = v;
    end
end

%% =========================================================================
% 局部函数 9：补充缺失参数
% =========================================================================

function params = complete_params_for_sensitivity(params)

    if ~isfield(params, 'p_0')
        params.p_0 = [0; 0; 3];
    end

    if ~isfield(params, 'v_0')
        params.v_0 = [0; 0; 0];
    end

    if ~isfield(params, 'p_goal')
        params.p_goal = [40; 40; 12];
    end

    if ~isfield(params, 'num_dynamic_obstacles')
        params.num_dynamic_obstacles = 5;
    end

    if ~isfield(params, 'num_sensitivity_repeats')
        params.num_sensitivity_repeats = 25;
    end

    if ~isfield(params, 'use_multi_obstacles')
        params.use_multi_obstacles = true;
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

    if ~isfield(params, 'default_motion_type')
        params.default_motion_type = params.motion_type_CV;
    end

    if ~isfield(params, 'heterogeneous_motion_types')
        params.heterogeneous_motion_types = [
            params.motion_type_CV, ...
            params.motion_type_CA, ...
            params.motion_type_SIN, ...
            params.motion_type_RAND
        ];
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

    if ~isfield(params, 'output_folder')
        params.output_folder = 'results_reviewer_response';
    end

    if ~isfield(params, 'figure_resolution')
        params.figure_resolution = 600;
    end

    if ~isfield(params, 'random_seed') || isempty(params.random_seed)
        params.random_seed = 2026;
    end

    if ~exist(params.output_folder, 'dir')
        mkdir(params.output_folder);
    end
end