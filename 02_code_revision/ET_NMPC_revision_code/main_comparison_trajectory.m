% ==============================================================
% main_comparison_trajectory.m
%
% 六种算法在同一复杂城市动态障碍物场景下的轨迹叠加对比图
%
% 本文件作用：
% 1. 在同一张城市地图中运行六种算法；
% 2. 六种算法使用相同的起点、终点、静态建筑物和动态障碍物真实轨迹；
% 3. 分别记录六种算法的 UAV 轨迹；
% 4. 将六条轨迹绘制在同一张三维图中；
% 5. 保存轨迹叠加图、每种算法的数值指标和轨迹数据。
%
% 对比算法：
% 1. Proposed ET-NMPC
% 2. C-NMPC
% 3. NP-ET-NMPC
% 4. SE-ET-NMPC
% 5. 3D-APF
% 6. RRT*
%
% 说明：
% 1. 本文件只用于“单场景轨迹可视化对比”；
% 2. 多次重复统计对比仍然使用 main_comparison.m；
% 3. 本文件适合生成论文中的算法轨迹对比图；
% 4. 为保证公平性，本文件先预生成动态障碍物真实轨迹，
%    然后所有算法都使用同一组动态障碍物真实轨迹。
%
% 对应审稿意见：
% Reviewer 1 第 5 条：
%   增加 continuous NMPC、event-triggered MPC/NMPC、RRT* 等 baseline。
%
% Reviewer 2 第 4 条：
%   增加预测型或搜索型轨迹规划方法对比。
% ==============================================================

clear; clc; close all;

%% =========================================================
% 1. 初始化参数
% =========================================================

params_base = init_params();
params_base = complete_params_for_trajectory_comparison(params_base);

if isfield(params_base, 'random_seed') && ~isempty(params_base.random_seed)
    rng(params_base.random_seed);
end

% 起点、终点
p_0 = params_base.p_0;
v_0 = params_base.v_0;
p_goal = params_base.p_goal;

% 名义路径方向
path_dir = (p_goal - p_0) / norm(p_goal - p_0);
path_len = norm(p_goal - p_0);

% 生成静态城市环境
buildings = generate_city_map();

% 输出文件夹
trajectory_folder = fullfile(params_base.output_folder, 'comparison_trajectory');

if ~exist(trajectory_folder, 'dir')
    mkdir(trajectory_folder);
end

%% =========================================================
% 2. 设置对比算法
% =========================================================

algo_names = { ...
    'Proposed ET-NMPC', ...
    'C-NMPC', ...
    'NP-ET-NMPC', ...
    'SE-ET-NMPC', ...
    '3D-APF', ...
    'RRT*' ...
};

num_algos = length(algo_names);

% 使用一个固定场景种子，保证所有算法面对同一场景
scenario_seed = params_base.random_seed + 999;

% 预生成动态障碍物真实轨迹
% 这样可以避免 RRT* 随机采样改变随机数流，从而导致不同算法遇到不同动态障碍物轨迹。
[obs_truth, obs_init] = precompute_dynamic_obstacle_truth_trajectory( ...
    params_base, p_0, p_goal, scenario_seed);

disp(' ');
disp('==============================================================');
disp('开始执行六算法轨迹叠加对比实验');
fprintf('动态障碍物数量：%d\n', params_base.num_dynamic_obstacles);
fprintf('静态建筑物数量：%d\n', length(buildings));
fprintf('场景随机种子：%d\n', scenario_seed);
disp('对比算法：');
for i = 1:num_algos
    fprintf('  %d. %s\n', i, algo_names{i});
end
disp('==============================================================');

%% =========================================================
% 3. 逐个运行算法并记录轨迹
% =========================================================

results = struct([]);
all_traj_rows = {};
metric_rows = {};

for algo_idx = 1:num_algos

    algo_name = algo_names{algo_idx};

    fprintf('\n==============================================================\n');
    fprintf('当前运行算法：%s\n', algo_name);
    fprintf('==============================================================\n');

    params = params_base;

    % 对比实验中关闭动画，只保存最后叠加图
    params.enable_animation = false;

    % 根据算法名称设置算法模式
    params = configure_algorithm_mode_traj(params, algo_name);

    % 每个算法使用不同控制器随机种子，但使用相同真实动态障碍物轨迹
    controller_seed = scenario_seed + 10000 * algo_idx;

    result = run_single_trajectory_comparison_simulation( ...
        params, ...
        algo_name, ...
        p_0, ...
        v_0, ...
        p_goal, ...
        path_dir, ...
        path_len, ...
        buildings, ...
        obs_truth, ...
        obs_init, ...
        controller_seed);

    results(algo_idx).algorithm = algo_name;
    results(algo_idx).history_x = result.history_x;
    results(algo_idx).history_u = result.history_u;
    results(algo_idx).history_sigma = result.history_sigma;
    results(algo_idx).valid_k = result.steps;
    results(algo_idx).success = result.success;
    results(algo_idx).collision = result.collision;
    results(algo_idx).trigger_rate = result.trigger_rate;
    results(algo_idx).avg_cpu = result.avg_cpu;
    results(algo_idx).max_cpu = result.max_cpu;
    results(algo_idx).std_cpu = result.std_cpu;
    results(algo_idx).min_dist_dyn = result.min_dist_dyn;
    results(algo_idx).min_clearance_dyn = result.min_clearance_dyn;
    results(algo_idx).control_effort = result.control_effort;
    results(algo_idx).total_climb = result.total_climb;

    % 保存指标行
    metric_rows = [metric_rows; { ...
        algo_name, ...
        params_base.num_dynamic_obstacles, ...
        length(buildings), ...
        result.success, ...
        result.collision, ...
        result.trigger_rate, ...
        result.avg_cpu, ...
        result.max_cpu, ...
        result.std_cpu, ...
        result.min_dist_dyn, ...
        result.min_clearance_dyn, ...
        result.control_effort, ...
        result.total_climb, ...
        result.steps ...
    }]; %#ok<AGROW>

    % 保存轨迹行
    for k = 1:result.steps
        all_traj_rows = [all_traj_rows; { ...
            algo_name, ...
            k, ...
            result.history_x(1, k), ...
            result.history_x(2, k), ...
            result.history_x(3, k), ...
            result.history_x(4, k), ...
            result.history_x(5, k), ...
            result.history_x(6, k), ...
            result.history_sigma(k) ...
        }]; %#ok<AGROW>
    end

    fprintf('算法 %s 运行结束：success=%d, collision=%d, steps=%d, clearance=%.2f m, avgCPU=%.2f ms\n', ...
        algo_name, ...
        result.success, ...
        result.collision, ...
        result.steps, ...
        result.min_clearance_dyn, ...
        result.avg_cpu);
end

%% =========================================================
% 4. 绘制六算法轨迹叠加图
% =========================================================

plot_trajectory_overlay_results( ...
    results, ...
    algo_names, ...
    buildings, ...
    obs_truth, ...
    p_0, ...
    p_goal, ...
    params_base, ...
    trajectory_folder);

%% =========================================================
% 5. 保存结果 CSV 和 MAT 文件
% =========================================================

metric_table = cell2table(metric_rows, ...
    'VariableNames', { ...
    'Algorithm', ...
    'NumDynamicObstacles', ...
    'NumStaticBuildings', ...
    'Success', ...
    'Collision', ...
    'TriggerRatePercent', ...
    'AvgCPUms', ...
    'MaxCPUms', ...
    'StdCPUms', ...
    'MinDistanceMeter', ...
    'MinClearanceMeter', ...
    'ControlEffort', ...
    'TotalClimbMeter', ...
    'Steps'});

metric_file = fullfile(trajectory_folder, 'trajectory_comparison_metrics.csv');
writetable(metric_table, metric_file);

traj_table = cell2table(all_traj_rows, ...
    'VariableNames', { ...
    'Algorithm', ...
    'Step', ...
    'px', ...
    'py', ...
    'pz', ...
    'vx', ...
    'vy', ...
    'vz', ...
    'TriggerSignal'});

traj_file = fullfile(trajectory_folder, 'trajectory_comparison_paths.csv');
writetable(traj_table, traj_file);

mat_file = fullfile(trajectory_folder, 'trajectory_comparison_workspace.mat');
save(mat_file, 'results', 'obs_truth', 'obs_init', 'buildings', 'params_base', 'metric_table', 'traj_table');

disp(' ');
disp('==============================================================');
disp('六算法轨迹叠加对比实验完成');
disp(['指标结果已保存：', metric_file]);
disp(['轨迹数据已保存：', traj_file]);
disp(['MAT 数据已保存：', mat_file]);
disp('==============================================================');

%% =========================================================================
% 局部函数 1：预生成动态障碍物真实轨迹
% =========================================================================

function [obs_truth, obs_init] = precompute_dynamic_obstacle_truth_trajectory(params, p_0, p_goal, scenario_seed)

    rng(scenario_seed);

    obs_array = initialize_dynamic_obstacles_traj(params, p_0, p_goal);
    obs_init = obs_array;

    num_obs = length(obs_array);

    obs_truth = struct();
    obs_truth.num_obs = num_obs;
    obs_truth.k_max = params.k_max;
    obs_truth.p = zeros(3, params.k_max, num_obs);
    obs_truth.v = zeros(3, params.k_max, num_obs);
    obs_truth.r = zeros(1, num_obs);
    obs_truth.motion_type = zeros(1, num_obs);
    obs_truth.cov = cell(1, num_obs);

    for i = 1:num_obs
        obs_truth.r(i) = obs_array(i).r_i;
        obs_truth.motion_type(i) = obs_array(i).motion_type;
        obs_truth.cov{i} = obs_array(i).cov;
    end

    for k = 1:params.k_max

        current_time = k * params.dt;

        obs_array = update_dynamic_obstacles_traj(obs_array, params, current_time);

        for i = 1:num_obs
            obs_truth.p(:, k, i) = obs_array(i).p;
            obs_truth.v(:, k, i) = obs_array(i).v;
        end
    end
end

%% =========================================================================
% 局部函数 2：单个算法轨迹仿真
% =========================================================================

function result = run_single_trajectory_comparison_simulation( ...
    params, ...
    algo_name, ...
    p_0, ...
    v_0, ...
    p_goal, ...
    path_dir, ...
    path_len, ...
    buildings, ...
    obs_truth, ...
    obs_init, ...
    controller_seed)

    rng(controller_seed);

    x_curr = [p_0; v_0];

    num_obs = obs_truth.num_obs;

    k = 0;
    k_last = -params.T_ref;
    sigma_k = 0;
    sigma_prev = 0;

    collision_flag = false;
    success_flag = false;

    history_x = zeros(6, params.k_max);
    history_u = zeros(3, params.k_max);
    history_sigma = zeros(1, params.k_max);
    history_t_solve = zeros(1, params.k_max);
    history_obs = zeros(3, params.k_max, num_obs);

    rrt_path = [];
    rrt_replan_counter = inf;

    while norm(x_curr(1:3) - p_goal) > params.eps_tol && k < params.k_max

        k = k + 1;

        % 读取当前时刻预生成的动态障碍物真实状态
        obs_array = get_obstacle_array_from_truth(obs_truth, obs_init, k);

        history_x(:, k) = x_curr;

        for i = 1:num_obs
            history_obs(:, k, i) = obs_array(i).p;
        end

        % 动态碰撞检测
        if params.check_dynamic_collision
            for i = 1:num_obs
                dist_to_obs = norm(x_curr(1:3) - obs_array(i).p);
                collision_dist = params.r_u + obs_array(i).r_i + params.collision_buffer;

                if dist_to_obs <= collision_dist
                    collision_flag = true;
                    break;
                end
            end
        end

        if collision_flag
            break;
        end

        % 静态碰撞检测
        if params.check_static_collision
            if check_static_collision_traj(x_curr, buildings, params)
                collision_flag = true;
                break;
            end
        end

        % 构造局部参考轨迹
        [x_ref, Pi_ref_local, dist_to_goal, use_direct_goal_guidance] = build_local_reference_traj( ...
            x_curr, ...
            p_0, ...
            p_goal, ...
            path_dir, ...
            path_len, ...
            buildings, ...
            obs_array, ...
            params);

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

        if ~params.use_prediction
            for i = 1:length(obs_pred)
                obs_pred(i).pos = repmat(obs_pred(i).pos(:, 1), 1, params.H);
                obs_pred(i).vel = zeros(3, params.H);
                obs_pred(i).inflation = zeros(1, params.H);
            end
        end

        % 控制计算
        t_start = tic;

        if strcmp(params.controller_type, 'NMPC')

            [R_k, ~, ~, ~] = compute_risk(x_curr, x_ref, obs_pred, buildings, params);

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

            if sigma_k == 1

                [u_cmd, ~, ~] = solve_nmpc_casadi( ...
                    x_curr, Pi_ref_local, obs_pred, buildings, params);

                k_last = k;

            else

                u_cmd = nominal_tracking_control_traj( ...
                    x_curr, x_ref, dist_to_goal, use_direct_goal_guidance, params);
            end

        elseif strcmp(params.controller_type, 'APF')

            sigma_k = 0;

            u_cmd = apf_controller_3d_traj( ...
                x_curr, p_goal, obs_array, buildings, params);

        elseif strcmp(params.controller_type, 'RRTSTAR')

            sigma_k = 0;

            if isempty(rrt_path) || rrt_replan_counter >= params.rrt_replan_interval
                rrt_path = rrtstar_plan_traj(x_curr(1:3), p_goal, obs_array, buildings, params);
                rrt_replan_counter = 0;
            end

            rrt_replan_counter = rrt_replan_counter + 1;

            u_cmd = rrtstar_tracking_control_traj(x_curr, rrt_path, p_goal, params);

        else

            error('未知 controller_type：%s', params.controller_type);
        end

        history_t_solve(k) = toc(t_start);

        % 状态更新
        x_next = zeros(6, 1);
        x_next(1:3) = x_curr(1:3) + x_curr(4:6) * params.dt;
        x_next(4:6) = x_curr(4:6) + u_cmd * params.dt;

        x_next(4:6) = limit_vector_norm_traj(x_next(4:6), params.v_max);
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

    [min_dist_dyn, min_clearance_dyn] = compute_dynamic_distance_metrics_traj( ...
        history_x, history_obs, obs_truth, valid_k, params);

    delta_z = diff(history_x(3, 1:valid_k));
    total_climb = sum(max(0, delta_z));

    control_effort = sum(sum(history_u(:, 1:valid_k).^2)) * params.dt;

    result.algorithm = algo_name;
    result.success = success_flag;
    result.collision = collision_flag;
    result.trigger_rate = sum(history_sigma(1:valid_k)) / valid_k * 100;
    result.avg_cpu = mean(history_t_solve(1:valid_k)) * 1000;
    result.max_cpu = max(history_t_solve(1:valid_k)) * 1000;
    result.std_cpu = std(history_t_solve(1:valid_k)) * 1000;
    result.min_dist_dyn = min_dist_dyn;
    result.min_clearance_dyn = min_clearance_dyn;
    result.control_effort = control_effort;
    result.total_climb = total_climb;
    result.steps = valid_k;
    result.history_x = history_x;
    result.history_u = history_u;
    result.history_sigma = history_sigma;
end

%% =========================================================================
% 局部函数 3：根据预生成轨迹读取当前障碍物状态
% =========================================================================

function obs_array = get_obstacle_array_from_truth(obs_truth, obs_init, k)

    num_obs = obs_truth.num_obs;

    obs_array = obs_init;

    k = min(k, obs_truth.k_max);

    for i = 1:num_obs
        obs_array(i).p = obs_truth.p(:, k, i);
        obs_array(i).v = obs_truth.v(:, k, i);
        obs_array(i).r_i = obs_truth.r(i);
        obs_array(i).motion_type = obs_truth.motion_type(i);
        obs_array(i).cov = obs_truth.cov{i};
    end
end

%% =========================================================================
% 局部函数 4：构造局部参考轨迹
% =========================================================================

function [x_ref, Pi_ref_local, dist_to_goal, use_direct_goal_guidance] = build_local_reference_traj( ...
    x_curr, ...
    p_0, ...
    p_goal, ...
    path_dir, ...
    path_len, ...
    buildings, ...
    obs_array, ...
    params)

    dist_to_goal = norm(p_goal - x_curr(1:3));

    goal_dir = p_goal - x_curr(1:3);

    if norm(goal_dir) > 1e-6
        goal_dir_unit = goal_dir / norm(goal_dir);
    else
        goal_dir_unit = zeros(3, 1);
    end

    line_to_goal_clear = check_line_to_goal_clear_traj( ...
        x_curr(1:3), p_goal, buildings, obs_array, params);

    use_direct_goal_guidance = ...
        (dist_to_goal <= params.goal_direct_radius) || ...
        (line_to_goal_clear && dist_to_goal <= params.goal_line_of_sight_radius);

    if use_direct_goal_guidance

        x_ref_pos = p_goal;

        if dist_to_goal > params.goal_slow_radius
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
end

%% =========================================================================
% 局部函数 5：算法模式设置
% =========================================================================

function params = configure_algorithm_mode_traj(params, algo_name)

    if strcmp(algo_name, 'Proposed ET-NMPC')

        params.use_event_trigger = true;
        params.use_prediction = true;
        params.use_asymmetric_energy = true;
        params.controller_type = 'NMPC';

    elseif strcmp(algo_name, 'C-NMPC')

        params.use_event_trigger = false;
        params.use_prediction = true;
        params.use_asymmetric_energy = true;
        params.controller_type = 'NMPC';

    elseif strcmp(algo_name, 'NP-ET-NMPC')

        params.use_event_trigger = true;
        params.use_prediction = false;
        params.use_asymmetric_energy = true;
        params.controller_type = 'NMPC';

    elseif strcmp(algo_name, 'SE-ET-NMPC')

        params.use_event_trigger = true;
        params.use_prediction = true;
        params.use_asymmetric_energy = false;
        params.c3 = 0.0;
        params.controller_type = 'NMPC';

    elseif strcmp(algo_name, '3D-APF')

        params.use_event_trigger = false;
        params.use_prediction = false;
        params.use_asymmetric_energy = false;
        params.controller_type = 'APF';

    elseif strcmp(algo_name, 'RRT*')

        params.use_event_trigger = false;
        params.use_prediction = false;
        params.use_asymmetric_energy = false;
        params.controller_type = 'RRTSTAR';

    else

        error('未知算法名称：%s', algo_name);
    end
end

%% =========================================================================
% 局部函数 6：初始化动态障碍物
% =========================================================================

function obs_array = initialize_dynamic_obstacles_traj(params, p_0, p_goal)

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
% 局部函数 7：更新动态障碍物
% =========================================================================

function obs_array = update_dynamic_obstacles_traj(obs_array, params, current_time)

    for i = 1:length(obs_array)

        motion_type = obs_array(i).motion_type;

        if motion_type == params.motion_type_CV

            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;

        elseif motion_type == params.motion_type_CA

            acc = params.obs_ca_acc * obs_array(i).acc_dir;
            obs_array(i).v = obs_array(i).v + acc * params.dt;
            obs_array(i).v = limit_vector_norm_traj(obs_array(i).v, params.obs_v_max);
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
            obs_array(i).v = limit_vector_norm_traj(obs_array(i).v, params.obs_v_max);

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

            obs_array(i).v = limit_vector_norm_traj(obs_array(i).v, params.obs_v_max);
            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;

        else

            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;
        end

        obs_array(i).p(3) = max(min(obs_array(i).p(3), params.z_max - 0.5), params.z_min + 0.5);
    end
end

%% =========================================================================
% 局部函数 8：名义跟踪控制
% =========================================================================

function u_cmd = nominal_tracking_control_traj(x_curr, x_ref, dist_to_goal, use_direct_goal_guidance, params)

    e_p = x_ref(1:3) - x_curr(1:3);
    e_v = x_ref(4:6) - x_curr(4:6);

    if use_direct_goal_guidance
        u_cmd_raw = 2.2 * e_p + 3.2 * e_v;
    elseif dist_to_goal < params.goal_slow_radius
        u_cmd_raw = 2.0 * e_p + 3.0 * e_v;
    else
        u_cmd_raw = 1.5 * e_p + 2.5 * e_v;
    end

    u_cmd = limit_vector_norm_traj(u_cmd_raw, params.u_max);
end

%% =========================================================================
% 局部函数 9：3D-APF 控制器
% =========================================================================

function u_cmd = apf_controller_3d_traj(x_curr, p_goal, obs_array, buildings, params)

    p = x_curr(1:3);
    v = x_curr(4:6);

    F_att = params.apf_k_att * (p_goal - p);

    F_rep_static = zeros(3, 1);

    for i = 1:length(buildings)

        b = buildings(i);

        if p(3) < b.h + 1.0

            if b.type == 1

                closest_x = min(max(p(1), b.x - b.w/2), b.x + b.w/2);
                closest_y = min(max(p(2), b.y - b.l/2), b.y + b.l/2);

                vec_xy = p(1:2) - [closest_x; closest_y];
                dist_xy = norm(vec_xy) - params.r_u;

            elseif b.type == 2

                vec_xy = p(1:2) - [b.x; b.y];
                dist_xy = norm(vec_xy) - b.r - params.r_u;

            elseif b.type == 3

                cone_scale = max(0, 1 - p(3) / max(b.h, 1e-6));
                cone_radius = b.r * cone_scale;

                vec_xy = p(1:2) - [b.x; b.y];
                dist_xy = norm(vec_xy) - cone_radius - params.r_u;

            else

                vec_xy = p(1:2) - [b.x; b.y];
                dist_xy = norm(vec_xy) - params.r_u;
            end

            if norm(vec_xy) > 1e-6
                dir_xy = vec_xy / norm(vec_xy);
            else
                dir_xy = [0; 0];
            end

            if dist_xy > 0.1 && dist_xy < params.apf_rho_0
                rep_mag = params.apf_k_rep * (1 / dist_xy - 1 / params.apf_rho_0) * (1 / dist_xy^2);
                F_rep_static(1:2) = F_rep_static(1:2) + rep_mag * dir_xy;
            elseif dist_xy <= 0.1
                F_rep_static(3) = F_rep_static(3) + params.u_max;
            end
        end
    end

    F_rep_dyn = zeros(3, 1);

    for i = 1:length(obs_array)

        vec = p - obs_array(i).p;
        dist = norm(vec) - obs_array(i).r_i - params.r_u;

        rho_dyn = params.apf_dynamic_influence_radius;

        if dist > 0.1 && dist < rho_dyn
            rep_mag = params.apf_k_rep_dynamic * (1 / dist - 1 / rho_dyn) * (1 / dist^2);
            F_rep_dyn = F_rep_dyn + rep_mag * (vec / norm(vec));
        elseif dist <= 0.1
            F_rep_dyn = F_rep_dyn + [0; 0; params.u_max];
        end
    end

    F_damp = -params.apf_damping * v;

    F_total = F_att + F_rep_static + F_rep_dyn + F_damp;

    u_cmd = limit_vector_norm_traj(F_total, params.u_max);
end

%% =========================================================================
% 局部函数 10：RRT* 路径规划器
% =========================================================================

function path = rrtstar_plan_traj(p_start, p_goal, obs_array, buildings, params)

    if is_segment_collision_free_traj(p_start, p_goal, obs_array, buildings, params)
        path = [p_start, p_goal];
        return;
    end

    max_iter = params.rrt_max_iter;
    step_size = params.rrt_step_size;
    goal_sample_rate = params.rrt_goal_sample_rate;
    goal_radius = params.rrt_goal_radius;
    search_radius = params.rrt_rewire_radius;

    nodes_pos = zeros(3, max_iter + 1);
    nodes_parent = zeros(1, max_iter + 1);
    nodes_cost = inf(1, max_iter + 1);

    nodes_pos(:, 1) = p_start;
    nodes_parent(1) = 0;
    nodes_cost(1) = 0;

    node_count = 1;
    goal_node = -1;

    for iter = 1:max_iter

        if rand < goal_sample_rate
            p_rand = p_goal;
        else
            p_rand = [
                rand_range_traj(params.x_lim(1), params.x_lim(2));
                rand_range_traj(params.y_lim(1), params.y_lim(2));
                rand_range_traj(params.z_min + 0.5, params.z_max - 0.5)
            ];
        end

        dist_all = sqrt(sum((nodes_pos(:, 1:node_count) - p_rand).^2, 1));
        [~, nearest_idx] = min(dist_all);

        p_nearest = nodes_pos(:, nearest_idx);

        direction = p_rand - p_nearest;

        if norm(direction) < 1e-6
            continue;
        end

        direction = direction / norm(direction);
        p_new = p_nearest + step_size * direction;

        p_new(1) = max(min(p_new(1), params.x_lim(2)), params.x_lim(1));
        p_new(2) = max(min(p_new(2), params.y_lim(2)), params.y_lim(1));
        p_new(3) = max(min(p_new(3), params.z_max), params.z_min);

        if ~is_segment_collision_free_traj(p_nearest, p_new, obs_array, buildings, params)
            continue;
        end

        dist_to_new = sqrt(sum((nodes_pos(:, 1:node_count) - p_new).^2, 1));
        near_indices = find(dist_to_new <= search_radius);

        best_parent = nearest_idx;
        best_cost = nodes_cost(nearest_idx) + norm(p_new - p_nearest);

        for j = 1:length(near_indices)

            idx = near_indices(j);
            p_near = nodes_pos(:, idx);
            candidate_cost = nodes_cost(idx) + norm(p_new - p_near);

            if candidate_cost < best_cost
                if is_segment_collision_free_traj(p_near, p_new, obs_array, buildings, params)
                    best_parent = idx;
                    best_cost = candidate_cost;
                end
            end
        end

        node_count = node_count + 1;
        nodes_pos(:, node_count) = p_new;
        nodes_parent(node_count) = best_parent;
        nodes_cost(node_count) = best_cost;

        new_idx = node_count;

        for j = 1:length(near_indices)

            idx = near_indices(j);

            if idx == best_parent
                continue;
            end

            p_near = nodes_pos(:, idx);
            candidate_cost = nodes_cost(new_idx) + norm(p_near - p_new);

            if candidate_cost < nodes_cost(idx)
                if is_segment_collision_free_traj(p_new, p_near, obs_array, buildings, params)
                    nodes_parent(idx) = new_idx;
                    nodes_cost(idx) = candidate_cost;
                end
            end
        end

        if norm(p_new - p_goal) < goal_radius
            if is_segment_collision_free_traj(p_new, p_goal, obs_array, buildings, params)

                node_count = node_count + 1;
                nodes_pos(:, node_count) = p_goal;
                nodes_parent(node_count) = new_idx;
                nodes_cost(node_count) = nodes_cost(new_idx) + norm(p_goal - p_new);

                goal_node = node_count;
                break;
            end
        end
    end

    if goal_node == -1
        dist_to_goal = sqrt(sum((nodes_pos(:, 1:node_count) - p_goal).^2, 1));
        [~, goal_node] = min(dist_to_goal);
    end

    path_rev = [];
    idx = goal_node;

    while idx > 0
        path_rev = [path_rev, nodes_pos(:, idx)]; %#ok<AGROW>
        idx = nodes_parent(idx);
    end

    path = fliplr(path_rev);

    path = shortcut_path_traj(path, obs_array, buildings, params);
end

%% =========================================================================
% 局部函数 11：RRT* 跟踪控制
% =========================================================================

function u_cmd = rrtstar_tracking_control_traj(x_curr, path, p_goal, params)

    p = x_curr(1:3);
    v = x_curr(4:6);

    if isempty(path)
        p_target = p_goal;
    elseif size(path, 2) == 1
        p_target = path(:, 1);
    else
        dist_to_nodes = sqrt(sum((path - p).^2, 1));
        [~, nearest_idx] = min(dist_to_nodes);

        target_idx = min(nearest_idx + params.rrt_lookahead_index, size(path, 2));
        p_target = path(:, target_idx);
    end

    if norm(p_goal - p) <= params.goal_direct_radius
        p_target = p_goal;
    end

    e_p = p_target - p;
    e_v = -v;

    u_raw = params.rrt_kp * e_p + params.rrt_kv * e_v;

    u_cmd = limit_vector_norm_traj(u_raw, params.u_max);
end

%% =========================================================================
% 局部函数 12：线段碰撞检测
% =========================================================================

function flag = is_segment_collision_free_traj(p1, p2, obs_array, buildings, params)

    flag = true;

    seg_len = norm(p2 - p1);
    n_check = max(2, ceil(seg_len / params.rrt_collision_check_resolution));

    for i = 0:n_check

        alpha = i / n_check;
        p = (1 - alpha) * p1 + alpha * p2;

        if p(1) < params.x_lim(1) || p(1) > params.x_lim(2) || ...
           p(2) < params.y_lim(1) || p(2) > params.y_lim(2) || ...
           p(3) < params.z_min || p(3) > params.z_max
            flag = false;
            return;
        end

        x_tmp = [p; 0; 0; 0];

        if check_static_collision_traj(x_tmp, buildings, params)
            flag = false;
            return;
        end

        for j = 1:length(obs_array)

            dist_dyn = norm(p - obs_array(j).p);
            safe_dist = params.r_u + obs_array(j).r_i + params.d_m + params.rrt_dynamic_buffer;

            if dist_dyn <= safe_dist
                flag = false;
                return;
            end
        end
    end
end

%% =========================================================================
% 局部函数 13：路径快捷平滑
% =========================================================================

function path_smooth = shortcut_path_traj(path, obs_array, buildings, params)

    if size(path, 2) <= 2
        path_smooth = path;
        return;
    end

    path_smooth = path;
    max_shortcut_iter = 30;

    for iter = 1:max_shortcut_iter

        n = size(path_smooth, 2);

        if n <= 2
            break;
        end

        i = randi([1, n - 1]);
        j = randi([i + 1, n]);

        if j <= i + 1
            continue;
        end

        if is_segment_collision_free_traj(path_smooth(:, i), path_smooth(:, j), obs_array, buildings, params)
            path_smooth = [path_smooth(:, 1:i), path_smooth(:, j:end)];
        end
    end
end

%% =========================================================================
% 局部函数 14：终点直线路径可见性检测
% =========================================================================

function flag = check_line_to_goal_clear_traj(p_start, p_goal, buildings, obs_array, params)

    flag = true;

    segment_len = norm(p_goal - p_start);

    if segment_len < 1e-6
        return;
    end

    n_check = max(3, ceil(segment_len / params.goal_line_check_resolution));

    for ii = 0:n_check

        alpha = ii / n_check;
        p = (1 - alpha) * p_start + alpha * p_goal;

        if is_point_inside_static_safety_zone_traj(p, buildings, params)
            flag = false;
            return;
        end

        if is_point_inside_dynamic_safety_zone_traj(p, obs_array, params)
            flag = false;
            return;
        end
    end
end

%% =========================================================================
% 局部函数 15：静态安全区检测
% =========================================================================

function flag = is_point_inside_static_safety_zone_traj(p, buildings, params)

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

%% =========================================================================
% 局部函数 16：动态安全区检测
% =========================================================================

function flag = is_point_inside_dynamic_safety_zone_traj(p, obs_array, params)

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

%% =========================================================================
% 局部函数 17：静态碰撞检测
% =========================================================================

function flag = check_static_collision_traj(x_curr, buildings, params)

    flag = false;
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
            horizontal_threshold = params.r_u + params.collision_buffer;

            if dist_to_rect <= horizontal_threshold
                flag = true;
                return;
            end

        elseif b.type == 2

            dist_xy = norm(p(1:2) - [b.x; b.y]);
            horizontal_threshold = b.r + params.r_u + params.collision_buffer;

            if dist_xy <= horizontal_threshold
                flag = true;
                return;
            end

        elseif b.type == 3

            if p(3) <= b.h + params.r_u

                cone_scale = max(0, 1 - p(3) / max(b.h, 1e-6));
                cone_radius_at_z = b.r * cone_scale;

                dist_xy = norm(p(1:2) - [b.x; b.y]);
                horizontal_threshold = cone_radius_at_z + params.r_u + params.collision_buffer;

                if dist_xy <= horizontal_threshold
                    flag = true;
                    return;
                end
            end
        end
    end
end

%% =========================================================================
% 局部函数 18：动态距离指标
% =========================================================================

function [min_dist_dyn, min_clearance_dyn] = compute_dynamic_distance_metrics_traj( ...
    history_x, history_obs, obs_truth, valid_k, params)

    num_obs = obs_truth.num_obs;

    min_dist_each = inf(1, num_obs);
    min_clearance_each = inf(1, num_obs);

    for i = 1:num_obs

        pos_uav = history_x(1:3, 1:valid_k);
        pos_obs = squeeze(history_obs(:, 1:valid_k, i));

        dist_i = sqrt(sum((pos_uav - pos_obs).^2, 1));

        min_dist_each(i) = min(dist_i);
        min_clearance_each(i) = min_dist_each(i) - params.r_u - obs_truth.r(i);
    end

    min_dist_dyn = min(min_dist_each);
    min_clearance_dyn = min(min_clearance_each);
end

%% =========================================================================
% 局部函数 19：绘制六算法轨迹叠加图
% =========================================================================

function plot_trajectory_overlay_results( ...
    results, ...
    algo_names, ...
    buildings, ...
    obs_truth, ...
    p_0, ...
    p_goal, ...
    params, ...
    trajectory_folder)

    fig = figure( ...
        'Name', 'Trajectory Comparison Overlay', ...
        'Color', 'w', ...
        'Position', [50, 50, 1300, 900]); % 更大

    hold on; grid on;

    % 绘制静态建筑物
    draw_buildings_traj(buildings);

    % 绘制名义直线路径
    plot3( ...
        [p_0(1), p_goal(1)], ...
        [p_0(2), p_goal(2)], ...
        [p_0(3), p_goal(3)], ...
        'k--', ...
        'LineWidth', 1.5, ...
        'DisplayName', 'Nominal Path');

    % 绘制动态障碍物真实轨迹
    for i = 1:obs_truth.num_obs

        obs_x = squeeze(obs_truth.p(1, :, i));
        obs_y = squeeze(obs_truth.p(2, :, i));
        obs_z = squeeze(obs_truth.p(3, :, i));

        valid_idx = 1:min(params.k_max, length(obs_x));

        plot3( ...
            obs_x(valid_idx), ...
            obs_y(valid_idx), ...
            obs_z(valid_idx), ...
            ':', ...
            'Color', [0.25, 0.25, 0.25], ...
            'LineWidth', 1.2, ...
            'HandleVisibility', 'off');

        plot3( ...
            obs_x(1), ...
            obs_y(1), ...
            obs_z(1), ...
            'ko', ...
            'MarkerSize', 5, ...
            'MarkerFaceColor', [0.25, 0.25, 0.25], ...
            'HandleVisibility', 'off');
    end

    % 只添加一次动态障碍物图例
    plot3(nan, nan, nan, ':', ...
        'Color', [0.25, 0.25, 0.25], ...
        'LineWidth', 1.2, ...
        'DisplayName', 'Dynamic Obstacle Trajectories');

    % 绘制起点和终点
    plot3( ...
        p_0(1), p_0(2), p_0(3), ...
        'bo', ...
        'MarkerSize', 8, ...
        'MarkerFaceColor', 'b', ...
        'DisplayName', 'Start');

    plot3( ...
        p_goal(1), p_goal(2), p_goal(3), ...
        'g*', ...
        'MarkerSize', 12, ...
        'LineWidth', 2.0, ...
        'DisplayName', 'Goal');

    % 轨迹颜色和线型
    color_list = [
        0.00, 0.20, 0.90;
        0.85, 0.10, 0.10;
        0.10, 0.60, 0.20;
        0.70, 0.20, 0.85;
        0.95, 0.55, 0.05;
        0.10, 0.10, 0.10
    ];

    line_style_list = {'-', '-', '-', '-', '-', '-'};

    for a = 1:length(results)

        valid_k = results(a).valid_k;
        traj = results(a).history_x(:, 1:valid_k);

        label = algo_names{a};

        if results(a).collision
            label = [label, ' (collision)'];
        elseif ~results(a).success
            label = [label, ' (failed)'];
        end

        plot3( ...
            traj(1, :), ...
            traj(2, :), ...
            traj(3, :), ...
            line_style_list{a}, ...
            'Color', color_list(a, :), ...
            'LineWidth', 2.4, ...
            'DisplayName', label);

        % 标记终止点
        plot3( ...
            traj(1, end), ...
            traj(2, end), ...
            traj(3, end), ...
            'o', ...
            'Color', color_list(a, :), ...
            'MarkerFaceColor', color_list(a, :), ...
            'MarkerSize', 5, ...
            'HandleVisibility', 'off');
         % 如果该算法发生碰撞，则在碰撞位置标记红色 ×
        if results(a).collision
            plot3( ...
                traj(1, end), ...
                traj(2, end), ...
                traj(3, end), ...
                'x', ...
                'Color', [1, 0, 0], ...
                'MarkerSize', 14, ...
                'LineWidth', 2.5, ...
                'HandleVisibility', 'off');
        end
    end

    xlabel('X [m]', 'FontSize', 14);
    ylabel('Y [m]', 'FontSize', 14);
    zlabel('Z [m]', 'FontSize', 14);

    set(gca, 'FontSize', 13);
    daspect([1 1 1]);

    xlim(params.x_lim);
    ylim(params.y_lim);
    zlim(params.z_lim);

    view([-35, 42]);
    camlight;
    lighting gouraud;

    legend('show', 'Location', 'northeast', 'FontSize', 12);

    png_file = fullfile(trajectory_folder, 'Comparison_Trajectory_Overlay.png');
    pdf_file = fullfile(trajectory_folder, 'Comparison_Trajectory_Overlay.pdf');
    fig_file = fullfile(trajectory_folder, 'Comparison_Trajectory_Overlay.fig');

    exportgraphics(fig, png_file, 'Resolution', params.figure_resolution);
    exportgraphics(fig, pdf_file, 'ContentType', 'vector');
    savefig(fig, fig_file);

    disp(' ');
    disp(['轨迹叠加图 PNG 已保存：', png_file]);
    disp(['轨迹叠加图 PDF 已保存：', pdf_file]);
    disp(['轨迹叠加图 FIG 已保存：', fig_file]);
end

%% =========================================================================
% 局部函数 20：绘制静态建筑物
% =========================================================================

function draw_buildings_traj(buildings)

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
                'FaceColor', [0.55, 0.65, 0.75], ...
                'FaceAlpha', 0.55, ...
                'EdgeColor', 'none', ...
                'HandleVisibility', 'off');

        elseif b.type == 2

            [xc, yc, zc] = cylinder(b.r, 30);

            surf( ...
                xc + b.x, ...
                yc + b.y, ...
                zc * b.h, ...
                'FaceColor', [0.75, 0.50, 0.50], ...
                'EdgeColor', 'none', ...
                'FaceAlpha', 0.55, ...
                'HandleVisibility', 'off');

        elseif b.type == 3

            [xc, yc, zc] = cylinder([b.r, 0], 30);

            surf( ...
                xc + b.x, ...
                yc + b.y, ...
                zc * b.h, ...
                'FaceColor', [0.50, 0.75, 0.50], ...
                'EdgeColor', 'none', ...
                'FaceAlpha', 0.55, ...
                'HandleVisibility', 'off');
        end
    end
end

%% =========================================================================
% 局部函数 21：随机数范围
% =========================================================================

function value = rand_range_traj(a, b)

    value = a + (b - a) * rand;
end

%% =========================================================================
% 局部函数 22：向量范数限幅
% =========================================================================

function v_limited = limit_vector_norm_traj(v, max_norm)

    if norm(v) > max_norm
        v_limited = v / norm(v) * max_norm;
    else
        v_limited = v;
    end
end

%% =========================================================================
% 局部函数 23：补充缺失参数
% =========================================================================

function params = complete_params_for_trajectory_comparison(params)

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

    if ~isfield(params, 'check_dynamic_collision')
        params.check_dynamic_collision = true;
    end

    if ~isfield(params, 'check_static_collision')
        params.check_static_collision = true;
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

    if ~isfield(params, 'save_figures')
        params.save_figures = true;
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

    if ~isfield(params, 'goal_direct_radius')
        params.goal_direct_radius = 8.0;
    end

    if ~isfield(params, 'goal_slow_radius')
        params.goal_slow_radius = 5.0;
    end

    if params.goal_direct_radius <= params.goal_slow_radius
        params.goal_direct_radius = params.goal_slow_radius + 3.0;
    end

    if ~isfield(params, 'goal_line_of_sight_radius')
        params.goal_line_of_sight_radius = 18.0;
    end

    if ~isfield(params, 'goal_line_check_resolution')
        params.goal_line_check_resolution = 0.8;
    end

    if ~isfield(params, 'goal_line_static_margin')
        params.goal_line_static_margin = 0.8;
    end

    if ~isfield(params, 'goal_line_dynamic_margin')
        params.goal_line_dynamic_margin = 1.0;
    end

    if ~isfield(params, 'apf_k_att')
        params.apf_k_att = 1.2;
    end

    if ~isfield(params, 'apf_k_rep')
        params.apf_k_rep = 30.0;
    end

    if ~isfield(params, 'apf_k_rep_dynamic')
        params.apf_k_rep_dynamic = 35.0;
    end

    if ~isfield(params, 'apf_rho_0')
        params.apf_rho_0 = 6.0;
    end

    if ~isfield(params, 'apf_dynamic_influence_radius')
        params.apf_dynamic_influence_radius = 6.0;
    end

    if ~isfield(params, 'apf_damping')
        params.apf_damping = 1.0;
    end

    if ~isfield(params, 'rrt_max_iter')
        params.rrt_max_iter = 350;
    end

    if ~isfield(params, 'rrt_step_size')
        params.rrt_step_size = 2.0;
    end

    if ~isfield(params, 'rrt_goal_sample_rate')
        params.rrt_goal_sample_rate = 0.15;
    end

    if ~isfield(params, 'rrt_goal_radius')
        params.rrt_goal_radius = 2.5;
    end

    if ~isfield(params, 'rrt_rewire_radius')
        params.rrt_rewire_radius = 4.0;
    end

    if ~isfield(params, 'rrt_collision_check_resolution')
        params.rrt_collision_check_resolution = 0.8;
    end

    if ~isfield(params, 'rrt_dynamic_buffer')
        params.rrt_dynamic_buffer = 0.8;
    end

    if ~isfield(params, 'rrt_replan_interval')
        params.rrt_replan_interval = 20;
    end

    if ~isfield(params, 'rrt_lookahead_index')
        params.rrt_lookahead_index = 2;
    end

    if ~isfield(params, 'rrt_kp')
        params.rrt_kp = 1.4;
    end

    if ~isfield(params, 'rrt_kv')
        params.rrt_kv = 2.2;
    end

    if ~exist(params.output_folder, 'dir')
        mkdir(params.output_folder);
    end
end