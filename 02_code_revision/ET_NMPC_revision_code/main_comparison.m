% ==============================================================
% main_comparison.m
%
% 多动态障碍物复杂城市环境下的对比实验主程序
%
% 本文件作用：
% 1. 专门用于对比 Proposed ET-NMPC 与其他 baseline；
% 2. 不再把 3D-APF、RRT* 等对比算法塞进 main_simulation.m；
% 3. 支持复杂城市峡谷、多动态障碍物、异质运动模式和观测噪声；
% 4. 输出成功率、碰撞率、触发率、CPU 时间、最小安全距离、控制能耗等指标；
% 5. 保存 CSV 数据结果；本版本不生成对比图片。
%
% 对比算法：
% 1. Proposed ET-NMPC
% 2. C-NMPC
% 3. NP-ET-NMPC
% 4. SE-ET-NMPC
% 5. 3D-APF
% 6. RRT*
%
% 本版本完善内容：
% 1. 保留原始整体结构，不对代码进行过分删减；
% 2. 不同算法在同一 repeat 下使用相同 scenario_seed，保证公平对比；
% 3. 同步 main_simulation.m 中的终点直接引导逻辑，避免终点附近绕大圈；
% 4. 静态建筑物碰撞检测改为更精确的矩形、圆柱、圆锥 footprint 检测；
% 5. 补充 goal_direct_radius、goal_slow_radius、goal_line_of_sight_radius 等缺失参数；
% 6. 对未成功到达终点的实验，仅统计成功率/碰撞率，其他性能指标记为 NaN；
% 7. 对比实验默认关闭动画且不生成图片，提高批量运行效率；
% 8. 增加 EstimatedEnergyJ / EstimatedEnergyWh，
%    用于回应能量模型验证和定量节能结果。
%    该指标采用简化物理能耗估计：
%    E = P_hover*T + m*g*TotalClimb/eta_climb + k_u*ControlEffort。
%
% 对应审稿意见：
% Reviewer 1 第 5 条：
%   当前只与 3D-APF 对比不足，需要增加 continuous NMPC、event-triggered MPC/NMPC、
%   sampling-based planners such as RRT* 等对比。
%
% Reviewer 2 第 4 条：
%   当前只与 3D-APF 和 continuous NMPC 对比不足，建议增加预测或搜索型轨迹规划方法。
% ==============================================================

clear; clc; close all;

%% =========================================================
% 1. 初始化参数
% =========================================================

params_base = init_params();
params_base = complete_params_for_comparison(params_base);

% =========================================================
% 本文件只用于批量采集对比实验数据，不生成其他图片。
% 轨迹叠加图请使用 main_comparison_trajectory.m 单独生成。
% =========================================================
params_base.save_figures = false;

if isfield(params_base, 'random_seed') && ~isempty(params_base.random_seed)
    rng(params_base.random_seed);
end

% 起点、终点和参考路径方向
p_0 = params_base.p_0;
v_0 = params_base.v_0;
p_goal = params_base.p_goal;

path_dir = (p_goal - p_0) / norm(p_goal - p_0);
path_len = norm(p_goal - p_0);

% 生成城市静态障碍物
% 所有算法共用同一张地图，保证对比公平
buildings = generate_city_map();

% 输出文件夹
comparison_folder = fullfile(params_base.output_folder, 'comparison_results');

if ~exist(comparison_folder, 'dir')
    mkdir(comparison_folder);
end

%% =========================================================
% 2. 对比实验设置
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

% 对比实验重复次数
% 调试时建议在 init_params.m 中设为 1 或 2；
% 正式论文实验建议设为 10、20 或更多。
num_repeats = params_base.num_comparison_repeats;

% 动态障碍物数量
num_dynamic_obstacles = params_base.num_dynamic_obstacles;

disp(' ');
disp('==============================================================');
disp('开始执行对比实验 main_comparison.m');
disp('当前版本：仅保存 CSV 数据，不生成对比图片。');
fprintf('动态障碍物数量：%d\n', num_dynamic_obstacles);
fprintf('每个算法重复次数：%d\n', num_repeats);
fprintf('静态建筑物数量：%d\n', length(buildings));
disp('对比算法：');
for i = 1:num_algos
    fprintf('  %d. %s\n', i, algo_names{i});
end
disp('说明：同一 repeat 下，所有算法使用相同随机场景，保证公平对比。');
disp('==============================================================');

%% =========================================================
% 3. 结果记录
% =========================================================

success_rate = zeros(num_algos, 1);
collision_rate = zeros(num_algos, 1);

% 下列性能指标只对成功到达终点的实验样本进行统计。
% 如果某个算法所有重复实验均失败，则对应汇总值保持 NaN，
% 后续论文表格中可以显示为 “-”。
trigger_rate = nan(num_algos, 1);
avg_cpu = nan(num_algos, 1);
max_cpu = nan(num_algos, 1);
std_cpu = nan(num_algos, 1);
min_dist = nan(num_algos, 1);
min_clearance = nan(num_algos, 1);
control_effort = nan(num_algos, 1);
total_climb = nan(num_algos, 1);
path_length = nan(num_algos, 1);
estimated_energy_j = nan(num_algos, 1);
estimated_energy_wh = nan(num_algos, 1);
avg_steps = nan(num_algos, 1);

% 成功样本数量和失败样本数量
successful_runs = zeros(num_algos, 1);
failed_runs = zeros(num_algos, 1);

detail_rows = {};

%% =========================================================
% 4. 主对比循环
% =========================================================

for algo_idx = 1:num_algos

    algo_name = algo_names{algo_idx};

    fprintf('\n==============================================================\n');
    fprintf('当前算法：%s\n', algo_name);
    fprintf('==============================================================\n');

    % success 和 collision 对所有实验均有意义，因此用 0 初始化
    temp_success = zeros(num_repeats, 1);
    temp_collision = zeros(num_repeats, 1);

    % 以下性能指标只对成功到达终点的实验有完整意义。
    % 若某次实验中途碰撞、超时或未到达终点，则保持 NaN。
    temp_trigger = nan(num_repeats, 1);
    temp_avg_cpu = nan(num_repeats, 1);
    temp_max_cpu = nan(num_repeats, 1);
    temp_std_cpu = nan(num_repeats, 1);
    temp_min_dist = nan(num_repeats, 1);
    temp_min_clearance = nan(num_repeats, 1);
    temp_control_effort = nan(num_repeats, 1);
    temp_total_climb = nan(num_repeats, 1);
    temp_path_length = nan(num_repeats, 1);
    temp_estimated_energy_j = nan(num_repeats, 1);
    temp_estimated_energy_wh = nan(num_repeats, 1);
    temp_steps = nan(num_repeats, 1);

    for r = 1:num_repeats

        fprintf('  重复实验 %d / %d\n', r, num_repeats);

        params = params_base;

        % 复杂场景设置
        params.num_dynamic_obstacles = num_dynamic_obstacles;
        params.use_multi_obstacles = true;
        params.use_heterogeneous_motion = true;
        params.use_unpredictable_motion = true;
        params.use_measurement_noise = true;

        % 对比实验中默认关闭动画，提高运行速度
        params.enable_animation = false;

        % 如果只想保存 CSV，不想生成图片，可以在 init_params.m 或此处设置 save_figures=false
        % 这里不强制关闭 save_figures，保持由全局参数控制。
        % params.save_figures = false;

        % 设置算法模式
        params = configure_algorithm_mode(params, algo_name);

        % =====================================================
        % 重要修改：
        % 同一个 repeat 下，所有算法使用相同随机种子。
        % 原代码中 scenario_seed 与 algo_idx 相关，会导致不同算法遇到不同障碍物场景。
        % 现在改为只与 r 相关，保证公平对比。
        % =====================================================
        scenario_seed = params_base.random_seed + r;

        % 运行一次实验
        result = run_single_comparison_simulation( ...
            params, algo_name, p_0, v_0, p_goal, path_dir, path_len, buildings, scenario_seed);

        % =====================================================
        % 记录结果
        %
        % 重要说明：
        % 1. success 和 collision 是任务级指标，所有实验都记录；
        % 2. CPU、能耗、爬升高度、最小安全间隔等性能指标只对成功样本取平均；
        % 3. 如果算法中途碰撞、达到最大步数或未到达终点，则这些性能指标记为 NaN；
        % 4. 导出到 CSV 后，NaN 可在论文表格中显示为 “-”。
        % =====================================================

        temp_success(r) = result.success;
        temp_collision(r) = result.collision;

        if result.success

            metric_valid = true;

            temp_trigger(r) = result.trigger_rate;
            temp_avg_cpu(r) = result.avg_cpu;
            temp_max_cpu(r) = result.max_cpu;
            temp_std_cpu(r) = result.std_cpu;
            temp_min_dist(r) = result.min_dist_dyn;
            temp_min_clearance(r) = result.min_clearance_dyn;
            temp_control_effort(r) = result.control_effort;
            temp_total_climb(r) = result.total_climb;
            temp_path_length(r) = result.path_length;
            temp_estimated_energy_j(r) = result.estimated_energy_j;
            temp_estimated_energy_wh(r) = result.estimated_energy_wh;
            temp_steps(r) = result.steps;

        else

            metric_valid = false;

            temp_trigger(r) = NaN;
            temp_avg_cpu(r) = NaN;
            temp_max_cpu(r) = NaN;
            temp_std_cpu(r) = NaN;
            temp_min_dist(r) = NaN;
            temp_min_clearance(r) = NaN;
            temp_control_effort(r) = NaN;
            temp_total_climb(r) = NaN;
            temp_path_length(r) = NaN;
            temp_estimated_energy_j(r) = NaN;
            temp_estimated_energy_wh(r) = NaN;
            temp_steps(r) = NaN;
        end

        detail_rows = [detail_rows; { ...
            algo_name, ...
            r, ...
            scenario_seed, ...
            num_dynamic_obstacles, ...
            length(buildings), ...
            result.success, ...
            result.collision, ...
            metric_valid, ...
            result.failure_reason, ...
            temp_trigger(r), ...
            temp_avg_cpu(r), ...
            temp_max_cpu(r), ...
            temp_std_cpu(r), ...
            temp_min_dist(r), ...
            temp_min_clearance(r), ...
            temp_control_effort(r), ...
            temp_total_climb(r), ...
            temp_path_length(r), ...
            temp_estimated_energy_j(r), ...
            temp_estimated_energy_wh(r), ...
            temp_steps(r) ...
        }]; %#ok<AGROW>
    end

    % =====================================================
    % 汇总平均结果
    %
    % success_rate 和 collision_rate 基于所有重复实验统计；
    % 其他性能指标只基于成功完成任务的实验统计。
    % 如果某算法一次都没有成功，则对应指标保持 NaN，
    % 后续论文表格中可显示为 “-”。
    % =====================================================

    success_rate(algo_idx) = mean(temp_success) * 100;
    collision_rate(algo_idx) = mean(temp_collision) * 100;

    successful_runs(algo_idx) = sum(temp_success == 1);
    failed_runs(algo_idx) = num_repeats - successful_runs(algo_idx);

    trigger_rate(algo_idx) = mean_omit_nan_local(temp_trigger);
    avg_cpu(algo_idx) = mean_omit_nan_local(temp_avg_cpu);
    max_cpu(algo_idx) = mean_omit_nan_local(temp_max_cpu);
    std_cpu(algo_idx) = mean_omit_nan_local(temp_std_cpu);
    min_dist(algo_idx) = mean_omit_nan_local(temp_min_dist);
    min_clearance(algo_idx) = mean_omit_nan_local(temp_min_clearance);
    control_effort(algo_idx) = mean_omit_nan_local(temp_control_effort);
    total_climb(algo_idx) = mean_omit_nan_local(temp_total_climb);
    path_length(algo_idx) = mean_omit_nan_local(temp_path_length);
    estimated_energy_j(algo_idx) = mean_omit_nan_local(temp_estimated_energy_j);
    estimated_energy_wh(algo_idx) = mean_omit_nan_local(temp_estimated_energy_wh);
    avg_steps(algo_idx) = mean_omit_nan_local(temp_steps);

    fprintf('\n算法 %s 汇总结果：\n', algo_name);
    fprintf('Success Rate      : %.1f %%\n', success_rate(algo_idx));
    fprintf('Collision Rate    : %.1f %%\n', collision_rate(algo_idx));
    fprintf('Successful Runs   : %d / %d\n', successful_runs(algo_idx), num_repeats);
    fprintf('Failed Runs       : %d / %d\n', failed_runs(algo_idx), num_repeats);
    fprintf('Trigger Rate      : %s %%\n', format_metric_local(trigger_rate(algo_idx), '%.1f'));
    fprintf('Avg CPU Time      : %s ms\n', format_metric_local(avg_cpu(algo_idx), '%.2f'));
    fprintf('Max CPU Time      : %s ms\n', format_metric_local(max_cpu(algo_idx), '%.2f'));
    fprintf('Min Distance      : %s m\n', format_metric_local(min_dist(algo_idx), '%.2f'));
    fprintf('Min Clearance     : %s m\n', format_metric_local(min_clearance(algo_idx), '%.2f'));
    fprintf('Control Effort    : %s\n', format_metric_local(control_effort(algo_idx), '%.2f'));
    fprintf('Total Climb       : %s m\n', format_metric_local(total_climb(algo_idx), '%.2f'));
    fprintf('Path Length       : %s m\n', format_metric_local(path_length(algo_idx), '%.2f'));
    fprintf('Estimated Energy  : %s J / %s Wh\n', ...
        format_metric_local(estimated_energy_j(algo_idx), '%.2f'), ...
        format_metric_local(estimated_energy_wh(algo_idx), '%.4f'));
end

%% =========================================================
% 5. 打印最终结果表
% =========================================================

disp(' ');
disp('==============================================================');
disp('对比实验最终结果');
disp('==============================================================');
fprintf('%-18s | Success | Collision | Valid | Trigger | Avg CPU | Clearance | Effort | Climb | EstEnergy(Wh)\n', 'Algorithm');
fprintf('--------------------------------------------------------------------------------------------------------------------------------\n');

for i = 1:num_algos
    fprintf('%-18s | %6.1f%% | %8.1f%% | %2d/%-2d | %7s | %7s | %9s | %6s | %5s | %13s\n', ...
        algo_names{i}, ...
        success_rate(i), ...
        collision_rate(i), ...
        successful_runs(i), ...
        num_repeats, ...
        format_metric_local(trigger_rate(i), '%.1f'), ...
        format_metric_local(avg_cpu(i), '%.2f'), ...
        format_metric_local(min_clearance(i), '%.2f'), ...
        format_metric_local(control_effort(i), '%.2f'), ...
        format_metric_local(total_climb(i), '%.2f'), ...
        format_metric_local(estimated_energy_wh(i), '%.4f'));
end

fprintf('--------------------------------------------------------------------------------------------------------------------------------\n');

%% =========================================================
% 6. 保存 CSV 结果
% =========================================================

summary_table = table( ...
    algo_names(:), ...
    repmat(num_dynamic_obstacles, num_algos, 1), ...
    repmat(length(buildings), num_algos, 1), ...
    repmat(num_repeats, num_algos, 1), ...
    successful_runs, ...
    failed_runs, ...
    success_rate, ...
    collision_rate, ...
    trigger_rate, ...
    avg_cpu, ...
    max_cpu, ...
    std_cpu, ...
    min_dist, ...
    min_clearance, ...
    control_effort, ...
    total_climb, ...
    path_length, ...
    estimated_energy_j, ...
    estimated_energy_wh, ...
    avg_steps, ...
    'VariableNames', { ...
    'Algorithm', ...
    'NumDynamicObstacles', ...
    'NumStaticBuildings', ...
    'TotalRuns', ...
    'SuccessfulRuns', ...
    'FailedRuns', ...
    'SuccessRatePercent', ...
    'CollisionRatePercent', ...
    'AvgTriggerRatePercent_SuccessOnly', ...
    'AvgCPUms_SuccessOnly', ...
    'MaxCPUms_SuccessOnly', ...
    'StdCPUms_SuccessOnly', ...
    'AvgMinDistanceMeter_SuccessOnly', ...
    'AvgMinClearanceMeter_SuccessOnly', ...
    'ControlEffort_SuccessOnly', ...
    'TotalClimbMeter_SuccessOnly', ...
    'PathLengthMeter_SuccessOnly', ...
    'EstimatedEnergyJ_SuccessOnly', ...
    'EstimatedEnergyWh_SuccessOnly', ...
    'AvgSteps_SuccessOnly'});

summary_file = fullfile(comparison_folder, 'comparison_summary_results.csv');
writetable(summary_table, summary_file);

detail_table = cell2table(detail_rows, ...
    'VariableNames', { ...
    'Algorithm', ...
    'RepeatIndex', ...
    'ScenarioSeed', ...
    'NumDynamicObstacles', ...
    'NumStaticBuildings', ...
    'Success', ...
    'Collision', ...
    'MetricValid', ...
    'FailureReason', ...
    'TriggerRatePercent', ...
    'AvgCPUms', ...
    'MaxCPUms', ...
    'StdCPUms', ...
    'MinDistanceMeter', ...
    'MinClearanceMeter', ...
    'ControlEffort', ...
    'TotalClimbMeter', ...
    'PathLengthMeter', ...
    'EstimatedEnergyJ', ...
    'EstimatedEnergyWh', ...
    'Steps'});

detail_file = fullfile(comparison_folder, 'comparison_detail_results.csv');
writetable(detail_table, detail_file);

disp(' ');
disp(['对比实验汇总结果已保存：', summary_file]);
disp(['对比实验详细结果已保存：', detail_file]);

%% =========================================================
% 7. 跳过绘图，只保存 CSV 数据
% =========================================================
% 本文件用于批量采集对比实验数据，不生成图片。
% 如果需要轨迹叠加对比图，请运行 main_comparison_trajectory.m。
% 如果需要柱状图，可以后续根据 CSV 单独绘制。

disp('已跳过对比实验图片生成，仅保存 CSV 数据。');

disp(' ');
disp('==============================================================');
disp('main_comparison.m 对比实验全部完成');
disp('==============================================================');

%% =========================================================================
% 局部函数 1：根据算法名称设置算法模式
% =========================================================================

function params = configure_algorithm_mode(params, algo_name)

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
% 局部函数 2：单次对比仿真
% =========================================================================

function result = run_single_comparison_simulation( ...
    params, algo_name, p_0, v_0, p_goal, path_dir, path_len, buildings, scenario_seed)

    rng(scenario_seed);

    x_curr = [p_0; v_0];

    obs_array = initialize_dynamic_obstacles_comp(params, p_0, p_goal);

    num_obs = length(obs_array);

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

    % RRT* 使用的路径缓存
    rrt_path = [];
    rrt_replan_counter = inf;

    while norm(x_curr(1:3) - p_goal) > params.eps_tol && k < params.k_max

        k = k + 1;
        current_time = k * params.dt;

        history_x(:, k) = x_curr;

        % 更新动态障碍物真实运动
        obs_array = update_dynamic_obstacles_comp(obs_array, params, current_time);

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
            if check_static_collision_comp(x_curr, buildings, params)
                collision_flag = true;
                break;
            end
        end

        %% =====================================================
        % 构造参考状态和局部参考轨迹
        % 同步 main_simulation.m 的终点直接引导逻辑
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

        % 判断当前到终点的直线路径是否可通行
        line_to_goal_clear = check_line_to_goal_clear_comp( ...
            x_curr(1:3), p_goal, buildings, obs_array, params);

        use_direct_goal_guidance = ...
            (dist_to_goal <= goal_direct_radius) || ...
            (line_to_goal_clear && dist_to_goal <= params.goal_line_of_sight_radius);

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
                u_cmd = nominal_tracking_control_comp(x_curr, x_ref, dist_to_goal, use_direct_goal_guidance, params);
            end

        elseif strcmp(params.controller_type, 'APF')

            sigma_k = 0;
            u_cmd = apf_controller_3d_comp(x_curr, p_goal, obs_array, buildings, params);

        elseif strcmp(params.controller_type, 'RRTSTAR')

            sigma_k = 0;

            % RRT* 不需要每一步都重新规划，否则计算量过大
            if isempty(rrt_path) || rrt_replan_counter >= params.rrt_replan_interval
                rrt_path = rrtstar_plan_comp(x_curr(1:3), p_goal, obs_array, buildings, params);
                rrt_replan_counter = 0;
            end

            rrt_replan_counter = rrt_replan_counter + 1;

            u_cmd = rrtstar_tracking_control_comp(x_curr, rrt_path, p_goal, params);

        else
            error('未知 controller_type：%s', params.controller_type);
        end

        history_t_solve(k) = toc(t_start);

        % 状态更新
        x_next = zeros(6, 1);
        x_next(1:3) = x_curr(1:3) + x_curr(4:6) * params.dt;
        x_next(4:6) = x_curr(4:6) + u_cmd * params.dt;

        x_next(4:6) = limit_vector_norm_comp(x_next(4:6), params.v_max);
        x_next(3) = max(min(x_next(3), params.z_max), params.z_min);

        history_u(:, k) = u_cmd;
        history_sigma(k) = sigma_k;

        x_curr = x_next;
        sigma_prev = sigma_k;
    end

    if ~collision_flag && norm(x_curr(1:3) - p_goal) <= params.eps_tol
        success_flag = true;
    end

    % =====================================================
    % 记录失败原因
    % Success        ：成功到达终点；
    % Collision      ：中途发生静态或动态碰撞；
    % MaxStepReached ：达到最大步数仍未到达终点；
    % NotReached     ：未碰撞但也未进入终点容差。
    % =====================================================
    if success_flag
        failure_reason = 'Success';
    elseif collision_flag
        failure_reason = 'Collision';
    elseif k >= params.k_max
        failure_reason = 'MaxStepReached';
    else
        failure_reason = 'NotReached';
    end

    valid_k = max(1, k);

    [min_dist_dyn, min_clearance_dyn] = compute_dynamic_distance_metrics_comp( ...
        history_x, history_obs, obs_array, valid_k, params);

    delta_pos = diff(history_x(1:3, 1:valid_k), 1, 2);
    path_length = sum(sqrt(sum(delta_pos.^2, 1)));

    delta_z = diff(history_x(3, 1:valid_k));
    total_climb = sum(max(0, delta_z));

    control_effort = sum(sum(history_u(:, 1:valid_k).^2)) * params.dt;

    % =====================================================
    % 简化物理能耗估计，用于 Reviewer 1 第 3 条“定量节能结果”
    %
    % 该估计量以 Joule 和 Wh 输出，便于在论文表格中报告：
    %   E_total = E_propulsion + E_climb + E_maneuver
    %   E_propulsion = P_hover * T
    %   E_climb     = m*g*total_climb/eta_climb
    %   E_maneuver  = k_u * ∫||u||^2dt
    %
    % 其中 P_hover*T 近似基础悬停/平飞推进功耗，
    % m*g*total_climb/eta_climb 近似垂直爬升所需势能，
    % k_u*control_effort 近似剧烈机动带来的附加能耗。
    % 该模型仍然是简化估计，不包含显式风场和完整旋翼气动模型。
    % =====================================================
    mass_uav = get_param_or_default_local(params, 'energy_mass_kg', 1.5);
    gravity_acc = get_param_or_default_local(params, 'energy_gravity', 9.81);
    eta_climb = get_param_or_default_local(params, 'energy_climb_efficiency', 0.70);
    p_hover = get_param_or_default_local(params, 'energy_hover_power_w', 180.0);
    k_u_energy = get_param_or_default_local(params, 'energy_control_coeff', 2.0);

    flight_time = valid_k * params.dt;

    estimated_energy_propulsion_j = p_hover * flight_time;
    estimated_energy_climb_j = mass_uav * gravity_acc * total_climb / max(eta_climb, 1e-6);
    estimated_energy_maneuver_j = k_u_energy * control_effort;

    estimated_energy_j = estimated_energy_propulsion_j + ...
                         estimated_energy_climb_j + ...
                         estimated_energy_maneuver_j;

    estimated_energy_wh = estimated_energy_j / 3600.0;

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
    result.path_length = path_length;
    result.estimated_energy_j = estimated_energy_j;
    result.estimated_energy_wh = estimated_energy_wh;
    result.steps = valid_k;
    result.failure_reason = failure_reason;

    fprintf('    %s | success=%d | collision=%d | reason=%s | cpu=%.2f ms | clearance=%.2f m | energy=%.4f Wh | steps=%d\n', ...
        algo_name, result.success, result.collision, result.failure_reason, result.avg_cpu, result.min_clearance_dyn, result.estimated_energy_wh, result.steps);
end

%% =========================================================================
% 局部函数 3：轻量级跟踪控制器
% =========================================================================

function u_cmd = nominal_tracking_control_comp(x_curr, x_ref, dist_to_goal, use_direct_goal_guidance, params)

    e_p = x_ref(1:3) - x_curr(1:3);
    e_v = x_ref(4:6) - x_curr(4:6);

    if use_direct_goal_guidance
        u_cmd_raw = 2.2 * e_p + 3.2 * e_v;
    elseif dist_to_goal < params.goal_slow_radius
        u_cmd_raw = 2.0 * e_p + 3.0 * e_v;
    else
        u_cmd_raw = 1.5 * e_p + 2.5 * e_v;
    end

    u_cmd = limit_vector_norm_comp(u_cmd_raw, params.u_max);
end

%% =========================================================================
% 局部函数 4：3D-APF 控制器
% =========================================================================

function u_cmd = apf_controller_3d_comp(x_curr, p_goal, obs_array, buildings, params)

    p = x_curr(1:3);
    v = x_curr(4:6);

    % 引力
    F_att = params.apf_k_att * (p_goal - p);

    % 静态建筑物斥力
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

    % 动态障碍物斥力
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

    % 速度阻尼，避免 APF 振荡过强
    F_damp = -params.apf_damping * v;

    F_total = F_att + F_rep_static + F_rep_dyn + F_damp;

    u_cmd = limit_vector_norm_comp(F_total, params.u_max);
end

%% =========================================================================
% 局部函数 5：RRT* 路径规划器
% =========================================================================

function path = rrtstar_plan_comp(p_start, p_goal, obs_array, buildings, params)

    % 如果直线可行，直接返回直线路径
    if is_segment_collision_free_comp(p_start, p_goal, obs_array, buildings, params)
        path = [p_start, p_goal];
        return;
    end

    max_iter = params.rrt_max_iter;
    step_size = params.rrt_step_size;
    goal_sample_rate = params.rrt_goal_sample_rate;
    goal_radius = params.rrt_goal_radius;
    search_radius = params.rrt_rewire_radius;

    % 节点结构
    nodes_pos = zeros(3, max_iter + 1);
    nodes_parent = zeros(1, max_iter + 1);
    nodes_cost = inf(1, max_iter + 1);

    nodes_pos(:, 1) = p_start;
    nodes_parent(1) = 0;
    nodes_cost(1) = 0;

    node_count = 1;
    goal_node = -1;

    for iter = 1:max_iter

        % 采样
        if rand < goal_sample_rate
            p_rand = p_goal;
        else
            p_rand = [
                rand_range(params.x_lim(1), params.x_lim(2));
                rand_range(params.y_lim(1), params.y_lim(2));
                rand_range(params.z_min + 0.5, params.z_max - 0.5)
            ];
        end

        % 找最近节点
        dist_all = sqrt(sum((nodes_pos(:, 1:node_count) - p_rand).^2, 1));
        [~, nearest_idx] = min(dist_all);

        p_nearest = nodes_pos(:, nearest_idx);

        % 朝采样点扩展
        direction = p_rand - p_nearest;

        if norm(direction) < 1e-6
            continue;
        end

        direction = direction / norm(direction);
        p_new = p_nearest + step_size * direction;

        % 边界限制
        p_new(1) = max(min(p_new(1), params.x_lim(2)), params.x_lim(1));
        p_new(2) = max(min(p_new(2), params.y_lim(2)), params.y_lim(1));
        p_new(3) = max(min(p_new(3), params.z_max), params.z_min);

        % 碰撞检测
        if ~is_segment_collision_free_comp(p_nearest, p_new, obs_array, buildings, params)
            continue;
        end

        % 找邻居节点
        dist_to_new = sqrt(sum((nodes_pos(:, 1:node_count) - p_new).^2, 1));
        near_indices = find(dist_to_new <= search_radius);

        % 选择最低代价父节点
        best_parent = nearest_idx;
        best_cost = nodes_cost(nearest_idx) + norm(p_new - p_nearest);

        for j = 1:length(near_indices)
            idx = near_indices(j);
            p_near = nodes_pos(:, idx);
            candidate_cost = nodes_cost(idx) + norm(p_new - p_near);

            if candidate_cost < best_cost
                if is_segment_collision_free_comp(p_near, p_new, obs_array, buildings, params)
                    best_parent = idx;
                    best_cost = candidate_cost;
                end
            end
        end

        % 添加新节点
        node_count = node_count + 1;
        nodes_pos(:, node_count) = p_new;
        nodes_parent(node_count) = best_parent;
        nodes_cost(node_count) = best_cost;

        new_idx = node_count;

        % 重连邻居节点
        for j = 1:length(near_indices)
            idx = near_indices(j);

            if idx == best_parent
                continue;
            end

            p_near = nodes_pos(:, idx);
            candidate_cost = nodes_cost(new_idx) + norm(p_near - p_new);

            if candidate_cost < nodes_cost(idx)
                if is_segment_collision_free_comp(p_new, p_near, obs_array, buildings, params)
                    nodes_parent(idx) = new_idx;
                    nodes_cost(idx) = candidate_cost;
                end
            end
        end

        % 判断是否到达目标
        if norm(p_new - p_goal) < goal_radius
            if is_segment_collision_free_comp(p_new, p_goal, obs_array, buildings, params)

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
        % 如果没有成功连接目标，选择最接近目标的节点作为临时目标
        dist_to_goal = sqrt(sum((nodes_pos(:, 1:node_count) - p_goal).^2, 1));
        [~, goal_node] = min(dist_to_goal);
    end

    % 回溯路径
    path_rev = [];
    idx = goal_node;

    while idx > 0
        path_rev = [path_rev, nodes_pos(:, idx)]; %#ok<AGROW>
        idx = nodes_parent(idx);
    end

    path = fliplr(path_rev);

    % 简单路径平滑
    path = shortcut_path_comp(path, obs_array, buildings, params);
end

%% =========================================================================
% 局部函数 6：RRT* 跟踪控制
% =========================================================================

function u_cmd = rrtstar_tracking_control_comp(x_curr, path, p_goal, params)

    p = x_curr(1:3);
    v = x_curr(4:6);

    if isempty(path)
        p_target = p_goal;
    elseif size(path, 2) == 1
        p_target = path(:, 1);
    else
        % 选择距离当前 UAV 最近之后的一个前视点
        dist_to_nodes = sqrt(sum((path - p).^2, 1));
        [~, nearest_idx] = min(dist_to_nodes);

        target_idx = min(nearest_idx + params.rrt_lookahead_index, size(path, 2));
        p_target = path(:, target_idx);
    end

    % 接近终点时直接追踪终点，避免 RRT* 跟踪路径尾段绕行
    if norm(p_goal - p) <= params.goal_direct_radius
        p_target = p_goal;
    end

    e_p = p_target - p;
    e_v = -v;

    u_raw = params.rrt_kp * e_p + params.rrt_kv * e_v;

    u_cmd = limit_vector_norm_comp(u_raw, params.u_max);
end

%% =========================================================================
% 局部函数 7：线段碰撞检测
% =========================================================================

function flag = is_segment_collision_free_comp(p1, p2, obs_array, buildings, params)

    flag = true;

    seg_len = norm(p2 - p1);
    n_check = max(2, ceil(seg_len / params.rrt_collision_check_resolution));

    for i = 0:n_check

        alpha = i / n_check;
        p = (1 - alpha) * p1 + alpha * p2;

        % 边界检查
        if p(1) < params.x_lim(1) || p(1) > params.x_lim(2) || ...
           p(2) < params.y_lim(1) || p(2) > params.y_lim(2) || ...
           p(3) < params.z_min || p(3) > params.z_max
            flag = false;
            return;
        end

        % 静态建筑物检查
        x_tmp = [p; 0; 0; 0];

        if check_static_collision_comp(x_tmp, buildings, params)
            flag = false;
            return;
        end

        % 动态障碍物检查
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
% 局部函数 8：RRT* 路径快捷平滑
% =========================================================================

function path_smooth = shortcut_path_comp(path, obs_array, buildings, params)

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

        if is_segment_collision_free_comp(path_smooth(:, i), path_smooth(:, j), obs_array, buildings, params)
            path_smooth = [path_smooth(:, 1:i), path_smooth(:, j:end)];
        end
    end
end

%% =========================================================================
% 局部函数 9：初始化动态障碍物
% =========================================================================

function obs_array = initialize_dynamic_obstacles_comp(params, p_0, p_goal)

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
% 局部函数 10：更新动态障碍物
% =========================================================================

function obs_array = update_dynamic_obstacles_comp(obs_array, params, current_time)

    for i = 1:length(obs_array)

        motion_type = obs_array(i).motion_type;

        if motion_type == params.motion_type_CV

            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;

        elseif motion_type == params.motion_type_CA

            acc = params.obs_ca_acc * obs_array(i).acc_dir;
            obs_array(i).v = obs_array(i).v + acc * params.dt;
            obs_array(i).v = limit_vector_norm_comp(obs_array(i).v, params.obs_v_max);
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
            obs_array(i).v = limit_vector_norm_comp(obs_array(i).v, params.obs_v_max);

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

            obs_array(i).v = limit_vector_norm_comp(obs_array(i).v, params.obs_v_max);
            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;

        else

            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;
        end

        obs_array(i).p(3) = max(min(obs_array(i).p(3), params.z_max - 0.5), params.z_min + 0.5);
    end
end

%% =========================================================================
% 局部函数 11：终点直线路径可见性检测
% =========================================================================

function flag = check_line_to_goal_clear_comp(p_start, p_goal, buildings, obs_array, params)

    flag = true;

    segment_len = norm(p_goal - p_start);

    if segment_len < 1e-6
        return;
    end

    n_check = max(3, ceil(segment_len / params.goal_line_check_resolution));

    for ii = 0:n_check

        alpha = ii / n_check;
        p = (1 - alpha) * p_start + alpha * p_goal;

        if is_point_inside_static_safety_zone_comp(p, buildings, params)
            flag = false;
            return;
        end

        if is_point_inside_dynamic_safety_zone_comp(p, obs_array, params)
            flag = false;
            return;
        end
    end
end

%% =========================================================================
% 局部函数 12：判断点是否进入静态建筑物安全区
% =========================================================================

function flag = is_point_inside_static_safety_zone_comp(p, buildings, params)

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
% 局部函数 13：判断点是否进入动态障碍物安全区
% =========================================================================

function flag = is_point_inside_dynamic_safety_zone_comp(p, obs_array, params)

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
% 局部函数 14：静态碰撞检测
% =========================================================================

function flag = check_static_collision_comp(x_curr, buildings, params)

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
% 局部函数 15：动态距离指标
% =========================================================================

function [min_dist_dyn, min_clearance_dyn] = compute_dynamic_distance_metrics_comp( ...
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
% 局部函数 16：随机数范围
% =========================================================================

function value = rand_range(a, b)

    value = a + (b - a) * rand;
end

%% =========================================================================
% 局部函数 17：向量范数限幅
% =========================================================================

function v_limited = limit_vector_norm_comp(v, max_norm)

    if norm(v) > max_norm
        v_limited = v / norm(v) * max_norm;
    else
        v_limited = v;
    end
end

%% =========================================================================
% 局部函数 18：补充缺失参数
% =========================================================================

function params = complete_params_for_comparison(params)

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

    if ~isfield(params, 'num_comparison_repeats')%循环次数
        params.num_comparison_repeats = 30;
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

    % 本文件只采集 CSV 数据，不生成图片。
    params.save_figures = false;

    % 简化物理能耗估计参数
    % energy_mass_kg: UAV 质量 kg
    % energy_hover_power_w: 基础悬停/平飞推进功率 W
    % energy_climb_efficiency: 爬升效率
    % energy_control_coeff: 控制输入平方积分到附加机动能耗的经验系数
    if ~isfield(params, 'energy_mass_kg')
        params.energy_mass_kg = 1.5;
    end

    if ~isfield(params, 'energy_gravity')
        params.energy_gravity = 9.81;
    end

    if ~isfield(params, 'energy_hover_power_w')
        params.energy_hover_power_w = 180.0;
    end

    if ~isfield(params, 'energy_climb_efficiency')
        params.energy_climb_efficiency = 0.70;
    end

    if ~isfield(params, 'energy_control_coeff')
        params.energy_control_coeff = 2.0;
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

    % =====================================================
    % 终点直接引导参数
    % 与 main_simulation.m 保持一致
    % =====================================================
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

    % APF 参数
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

    % RRT* 参数
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

%% =========================================================================
% 局部函数 19：读取参数，不存在时使用默认值
% =========================================================================

function value = get_param_or_default_local(params, field_name, default_value)

    if isfield(params, field_name)
        value = params.(field_name);
    else
        value = default_value;
    end
end

%% =========================================================================
% 局部函数 20：忽略 NaN 求平均
% =========================================================================

function value = mean_omit_nan_local(x)

    valid_x = x(~isnan(x));

    if isempty(valid_x)
        value = NaN;
    else
        value = mean(valid_x);
    end
end

%% =========================================================================
% 局部函数 21：格式化显示指标
% =========================================================================

function str = format_metric_local(value, format_str)

    if isnan(value)
        str = '-';
    else
        str = sprintf(format_str, value);
    end
end