% ==============================================================
% main_monte_carlo.m
%
% 多动态障碍物可扩展性 Monte Carlo 分析脚本（最终完善版）
%
% 本文件作用：
% 1. 自动测试不同动态障碍物数量下的避障性能；
% 2. 支持动态障碍物数量为 1、3、5；
% 3. 支持异质动态障碍物运动模式，包括 CV、CA、SIN、RAND；
% 4. 对 Proposed ET-NMPC、C-NMPC、NP-ET-NMPC、SE-ET-NMPC 进行统计比较；
% 5. 输出 Success Rate、Collision Rate、Trigger Rate、Avg CPU、
%    Minimum Clearance、Control Effort、Climb、Estimated Energy 等指标；
% 6. Success 以百分比形式输出；
% 7. 其余连续指标同时输出 mean、std 和 mean +/- std 文本形式；
% 8. 程序后台额外保留 Collision Rate 和每次实验的 Collision 标志，便于备用分析；
% 9. 保存 CSV 表格，包括汇总结果和每次 Monte Carlo 的详细结果；
% 10. 本版本默认不生成可扩展性分析图片，避免产生不需要的图片文件。
%
% 对应审稿意见：
% Reviewer 1 第 4 条：
%   需要增加多个动态障碍物和异质运动模式；
%   需要分析计算时间和触发率随障碍物数量变化的可扩展性。
%
% Reviewer 2 第 2 条：
%   需要测试更随机、更不可预测的动态障碍物运动场景。
%
% 说明：
% 该脚本只负责生成 Monte Carlo 数据。如果需要绘图，可使用本文件
% 保留的 plot_scalability_results 函数，或单独编写绘图脚本读取 CSV。
% ==============================================================

clear; clc; close all;

%% =========================================================
% 1. 初始化基础参数
% =========================================================

params_base = init_params();
params_base = complete_params_for_monte_carlo(params_base);

% 本脚本默认只用于生成 Monte Carlo 数据，不生成任何图片。
params_base.save_csv = true;
params_base.save_figures = false;

% 固定随机种子，保证 Monte Carlo 实验可复现
if isfield(params_base, 'random_seed') && ~isempty(params_base.random_seed)
    rng(params_base.random_seed);
end

% 起点、终点和参考路径方向
p_0 = params_base.p_0;
v_0 = params_base.v_0;
p_goal = params_base.p_goal;

path_dir = (p_goal - p_0) / norm(p_goal - p_0);
path_len = norm(p_goal - p_0);

% 生成静态城市环境
buildings = generate_city_map();

% 动态障碍物数量列表（固定为 1、3、5）
obs_count_list = [1, 3, 5];

% 每个障碍物数量下的 Monte Carlo 重复次数
num_repeats = params_base.num_scalability_repeats;

% 对比算法设置
algo_names = {
    'Proposed ET-NMPC';
    'C-NMPC';
    'NP-ET-NMPC';
    'SE-ET-NMPC'
};

num_algos = length(algo_names);
num_obs_cases = length(obs_count_list);

% 汇总结果矩阵
success_rate_mat        = zeros(num_obs_cases, num_algos);
collision_rate_mat      = zeros(num_obs_cases, num_algos);

trigger_mean_mat        = zeros(num_obs_cases, num_algos);
trigger_std_mat         = zeros(num_obs_cases, num_algos);

avg_cpu_mean_mat        = zeros(num_obs_cases, num_algos);
avg_cpu_std_mat         = zeros(num_obs_cases, num_algos);
max_cpu_mean_mat        = zeros(num_obs_cases, num_algos);
max_cpu_std_mat         = zeros(num_obs_cases, num_algos);

clearance_mean_mat      = zeros(num_obs_cases, num_algos);
clearance_std_mat       = zeros(num_obs_cases, num_algos);

effort_mean_mat         = zeros(num_obs_cases, num_algos);
effort_std_mat          = zeros(num_obs_cases, num_algos);

climb_mean_mat          = zeros(num_obs_cases, num_algos);
climb_std_mat           = zeros(num_obs_cases, num_algos);

energy_mean_mat         = zeros(num_obs_cases, num_algos);
energy_std_mat          = zeros(num_obs_cases, num_algos);

steps_mean_mat          = zeros(num_obs_cases, num_algos);
steps_std_mat           = zeros(num_obs_cases, num_algos);

valid_metric_count_mat  = zeros(num_obs_cases, num_algos);

% 保存每一次 Monte Carlo 的详细结果
detail_rows = {};

disp(' ');
disp('==============================================================');
disp('开始执行多动态障碍物可扩展性 Monte Carlo 分析');
fprintf('障碍物数量列表：%s\n', mat2str(obs_count_list));
fprintf('每组重复次数：%d\n', num_repeats);
disp('对比算法：Proposed ET-NMPC, C-NMPC, NP-ET-NMPC, SE-ET-NMPC');
disp('统计格式：Success 用百分比，其余连续指标输出 mean +/- std。');
disp('后台保留 Collision，用于备用分析。');
disp('注意：当前版本只保存 CSV 数据，不生成可扩展性图片。');
disp('==============================================================');

%% =========================================================
% 2. 主 Monte Carlo 循环
% =========================================================

for obs_case_idx = 1:num_obs_cases

    num_obs = obs_count_list(obs_case_idx);

    fprintf('\n==============================================================\n');
    fprintf('当前测试动态障碍物数量：%d\n', num_obs);
    fprintf('==============================================================\n');

    for algo_idx = 1:num_algos

        algo_name = algo_names{algo_idx};

        fprintf('\n--- 当前算法：%s ---\n', algo_name);

        % 当前算法在当前障碍物数量下的单次结果
        temp_success       = zeros(num_repeats, 1);
        temp_collision     = zeros(num_repeats, 1);

        temp_trigger       = zeros(num_repeats, 1);
        temp_avg_cpu       = zeros(num_repeats, 1);
        temp_max_cpu       = zeros(num_repeats, 1);
        temp_std_cpu       = zeros(num_repeats, 1);

        temp_min_dist      = zeros(num_repeats, 1);
        temp_min_clearance = zeros(num_repeats, 1);

        temp_effort        = zeros(num_repeats, 1);
        temp_climb         = zeros(num_repeats, 1);
        temp_energy_wh     = zeros(num_repeats, 1);

        temp_steps         = zeros(num_repeats, 1);

        for mc = 1:num_repeats

            fprintf('  Monte Carlo %d / %d\n', mc, num_repeats);

            % 为每次实验设置参数
            params = params_base;
            params.num_dynamic_obstacles = num_obs;
            params.use_multi_obstacles = true;
            params.use_heterogeneous_motion = true;
            params.use_unpredictable_motion = true;
            params.use_measurement_noise = true;

            % Monte Carlo 批量实验不显示动画、不保存图片，只保存数值结果
            params.enable_animation = false;
            params.save_figures = false;
            params.save_csv = true;

            % 设置算法模式
            params = apply_algorithm_mode_mc(params, algo_name);

            % 使用不同随机种子生成每次 Monte Carlo 场景
            scenario_seed = params_base.random_seed + 10000 * obs_case_idx + 1000 * algo_idx + mc;

            % 运行单次仿真
            result = run_single_monte_carlo_simulation( ...
                params, p_0, v_0, p_goal, path_dir, path_len, buildings, scenario_seed);

            % 记录单次结果
            temp_success(mc)       = result.success;
            temp_collision(mc)     = result.collision;

            temp_trigger(mc)       = result.trigger_rate;
            temp_avg_cpu(mc)       = result.avg_cpu;
            temp_max_cpu(mc)       = result.max_cpu;
            temp_std_cpu(mc)       = result.std_cpu;

            temp_min_dist(mc)      = result.min_dist_dyn;
            temp_min_clearance(mc) = result.min_clearance_dyn;

            temp_effort(mc)        = result.control_effort;
            temp_climb(mc)         = result.total_climb;
            temp_energy_wh(mc)     = result.est_energy_wh;

            temp_steps(mc)         = result.steps;

            % 保存详细行
            detail_rows = [detail_rows; { ...
                num_obs, ...
                algo_name, ...
                mc, ...
                scenario_seed, ...
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
                result.est_energy_wh, ...
                result.steps ...
            }]; %#ok<AGROW>
        end

        % =====================================================
        % 统计当前算法在当前障碍物数量下的结果
        % Success 和 Collision 用所有 Monte Carlo 试验统计。
        % 其余连续指标优先对成功样本统计。
        % 如果没有成功样本，则对全部样本统计，避免输出空值。
        % =====================================================

        success_rate_mat(obs_case_idx, algo_idx) = mean(temp_success) * 100;
        collision_rate_mat(obs_case_idx, algo_idx) = mean(temp_collision) * 100;

        valid_idx = temp_success == 1;

        if ~any(valid_idx)
            valid_idx = true(num_repeats, 1);
        end

        valid_metric_count_mat(obs_case_idx, algo_idx) = sum(valid_idx);

        [trigger_mean_mat(obs_case_idx, algo_idx), trigger_std_mat(obs_case_idx, algo_idx)] = ...
            mean_std_mc(temp_trigger(valid_idx));

        [avg_cpu_mean_mat(obs_case_idx, algo_idx), avg_cpu_std_mat(obs_case_idx, algo_idx)] = ...
            mean_std_mc(temp_avg_cpu(valid_idx));

        [max_cpu_mean_mat(obs_case_idx, algo_idx), max_cpu_std_mat(obs_case_idx, algo_idx)] = ...
            mean_std_mc(temp_max_cpu(valid_idx));

        [clearance_mean_mat(obs_case_idx, algo_idx), clearance_std_mat(obs_case_idx, algo_idx)] = ...
            mean_std_mc(temp_min_clearance(valid_idx));

        [effort_mean_mat(obs_case_idx, algo_idx), effort_std_mat(obs_case_idx, algo_idx)] = ...
            mean_std_mc(temp_effort(valid_idx));

        [climb_mean_mat(obs_case_idx, algo_idx), climb_std_mat(obs_case_idx, algo_idx)] = ...
            mean_std_mc(temp_climb(valid_idx));

        [energy_mean_mat(obs_case_idx, algo_idx), energy_std_mat(obs_case_idx, algo_idx)] = ...
            mean_std_mc(temp_energy_wh(valid_idx));

        [steps_mean_mat(obs_case_idx, algo_idx), steps_std_mat(obs_case_idx, algo_idx)] = ...
            mean_std_mc(temp_steps(valid_idx));

        fprintf('\n当前结果汇总 | Obs = %d | Algo = %s\n', num_obs, algo_name);
        fprintf('Success Rate      : %.1f %%\n', success_rate_mat(obs_case_idx, algo_idx));
        fprintf('Collision Rate    : %.1f %%\n', collision_rate_mat(obs_case_idx, algo_idx));
        fprintf('Trigger Rate      : %s %%\n', meanstd_text_mc(trigger_mean_mat(obs_case_idx, algo_idx), trigger_std_mat(obs_case_idx, algo_idx), 1));
        fprintf('Avg CPU Time      : %s ms\n', meanstd_text_mc(avg_cpu_mean_mat(obs_case_idx, algo_idx), avg_cpu_std_mat(obs_case_idx, algo_idx), 2));
        fprintf('Min Clearance     : %s m\n', meanstd_text_mc(clearance_mean_mat(obs_case_idx, algo_idx), clearance_std_mat(obs_case_idx, algo_idx), 2));
        fprintf('Control Effort    : %s\n', meanstd_text_mc(effort_mean_mat(obs_case_idx, algo_idx), effort_std_mat(obs_case_idx, algo_idx), 2));
        fprintf('Total Climb       : %s m\n', meanstd_text_mc(climb_mean_mat(obs_case_idx, algo_idx), climb_std_mat(obs_case_idx, algo_idx), 2));
        fprintf('Est. Energy       : %s Wh\n', meanstd_text_mc(energy_mean_mat(obs_case_idx, algo_idx), energy_std_mat(obs_case_idx, algo_idx), 4));
        fprintf('Valid Metric Runs : %d / %d\n', valid_metric_count_mat(obs_case_idx, algo_idx), num_repeats);
    end
end

%% =========================================================
% 3. 打印最终汇总表
% =========================================================

disp(' ');
disp('==============================================================');
disp('多动态障碍物可扩展性 Monte Carlo 最终结果');
disp('==============================================================');

for obs_case_idx = 1:num_obs_cases

    num_obs = obs_count_list(obs_case_idx);

    fprintf('\n动态障碍物数量 = %d\n', num_obs);
    fprintf('-----------------------------------------------------------------------------------------------------------------------------\n');
    fprintf('%-18s | Success | Collision | Trigger(mean+/-std) | Avg CPU(mean+/-std) | Clearance(mean+/-std) | Energy(mean+/-std)\n', ...
        'Algorithm');
    fprintf('-----------------------------------------------------------------------------------------------------------------------------\n');

    for algo_idx = 1:num_algos

        fprintf('%-18s | %6.1f%% | %8.1f%% | %18s | %20s | %21s | %18s\n', ...
            algo_names{algo_idx}, ...
            success_rate_mat(obs_case_idx, algo_idx), ...
            collision_rate_mat(obs_case_idx, algo_idx), ...
            [meanstd_text_mc(trigger_mean_mat(obs_case_idx, algo_idx), trigger_std_mat(obs_case_idx, algo_idx), 1), '%'], ...
            [meanstd_text_mc(avg_cpu_mean_mat(obs_case_idx, algo_idx), avg_cpu_std_mat(obs_case_idx, algo_idx), 2), ' ms'], ...
            [meanstd_text_mc(clearance_mean_mat(obs_case_idx, algo_idx), clearance_std_mat(obs_case_idx, algo_idx), 2), ' m'], ...
            [meanstd_text_mc(energy_mean_mat(obs_case_idx, algo_idx), energy_std_mat(obs_case_idx, algo_idx), 4), ' Wh']);
    end

    fprintf('-----------------------------------------------------------------------------------------------------------------------------\n');
end

%% =========================================================
% 4. 保存 CSV 结果
% =========================================================

if params_base.save_csv

    if ~exist(params_base.output_folder, 'dir')
        mkdir(params_base.output_folder);
    end

    % 保存汇总结果
    summary_rows = {};

    for obs_case_idx = 1:num_obs_cases

        for algo_idx = 1:num_algos

            summary_rows = [summary_rows; { ...
                obs_count_list(obs_case_idx), ...
                algo_names{algo_idx}, ...
                num_repeats, ...
                valid_metric_count_mat(obs_case_idx, algo_idx), ...
                success_rate_mat(obs_case_idx, algo_idx), ...
                collision_rate_mat(obs_case_idx, algo_idx), ...
                trigger_mean_mat(obs_case_idx, algo_idx), ...
                trigger_std_mat(obs_case_idx, algo_idx), ...
                meanstd_text_mc(trigger_mean_mat(obs_case_idx, algo_idx), trigger_std_mat(obs_case_idx, algo_idx), 1), ...
                avg_cpu_mean_mat(obs_case_idx, algo_idx), ...
                avg_cpu_std_mat(obs_case_idx, algo_idx), ...
                meanstd_text_mc(avg_cpu_mean_mat(obs_case_idx, algo_idx), avg_cpu_std_mat(obs_case_idx, algo_idx), 2), ...
                max_cpu_mean_mat(obs_case_idx, algo_idx), ...
                max_cpu_std_mat(obs_case_idx, algo_idx), ...
                meanstd_text_mc(max_cpu_mean_mat(obs_case_idx, algo_idx), max_cpu_std_mat(obs_case_idx, algo_idx), 2), ...
                clearance_mean_mat(obs_case_idx, algo_idx), ...
                clearance_std_mat(obs_case_idx, algo_idx), ...
                meanstd_text_mc(clearance_mean_mat(obs_case_idx, algo_idx), clearance_std_mat(obs_case_idx, algo_idx), 2), ...
                effort_mean_mat(obs_case_idx, algo_idx), ...
                effort_std_mat(obs_case_idx, algo_idx), ...
                meanstd_text_mc(effort_mean_mat(obs_case_idx, algo_idx), effort_std_mat(obs_case_idx, algo_idx), 2), ...
                climb_mean_mat(obs_case_idx, algo_idx), ...
                climb_std_mat(obs_case_idx, algo_idx), ...
                meanstd_text_mc(climb_mean_mat(obs_case_idx, algo_idx), climb_std_mat(obs_case_idx, algo_idx), 2), ...
                energy_mean_mat(obs_case_idx, algo_idx), ...
                energy_std_mat(obs_case_idx, algo_idx), ...
                meanstd_text_mc(energy_mean_mat(obs_case_idx, algo_idx), energy_std_mat(obs_case_idx, algo_idx), 4), ...
                steps_mean_mat(obs_case_idx, algo_idx), ...
                steps_std_mat(obs_case_idx, algo_idx), ...
                meanstd_text_mc(steps_mean_mat(obs_case_idx, algo_idx), steps_std_mat(obs_case_idx, algo_idx), 2) ...
            }]; %#ok<AGROW>
        end
    end

    summary_table = cell2table(summary_rows, ...
        'VariableNames', { ...
        'NumDynamicObstacles', ...
        'Algorithm', ...
        'NumRepeats', ...
        'NumRunsForContinuousMetrics', ...
        'SuccessRatePercent', ...
        'CollisionRatePercent', ...
        'TriggerRateMeanPercent', ...
        'TriggerRateStdPercent', ...
        'TriggerRateMeanStdText', ...
        'AvgCPUmeanMs', ...
        'AvgCPUstdMs', ...
        'AvgCPUMeanStdText', ...
        'MaxCPUmeanMs', ...
        'MaxCPUstdMs', ...
        'MaxCPUMeanStdText', ...
        'ClearanceMeanMeter', ...
        'ClearanceStdMeter', ...
        'ClearanceMeanStdText', ...
        'EffortMean', ...
        'EffortStd', ...
        'EffortMeanStdText', ...
        'ClimbMeanMeter', ...
        'ClimbStdMeter', ...
        'ClimbMeanStdText', ...
        'EstEnergyMeanWh', ...
        'EstEnergyStdWh', ...
        'EstEnergyMeanStdText', ...
        'StepsMean', ...
        'StepsStd', ...
        'StepsMeanStdText'});

    summary_file = fullfile(params_base.output_folder, params_base.scalability_result_file);
    writetable(summary_table, summary_file);

    disp(' ');
    disp(['可扩展性汇总结果已保存：', summary_file]);

    % 保存每次 Monte Carlo 的详细结果
    detail_table = cell2table(detail_rows, ...
        'VariableNames', { ...
        'NumDynamicObstacles', ...
        'Algorithm', ...
        'MonteCarloIndex', ...
        'ScenarioSeed', ...
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
        'EstEnergyWh', ...
        'Steps'});

    detail_file = fullfile(params_base.output_folder, 'scalability_multi_obstacle_detail_results.csv');
    writetable(detail_table, detail_file);

    disp(['可扩展性详细结果已保存：', detail_file]);
else

    disp(' ');
    disp('当前 params_base.save_csv = false，未保存 CSV 文件。');
end

%% =========================================================
% 5. 跳过可扩展性分析图生成
% =========================================================
% 说明：
% 本版本只需要 Monte Carlo 数值数据，因此不再调用 plot_scalability_results。
% 如果以后需要重新生成图片，可以把下面的注释取消，并设置：
% params_base.save_figures = true;
%
% plot_scalability_results( ...
%     obs_count_list, ...
%     algo_names, ...
%     success_rate_mat, ...
%     collision_rate_mat, ...
%     trigger_mean_mat, ...
%     avg_cpu_mean_mat, ...
%     clearance_mean_mat, ...
%     energy_mean_mat, ...
%     params_base);
% ==============================================================

disp(' ');
disp('已按要求跳过可扩展性图片生成，仅保存 Monte Carlo 数据 CSV。');

disp(' ');
disp('==============================================================');
disp('多动态障碍物可扩展性 Monte Carlo 分析全部完成');
disp('==============================================================');

%% =========================================================================
% 局部函数 1：单次 Monte Carlo 仿真
% =========================================================================

function result = run_single_monte_carlo_simulation( ...
    params, p_0, v_0, p_goal, path_dir, path_len, buildings, scenario_seed)

    % 固定当前场景随机种子
    rng(scenario_seed);

    % 初始化无人机状态
    x_curr = [p_0; v_0];

    % 初始化多个动态障碍物
    obs_array = initialize_dynamic_obstacles_mc(params, p_0, p_goal);

    num_obs = length(obs_array);

    % 初始化事件触发变量
    k = 0;
    k_last = -params.T_ref;
    sigma_k = 0;
    sigma_prev = 0;

    % 初始化记录数组
    history_x = zeros(6, params.k_max);
    history_u = zeros(3, params.k_max);
    history_sigma = zeros(1, params.k_max);
    history_t_solve = zeros(1, params.k_max);
    history_obs = zeros(3, params.k_max, num_obs);

    % 初始化任务状态
    collision_flag = false;
    success_flag = false;

    % 主循环
    while norm(x_curr(1:3) - p_goal) > params.eps_tol && k < params.k_max

        k = k + 1;
        current_time = k * params.dt;

        % 记录无人机状态
        history_x(:, k) = x_curr;

        % 更新动态障碍物真实运动
        obs_array = update_dynamic_obstacles_mc(obs_array, params, current_time);

        for i = 1:num_obs
            history_obs(:, k, i) = obs_array(i).p;
        end

        % 动态障碍物碰撞检测
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

        % 静态建筑物碰撞检测
        if params.check_static_collision
            if check_static_collision_mc(x_curr, buildings, params)
                collision_flag = true;
                break;
            end
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

        % 构造动态障碍物观测数据
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

        % 卡尔曼预测
        obs_pred = kalman_predict(obs_data, x_curr, params);

        % 预测消融逻辑：NP-ET-NMPC 使用当前障碍物位置作为未来位置
        if ~params.use_prediction
            for i = 1:length(obs_pred)
                obs_pred(i).pos = repmat(obs_pred(i).pos(:, 1), 1, params.H);
                obs_pred(i).vel = zeros(3, params.H);
                obs_pred(i).inflation = zeros(1, params.H);
            end
        end

        % 计算复合风险
        [R_k, ~, ~, ~] = compute_risk(x_curr, x_ref, obs_pred, buildings, params);

        % 事件触发逻辑
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

        % 求解控制输入并计时
        t_start = tic;

        if sigma_k == 1

            [u_cmd, ~, ~] = solve_nmpc_casadi( ...
                x_curr, Pi_ref_local, obs_pred, buildings, params);

            k_last = k;

        else

            % 低风险模式：轻量级 PD 跟踪控制
            e_p = x_ref(1:3) - x_curr(1:3);
            e_v = x_ref(4:6) - x_curr(4:6);

            if dist_to_goal < 2.0
                u_cmd_raw = 2.0 * e_p + 3.0 * e_v;
            else
                u_cmd_raw = 1.5 * e_p + 2.5 * e_v;
            end

            u_cmd = limit_vector_norm_mc(u_cmd_raw, params.u_max);
        end

        history_t_solve(k) = toc(t_start);

        % 状态更新
        x_next = zeros(6, 1);
        x_next(1:3) = x_curr(1:3) + x_curr(4:6) * params.dt;
        x_next(4:6) = x_curr(4:6) + u_cmd * params.dt;

        % 速度限幅
        x_next(4:6) = limit_vector_norm_mc(x_next(4:6), params.v_max);

        % 高度限幅
        x_next(3) = max(min(x_next(3), params.z_max), params.z_min);

        % 记录
        history_u(:, k) = u_cmd;
        history_sigma(k) = sigma_k;

        % 更新状态
        x_curr = x_next;
        sigma_prev = sigma_k;
    end

    % 任务成功判断
    if ~collision_flag && norm(x_curr(1:3) - p_goal) <= params.eps_tol
        success_flag = true;
    end

    valid_k = max(1, k);

    % 统计触发率和 CPU
    trigger_rate = sum(history_sigma(1:valid_k)) / valid_k * 100;
    avg_cpu = mean(history_t_solve(1:valid_k)) * 1000;
    max_cpu = max(history_t_solve(1:valid_k)) * 1000;
    std_cpu = std(history_t_solve(1:valid_k)) * 1000;

    % 统计最小动态障碍距离
    [min_dist_dyn, min_clearance_dyn] = compute_dynamic_distance_metrics_mc( ...
        history_x, history_obs, obs_array, valid_k, params);

    % 统计控制努力、爬升和估计能耗
    [control_effort, total_climb, est_energy_wh] = compute_energy_metrics_mc( ...
        history_x, history_u, valid_k, params);

    % 输出结构体
    result.success = success_flag;
    result.collision = collision_flag;

    result.trigger_rate = trigger_rate;
    result.avg_cpu = avg_cpu;
    result.max_cpu = max_cpu;
    result.std_cpu = std_cpu;

    result.min_dist_dyn = min_dist_dyn;
    result.min_clearance_dyn = min_clearance_dyn;

    result.control_effort = control_effort;
    result.total_climb = total_climb;
    result.est_energy_wh = est_energy_wh;

    result.steps = valid_k;
end

%% =========================================================================
% 局部函数 2：初始化多个动态障碍物
% =========================================================================

function obs_array = initialize_dynamic_obstacles_mc(params, p_0, p_goal)

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

    % 动态障碍物沿路径不同位置分布
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

        % 让障碍物大致朝向无人机名义路径运动
        to_path = center_on_path - p_init;

        if norm(to_path) < 1e-6
            v_dir = -side_sign * lateral_dir_1;
        else
            v_dir = to_path / norm(to_path);
        end

        speed_i = 0.7 + 0.7 * rand;
        v_init = speed_i * v_dir;
        v_init(3) = 0.2 * (rand - 0.5);

        % 设置障碍物半径
        if i == 1
            % 保留原始论文中的典型横穿障碍物
            p_init = [14; 30; 8] + [randn * 0.8; randn * 1.5; randn * 0.8];
            v_init = [0.8; -0.8; 0] + [randn * 0.15; randn * 0.15; randn * 0.05];
            r_i = params.large_obs_radius;
        else
            r_i = params.default_obs_radius * (0.85 + 0.3 * rand);
        end

        % 设置运动模式
        if params.use_heterogeneous_motion
            mode_list = params.heterogeneous_motion_types;
            motion_type = mode_list(mod(i - 1, length(mode_list)) + 1);
        else
            motion_type = params.default_motion_type;
        end

        % 单障碍物场景时，使用 CV，作为与原始实验接近的基础情况
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
% 局部函数 3：更新多个动态障碍物真实状态
% =========================================================================

function obs_array = update_dynamic_obstacles_mc(obs_array, params, current_time)

    for i = 1:length(obs_array)

        motion_type = obs_array(i).motion_type;

        if motion_type == params.motion_type_CV

            % CV：匀速直线运动
            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;

        elseif motion_type == params.motion_type_CA

            % CA：带恒定加速度运动
            acc = params.obs_ca_acc * obs_array(i).acc_dir;

            obs_array(i).v = obs_array(i).v + acc * params.dt;
            obs_array(i).v = limit_vector_norm_mc(obs_array(i).v, params.obs_v_max);

            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;

        elseif motion_type == params.motion_type_SIN

            % SIN：带正弦横向和垂直扰动的非线性运动
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
            obs_array(i).v = limit_vector_norm_mc(obs_array(i).v, params.obs_v_max);

        elseif motion_type == params.motion_type_RAND

            % RAND：随机机动或不可预测运动
            if params.use_unpredictable_motion

                acc_random = params.obs_random_acc_std * randn(3, 1);

                if norm(acc_random) > params.obs_random_acc_max
                    acc_random = acc_random / norm(acc_random) * params.obs_random_acc_max;
                end

                obs_array(i).v = obs_array(i).v + acc_random * params.dt;

                % 以一定概率发生突然速度变化
                if rand < params.obs_maneuver_prob
                    dv = 2 * rand(3, 1) - 1;

                    if norm(dv) > 1e-6
                        dv = dv / norm(dv) * params.obs_velocity_jump_max;
                    end

                    obs_array(i).v = obs_array(i).v + dv;
                end
            end

            obs_array(i).v = limit_vector_norm_mc(obs_array(i).v, params.obs_v_max);
            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;

        else

            % 未知模式默认使用 CV
            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;
        end

        % 限制动态障碍物高度，避免跑出场景
        obs_array(i).p(3) = max(min(obs_array(i).p(3), params.z_max - 0.5), params.z_min + 0.5);
    end
end

%% =========================================================================
% 局部函数 4：计算动态障碍物距离指标
% =========================================================================

function [min_dist_dyn, min_clearance_dyn] = compute_dynamic_distance_metrics_mc( ...
    history_x, history_obs, obs_array, valid_k, params)

    num_obs = length(obs_array);

    min_dist_each = inf(1, num_obs);
    min_clearance_each = inf(1, num_obs);

    for i = 1:num_obs

        pos_uav = history_x(1:3, 1:valid_k);
        pos_obs = squeeze(history_obs(:, 1:valid_k, i));

        if size(pos_obs, 1) ~= 3
            pos_obs = reshape(pos_obs, 3, []);
        end

        dist_i = sqrt(sum((pos_uav - pos_obs).^2, 1));

        min_dist_each(i) = min(dist_i);
        min_clearance_each(i) = min_dist_each(i) - params.r_u - obs_array(i).r_i;
    end

    min_dist_dyn = min(min_dist_each);
    min_clearance_dyn = min(min_clearance_each);
end

%% =========================================================================
% 局部函数 5：计算控制努力、爬升和估计能耗
% =========================================================================

function [control_effort, total_climb, est_energy_wh] = compute_energy_metrics_mc( ...
    history_x, history_u, valid_k, params)

    if valid_k <= 1
        control_effort = 0;
        total_climb = 0;
        est_energy_wh = 0;
        return;
    end

    u_hist = history_u(:, 1:valid_k);
    x_hist = history_x(:, 1:valid_k);

    % 控制努力：离散控制输入平方和
    control_effort = sum(sum(u_hist.^2, 1)) * params.dt;

    % 正向爬升高度
    z_hist = x_hist(3, :);
    dz = diff(z_hist);
    total_climb = sum(max(0, dz));

    % 简化物理估计能耗
    T_total = valid_k * params.dt;
    E_est = params.P_base * T_total ...
        + (params.uav_mass * params.gravity * total_climb) / params.climb_efficiency ...
        + params.k_u_energy * control_effort;

    est_energy_wh = E_est / 3600;
end

%% =========================================================================
% 局部函数 6：静态建筑物碰撞检测
% =========================================================================

function flag = check_static_collision_mc(x_curr, buildings, params)

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
% 局部函数 7：绘制可扩展性结果图
% =========================================================================
% 说明：
% 当前 main_monte_carlo.m 默认不调用该函数。
% 保留该函数只是为了兼容以后需要重新生成图片的情况。
% 如果需要图片，请在主程序第 5 节取消注释调用，并设置：
% params_base.save_figures = true;
% =========================================================================

function plot_scalability_results( ...
    obs_count_list, ...
    algo_names, ...
    success_rate_mat, ...
    collision_rate_mat, ...
    trigger_mean_mat, ...
    avg_cpu_mean_mat, ...
    clearance_mean_mat, ...
    energy_mean_mat, ...
    params)

    if ~params.save_figures
        return;
    end

    if ~exist(params.output_folder, 'dir')
        mkdir(params.output_folder);
    end

    % ---------------------------------------------------------
    % 图 1：平均 CPU 时间随动态障碍物数量变化
    % ---------------------------------------------------------
    fig_cpu = figure('Name', 'Scalability CPU Time', 'Color', 'w');

    hold on; grid on;

    for a = 1:length(algo_names)
        plot(obs_count_list, avg_cpu_mean_mat(:, a), ...
            '-o', 'LineWidth', 2.0, 'MarkerSize', 8, ...
            'DisplayName', algo_names{a});
    end

    xlabel('Number of Dynamic Obstacles', 'FontSize', 13);
    ylabel('Average CPU Time (ms)', 'FontSize', 13);
    title('Computational Scalability versus Obstacle Number', 'FontSize', 14);
    legend('show', 'Location', 'northwest');
    set(gca, 'FontSize', 12);

    file_cpu = fullfile(params.output_folder, 'Scalability_CPU_Time.png');
    exportgraphics(fig_cpu, file_cpu, 'Resolution', params.figure_resolution);

    % ---------------------------------------------------------
    % 图 2：触发率随动态障碍物数量变化
    % ---------------------------------------------------------
    fig_trigger = figure('Name', 'Scalability Trigger Rate', 'Color', 'w');

    hold on; grid on;

    for a = 1:length(algo_names)
        plot(obs_count_list, trigger_mean_mat(:, a), ...
            '-s', 'LineWidth', 2.0, 'MarkerSize', 8, ...
            'DisplayName', algo_names{a});
    end

    xlabel('Number of Dynamic Obstacles', 'FontSize', 13);
    ylabel('Average Trigger Rate (%)', 'FontSize', 13);
    title('Trigger Rate versus Obstacle Number', 'FontSize', 14);
    legend('show', 'Location', 'northwest');
    set(gca, 'FontSize', 12);

    file_trigger = fullfile(params.output_folder, 'Scalability_Trigger_Rate.png');
    exportgraphics(fig_trigger, file_trigger, 'Resolution', params.figure_resolution);

    % ---------------------------------------------------------
    % 图 3：最小净安全间隔随动态障碍物数量变化
    % ---------------------------------------------------------
    fig_clearance = figure('Name', 'Scalability Clearance', 'Color', 'w');

    hold on; grid on;

    for a = 1:length(algo_names)
        plot(obs_count_list, clearance_mean_mat(:, a), ...
            '-^', 'LineWidth', 2.0, 'MarkerSize', 8, ...
            'DisplayName', algo_names{a});
    end

    h_boundary = yline(0, 'r--', 'Collision Boundary', 'LineWidth', 1.5);
    h_boundary.HandleVisibility = 'off';

    xlabel('Number of Dynamic Obstacles', 'FontSize', 13);
    ylabel('Minimum Clearance (m)', 'FontSize', 13);
    title('Safety Margin versus Obstacle Number', 'FontSize', 14);
    legend('show', 'Location', 'southwest');
    set(gca, 'FontSize', 12);

    file_clearance = fullfile(params.output_folder, 'Scalability_Min_Clearance.png');
    exportgraphics(fig_clearance, file_clearance, 'Resolution', params.figure_resolution);

    % ---------------------------------------------------------
    % 图 4：成功率和碰撞率
    % ---------------------------------------------------------
    fig_success = figure('Name', 'Scalability Success Collision', 'Color', 'w');

    tiledlayout(2, 1);

    nexttile;
    hold on; grid on;

    for a = 1:length(algo_names)
        plot(obs_count_list, success_rate_mat(:, a), ...
            '-o', 'LineWidth', 2.0, 'MarkerSize', 8, ...
            'DisplayName', algo_names{a});
    end

    ylabel('Success Rate (%)', 'FontSize', 13);
    title('Success Rate versus Obstacle Number', 'FontSize', 14);
    ylim([0, 105]);
    legend('show', 'Location', 'southwest');
    set(gca, 'FontSize', 12);

    nexttile;
    hold on; grid on;

    for a = 1:length(algo_names)
        plot(obs_count_list, collision_rate_mat(:, a), ...
            '-s', 'LineWidth', 2.0, 'MarkerSize', 8, ...
            'DisplayName', algo_names{a});
    end

    xlabel('Number of Dynamic Obstacles', 'FontSize', 13);
    ylabel('Collision Rate (%)', 'FontSize', 13);
    title('Collision Rate versus Obstacle Number', 'FontSize', 14);
    ylim([0, 105]);
    set(gca, 'FontSize', 12);

    file_success = fullfile(params.output_folder, 'Scalability_Success_Collision.png');
    exportgraphics(fig_success, file_success, 'Resolution', params.figure_resolution);

    % ---------------------------------------------------------
    % 图 5：估计能耗随动态障碍物数量变化
    % ---------------------------------------------------------
    fig_energy = figure('Name', 'Scalability Energy', 'Color', 'w');

    hold on; grid on;

    for a = 1:length(algo_names)
        plot(obs_count_list, energy_mean_mat(:, a), ...
            '-d', 'LineWidth', 2.0, 'MarkerSize', 8, ...
            'DisplayName', algo_names{a});
    end

    xlabel('Number of Dynamic Obstacles', 'FontSize', 13);
    ylabel('Estimated Energy (Wh)', 'FontSize', 13);
    title('Estimated Energy versus Obstacle Number', 'FontSize', 14);
    legend('show', 'Location', 'northwest');
    set(gca, 'FontSize', 12);

    file_energy = fullfile(params.output_folder, 'Scalability_Energy.png');
    exportgraphics(fig_energy, file_energy, 'Resolution', params.figure_resolution);

    disp(['可扩展性 CPU 图已保存：', file_cpu]);
    disp(['可扩展性触发率图已保存：', file_trigger]);
    disp(['可扩展性安全间隔图已保存：', file_clearance]);
    disp(['可扩展性成功率碰撞率图已保存：', file_success]);
    disp(['可扩展性能耗图已保存：', file_energy]);
end

%% =========================================================================
% 局部函数 8：设置算法模式
% =========================================================================

function params = apply_algorithm_mode_mc(params, algo_name)

    % 恢复默认关键开关
    params.use_event_trigger = true;
    params.use_prediction = true;
    params.use_asymmetric_energy = true;

    if strcmp(algo_name, 'Proposed ET-NMPC')

        params.use_event_trigger = true;
        params.use_prediction = true;
        params.use_asymmetric_energy = true;

    elseif strcmp(algo_name, 'C-NMPC')

        % C-NMPC：连续 NMPC，每一步都求解
        params.use_event_trigger = false;
        params.use_prediction = true;
        params.use_asymmetric_energy = true;

    elseif strcmp(algo_name, 'NP-ET-NMPC')

        % NP-ET-NMPC：事件触发开启，但关闭动态预测
        params.use_event_trigger = true;
        params.use_prediction = false;
        params.use_asymmetric_energy = true;

    elseif strcmp(algo_name, 'SE-ET-NMPC')

        % SE-ET-NMPC：事件触发和预测保留，但关闭非对称爬升惩罚
        params.use_event_trigger = true;
        params.use_prediction = true;
        params.use_asymmetric_energy = false;

        % 如果求解器直接读取 c3，则将其置零，避免残留非对称爬升惩罚。
        if isfield(params, 'c3')
            params.c3 = 0;
        end

    else
        error('未知算法名称：%s', algo_name);
    end
end

%% =========================================================================
% 局部函数 9：向量范数限幅
% =========================================================================

function v_limited = limit_vector_norm_mc(v, max_norm)

    if norm(v) > max_norm
        v_limited = v / norm(v) * max_norm;
    else
        v_limited = v;
    end
end

%% =========================================================================
% 局部函数 10：均值和标准差
% =========================================================================

function [m, s] = mean_std_mc(x)

    x = x(:);
    x = x(~isnan(x) & ~isinf(x));

    if isempty(x)
        m = NaN;
        s = NaN;
    elseif numel(x) == 1
        m = mean(x);
        s = 0;
    else
        m = mean(x);
        s = std(x);
    end
end

%% =========================================================================
% 局部函数 11：生成 mean +/- std 字符串
% =========================================================================

function str = meanstd_text_mc(m, s, precision)

    if nargin < 3
        precision = 2;
    end

    if isnan(m) || isnan(s)
        str = 'NaN';
        return;
    end

    fmt = ['%.', num2str(precision), 'f +/- %.', num2str(precision), 'f'];
    str = sprintf(fmt, m, s);
end

%% =========================================================================
% 局部函数 12：补充缺失参数
% =========================================================================

function params = complete_params_for_monte_carlo(params)

    if ~isfield(params, 'p_0')
        params.p_0 = [0; 0; 3];
    end

    if ~isfield(params, 'v_0')
        params.v_0 = [0; 0; 0];
    end

    if ~isfield(params, 'p_goal')
        params.p_goal = [40; 40; 12];
    end

    if ~isfield(params, 'use_multi_obstacles')
        params.use_multi_obstacles = true;
    end

    if ~isfield(params, 'num_dynamic_obstacles')
        params.num_dynamic_obstacles = 5;
    end

    % 本实验只采用 1、3、5 个动态障碍物，不采用 8 个动态障碍物。
    params.scalability_obs_list = [1, 3, 5];

    if ~isfield(params, 'num_scalability_repeats')
        % 正式论文建议每组至少 30 次；如果调试，可手动改小。
        params.num_scalability_repeats = 100;
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

    if ~isfield(params, 'save_csv')
        params.save_csv = true;
    end

    % Monte Carlo 数据实验默认不保存图片。
    params.save_figures = false;

    if ~isfield(params, 'figure_resolution')
        params.figure_resolution = 300;
    end

    if ~isfield(params, 'output_folder')
        params.output_folder = 'results_reviewer_response';
    end

    if ~isfield(params, 'scalability_result_file')
        params.scalability_result_file = 'scalability_multi_obstacle_results.csv';
    end

    if ~isfield(params, 'random_seed') || isempty(params.random_seed)
        params.random_seed = 2026;
    end

    % 能耗估计参数
    if ~isfield(params, 'P_base')
        params.P_base = 45.0;             % W
    end

    if ~isfield(params, 'uav_mass')
        params.uav_mass = 1.2;            % kg
    end

    if ~isfield(params, 'gravity')
        params.gravity = 9.81;            % m/s^2
    end

    if ~isfield(params, 'climb_efficiency')
        params.climb_efficiency = 0.70;
    end

    if ~isfield(params, 'k_u_energy')
        params.k_u_energy = 0.08;
    end

    if ~exist(params.output_folder, 'dir')
        mkdir(params.output_folder);
    end
end
