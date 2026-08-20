% ==============================================================
% main_proposed_et_nmpc_profiling.m
%
% Proposed ET-NMPC 主场景 30 次运行时剖析脚本
%
% 本脚本用于回应审稿意见 “Computational Performance Disclosure”。
% 它只测试 Proposed ET-NMPC，在主复杂动态城市场景下运行 30 次，记录：
% 1. 每一步 CPU time；
% 2. 触发步 CPU time；
% 3. 每次 IPOPT iteration；
% 4. 最大 / 平均 IPOPT iteration；
% 5. solver failure / fallback 次数；
% 6. MATLAB 运行前后内存，以及运行中观测到的最大内存；
% 7. trigger rate。
%
% 运行要求：
% 1. 当前文件应与 init_params.m、generate_city_map.m、kalman_predict.m、
%    compute_risk.m 等文件位于同一 MATLAB 路径下；
% 2. 当前文件内置了一个 solve_nmpc_casadi_profile()，不会覆盖你原来的
%    solve_nmpc_casadi.m；
% 3. 运行结束后会生成两个 CSV 文件：
%    - profiling_step_detail_results.csv
%    - profiling_summary_results.csv
% ============================================================== 

clear; clc; close all;

%% =========================================================
% 0. 实验设置
% =========================================================

num_runs = 30;
base_seed = 20260615;

output_folder = fullfile(pwd, 'results_proposed_et_nmpc_profiling');
if ~exist(output_folder, 'dir')
    mkdir(output_folder);
end

% IPOPT / CasADi 设置。论文中可如实写入这些设置。
profile_solver_settings = struct();
profile_solver_settings.ipopt_max_iter = 100;
profile_solver_settings.ipopt_tol = 1e-4;
profile_solver_settings.ipopt_acceptable_tol = 1e-3;
profile_solver_settings.ipopt_print_level = 0;
profile_solver_settings.ipopt_linear_solver = 'mumps';
profile_solver_settings.casadi_expand = true;

fprintf('==============================================================\n');
fprintf('Proposed ET-NMPC 运行时剖析实验开始\n');
fprintf('独立运行次数：%d\n', num_runs);
fprintf('输出文件夹：%s\n', output_folder);
fprintf('==============================================================\n');

all_step_rows = table();
all_run_rows = table();

%% =========================================================
% 1. 30 次代表性实验
% =========================================================

for run_id = 1:num_runs

    fprintf('\n==============================================================\n');
    fprintf('运行 Proposed ET-NMPC profiling trial %d / %d\n', run_id, num_runs);
    fprintf('==============================================================\n');

    rng(base_seed + run_id);

    mem_before = get_matlab_memory_mb();

    [run_result, step_table] = run_single_proposed_et_nmpc_profile( ...
        run_id, base_seed + run_id, output_folder, profile_solver_settings);

    mem_after = get_matlab_memory_mb();

    run_result.MemoryBeforeMB = mem_before;
    run_result.MemoryAfterMB = mem_after;
    run_result.MemoryDeltaMB = mem_after - mem_before;

    all_step_rows = [all_step_rows; step_table]; %#ok<AGROW>
    all_run_rows = [all_run_rows; struct2table(run_result)]; %#ok<AGROW>

    fprintf('Run %d finished | Success = %d | Collision = %d | Trigger = %.1f%% | Avg CPU = %.2f ms | Avg IPOPT Iter = %.2f | Fallback = %d\n', ...
        run_id, run_result.Success, run_result.Collision, run_result.TriggerRatePercent, ...
        run_result.AvgCPUms, run_result.AvgIPOPTIterations, run_result.FallbackCount);
end

%% =========================================================
% 2. 汇总统计
% =========================================================

triggered_rows = all_step_rows(all_step_rows.Sigma == 1, :);
valid_iter_rows = triggered_rows(~isnan(triggered_rows.IPOPTIterations), :);

summary = table();
summary.NumRuns = num_runs;
summary.TotalSteps = height(all_step_rows);
summary.TotalTriggeredSteps = height(triggered_rows);
summary.SuccessRatePercent = mean(all_run_rows.Success) * 100;
summary.CollisionRatePercent = mean(all_run_rows.Collision) * 100;
summary.TriggerRateMeanPercent = mean(all_run_rows.TriggerRatePercent);
summary.TriggerRateStdPercent = std(all_run_rows.TriggerRatePercent);
summary.AvgCPUPerStepMean_ms = mean(all_run_rows.AvgCPUms);
summary.AvgCPUPerStepStd_ms = std(all_run_rows.AvgCPUms);
summary.MaxCPUPerStep_ms = max(all_run_rows.MaxCPUms);
summary.StdCPUPerStepMean_ms = mean(all_run_rows.StdCPUms);
summary.AvgTriggeredCPU_ms = mean(triggered_rows.CPUTimeMs, 'omitnan');
summary.StdTriggeredCPU_ms = std(triggered_rows.CPUTimeMs, 'omitnan');
summary.MaxTriggeredCPU_ms = max(triggered_rows.CPUTimeMs);
summary.AvgIPOPTIterations = mean(valid_iter_rows.IPOPTIterations, 'omitnan');
summary.StdIPOPTIterations = std(valid_iter_rows.IPOPTIterations, 'omitnan');
summary.MaxIPOPTIterations = max(valid_iter_rows.IPOPTIterations);
summary.TotalFallbackCount = sum(all_run_rows.FallbackCount);
summary.TotalSolverFailureCount = sum(all_run_rows.SolverFailureCount);
summary.MemoryBeforeMeanMB = mean(all_run_rows.MemoryBeforeMB, 'omitnan');
summary.MemoryAfterMeanMB = mean(all_run_rows.MemoryAfterMB, 'omitnan');
summary.MemoryDeltaMeanMB = mean(all_run_rows.MemoryDeltaMB, 'omitnan');
summary.PeakObservedMemoryMB = max(all_run_rows.PeakMemoryMB);
summary.IPOPTMaxIterSetting = profile_solver_settings.ipopt_max_iter;
summary.IPOPTTolSetting = profile_solver_settings.ipopt_tol;
summary.IPOPTAcceptableTolSetting = profile_solver_settings.ipopt_acceptable_tol;
summary.IPOPTLinearSolver = string(profile_solver_settings.ipopt_linear_solver);
summary.WarmStart = string('Shifted reference initialization');

%% =========================================================
% 3. 保存 CSV
% =========================================================

step_file = fullfile(output_folder, 'profiling_step_detail_results.csv');
run_file = fullfile(output_folder, 'profiling_run_results.csv');
summary_file = fullfile(output_folder, 'profiling_summary_results.csv');

writetable(all_step_rows, step_file);
writetable(all_run_rows, run_file);
writetable(summary, summary_file);

fprintf('\n==============================================================\n');
fprintf('Proposed ET-NMPC 运行时剖析实验完成\n');
fprintf('逐步详情文件：%s\n', step_file);
fprintf('单次运行汇总：%s\n', run_file);
fprintf('总汇总文件：%s\n', summary_file);
fprintf('==============================================================\n');

disp(summary);

%% =========================================================
% 4. 单次运行函数
% =========================================================

function [run_result, step_table] = run_single_proposed_et_nmpc_profile( ...
    run_id, random_seed, output_folder, solver_settings)

    params = init_params();
    params = complete_params_for_main_simulation(params);

    % Proposed ET-NMPC 设置
    params.use_event_trigger = true;
    params.use_prediction = true;
    params.use_asymmetric_energy = true;
    params.use_heterogeneous_motion = true;
    params.use_unpredictable_motion = true;
    params.use_measurement_noise = true;

    % 关闭绘图和文件保存，加快 profiling
    params.enable_animation = false;
    params.save_figures = false;
    params.save_csv = false;
    params.output_folder = output_folder;
    params.random_seed = random_seed;

    % IPOPT 设置
    params.ipopt_max_iter = solver_settings.ipopt_max_iter;
    params.ipopt_tol = solver_settings.ipopt_tol;
    params.ipopt_acceptable_tol = solver_settings.ipopt_acceptable_tol;
    params.ipopt_print_level = solver_settings.ipopt_print_level;
    params.ipopt_linear_solver = solver_settings.ipopt_linear_solver;
    params.casadi_expand = solver_settings.casadi_expand;

    rng(random_seed);

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
    ]; %#ok<NASGU>

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
    static_hit_info = struct(); %#ok<NASGU>

    history_x = zeros(6, params.k_max);
    history_u = zeros(3, params.k_max);
    history_R = zeros(1, params.k_max);
    history_sigma = zeros(1, params.k_max);
    history_t_solve = zeros(1, params.k_max);
    history_obs = zeros(3, params.k_max, num_obs);

    step_RunID = zeros(params.k_max, 1);
    step_k = zeros(params.k_max, 1);
    step_sigma = zeros(params.k_max, 1);
    step_R = zeros(params.k_max, 1);
    step_cpu_ms = zeros(params.k_max, 1);
    step_triggered_cpu_ms = nan(params.k_max, 1);
    step_ipopt_iter = nan(params.k_max, 1);
    step_solver_success = false(params.k_max, 1);
    step_fallback = false(params.k_max, 1);
    step_return_status = strings(params.k_max, 1);
    step_memory_mb = nan(params.k_max, 1);

    peak_memory_mb = get_matlab_memory_mb();
    solver_failure_count = 0;
    fallback_count = 0;

    while norm(x_curr(1:3) - p_goal) > params.eps_tol && k < params.k_max

        k = k + 1;
        current_time = k * params.dt;
        history_x(:, k) = x_curr;

        %% 更新动态障碍物
        obs_array = update_dynamic_obstacles(obs_array, params, current_time);
        for i = 1:num_obs
            history_obs(:, k, i) = obs_array(i).p;
        end

        %% 动态碰撞检测
        if params.check_dynamic_collision
            for i = 1:num_obs
                dist_to_obs_i = norm(x_curr(1:3) - obs_array(i).p);
                collision_dist_i = params.r_u + obs_array(i).r_i + params.collision_buffer;
                if dist_to_obs_i <= collision_dist_i
                    collision_flag = true;
                    break;
                end
            end
        end
        if collision_flag
            break;
        end

        %% 静态建筑物碰撞检测
        if params.check_static_collision
            [static_collision, hit_id, hit_info] = check_static_collision_precise( ...
                x_curr, buildings, params);
            if static_collision
                collision_flag = true;
                static_hit_id = hit_id;
                static_hit_info = hit_info; %#ok<NASGU>
                break;
            end
        end

        %% 构造局部参考轨迹
        dist_to_goal = norm(p_goal - x_curr(1:3));
        goal_direct_radius = params.goal_direct_radius;
        goal_slow_radius = params.goal_slow_radius;
        goal_dir = p_goal - x_curr(1:3);

        if norm(goal_dir) > 1e-6
            goal_dir_unit = goal_dir / norm(goal_dir);
        else
            goal_dir_unit = zeros(3, 1);
        end

        line_to_goal_clear = check_line_to_goal_clear( ...
            x_curr(1:3), p_goal, buildings, obs_array, params);

        s_curr_for_goal = dot(x_curr(1:3) - p_0, path_dir);
        s_curr_for_goal = max(0, min(s_curr_for_goal, path_len));
        path_progress = s_curr_for_goal / path_len;

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

        %% 生成带噪声动态观测
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

        %% 卡尔曼预测
        obs_pred = kalman_predict(obs_data, x_curr, params);

        %% 复合风险计算
        [R_k, ~, ~, ~] = compute_risk(x_curr, x_ref, obs_pred, buildings, params);

        %% 事件触发
        if params.use_event_trigger
            if R_k <= params.R_off
                sigma_k = 0;
            elseif use_direct_goal_guidance
                sigma_k = 1;
            elseif (R_k >= params.R_on) && ((k - k_last) >= params.T_ref)
                sigma_k = 1;
            else
                sigma_k = sigma_prev;
            end
        else
            sigma_k = 1;
        end

        %% 计算控制输入并记录 CPU / IPOPT
        solver_info = struct();
        solver_info.success = true;
        solver_info.fallback_used = false;
        solver_info.iter_count = NaN;
        solver_info.return_status = "not_triggered";

        t_start = tic;
        if sigma_k == 1
            [u_cmd, ~, ~, solver_info] = solve_nmpc_casadi_profile( ...
                x_curr, Pi_ref_local, obs_pred, buildings, params);
            k_last = k;
            if ~solver_info.success
                solver_failure_count = solver_failure_count + 1;
            end
            if solver_info.fallback_used
                fallback_count = fallback_count + 1;
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
        end
        cpu_sec = toc(t_start);
        history_t_solve(k) = cpu_sec;

        %% UAV 状态更新
        x_next = zeros(6, 1);
        x_next(1:3) = x_curr(1:3) + x_curr(4:6) * params.dt;
        x_next(4:6) = x_curr(4:6) + u_cmd * params.dt;
        x_next(4:6) = limit_vector_norm(x_next(4:6), params.v_max);
        x_next(3) = max(min(x_next(3), params.z_max), params.z_min);

        history_u(:, k) = u_cmd;
        history_R(k) = R_k;
        history_sigma(k) = sigma_k;

        current_memory_mb = get_matlab_memory_mb();
        peak_memory_mb = max(peak_memory_mb, current_memory_mb);

        step_RunID(k) = run_id;
        step_k(k) = k;
        step_sigma(k) = sigma_k;
        step_R(k) = R_k;
        step_cpu_ms(k) = cpu_sec * 1000;
        if sigma_k == 1
            step_triggered_cpu_ms(k) = cpu_sec * 1000;
        end
        step_ipopt_iter(k) = solver_info.iter_count;
        step_solver_success(k) = solver_info.success;
        step_fallback(k) = solver_info.fallback_used;
        step_return_status(k) = string(solver_info.return_status);
        step_memory_mb(k) = current_memory_mb;

        x_curr = x_next;
        sigma_prev = sigma_k;
    end

    if ~collision_flag && norm(x_curr(1:3) - p_goal) <= params.eps_tol
        success_flag = true;
    end

    valid_k = max(1, k);
    trigger_rate = (sum(history_sigma(1:valid_k)) / valid_k) * 100;
    avg_t_solve = mean(history_t_solve(1:valid_k)) * 1000;
    max_t_solve = max(history_t_solve(1:valid_k)) * 1000;
    std_t_solve = std(history_t_solve(1:valid_k)) * 1000;

    triggered_idx = history_sigma(1:valid_k) == 1;
    if any(triggered_idx)
        avg_triggered_cpu_ms = mean(history_t_solve(triggered_idx)) * 1000;
        max_triggered_cpu_ms = max(history_t_solve(triggered_idx)) * 1000;
    else
        avg_triggered_cpu_ms = NaN;
        max_triggered_cpu_ms = NaN;
    end

    iter_values = step_ipopt_iter(1:valid_k);
    iter_values = iter_values(~isnan(iter_values));
    if isempty(iter_values)
        avg_iter = NaN;
        max_iter = NaN;
    else
        avg_iter = mean(iter_values);
        max_iter = max(iter_values);
    end

    run_result = struct();
    run_result.RunID = run_id;
    run_result.RandomSeed = random_seed;
    run_result.NumSteps = valid_k;
    run_result.NumDynamicObstacles = num_obs;
    run_result.Success = double(success_flag);
    run_result.Collision = double(collision_flag);
    run_result.StaticCollisionBuildingID = static_hit_id;
    run_result.TriggerRatePercent = trigger_rate;
    run_result.AvgCPUms = avg_t_solve;
    run_result.StdCPUms = std_t_solve;
    run_result.MaxCPUms = max_t_solve;
    run_result.AvgTriggeredCPUms = avg_triggered_cpu_ms;
    run_result.MaxTriggeredCPUms = max_triggered_cpu_ms;
    run_result.AvgIPOPTIterations = avg_iter;
    run_result.MaxIPOPTIterations = max_iter;
    run_result.SolverFailureCount = solver_failure_count;
    run_result.FallbackCount = fallback_count;
    run_result.PeakMemoryMB = peak_memory_mb;

    step_table = table( ...
        step_RunID(1:valid_k), ...
        step_k(1:valid_k), ...
        step_sigma(1:valid_k), ...
        step_R(1:valid_k), ...
        step_cpu_ms(1:valid_k), ...
        step_triggered_cpu_ms(1:valid_k), ...
        step_ipopt_iter(1:valid_k), ...
        step_solver_success(1:valid_k), ...
        step_fallback(1:valid_k), ...
        step_return_status(1:valid_k), ...
        step_memory_mb(1:valid_k), ...
        'VariableNames', { ...
        'RunID', 'Step', 'Sigma', 'Risk', 'CPUTimeMs', ...
        'TriggeredCPUTimeMs', 'IPOPTIterations', 'SolverSuccess', ...
        'FallbackUsed', 'ReturnStatus', 'MATLABMemoryMB'});
end

%% =========================================================
% 5. 内存读取函数
% =========================================================

function mem_mb = get_matlab_memory_mb()
    mem_mb = NaN;
    try
        m = memory;
        if isfield(m, 'MemUsedMATLAB')
            mem_mb = double(m.MemUsedMATLAB) / 1024^2;
        elseif isfield(m, 'MemUsedMATLABBytes')
            mem_mb = double(m.MemUsedMATLABBytes) / 1024^2;
        end
    catch
        try
            [~, out] = system('wmic process where name="MATLAB.exe" get WorkingSetSize /value');
            tokens = regexp(out, 'WorkingSetSize=(\d+)', 'tokens');
            if ~isempty(tokens)
                vals = cellfun(@(c) str2double(c{1}), tokens);
                mem_mb = max(vals) / 1024^2;
            end
        catch
            mem_mb = NaN;
        end
    end
end



function [u_cmd, X_opt, U_opt, solver_info] = solve_nmpc_casadi_profile(x_curr, Pi_ref_local, obs_pred, buildings, params)
    % =========================================================================
    % solve_nmpc_casadi.m
    %
    % 基于 CasADi 和 IPOPT 的 3D UAV 动态避障 NMPC 求解器
    %
    % 本版本修改重点：
    % 1. 增强静态建筑物避障代价，避免无人机贴墙飞行；
    % 2. 对圆柱体、圆锥体、长方体分别构造连续可导软惩罚；
    % 3. 不使用 if dist < safe_r 这类 CasADi 不支持的符号判断；
    % 4. 增加静态障碍物额外安全缓冲 static_extra_margin；
    % 5. 保留飞越低矮建筑物的能力；
    % 6. 求解失败时使用安全备份控制。
    %
    % 进一步修改完善：
    % 1. 保留原始代码结构，不删减原有功能；
    % 2. 增强终端位置代价和终端速度代价，减少终点附近绕圈；
    % 3. 增加预测时域内的渐进终点吸引代价；
    % 4. 在靠近终点时对障碍物软风险进行温和衰减，避免前方无障碍时仍过度绕行；
    % 5. 保留静态/动态障碍物避障安全性，不关闭避障项。
    %
    % 输入：
    %   x_curr        : 当前无人机状态 [px, py, pz, vx, vy, vz]'
    %   Pi_ref_local : 局部参考轨迹，维度为 3 x H
    %   obs_pred      : 动态障碍物预测结构体数组
    %   buildings     : 静态建筑物结构体数组
    %   params        : 全局参数结构体
    %
    % 输出：
    %   u_cmd : 当前执行控制输入，3 x 1
    %   X_opt : 预测状态轨迹，6 x H+1
    %   U_opt : 预测控制序列，3 x H
    % =========================================================================

    import casadi.*

    solver_info = struct();
    solver_info.success = false;
    solver_info.fallback_used = false;
    solver_info.iter_count = NaN;
    solver_info.return_status = 'not_started';
    solver_info.solve_time_sec = NaN;

    %% =========================================================
    % 1. 参数读取与默认值保护
    % =========================================================

    H = params.H;
    dt = params.dt;

    if ~isfield(params, 'ipopt_max_iter')
        params.ipopt_max_iter = 100;
    end

    if ~isfield(params, 'ipopt_acceptable_tol')
        params.ipopt_acceptable_tol = 1e-4;
    end

    if ~isfield(params, 'ipopt_print_level')
        params.ipopt_print_level = 0;
    end

    if ~isfield(params, 'casadi_expand')
        params.casadi_expand = true;
    end

    if ~isfield(params, 'r_u')
        params.r_u = 0.3;
    end

    if ~isfield(params, 'd_m')
        params.d_m = 0.2;
    end

    if ~isfield(params, 'u_max')
        params.u_max = 3.0;
    end

    if ~isfield(params, 'v_max')
        params.v_max = 5.0;
    end

    if ~isfield(params, 'z_min')
        params.z_min = 0.5;
    end

    if ~isfield(params, 'z_max')
        params.z_max = 20.0;
    end

    if ~isfield(params, 'alpha')
        params.alpha = 1.0;
    end

    if ~isfield(params, 'beta')
        params.beta = 0.5;
    end

    if ~isfield(params, 'gamma')
        params.gamma = 0.8;
    end

    if ~isfield(params, 'delta')
        params.delta = 5.0;
    end

    if ~isfield(params, 'c1')
        params.c1 = 1.0;
    end

    if ~isfield(params, 'c2')
        params.c2 = 0.1;
    end

    if ~isfield(params, 'c3')
        params.c3 = 3.0;
    end

    if ~isfield(params, 'use_asymmetric_energy')
        params.use_asymmetric_energy = true;
    end

    if ~isfield(params, 'terminal_pos_weight')
        params.terminal_pos_weight = 120.0;
    end

    if ~isfield(params, 'terminal_vel_weight')
        params.terminal_vel_weight = 20.0;
    end

    % =====================================================
    % 新增：终点收敛增强参数
    % =====================================================
    % 注意：
    % 这里不是删除原来的终端代价，而是在原有基础上增强。
    % 如果 init_params.m 里设置较小，这里给一个有效下限，
    % 避免终端代价过弱导致靠近终点时绕圈。
    % =====================================================

    if params.terminal_pos_weight < 120.0
        params.terminal_pos_weight = 120.0;
    end

    if params.terminal_vel_weight < 20.0
        params.terminal_vel_weight = 20.0;
    end

    % 预测时域内的渐进终点吸引代价权重
    if ~isfield(params, 'terminal_path_weight')
        params.terminal_path_weight = 8.0;
    end

    % 终点附近风险代价衰减半径，单位 m
    % 距离终点越近，静态/动态软风险会被适度降低。
    % 这不是关闭避障，而是避免已经无明显障碍时继续绕圈。
    if ~isfield(params, 'terminal_risk_decay_radius')
        params.terminal_risk_decay_radius = 12.0;
    end

    % 风险代价最小保留比例
    % 例如 0.55 表示靠近终点时仍保留至少 55% 避障风险代价。
    if ~isfield(params, 'terminal_risk_min_factor')
        params.terminal_risk_min_factor = 0.45;
    end

    % 防止设置过小导致避障被削弱过多
    if params.terminal_risk_min_factor < 0.45
        params.terminal_risk_min_factor = 0.45;
    end

    if params.terminal_risk_min_factor > 1.0
        params.terminal_risk_min_factor = 1.0;
    end

    % 终点真实吸引半径，单位 m
    % 当无人机进入该半径后，即使 Pi_ref_local 仍来自名义路径，
    % NMPC 的终端参考点也直接使用 params.p_goal，避免绕过最后障碍物后
    % 又被局部名义轨迹终点拉回去。
    if ~isfield(params, 'terminal_goal_attract_radius')
        params.terminal_goal_attract_radius = 25.0;
    end

    if params.terminal_goal_attract_radius <= 0
        params.terminal_goal_attract_radius = 25.0;
    end

    % 平滑函数小量
    smooth_eps = 1e-4;

    % 动态障碍物避障代价权重
    dyn_risk_gain = 3000;

    % 静态障碍物避障代价权重
    % 你之前撞到 B9，说明静态障碍物权重偏弱，所以这里明显提高。
    static_risk_gain = 12000;

    % 静态障碍物额外安全缓冲，单位 m
    % 作用：让 NMPC 在规划时比真实碰撞检测更保守一些。
    static_extra_margin = 0.55;

    % 靠近建筑物边界时的额外强惩罚权重
    static_near_gain = 2500;

    %% =========================================================
    % 2. 初始化 CasADi Opti 优化器
    % =========================================================

    opti = casadi.Opti();

    % 状态变量 X = [px, py, pz, vx, vy, vz]
    X = opti.variable(6, H + 1);

    % 控制变量 U = [ux, uy, uz]
    U = opti.variable(3, H);

    % 初始状态约束
    opti.subject_to(X(:, 1) == x_curr);

    %% =========================================================
    % 3. 初始化目标函数
    % =========================================================

    J_path = 0;
    J_smooth = 0;
    J_energy = 0;
    J_risk = 0;

    % 新增：预测时域内渐进终点吸引代价
    % 该项不会替代路径跟踪项，只是在预测窗口后段加强向终点收敛。
    J_terminal_progress = 0;

    % 终端参考点
    % 默认使用局部参考轨迹末端；
    % 但当无人机已经进入终点吸引半径时，直接使用真实终点 params.p_goal。
    % 这样可以避免绕过最后一个障碍物之后，NMPC 仍被 Pi_ref_local(:, end)
    % 这个局部名义路径点牵引，导致先绕回名义路径再去终点。
    if isfield(params, 'p_goal') && ...
            norm(x_curr(1:3) - params.p_goal) <= params.terminal_goal_attract_radius
        p_terminal_ref = params.p_goal;
    else
        p_terminal_ref = Pi_ref_local(:, end);
    end

    %% =========================================================
    % 4. 构造预测时域内约束和代价
    % =========================================================

    for k = 1:H

        % =====================================================
        % 4.1 无人机离散运动学模型
        % =====================================================

        p_next = X(1:3, k) + X(4:6, k) * dt;
        v_next = X(4:6, k) + U(:, k) * dt;

        opti.subject_to(X(1:3, k + 1) == p_next);
        opti.subject_to(X(4:6, k + 1) == v_next);

        % =====================================================
        % 4.2 物理约束
        % =====================================================

        opti.subject_to(X(4:6, k + 1) <= params.v_max);
        opti.subject_to(X(4:6, k + 1) >= -params.v_max);

        opti.subject_to(U(:, k) <= params.u_max);
        opti.subject_to(U(:, k) >= -params.u_max);

        opti.subject_to(X(3, k + 1) <= params.z_max);
        opti.subject_to(X(3, k + 1) >= params.z_min);

        % =====================================================
        % 4.3 路径跟踪代价
        % =====================================================

        ref_k = Pi_ref_local(:, min(k, size(Pi_ref_local, 2)));

        J_path = J_path + sumsqr(X(1:3, k + 1) - ref_k);

        % =====================================================
        % 新增：预测窗口内渐进终点吸引代价
        % =====================================================
        % 作用：
        % 1. 在预测窗口前段影响较弱；
        % 2. 在预测窗口后段影响增强；
        % 3. 避免无人机接近终点后仍然沿大弧线绕行。
        % =====================================================

        terminal_stage_weight = (k / H)^2;
        J_terminal_progress = J_terminal_progress ...
            + params.terminal_path_weight ...
            * terminal_stage_weight ...
            * sumsqr(X(1:3, k + 1) - p_terminal_ref);

        % =====================================================
        % 新增：终点附近风险代价衰减因子
        % =====================================================
        % terminal_risk_decay_k 是一个连续可导表达式：
        % - 离终点远时接近 1；
        % - 离终点近时接近 terminal_risk_min_factor；
        % - 不会变成 0，因此不会关闭避障。
        % =====================================================

        dist_to_terminal_k = sqrt(sumsqr(X(1:3, k + 1) - p_terminal_ref) + 1e-6);

        terminal_risk_decay_k = params.terminal_risk_min_factor ...
            + (1.0 - params.terminal_risk_min_factor) ...
            * dist_to_terminal_k ...
            / (dist_to_terminal_k + params.terminal_risk_decay_radius);

        % =====================================================
        % 4.4 控制平滑性代价
        % =====================================================

        if k < H
            J_smooth = J_smooth + sumsqr(U(:, k + 1) - U(:, k));
        end

        % =====================================================
        % 4.5 能耗代理代价
        % =====================================================

        dp = X(1:3, k + 1) - X(1:3, k);
        dz = X(3, k + 1) - X(3, k);

        if params.use_asymmetric_energy
            climb_positive = 0.5 * (dz + sqrt(dz^2 + smooth_eps));
            climb_penalty = params.c3 * climb_positive;
        else
            climb_penalty = 0;
        end

        J_energy = J_energy ...
            + params.c1 * sumsqr(dp) ...
            + params.c2 * sumsqr(U(:, k)) ...
            + climb_penalty;

        % =====================================================
        % 4.6 多动态障碍物软避障代价
        % =====================================================

        if ~isempty(obs_pred)

            for i = 1:length(obs_pred)

                obs_pos = obs_pred(i).pos(:, min(k, size(obs_pred(i).pos, 2)));

                if isfield(obs_pred(i), 'r_i') && ~isempty(obs_pred(i).r_i)
                    r_i = obs_pred(i).r_i;
                else
                    r_i = 1.5;
                end

                if isfield(obs_pred(i), 'inflation') && ~isempty(obs_pred(i).inflation)
                    inflation_k = obs_pred(i).inflation(min(k, length(obs_pred(i).inflation)));
                else
                    inflation_k = 0;
                end

                rho_dyn = params.r_u + r_i + inflation_k + params.d_m;

                dist_dyn = sqrt(sumsqr(X(1:3, k + 1) - obs_pos) + 1e-6);

                margin_dyn = rho_dyn - dist_dyn;

                violation_dyn = smooth_positive_part(margin_dyn, smooth_eps);

                % 三次惩罚：进入危险区后惩罚快速增大
                % 新增 terminal_risk_decay_k：
                % 靠近终点且直达目标时，避免动态风险软代价继续导致绕圈。
                J_risk = J_risk ...
                    + terminal_risk_decay_k ...
                    * dyn_risk_gain ...
                    * violation_dyn^3;

                % 弱近距离惩罚：还没碰撞时也鼓励远离
                % 同样加入终点附近温和衰减。
                % 修改说明：
                % 原来的 1/(dist_dyn-rho_dyn+3) 在极端情况下可能出现分母过小，
                % 这里改为平滑正部函数形式，只在接近安全边界时产生有限惩罚。
                near_margin_dyn = rho_dyn + 3.0 - dist_dyn;
                near_dyn = smooth_positive_part(near_margin_dyn, smooth_eps);
                J_risk = J_risk ...
                    + terminal_risk_decay_k ...
                    * 2.0 ...
                    * near_dyn^2;
            end
        end

        % =====================================================
        % 4.7 静态建筑物软避障代价
        % =====================================================
        % 核心思想：
        % 1. 真实碰撞检测可以较精确；
        % 2. NMPC 规划时必须更保守，提前远离建筑物；
        % 3. 低于楼顶时避障权重大；
        % 4. 高于楼顶后水平避障权重减弱，允许飞越低矮建筑。
        % =====================================================

        if ~isempty(buildings)

            for j = 1:length(buildings)

                b = buildings(j);

                px = X(1, k + 1);
                py = X(2, k + 1);
                pz = X(3, k + 1);

                % ---------------------------------------------
                % 高度权重
                % below_roof_weight 接近 1：无人机在楼顶以下，需要强避障
                % below_roof_weight 接近 0：无人机已经飞过楼顶，避障减弱
                % ---------------------------------------------
                height_margin = b.h + params.r_u + params.d_m - pz;
                below_roof_weight = 0.5 * (tanh(3.0 * height_margin) + 1);

                % ---------------------------------------------
                % 轻微爬升激励
                % 如果无人机在建筑物水平附近且低于楼顶，会鼓励往上飞一点。
                % 这个不是硬约束，只是帮助 NMPC 不贴墙。
                % ---------------------------------------------
                climb_help = smooth_positive_part(height_margin, smooth_eps);

                if b.type == 1

                    % =================================================
                    % 长方体建筑物：矩形 footprint 的平滑外部距离
                    % =================================================

                    dx_raw = sqrt((px - b.x)^2 + smooth_eps) - b.w / 2;
                    dy_raw = sqrt((py - b.y)^2 + smooth_eps) - b.l / 2;

                    dx_out = smooth_positive_part(dx_raw, smooth_eps);
                    dy_out = smooth_positive_part(dy_raw, smooth_eps);

                    dist_xy = sqrt(dx_out^2 + dy_out^2 + 1e-6);

                    safe_r = params.r_u + params.d_m + static_extra_margin;

                    margin_static = safe_r - dist_xy;

                    violation_static = smooth_positive_part(margin_static, smooth_eps);

                    % 主惩罚
                    % 新增 terminal_risk_decay_k：
                    % 靠近终点且直线路径可达时，避免静态软风险过度拉弯轨迹。
                    J_risk = J_risk ...
                        + terminal_risk_decay_k ...
                        * static_risk_gain ...
                        * violation_static^4 ...
                        * below_roof_weight;

                    % 近距离缓冲惩罚
                    near_static = smooth_positive_part(safe_r + 1.0 - dist_xy, smooth_eps);
                    J_risk = J_risk ...
                        + terminal_risk_decay_k ...
                        * static_near_gain ...
                        * near_static^2 ...
                        * below_roof_weight;

                    % 附加爬升激励，仅在靠近建筑物时生效
                    J_risk = J_risk ...
                        + terminal_risk_decay_k ...
                        * 80 ...
                        * near_static^2 ...
                        * climb_help ...
                        * below_roof_weight;

                elseif b.type == 2

                    % =================================================
                    % 圆柱体建筑物
                    % =================================================

                    dist_xy = sqrt((px - b.x)^2 + (py - b.y)^2 + 1e-6);

                    safe_r = b.r + params.r_u + params.d_m + static_extra_margin;

                    margin_static = safe_r - dist_xy;

                    violation_static = smooth_positive_part(margin_static, smooth_eps);

                    % 主惩罚
                    J_risk = J_risk ...
                        + terminal_risk_decay_k ...
                        * static_risk_gain ...
                        * violation_static^4 ...
                        * below_roof_weight;

                    % 近距离缓冲惩罚
                    near_static = smooth_positive_part(safe_r + 1.2 - dist_xy, smooth_eps);
                    J_risk = J_risk ...
                        + terminal_risk_decay_k ...
                        * static_near_gain ...
                        * near_static^2 ...
                        * below_roof_weight;

                    % 附加爬升激励
                    J_risk = J_risk ...
                        + terminal_risk_decay_k ...
                        * 100 ...
                        * near_static^2 ...
                        * climb_help ...
                        * below_roof_weight;

                elseif b.type == 3

                    % =================================================
                    % 圆锥体建筑物
                    % =================================================

                    cone_scale_raw = 1 - pz / max(b.h, 1e-6);
                    cone_scale = smooth_positive_part(cone_scale_raw, smooth_eps);

                    cone_radius = b.r * cone_scale;

                    dist_xy = sqrt((px - b.x)^2 + (py - b.y)^2 + 1e-6);

                    safe_r = cone_radius + params.r_u + params.d_m + static_extra_margin;

                    margin_static = safe_r - dist_xy;

                    violation_static = smooth_positive_part(margin_static, smooth_eps);

                    % 主惩罚
                    J_risk = J_risk ...
                        + terminal_risk_decay_k ...
                        * static_risk_gain ...
                        * violation_static^4 ...
                        * below_roof_weight;

                    % 近距离缓冲惩罚
                    near_static = smooth_positive_part(safe_r + 1.0 - dist_xy, smooth_eps);
                    J_risk = J_risk ...
                        + terminal_risk_decay_k ...
                        * static_near_gain ...
                        * near_static^2 ...
                        * below_roof_weight;

                    % 附加爬升激励
                    J_risk = J_risk ...
                        + terminal_risk_decay_k ...
                        * 80 ...
                        * near_static^2 ...
                        * climb_help ...
                        * below_roof_weight;
                end
            end
        end
    end

    %% =========================================================
    % 5. 组合总代价函数
    % =========================================================
    % 增加终端代价：
    % 1. 强化预测时域末端靠近局部参考轨迹最后一个点；
    % 2. 当 main_simulation.m 进入终点直接引导阶段时，
    %    Pi_ref_local(:, end) 会接近或等于 p_goal；
    % 3. 终端速度代价用于抑制冲过终点后绕圈。
    % =========================================================

    % p_terminal_ref 已经在目标函数初始化阶段定义，
    % 这里继续使用，避免重复定义导致逻辑不一致。

    J_terminal_pos = params.terminal_pos_weight * sumsqr(X(1:3, H + 1) - p_terminal_ref);
    J_terminal_vel = params.terminal_vel_weight * sumsqr(X(4:6, H + 1));

    J_total = params.alpha * J_path ...
            + params.beta  * J_smooth ...
            + params.gamma * J_energy ...
            + params.delta * J_risk ...
            + J_terminal_progress ...
            + J_terminal_pos ...
            + J_terminal_vel;

    opti.minimize(J_total);

    %% =========================================================
    % 6. 初始猜测
    % =========================================================

    X_guess = zeros(6, H + 1);
    U_guess = zeros(3, H);

    X_guess(:, 1) = x_curr;

    for k = 1:H
        if isfield(params, 'p_goal') && ...
                norm(x_curr(1:3) - params.p_goal) <= params.terminal_goal_attract_radius
            alpha_guess = min(1.0, k / H);
            X_guess(1:3, k + 1) = x_curr(1:3) + alpha_guess * (p_terminal_ref - x_curr(1:3));
        else
            X_guess(1:3, k + 1) = Pi_ref_local(:, min(k, size(Pi_ref_local, 2)));
        end
        X_guess(4:6, k + 1) = x_curr(4:6);
    end

    opti.set_initial(X, X_guess);
    opti.set_initial(U, U_guess);

    %% =========================================================
    % 7. IPOPT 求解器配置
    % =========================================================

    p_opts = struct();
    p_opts.expand = params.casadi_expand;
    p_opts.print_time = false;

    s_opts = struct();
    s_opts.max_iter = params.ipopt_max_iter;
    if isfield(params, 'ipopt_tol')
        s_opts.tol = params.ipopt_tol;
    else
        end
    s_opts.print_level = 0;
    s_opts.sb = 'yes';
    s_opts.acceptable_tol = params.ipopt_acceptable_tol;
    s_opts.acceptable_iter = 5;
    s_opts.acceptable_obj_change_tol = 1e-5;
    if isfield(params, 'ipopt_linear_solver') && ~isempty(params.ipopt_linear_solver)
        s_opts.linear_solver = params.ipopt_linear_solver;
    end

    opti.solver('ipopt', p_opts, s_opts);

    %% =========================================================
    % 8. 求解优化问题
    % =========================================================

    try

        solve_timer = tic;
        sol = opti.solve();
        solver_info.solve_time_sec = toc(solve_timer);

        try
            stats = opti.stats();
            solver_info.success = true;
            solver_info.fallback_used = false;
            if isfield(stats, 'iter_count')
                solver_info.iter_count = stats.iter_count;
            elseif isfield(stats, 'iterations')
                solver_info.iter_count = stats.iterations;
            end
            if isfield(stats, 'return_status')
                solver_info.return_status = stats.return_status;
            else
                solver_info.return_status = 'Solve_Succeeded';
            end
        catch
            solver_info.success = true;
            solver_info.fallback_used = false;
            solver_info.return_status = 'Solve_Succeeded';
        end

        u_cmd = sol.value(U(:, 1));
        X_opt = sol.value(X);
        U_opt = sol.value(U);

        u_cmd = limit_vector_norm_local(u_cmd, params.u_max);

    catch ME

        solver_info.success = false;
        solver_info.fallback_used = true;
        solver_info.return_status = ME.message;
        try
            stats = opti.stats();
            if isfield(stats, 'iter_count')
                solver_info.iter_count = stats.iter_count;
            elseif isfield(stats, 'iterations')
                solver_info.iter_count = stats.iterations;
            end
            if isfield(stats, 'return_status')
                solver_info.return_status = stats.return_status;
            end
        catch
        end

        warning('CasADi IPOPT 求解失败，使用备份控制。错误信息：%s', ME.message);

        try
            X_debug = opti.debug.value(X);
            U_debug = opti.debug.value(U);

            if all(isfinite(X_debug(:))) && all(isfinite(U_debug(:)))
                X_opt = X_debug;
                U_opt = U_debug;
            else
                [X_opt, U_opt] = build_fallback_prediction_local(x_curr, Pi_ref_local, params);
            end

        catch
            [X_opt, U_opt] = build_fallback_prediction_local(x_curr, Pi_ref_local, params);
        end

        u_cmd = fallback_safe_control_local(x_curr, Pi_ref_local, obs_pred, buildings, params);
        u_cmd = limit_vector_norm_local(u_cmd, params.u_max);
    end
end

%% =========================================================================
% 局部函数 1：平滑正部函数
% =========================================================================
function y = smooth_positive_part(x, eps_val)
    % =====================================================
    % 平滑近似 max(0, x)
    % y = 0.5 * (x + sqrt(x^2 + eps))
    % =====================================================

    y = 0.5 * (x + sqrt(x^2 + eps_val));
end

%% =========================================================================
% 局部函数 2：求解失败时的备份安全控制
% =========================================================================
function u_cmd = fallback_safe_control_local(x_curr, Pi_ref_local, obs_pred, buildings, params)

    p = x_curr(1:3);
    v = x_curr(4:6);

    if ~isempty(Pi_ref_local)

        % =====================================================
        % 修改：
        % 如果当前已经接近局部预测轨迹终端点，
        % fallback 控制也优先追踪终端点，
        % 避免求解失败时重新追踪 Pi_ref_local(:,1) 造成绕圈。
        % =====================================================

        p_first = Pi_ref_local(:, 1);
        p_last = Pi_ref_local(:, end);

        if isfield(params, 'p_goal') && ...
                isfield(params, 'terminal_goal_attract_radius') && ...
                norm(p - params.p_goal) <= params.terminal_goal_attract_radius
            p_ref = params.p_goal;
        elseif norm(p_last - p) < 10.0
            p_ref = p_last;
        else
            p_ref = p_first;
        end
    else
        p_ref = p;
    end

    % 基础跟踪控制
    k_p = 1.2;
    k_v = 2.0;

    u_track = k_p * (p_ref - p) - k_v * v;

    % 动态障碍物排斥
    u_rep_dyn = zeros(3, 1);

    if ~isempty(obs_pred)
        for i = 1:length(obs_pred)

            obs_pos = obs_pred(i).pos(:, 1);

            if isfield(obs_pred(i), 'r_i') && ~isempty(obs_pred(i).r_i)
                r_i = obs_pred(i).r_i;
            else
                r_i = 1.5;
            end

            safe_dist = params.r_u + r_i + params.d_m + 2.0;

            vec_away = p - obs_pos;
            dist = norm(vec_away);

            if dist < safe_dist && dist > 1e-6
                rep_gain = 2.5 * (1 / dist - 1 / safe_dist) / (dist^2);
                u_rep_dyn = u_rep_dyn + rep_gain * vec_away / dist;
            elseif dist <= 1e-6
                u_rep_dyn = u_rep_dyn + [0; 0; params.u_max];
            end
        end
    end

    % 静态建筑物排斥
    u_rep_static = zeros(3, 1);

    if ~isempty(buildings)

        for j = 1:length(buildings)

            b = buildings(j);

            % 只在低于楼顶附近时考虑静态排斥
            if p(3) <= b.h + params.r_u + params.d_m + 1.0

                if b.type == 1

                    % 长方体：用最近矩形点计算排斥方向
                    closest_x = min(max(p(1), b.x - b.w/2), b.x + b.w/2);
                    closest_y = min(max(p(2), b.y - b.l/2), b.y + b.l/2);

                    vec_xy = p(1:2) - [closest_x; closest_y];
                    dist_xy = norm(vec_xy);

                    safe_dist = params.r_u + params.d_m + 1.2;

                    if dist_xy < safe_dist && dist_xy > 1e-6
                        rep_gain = 3.0 * (1 / dist_xy - 1 / safe_dist) / (dist_xy^2);
                        u_rep_static(1:2) = u_rep_static(1:2) + rep_gain * vec_xy / dist_xy;
                    elseif dist_xy <= 1e-6
                        u_rep_static(3) = u_rep_static(3) + params.u_max;
                    end

                elseif b.type == 2

                    vec_xy = p(1:2) - [b.x; b.y];
                    dist_center = norm(vec_xy);
                    dist_edge = dist_center - b.r;

                    safe_dist = params.r_u + params.d_m + 1.2;

                    if dist_edge < safe_dist && dist_center > 1e-6
                        rep_gain = 3.0 * (1 / max(dist_edge, 0.1) - 1 / safe_dist) / (max(dist_edge, 0.1)^2);
                        u_rep_static(1:2) = u_rep_static(1:2) + rep_gain * vec_xy / dist_center;
                    elseif dist_center <= 1e-6
                        u_rep_static(3) = u_rep_static(3) + params.u_max;
                    end

                elseif b.type == 3

                    cone_scale = max(0, 1 - p(3) / max(b.h, 1e-6));
                    cone_radius = b.r * cone_scale;

                    vec_xy = p(1:2) - [b.x; b.y];
                    dist_center = norm(vec_xy);
                    dist_edge = dist_center - cone_radius;

                    safe_dist = params.r_u + params.d_m + 1.2;

                    if dist_edge < safe_dist && dist_center > 1e-6
                        rep_gain = 2.5 * (1 / max(dist_edge, 0.1) - 1 / safe_dist) / (max(dist_edge, 0.1)^2);
                        u_rep_static(1:2) = u_rep_static(1:2) + rep_gain * vec_xy / dist_center;
                    elseif dist_center <= 1e-6
                        u_rep_static(3) = u_rep_static(3) + params.u_max;
                    end
                end
            end
        end
    end

    u_cmd = u_track + u_rep_dyn + u_rep_static;

    % 高度边界保护
    if p(3) < params.z_min + 1.0
        u_cmd(3) = abs(u_cmd(3));
    elseif p(3) > params.z_max - 1.0
        u_cmd(3) = -abs(u_cmd(3));
    end

    u_cmd = limit_vector_norm_local(u_cmd, params.u_max);
end

%% =========================================================================
% 局部函数 3：构造失败时的预测轨迹
% =========================================================================
function [X_opt, U_opt] = build_fallback_prediction_local(x_curr, Pi_ref_local, params)

    H = params.H;

    X_opt = zeros(6, H + 1);
    U_opt = zeros(3, H);

    X_opt(:, 1) = x_curr;

    for k = 1:H
        if ~isempty(Pi_ref_local)
            X_opt(1:3, k + 1) = Pi_ref_local(:, min(k, size(Pi_ref_local, 2)));
        else
            X_opt(1:3, k + 1) = x_curr(1:3);
        end

        X_opt(4:6, k + 1) = x_curr(4:6);
    end
end

%% =========================================================================
% 局部函数 4：向量范数限幅
% =========================================================================
function v_limited = limit_vector_norm_local(v, max_norm)

    if norm(v) > max_norm
        v_limited = v / norm(v) * max_norm;
    else
        v_limited = v;
    end
end

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