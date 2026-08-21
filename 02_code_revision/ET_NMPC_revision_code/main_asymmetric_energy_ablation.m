% main_asymmetric_energy_ablation.m
%
% This script is used for the Reviewer #1 Comment 4
% asymmetric-energy ablation study and reuses the
% current revision NMPC solver and simulation logic.
%
% It intentionally does not modify the existing Monte Carlo, solver,
% parameter, baseline, or result files.  The only controller difference
% within each paired trial is the asymmetric positive-climb penalty.

clear; clc;

%% Experiment configuration
params_base = init_params();
% Reviewer #1 Comment 4 energy-estimation defaults.  These match the
% current Monte Carlo convention without modifying init_params.m.
if ~isfield(params_base, 'P_base')
    params_base.P_base = 45.0;
end
if ~isfield(params_base, 'uav_mass')
    params_base.uav_mass = 1.2;
end
if ~isfield(params_base, 'gravity')
    params_base.gravity = 9.81;
end
if ~isfield(params_base, 'climb_efficiency')
    params_base.climb_efficiency = 0.70;
end
if ~isfield(params_base, 'k_u_energy')
    params_base.k_u_energy = 0.08;
end
params_base.use_event_trigger = true;
params_base.use_prediction = true;
params_base.use_multi_obstacles = true;
params_base.use_heterogeneous_motion = true;
params_base.use_unpredictable_motion = true;
params_base.use_measurement_noise = true;
params_base.enable_animation = false;
params_base.save_figures = false;

num_trials = 30;
base_seed = params_base.random_seed;
if isempty(base_seed)
    base_seed = 2026;
end

% Use one fixed map for every paired trial and both controller modes.
rng(base_seed, 'twister');
buildings = generate_city_map();

result_folder = 'H:\1.school\3new_paper\IoTJ_R1_revision\03_new_experiments\reviewer1_comment4_asymmetric_energy_ablation\results';
if ~exist(result_folder, 'dir')
    mkdir(result_folder);
end

output_files = { ...
    fullfile(result_folder, 'asymmetric_energy_ablation_detail.csv'), ...
    fullfile(result_folder, 'asymmetric_energy_ablation_trajectory.csv'), ...
    fullfile(result_folder, 'asymmetric_energy_ablation_summary.csv')};
for output_index = 1:length(output_files)
    if exist(output_files{output_index}, 'file')
        error('Refusing to overwrite an existing ablation result: %s', ...
            output_files{output_index});
    end
end

detail_rows = cell(2 * num_trials, 17);
trajectory_rows = cell(0, 8);
result_template = struct( ...
    'Success', 0, ...
    'Collision', 0, ...
    'TotalClimbMeter', 0, ...
    'EstimatedEnergyWh', 0, ...
    'PathLengthMeter', 0, ...
    'FlightTimeSec', 0, ...
    'MinClearanceMeter', NaN, ...
    'TriggerRatePercent', 0, ...
    'ControlEffort', 0, ...
    'Steps', 0, ...
    'Trajectory', zeros(3, 0));
paired_results = repmat(result_template, num_trials, 2);
detail_row_index = 0;

fprintf('Paired asymmetric-energy ablation: %d trials\n', num_trials);
fprintf('Seed rule: scenario_seed = %d + TrialIndex\n', base_seed);

for trial_index = 1:num_trials
    scenario_seed = base_seed + trial_index;

    % The RNG is reset inside each call.  The asymmetric mode is run first.
    params_asymmetric = configure_ablation_mode(params_base, true);
    asymmetric_result = run_single_ablation_trial( ...
        params_asymmetric, buildings, scenario_seed);

    params_symmetric = configure_ablation_mode(params_base, false);
    symmetric_result = run_single_ablation_trial( ...
        params_symmetric, buildings, scenario_seed);

    paired_results(trial_index, 1) = asymmetric_result;
    paired_results(trial_index, 2) = symmetric_result;

    detail_row_index = detail_row_index + 1;
    detail_rows(detail_row_index, :) = make_detail_row( ...
        trial_index, scenario_seed, 'Asymmetric ET-NMPC', ...
        params_asymmetric, asymmetric_result);
    trajectory_rows = [trajectory_rows; make_trajectory_rows( ...
        trial_index, scenario_seed, 'Asymmetric ET-NMPC', ...
        asymmetric_result, params_asymmetric.dt)]; %#ok<AGROW>

    detail_row_index = detail_row_index + 1;
    detail_rows(detail_row_index, :) = make_detail_row( ...
        trial_index, scenario_seed, 'Symmetric-energy ET-NMPC', ...
        params_symmetric, symmetric_result);
    trajectory_rows = [trajectory_rows; make_trajectory_rows( ...
        trial_index, scenario_seed, 'Symmetric-energy ET-NMPC', ...
        symmetric_result, params_symmetric.dt)]; %#ok<AGROW>
end

detail_table = cell2table(detail_rows, 'VariableNames', { ...
    'TrialIndex', 'ScenarioSeed', 'AlgorithmMode', 'UseAsymmetricEnergy', ...
    'c1', 'c2', 'c3', 'Success', 'Collision', 'TotalClimbMeter', ...
    'EstimatedEnergyWh', 'PathLengthMeter', 'FlightTimeSec', ...
    'MinClearanceMeter', 'TriggerRatePercent', 'ControlEffort', 'Steps'});

trajectory_table = cell2table(trajectory_rows, 'VariableNames', { ...
    'TrialIndex', 'ScenarioSeed', 'AlgorithmMode', 'SampleIndex', ...
    'TimeSec', 'X', 'Y', 'Z'});

summary_table = build_summary_table(paired_results);

writetable(detail_table, fullfile(result_folder, ...
    'asymmetric_energy_ablation_detail.csv'));
writetable(trajectory_table, fullfile(result_folder, ...
    'asymmetric_energy_ablation_trajectory.csv'));
writetable(summary_table, fullfile(result_folder, ...
    'asymmetric_energy_ablation_summary.csv'));

fprintf('Ablation results written to: %s\n', result_folder);

%% Local functions

function params = configure_ablation_mode(params, use_asymmetric_energy)
    params.use_event_trigger = true;
    params.use_prediction = true;
    params.use_asymmetric_energy = use_asymmetric_energy;

    if use_asymmetric_energy
        params.c3 = 3.0;
    else
        params.c3 = 0.0;
    end
end

function result = run_single_ablation_trial(params, buildings, scenario_seed)
    % Reset before every controller run so the pair sees the same stochastic
    % obstacle initialization, random motion, and measurement noise.
    rng(scenario_seed, 'twister');

    p_0 = params.p_0;
    v_0 = params.v_0;
    p_goal = params.p_goal;
    path_vector = p_goal - p_0;
    path_length = norm(path_vector);
    path_direction = path_vector / path_length;

    x_current = [p_0; v_0];
    obstacles = initialize_dynamic_obstacles_ablation(params, p_0, p_goal);
    num_obstacles = length(obstacles);

    % The state history has one more sample than executed control steps.
    history_x = zeros(6, params.k_max + 1);
    history_x(:, 1) = x_current;
    history_u = zeros(3, params.k_max);
    history_sigma = zeros(1, params.k_max);

    executed_steps = 0;
    k_last = -params.T_ref;
    sigma_previous = 0;
    collision_flag = false;
    min_clearance = inf;

    while norm(x_current(1:3) - p_goal) > params.eps_tol && ...
            executed_steps < params.k_max

        step_index = executed_steps + 1;
        current_time = step_index * params.dt;
        obstacles = update_dynamic_obstacles_ablation(obstacles, params, current_time);

        [step_collision, step_clearance] = dynamic_collision_and_clearance( ...
            x_current(1:3), obstacles, params);
        min_clearance = min(min_clearance, step_clearance);
        if params.check_dynamic_collision && step_collision
            collision_flag = true;
            break;
        end

        if params.check_static_collision && ...
                check_static_collision_ablation(x_current, buildings, params)
            collision_flag = true;
            break;
        end

        s_current = dot(x_current(1:3) - p_0, path_direction);
        s_current = max(0, min(s_current, path_length));
        s_reference = min(s_current + 2.0, path_length);
        x_reference_position = p_0 + s_reference * path_direction;

        distance_to_goal = norm(p_goal - x_current(1:3));
        if distance_to_goal > 2.0
            v_reference_nominal = params.v_max * 0.8 * path_direction;
        else
            v_reference_nominal = [0; 0; 0];
        end
        x_reference = [x_reference_position; v_reference_nominal];

        local_reference = zeros(3, params.H);
        for horizon_index = 1:params.H
            s_h = min(s_current + horizon_index * 1.5 * ...
                (params.v_max * 0.8 * params.dt), path_length);
            local_reference(:, horizon_index) = p_0 + s_h * path_direction;
        end

        observation_data = struct([]);
        for obstacle_index = 1:num_obstacles
            if params.use_measurement_noise
                measured_position = obstacles(obstacle_index).p + ...
                    randn(3, 1) * params.noise_pos_std;
                measured_velocity = obstacles(obstacle_index).v + ...
                    randn(3, 1) * params.noise_vel_std;
            else
                measured_position = obstacles(obstacle_index).p;
                measured_velocity = obstacles(obstacle_index).v;
            end

            observation_data(obstacle_index).state = ...
                [measured_position; measured_velocity];
            observation_data(obstacle_index).cov = obstacles(obstacle_index).cov;
            observation_data(obstacle_index).r_i = obstacles(obstacle_index).r_i;
            observation_data(obstacle_index).motion_type = ...
                obstacles(obstacle_index).motion_type;
        end

        obstacle_prediction = kalman_predict(observation_data, x_current, params);
        if ~params.use_prediction
            for obstacle_index = 1:length(obstacle_prediction)
                obstacle_prediction(obstacle_index).pos = repmat( ...
                    obstacle_prediction(obstacle_index).pos(:, 1), 1, params.H);
                obstacle_prediction(obstacle_index).vel = zeros(3, params.H);
                obstacle_prediction(obstacle_index).inflation = zeros(1, params.H);
            end
        end

        [risk_value, ~, ~, ~] = compute_risk( ...
            x_current, x_reference, obstacle_prediction, buildings, params);

        if risk_value >= params.R_on && (step_index - k_last) >= params.T_ref
            sigma_current = 1;
        elseif risk_value <= params.R_off
            sigma_current = 0;
        else
            sigma_current = sigma_previous;
        end

        if sigma_current == 1
            [u_command, ~, ~] = solve_nmpc_casadi( ...
                x_current, local_reference, obstacle_prediction, buildings, params);
            k_last = step_index;
        else
            position_error = x_reference(1:3) - x_current(1:3);
            velocity_error = x_reference(4:6) - x_current(4:6);
            if distance_to_goal < 2.0
                u_raw = 2.0 * position_error + 3.0 * velocity_error;
            else
                u_raw = 1.5 * position_error + 2.5 * velocity_error;
            end
            u_command = limit_vector_norm_ablation(u_raw, params.u_max);
        end

        x_next = zeros(6, 1);
        x_next(1:3) = x_current(1:3) + x_current(4:6) * params.dt;
        x_next(4:6) = x_current(4:6) + u_command * params.dt;
        x_next(4:6) = limit_vector_norm_ablation(x_next(4:6), params.v_max);
        x_next(3) = max(min(x_next(3), params.z_max), params.z_min);

        history_u(:, step_index) = u_command;
        history_sigma(step_index) = sigma_current;
        history_x(:, step_index + 1) = x_next;
        executed_steps = step_index;
        x_current = x_next;
        sigma_previous = sigma_current;
    end

    % Verify the final saved state as well, because the loop can terminate
    % immediately after a state update when the goal or k_max condition is met.
    [final_dynamic_collision, final_clearance] = dynamic_collision_and_clearance( ...
        x_current(1:3), obstacles, params);
    min_clearance = min(min_clearance, final_clearance);
    if params.check_dynamic_collision && final_dynamic_collision
        collision_flag = true;
    end
    if params.check_static_collision && ...
            check_static_collision_ablation(x_current, buildings, params)
        collision_flag = true;
    end

    success_flag = ~collision_flag && ...
        norm(x_current(1:3) - p_goal) <= params.eps_tol;

    trajectory = history_x(1:3, 1:(executed_steps + 1));
    control_history = history_u(:, 1:executed_steps);
    trigger_history = history_sigma(1:executed_steps);

    if isinf(min_clearance)
        min_clearance = NaN;
    end

    result.Success = double(success_flag);
    result.Collision = double(collision_flag);
    result.TotalClimbMeter = sum(max(0, diff(trajectory(3, :))));
    result.PathLengthMeter = sum(sqrt(sum(diff(trajectory, 1, 2).^2, 1)));
    result.FlightTimeSec = executed_steps * params.dt;
    result.MinClearanceMeter = min_clearance;
    if executed_steps == 0
        result.TriggerRatePercent = 0;
        result.ControlEffort = 0;
    else
        result.TriggerRatePercent = sum(trigger_history) / executed_steps * 100;
        result.ControlEffort = sum(sum(control_history.^2, 1)) * params.dt;
    end
    result.EstimatedEnergyWh = estimated_energy_wh( ...
        result.FlightTimeSec, result.TotalClimbMeter, result.ControlEffort, params);
    result.Steps = executed_steps;
    result.Trajectory = trajectory;
end

function obs_array = initialize_dynamic_obstacles_ablation(params, p_0, p_goal)
    num_obs = params.num_dynamic_obstacles;
    obs_array = struct([]);
    path_vector = p_goal - p_0;
    path_direction = path_vector / norm(path_vector);
    vertical_direction = [0; 0; 1];
    lateral_direction_1 = cross(path_direction, vertical_direction);
    if norm(lateral_direction_1) < 1e-6
        lateral_direction_1 = [1; 0; 0];
    else
        lateral_direction_1 = lateral_direction_1 / norm(lateral_direction_1);
    end
    lateral_direction_2 = cross(path_direction, lateral_direction_1);
    lateral_direction_2 = lateral_direction_2 / norm(lateral_direction_2);

    if num_obs == 1
        s_list = 0.35;
    else
        s_list = linspace(0.25, 0.78, num_obs);
    end

    for i = 1:num_obs
        center_on_path = p_0 + s_list(i) * path_vector;
        side_sign = (-1)^i;
        lateral_offset = side_sign * (5.0 + 2.0 * rand) * lateral_direction_1;
        vertical_offset = (rand - 0.5) * 3.5 * lateral_direction_2;
        p_init = center_on_path + lateral_offset + vertical_offset;
        p_init(3) = max(min(p_init(3), params.z_max - 3.0), params.z_min + 3.0);

        to_path = center_on_path - p_init;
        if norm(to_path) < 1e-6
            v_direction = -side_sign * lateral_direction_1;
        else
            v_direction = to_path / norm(to_path);
        end
        speed_i = 0.7 + 0.7 * rand;
        v_init = speed_i * v_direction;
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
        obs_array(i).lateral_dir = side_sign * lateral_direction_1;
        acc_direction = 0.7 * v_direction + 0.3 * side_sign * lateral_direction_1;
        if norm(acc_direction) < 1e-6
            acc_direction = v_direction;
        end
        obs_array(i).acc_dir = acc_direction / norm(acc_direction);
    end
end

function obs_array = update_dynamic_obstacles_ablation(obs_array, params, current_time)
    for i = 1:length(obs_array)
        motion_type = obs_array(i).motion_type;
        if motion_type == params.motion_type_CV
            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;
        elseif motion_type == params.motion_type_CA
            acceleration = params.obs_ca_acc * obs_array(i).acc_dir;
            obs_array(i).v = obs_array(i).v + acceleration * params.dt;
            obs_array(i).v = limit_vector_norm_ablation(obs_array(i).v, params.obs_v_max);
            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;
        elseif motion_type == params.motion_type_SIN
            base_position = obs_array(i).p0 + obs_array(i).v0 * current_time;
            lateral_offset = params.obs_sin_amp * sin( ...
                params.obs_sin_omega * current_time + obs_array(i).phase) * ...
                obs_array(i).lateral_dir;
            vertical_offset = [0; 0; params.obs_vertical_amp * sin( ...
                params.obs_vertical_omega * current_time + obs_array(i).phase)];
            obs_array(i).p = base_position + lateral_offset + vertical_offset;
            lateral_velocity = params.obs_sin_amp * params.obs_sin_omega * cos( ...
                params.obs_sin_omega * current_time + obs_array(i).phase) * ...
                obs_array(i).lateral_dir;
            vertical_velocity = [0; 0; params.obs_vertical_amp * ...
                params.obs_vertical_omega * cos(params.obs_vertical_omega * ...
                current_time + obs_array(i).phase)];
            obs_array(i).v = obs_array(i).v0 + lateral_velocity + vertical_velocity;
            obs_array(i).v = limit_vector_norm_ablation(obs_array(i).v, params.obs_v_max);
        elseif motion_type == params.motion_type_RAND
            if params.use_unpredictable_motion
                acceleration_random = params.obs_random_acc_std * randn(3, 1);
                if norm(acceleration_random) > params.obs_random_acc_max
                    acceleration_random = acceleration_random / norm(acceleration_random) * ...
                        params.obs_random_acc_max;
                end
                obs_array(i).v = obs_array(i).v + acceleration_random * params.dt;
                if rand < params.obs_maneuver_prob
                    delta_velocity = 2 * rand(3, 1) - 1;
                    if norm(delta_velocity) > 1e-6
                        delta_velocity = delta_velocity / norm(delta_velocity) * ...
                            params.obs_velocity_jump_max;
                    end
                    obs_array(i).v = obs_array(i).v + delta_velocity;
                end
            end
            obs_array(i).v = limit_vector_norm_ablation(obs_array(i).v, params.obs_v_max);
            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;
        else
            obs_array(i).p = obs_array(i).p + obs_array(i).v * params.dt;
        end
        obs_array(i).p(3) = max(min(obs_array(i).p(3), params.z_max - 0.5), ...
            params.z_min + 0.5);
    end
end

function [collision, min_clearance] = dynamic_collision_and_clearance(position, obstacles, params)
    min_clearance = inf;
    collision = false;
    for i = 1:length(obstacles)
        distance = norm(position - obstacles(i).p);
        collision_distance = params.r_u + obstacles(i).r_i + params.collision_buffer;
        min_clearance = min(min_clearance, distance - params.r_u - obstacles(i).r_i);
        if distance <= collision_distance
            collision = true;
        end
    end
end

function flag = check_static_collision_ablation(x_current, buildings, params)
    flag = false;
    position = x_current(1:3);
    for i = 1:length(buildings)
        building = buildings(i);
        if building.type == 1
            effective_radius = sqrt((building.w / 2)^2 + (building.l / 2)^2);
        elseif building.type == 2
            effective_radius = building.r;
        elseif building.type == 3
            effective_radius = building.r * max(0, 1 - position(3) / max(building.h, 1e-6));
        else
            effective_radius = building.r;
        end
        distance_xy = norm(position(1:2) - [building.x; building.y]);
        if distance_xy <= effective_radius + params.r_u && ...
                position(3) <= building.h + params.r_u
            flag = true;
            return;
        end
    end
end

function vector_limited = limit_vector_norm_ablation(vector, maximum_norm)
    if norm(vector) > maximum_norm
        vector_limited = vector / norm(vector) * maximum_norm;
    else
        vector_limited = vector;
    end
end

function energy_wh = estimated_energy_wh(flight_time, total_climb, control_effort, params)
    energy_joule = params.P_base * flight_time + ...
        (params.uav_mass * params.gravity * total_climb) / params.climb_efficiency + ...
        params.k_u_energy * control_effort;
    energy_wh = energy_joule / 3600;
end

function row = make_detail_row(trial_index, scenario_seed, mode_name, params, result)
    row = {trial_index, scenario_seed, mode_name, ...
        params.use_asymmetric_energy, params.c1, params.c2, params.c3, ...
        result.Success, result.Collision, result.TotalClimbMeter, ...
        result.EstimatedEnergyWh, result.PathLengthMeter, result.FlightTimeSec, ...
        result.MinClearanceMeter, result.TriggerRatePercent, ...
        result.ControlEffort, result.Steps};
end

function rows = make_trajectory_rows(trial_index, scenario_seed, mode_name, result, dt)
    num_samples = size(result.Trajectory, 2);
    rows = cell(num_samples, 8);
    for sample_index = 1:num_samples
        rows(sample_index, :) = {trial_index, scenario_seed, mode_name, ...
            sample_index, (sample_index - 1) * dt, ...
            result.Trajectory(1, sample_index), result.Trajectory(2, sample_index), ...
            result.Trajectory(3, sample_index)};
    end
end

function summary_table = build_summary_table(paired_results)
    metric_names = { ...
        'TotalClimbMeter', 'EstimatedEnergyWh', 'PathLengthMeter', ...
        'FlightTimeSec', 'MinClearanceMeter', 'TriggerRatePercent', ...
        'ControlEffort', 'SuccessPercent', 'CollisionPercent'};
    mode_names = {'Asymmetric ET-NMPC', 'Symmetric-energy ET-NMPC'};
    summary_rows = cell(0, 9);

    for mode_index = 1:2
        mode_results = paired_results(:, mode_index);
        for metric_index = 1:length(metric_names)
            values = metric_values(mode_results, metric_names{metric_index});
            [mean_value, std_value, median_value, ci_low, ci_high] = summary_statistics(values);
            summary_rows(end + 1, :) = {'Mode', mode_names{mode_index}, ...
                metric_names{metric_index}, sum(isfinite(values)), mean_value, std_value, ...
                median_value, ci_low, ci_high}; %#ok<AGROW>
        end
    end

    for metric_index = 1:length(metric_names)
        asymmetric_values = metric_values(paired_results(:, 1), metric_names{metric_index});
        symmetric_values = metric_values(paired_results(:, 2), metric_names{metric_index});
        paired_difference = asymmetric_values - symmetric_values;
        [mean_value, std_value, median_value, ci_low, ci_high] = ...
            summary_statistics(paired_difference);
        summary_rows(end + 1, :) = {'PairedDifference', ...
            'Asymmetric - Symmetric', metric_names{metric_index}, ...
            sum(isfinite(paired_difference)), mean_value, std_value, median_value, ...
            ci_low, ci_high}; %#ok<AGROW>
    end

    summary_table = cell2table(summary_rows, 'VariableNames', { ...
        'SummaryType', 'AlgorithmMode', 'Metric', 'N', 'Mean', 'Std', ...
        'Median', 'CI95Lower', 'CI95Upper'});
end

function values = metric_values(results, metric_name)
    switch metric_name
        case 'SuccessPercent'
            values = 100 * [results.Success]';
        case 'CollisionPercent'
            values = 100 * [results.Collision]';
        otherwise
            values = [results.(metric_name)]';
    end
end

function [mean_value, std_value, median_value, ci_low, ci_high] = summary_statistics(values)
    values = values(:);
    values = values(isfinite(values));
    n = numel(values);
    if n == 0
        mean_value = NaN;
        std_value = NaN;
        median_value = NaN;
        ci_low = NaN;
        ci_high = NaN;
        return;
    end

    mean_value = mean(values);
    median_value = median(values);
    if n == 1
        std_value = 0;
        ci_low = mean_value;
        ci_high = mean_value;
        return;
    end

    std_value = std(values);
    half_width = tinv(0.975, n - 1) * std_value / sqrt(n);
    ci_low = mean_value - half_width;
    ci_high = mean_value + half_width;
end
