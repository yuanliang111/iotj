function obs_pred = kalman_predict(obs_data, x_curr, params)
    % =========================================================================
    % kalman_predict.m
    %
    % 动态障碍物卡尔曼预测与不确定性膨胀函数
    %
    % 本文件作用：
    % 1. 对一个或多个动态障碍物进行未来 H 步位置预测；
    % 2. 默认采用 CV，constant velocity，匀速模型进行在线预测；
    % 3. 在复杂运动或不可预测运动场景下，自动增大过程噪声；
    % 4. 通过协方差传播计算障碍物预测不确定性膨胀半径；
    % 5. 为 NMPC 避障约束和复合风险计算提供 obs_pred。
    %
    % 为什么还保留 CV 预测模型：
    % 论文方法中的在线预测模型仍然可以使用轻量级 CV 模型，
    % 这样可以保持计算效率；但是在实验中，动态障碍物真实运动
    % 可以是 CA、SIN、RAND 等非 CV 运动，从而形成模型失配测试。
    % 这正好回应 Reviewer 2 第 2 条关于 CV 模型鲁棒性的质疑。
    %
    % 输入：
    %   obs_data : 动态障碍物观测结构体数组
    %              obs_data(i).state = [px, py, pz, vx, vy, vz]'
    %              obs_data(i).cov   = 6x6 协方差矩阵
    %              obs_data(i).r_i   = 障碍物半径
    %
    %              可选字段：
    %              obs_data(i).motion_type
    %              如果没有该字段，则默认按照 CV 预测处理。
    %
    %   x_curr   : 当前无人机状态 [px, py, pz, vx, vy, vz]'
    %              当前版本中主要保留该输入以兼容旧代码和后续扩展。
    %
    %   params   : 全局参数结构体
    %
    % 输出：
    %   obs_pred : 动态障碍物预测结构体数组
    %              obs_pred(i).pos       = 3 x H 预测位置
    %              obs_pred(i).vel       = 3 x H 预测速度
    %              obs_pred(i).inflation = 1 x H 不确定性膨胀半径
    %              obs_pred(i).r_i       = 障碍物半径
    %              obs_pred(i).motion_type = 运动模式编号
    % =========================================================================

    %#ok<INUSD>
    % x_curr 当前没有直接参与计算，但保留是为了兼容原始接口，
    % 后续如果需要根据无人机距离自适应调整预测保守性，可以使用该变量。

    %% =========================================================
    % 1. 基础参数读取与兼容性保护
    % =========================================================

    H = params.H;
    dt = params.dt;

    % 如果某些新增参数不存在，则在这里补默认值，避免旧版本 init_params.m 报错
    if ~isfield(params, 'sigma_q')
        params.sigma_q = 0.1;
    end

    if ~isfield(params, 'sigma_r')
        params.sigma_r = 0.05;
    end

    if ~isfield(params, 'chi2_alpha')
        params.chi2_alpha = 7.81;
    end

    if ~isfield(params, 'use_unpredictable_motion')
        params.use_unpredictable_motion = false;
    end

    if ~isfield(params, 'use_heterogeneous_motion')
        params.use_heterogeneous_motion = false;
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

    if ~isfield(params, 'obs_random_acc_std')
        params.obs_random_acc_std = 0.35;
    end

    if ~isfield(params, 'obs_random_acc_max')
        params.obs_random_acc_max = 0.8;
    end

    if ~isfield(params, 'noise_pos_std')
        params.noise_pos_std = params.sigma_r;
    end

    if ~isfield(params, 'noise_vel_std')
        params.noise_vel_std = params.sigma_r;
    end

    %% =========================================================
    % 2. 构造 CV 状态转移矩阵
    % =========================================================

    % 状态向量：
    % o = [px, py, pz, vx, vy, vz]'
    %
    % CV 模型：
    % p(k+1) = p(k) + v(k) * dt
    % v(k+1) = v(k)
    A = eye(6);
    A(1:3, 4:6) = eye(3) * dt;

    % 加速度扰动输入矩阵
    % 如果真实障碍物出现加速度变化，则其不确定性通过该矩阵传播到位置和速度
    G = [
        0.5 * dt^2 * eye(3);
        dt * eye(3)
    ];

    %% =========================================================
    % 3. 空输入保护
    % =========================================================

    obs_pred = struct([]);

    if isempty(obs_data)
        return;
    end

    %% =========================================================
    % 4. 对每一个动态障碍物进行 H 步预测
    % =========================================================

    for i = 1:length(obs_data)

        % =====================================================
        % 4.1 读取当前障碍物观测状态
        % =====================================================

        o_curr = obs_data(i).state;

        % 如果没有协方差矩阵，则给一个默认值
        if isfield(obs_data(i), 'cov') && ~isempty(obs_data(i).cov)
            P_curr = obs_data(i).cov;
        else
            P_curr = diag([
                params.noise_pos_std^2 * ones(1, 3), ...
                params.noise_vel_std^2 * ones(1, 3)
            ]);
        end

        % 如果没有障碍物半径，则使用默认半径
        if isfield(obs_data(i), 'r_i') && ~isempty(obs_data(i).r_i)
            r_i = obs_data(i).r_i;
        else
            if isfield(params, 'default_obs_radius')
                r_i = params.default_obs_radius;
            else
                r_i = 1.5;
            end
        end

        % 如果传入了运动模式，则记录运动模式；
        % 如果没有传入，则默认认为预测模型不知道真实运动类型，按 CV 处理。
        if isfield(obs_data(i), 'motion_type') && ~isempty(obs_data(i).motion_type)
            motion_type = obs_data(i).motion_type;
        else
            motion_type = params.motion_type_CV;
        end

        % =====================================================
        % 4.2 根据场景复杂程度设置过程噪声
        % =====================================================

        % 基础过程噪声：对应普通 CV 预测
        sigma_acc = params.sigma_q;

        % 如果启用不可预测运动，则整体增大过程噪声
        % 这样可以让预测占据区域更保守，避免过度相信 CV 模型。
        if params.use_unpredictable_motion
            sigma_acc = max(sigma_acc, params.obs_random_acc_std);
        end

        % 如果已知该障碍物是非 CV 运动，则进一步增大不确定性
        % 注意：这里并不是让预测模型“作弊”知道真实未来轨迹，
        % 而是根据运动类别提高预测保守性。
        if motion_type == params.motion_type_CA
            sigma_acc = max(sigma_acc, 0.25);
        elseif motion_type == params.motion_type_SIN
            sigma_acc = max(sigma_acc, 0.35);
        elseif motion_type == params.motion_type_RAND
            sigma_acc = max(sigma_acc, params.obs_random_acc_max);
        end

        % 构造过程噪声协方差矩阵
        Q = G * (sigma_acc^2 * eye(3)) * G';

        % 构造测量噪声协方差矩阵
        R = diag([
            params.noise_pos_std^2 * ones(1, 3), ...
            params.noise_vel_std^2 * ones(1, 3)
        ]);

        % 当前简化处理：把当前观测作为后验估计
        % 为了反映测量噪声，将观测噪声合入初始协方差
        P_curr = P_curr + R;

        % =====================================================
        % 4.3 预分配预测结果数组
        % =====================================================

        pred_pos = zeros(3, H);
        pred_vel = zeros(3, H);
        inflation = zeros(1, H);

        o_pred = o_curr;
        P_pred = P_curr;

        %% =====================================================
        % 4.4 执行 H 步开环预测
        % =====================================================

        for tau = 1:H

            % -------------------------------------------------
            % 状态预测
            % -------------------------------------------------
            % 仍采用 CV 预测模型：
            % o(k+tau|k) = A * o(k+tau-1|k)
            %
            % 注意：
            % 真实障碍物运动可以是 CA、SIN、RAND；
            % 这里故意保持 CV 预测，用于测试模型失配情况下的鲁棒性。
            o_pred = A * o_pred;

            % 保存预测位置和速度
            pred_pos(:, tau) = o_pred(1:3);
            pred_vel(:, tau) = o_pred(4:6);

            % -------------------------------------------------
            % 协方差传播
            % -------------------------------------------------
            P_pred = A * P_pred * A' + Q;

            % -------------------------------------------------
            % 提取位置协方差子矩阵
            % -------------------------------------------------
            P_pos = P_pred(1:3, 1:3);

            % 数值对称化，避免浮点误差导致特征值出现微小负数
            P_pos = 0.5 * (P_pos + P_pos');

            % -------------------------------------------------
            % 计算最大特征值
            % -------------------------------------------------
            eig_values = eig(P_pos);
            max_eig = max(real(eig_values));
            max_eig = max(max_eig, 0);

            % -------------------------------------------------
            % 计算不确定性膨胀半径
            % -------------------------------------------------
            % Delta_i(k+tau) = sqrt(chi2_alpha * lambda_max(P_pos))
            inflation_tau = sqrt(params.chi2_alpha * max_eig);

            % -------------------------------------------------
            % 对非 CV 或随机运动进一步增加保守膨胀
            % -------------------------------------------------
            % 这一步用于应对真实运动与 CV 预测模型不完全匹配的问题。
            % tau 越大，模型失配可能越明显，因此使用随 tau 增长的附加项。
            mismatch_scale = tau * dt;

            if motion_type == params.motion_type_CA
                inflation_tau = inflation_tau + 0.10 * mismatch_scale;
            elseif motion_type == params.motion_type_SIN
                inflation_tau = inflation_tau + 0.15 * mismatch_scale;
            elseif motion_type == params.motion_type_RAND
                inflation_tau = inflation_tau + 0.25 * mismatch_scale;
            elseif params.use_unpredictable_motion
                inflation_tau = inflation_tau + 0.10 * mismatch_scale;
            end

            % 保存该步预测不确定性膨胀半径
            inflation(tau) = inflation_tau;
        end

        %% =====================================================
        % 4.5 打包第 i 个障碍物预测结果
        % =====================================================

        obs_item.pos = pred_pos;
        obs_item.vel = pred_vel;
        obs_item.inflation = inflation;
        obs_item.r_i = r_i;
        obs_item.motion_type = motion_type;

        obs_pred = [obs_pred, obs_item]; %#ok<AGROW>
    end

end