function [R_k, R_tcpa, R_pred, R_dev] = compute_risk(x_curr, x_ref, obs_pred, buildings, params)
    % =========================================================================
    % compute_risk.m
    %
    % 复合风险评估函数
    %
    % 本文件作用：
    % 1. 计算动态障碍物会遇风险 R_tcpa；
    % 2. 计算未来预测占据风险 R_pred；
    % 3. 计算轨迹偏离风险 R_dev；
    % 4. 计算静态建筑物风险 R_static；
    % 5. 融合得到总复合风险 R_k；
    % 6. 为事件触发机制提供判断依据。
    %
    % 输入：
    %   x_curr   : 当前无人机状态，维度为 6 x 1
    %              x_curr = [px, py, pz, vx, vy, vz]'
    %
    %   x_ref    : 当前参考状态或未来参考轨迹
    %              情况 1：6 x 1，表示当前参考状态 [p_ref; v_ref]
    %              情况 2：3 x H，表示未来 H 步参考位置轨迹
    %
    %   obs_pred : 动态障碍物预测结构体数组
    %              obs_pred(i).pos       = 3 x H
    %              obs_pred(i).vel       = 3 x H
    %              obs_pred(i).inflation = 1 x H
    %              obs_pred(i).r_i       = 障碍物半径
    %
    %   buildings : 静态建筑物结构体数组
    %
    %   params    : 全局参数结构体
    %
    % 输出：
    %   R_k     : 总复合风险
    %   R_tcpa  : 动态障碍物 TCPA/DCPA 会遇风险
    %   R_pred  : 未来预测占据风险
    %   R_dev   : 轨迹偏离风险
    % =========================================================================


    %% =========================================================
    % 1. 参数读取与兼容性保护
    % =========================================================

    if ~isfield(params, 'H')
        params.H = 15;
    end

    if ~isfield(params, 'dt')
        params.dt = 0.1;
    end

    if ~isfield(params, 'eps')
        params.eps = 1e-4;
    end

    if ~isfield(params, 'T_c')
        params.T_c = 3.0;
    end

    if ~isfield(params, 'D_c')
        params.D_c = 2.0;
    end

    if ~isfield(params, 'e_max')
        params.e_max = 2.0;
    end

    if ~isfield(params, 'r_u')
        params.r_u = 0.3;
    end

    if ~isfield(params, 'd_m')
        params.d_m = 0.2;
    end

    if ~isfield(params, 'w1')
        params.w1 = 0.4;
    end

    if ~isfield(params, 'w2')
        params.w2 = 0.4;
    end

    if ~isfield(params, 'w3')
        params.w3 = 0.2;
    end

    % 风险上限，防止距离极近时数值爆炸
    risk_cap = 1.5;

    % 静态障碍物风险影响距离，单位：m
    static_influence_dist = 4.0;

    % 当前无人机位置和速度
    p_k = x_curr(1:3);
    v_k = x_curr(4:6);

    % 初始化风险分量
    R_tcpa = 0;
    R_pred = 0;
    R_dev = 0;
    R_static = 0;


    %% =========================================================
    % 2. 动态障碍物 TCPA/DCPA 会遇风险
    % =========================================================
    % TCPA：Time to Closest Point of Approach，最近会遇时间
    % DCPA：Distance at Closest Point of Approach，最近会遇距离
    %
    % 直观理解：
    % 如果障碍物正在靠近无人机，并且最近会遇距离较小，
    % 则 R_tcpa 较大；如果障碍物正在远离，则风险较低。
    % =========================================================

    if ~isempty(obs_pred)

        for i = 1:length(obs_pred)

            % 取当前预测序列中的第一步作为当前障碍物估计状态
            if isempty(obs_pred(i).pos)
                continue;
            end

            p_oi = obs_pred(i).pos(:, 1);

            if isfield(obs_pred(i), 'vel') && ~isempty(obs_pred(i).vel)
                v_oi = obs_pred(i).vel(:, 1);
            else
                v_oi = zeros(3, 1);
            end

            % 相对位置和相对速度
            p_rel = p_k - p_oi;
            v_rel = v_k - v_oi;

            % 相对速度很小时，说明双方运动趋势接近，不直接计算 TCPA
            v_rel_norm_sq = norm(v_rel)^2;

            if v_rel_norm_sq > params.eps

                % 最近会遇时间
                tcpa = -(p_rel' * v_rel) / (v_rel_norm_sq + params.eps);

                % 只考虑未来会遇；如果 tcpa <= 0，说明最近点已经过去或正在远离
                if tcpa > 0

                    % 最近会遇距离
                    dcpa = norm(p_rel + tcpa * v_rel);

                    % 指数型风险函数
                    risk_i = exp(-tcpa / params.T_c) * exp(-dcpa / params.D_c);

                    % 风险上限裁剪
                    risk_i = min(risk_cap, risk_i);

                    % 多障碍物取最大风险
                    R_tcpa = max(R_tcpa, risk_i);
                end
            end
        end
    end


    %% =========================================================
    % 3. 未来预测占据风险
    % =========================================================
    % 原始版本中使用的是：
    %   当前无人机位置 p_k 与障碍物未来位置比较
    %
    % 现在改为：
    %   未来参考点 r_{k+tau} 与障碍物未来预测位置比较
    %
    % 这样更符合论文中的 predictive occupancy risk：
    % 判断无人机未来计划路径是否会接近未来障碍物占据区域。
    % =========================================================

    if ~isempty(obs_pred)

        for i = 1:length(obs_pred)

            if isempty(obs_pred(i).pos)
                continue;
            end

            % 当前障碍物预测步数
            H_obs = size(obs_pred(i).pos, 2);
            H_use = min(params.H, H_obs);

            for tau = 1:H_use

                % 取得未来参考点 r_{k+tau}
                r_tau = get_reference_point(x_ref, tau, params);

                % 障碍物未来位置
                obs_tau = obs_pred(i).pos(:, tau);

                % 障碍物半径
                if isfield(obs_pred(i), 'r_i') && ~isempty(obs_pred(i).r_i)
                    r_i = obs_pred(i).r_i;
                else
                    r_i = 1.5;
                end

                % 预测不确定性膨胀半径
                if isfield(obs_pred(i), 'inflation') && ~isempty(obs_pred(i).inflation)
                    inflation_tau = obs_pred(i).inflation(min(tau, length(obs_pred(i).inflation)));
                else
                    inflation_tau = 0;
                end

                % 总安全占据半径
                rho_tau = params.r_u + r_i + inflation_tau + params.d_m;

                % 未来参考点到障碍物未来位置的距离
                dist_tau = norm(r_tau - obs_tau);

                % 预测风险
                % 当 dist_tau 接近 rho_tau 或小于 rho_tau 时，风险会升高
                pred_val = rho_tau / (dist_tau + params.eps);

                % 风险上限裁剪
                pred_val = min(risk_cap, pred_val);

                % 多障碍物、多预测步取最大风险
                R_pred = max(R_pred, pred_val);
            end
        end
    end


    %% =========================================================
    % 4. 静态建筑物风险
    % =========================================================
    % 这里使用保守圆柱包络近似建筑物水平范围：
    % 1. 长方体使用外接圆半径；
    % 2. 圆柱体使用自身半径；
    % 3. 圆锥体半径随高度升高而减小。
    %
    % 当无人机在建筑物高度附近且水平距离过近时，静态风险增大。
    % =========================================================

    if ~isempty(buildings)

        for j = 1:length(buildings)

            b = buildings(j);

            % 计算不同建筑物类型的等效水平半径
            if b.type == 1

                % 长方体建筑物，使用水平外接圆半径
                eff_r = sqrt((b.w / 2)^2 + (b.l / 2)^2);

            elseif b.type == 2

                % 圆柱体建筑物
                eff_r = b.r;

            elseif b.type == 3

                % 圆锥体建筑物
                % 高度越高，圆锥半径越小
                eff_r = b.r * max(0, 1 - p_k(3) / max(b.h, params.eps));

            else

                % 未知类型建筑物，使用默认半径
                eff_r = b.r;
            end

            % 水平距离
            dist_xy = norm(p_k(1:2) - [b.x; b.y]);

            % 与建筑物边界之间的水平净距离
            margin = dist_xy - (eff_r + params.r_u + params.d_m);

            % 只有当无人机高度低于楼顶附近时，才考虑静态建筑物水平风险
            if p_k(3) < b.h + 1.0

                % 如果 margin 小于影响距离，则风险逐渐增大
                static_val = max(0, 1.0 - margin / static_influence_dist);

                % 风险上限裁剪
                static_val = min(risk_cap, static_val);

                R_static = max(R_static, static_val);
            end
        end
    end


    %% =========================================================
    % 5. 轨迹偏离风险
    % =========================================================
    % 当前 UAV 位置与当前参考位置之间的偏差越大，风险越高。
    % 这可以避免无人机过度偏离参考轨迹。
    % =========================================================

    p_ref_now = get_reference_point(x_ref, 1, params);

    e_k = norm(p_k - p_ref_now);

    R_dev = e_k / (params.e_max + params.eps);

    % 风险上限裁剪
    R_dev = min(risk_cap, R_dev);


    %% =========================================================
    % 6. 融合复合风险
    % =========================================================
    % 动态风险部分：
    %   R_dyn = w1 * R_tcpa + w2 * R_pred + w3 * R_dev
    %
    % 总风险：
    %   R_k = max(R_dyn, R_static)
    %
    % 这样设计的含义是：
    % 1. 动态风险由会遇风险、预测占据风险、偏离风险加权融合；
    % 2. 静态建筑物风险作为独立安全风险，取最大值参与触发。
    % =========================================================

    R_dyn = params.w1 * R_tcpa + params.w2 * R_pred + params.w3 * R_dev;

    R_k = max(R_dyn, R_static);

    % 总风险上限裁剪
    R_k = min(risk_cap, R_k);

end


%% =========================================================================
% 局部函数：根据 x_ref 获取第 tau 步参考点
% =========================================================================
function r_tau = get_reference_point(x_ref, tau, params)
    % =====================================================
    % 根据输入 x_ref 的格式返回未来第 tau 步参考位置
    %
    % 支持两种输入格式：
    %
    % 情况 1：
    %   x_ref 为 6 x 1 当前参考状态：
    %   x_ref = [p_ref; v_ref]
    %
    %   此时使用：
    %   r_tau = p_ref + (tau - 1) * dt * v_ref
    %
    % 情况 2：
    %   x_ref 为 3 x H 未来参考轨迹：
    %   此时直接取第 tau 个参考点。
    % =====================================================

    if size(x_ref, 1) == 3 && size(x_ref, 2) >= 1

        % x_ref 是 3 x H 未来参考位置序列
        idx = min(tau, size(x_ref, 2));
        r_tau = x_ref(:, idx);

    elseif numel(x_ref) >= 6

        % x_ref 是 6 x 1 当前参考状态
        p_ref = x_ref(1:3);
        v_ref = x_ref(4:6);

        r_tau = p_ref + (tau - 1) * params.dt * v_ref;

    elseif numel(x_ref) >= 3

        % x_ref 只有参考位置
        r_tau = x_ref(1:3);

    else

        error('x_ref 输入格式错误：应为 6x1 参考状态或 3xH 参考轨迹。');
    end
end