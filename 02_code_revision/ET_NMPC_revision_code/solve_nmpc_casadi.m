function [u_cmd, X_opt, U_opt] = solve_nmpc_casadi(x_curr, Pi_ref_local, obs_pred, buildings, params)
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
    s_opts.print_level = 0;
    s_opts.sb = 'yes';
    s_opts.tol = 1e-4;
    s_opts.acceptable_tol = params.ipopt_acceptable_tol;
    s_opts.acceptable_iter = 5;
    s_opts.acceptable_obj_change_tol = 1e-5;

    opti.solver('ipopt', p_opts, s_opts);

    %% =========================================================
    % 8. 求解优化问题
    % =========================================================

    try

        sol = opti.solve();

        u_cmd = sol.value(U(:, 1));
        X_opt = sol.value(X);
        U_opt = sol.value(U);

        u_cmd = limit_vector_norm_local(u_cmd, params.u_max);

    catch ME

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