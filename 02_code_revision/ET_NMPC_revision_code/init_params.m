function params = init_params()
    % =========================================================================
    % init_params.m
    %
    % ET-NMPC 无人机动态避障仿真的全局参数配置文件
    %
    % 本文件作用：
    % 1. 统一管理仿真中的所有参数；
    % 2. 保留原论文中的基础实验参数；
    % 3. 新增多动态障碍物实验参数；
    % 4. 新增非匀速、随机、突变运动障碍物参数；
    % 5. 新增可扩展性分析参数；
    % 6. 新增终点直接引导参数和终端代价参数；
    % 7. 为回应审稿人意见做准备。
    %
    % 对应审稿意见：
    % Reviewer 1 第 4 条：
    %   当前 urban canyon 场景较简单，需要增加多个动态障碍物、
    %   异质运动模式，并分析计算时间和触发率随障碍物数量变化的可扩展性。
    %
    % Reviewer 2 第 2 条：
    %   当前动态障碍物实验主要基于 constant-velocity，CV，模型，
    %   需要测试更随机、更不可预测的动态障碍物运动场景。
    % =========================================================================


    %% ========================================================================
    % 0. 算法模式与消融实验开关
    % ========================================================================

    % 是否启用事件触发机制
    % true  ：使用本文提出的 ET-NMPC
    % false ：退化为连续求解 NMPC，即 C-NMPC baseline
    params.use_event_trigger = true;

    % 是否启用动态障碍物预测
    % true  ：使用预测模型，属于 Proposed ET-NMPC
    % false ：关闭预测，退化为 NP-ET-NMPC，即 No Prediction baseline
    params.use_prediction = true;

    % 是否启用非对称爬升能耗惩罚
    % true  ：使用本文提出的非对称爬升惩罚
    % false ：关闭该项，作为 SE-ET-NMPC，即 Symmetric Energy baseline
    params.use_asymmetric_energy = true;

    % 是否启用多个动态障碍物
    % true  ：启用多动态障碍物场景，用于回应审稿意见
    % false ：使用原始单动态障碍物场景
    params.use_multi_obstacles = true;

    % 是否启用异质障碍物运动模式
    % true  ：不同障碍物采用不同运动模式，例如 CV、CA、SIN、RAND
    % false ：所有障碍物都采用 constant-velocity，CV，运动模式
    params.use_heterogeneous_motion = true;

    % 是否启用不可预测运动扰动
    % true  ：动态障碍物会出现随机加速度、随机转向或速度突变
    % false ：动态障碍物按照设定模型平稳运动
    params.use_unpredictable_motion = true;

    % 是否加入传感器观测噪声
    % true  ：观测位置和速度中加入高斯噪声
    % false ：直接使用动态障碍物真实状态
    params.use_measurement_noise = true;


    %% ========================================================================
    % 1. 时间参数与预测时域参数
    % ========================================================================

    % 仿真采样时间，单位：秒
    params.dt = 0.1;

    % NMPC 预测步长，即每次优化向前预测多少个采样步
    params.H = 15;

    % 最大仿真步数，用于防止仿真无法结束
    params.k_max = 1000;


    %% ========================================================================
    % 2. 无人机运动学约束参数
    % ========================================================================

    % 无人机最大速度，单位：m/s
    params.v_max = 5.0;

    % 无人机最大加速度，也就是控制输入最大值，单位：m/s^2
    params.u_max = 3.0;

    % 最小飞行高度，单位：m
    params.z_min = 0.5;

    % 最大飞行高度，单位：m
    params.z_max = 20.0;

    % 无人机等效安全半径，单位：m
    params.r_u = 0.3;


    %% ========================================================================
    % 3. 任务起点、终点与参考路径参数
    % ========================================================================

    % 无人机起始位置，单位：m
    params.p_0 = [0; 0; 3];

    % 无人机初始速度，单位：m/s
    params.v_0 = [0; 0; 0];

    % 无人机目标位置，单位：m
    params.p_goal = [40; 40; 12];

    % 到达目标点的距离容差，单位：m
    % 当无人机距离目标点小于该值时，认为任务完成
    params.eps_tol = 0.5;

    % 全局参考路径离散点数量
    params.N_ref = 300;

    % -------------------------------------------------------------------------
    % 3.1 终点直接引导参数
    % -------------------------------------------------------------------------

    % 当无人机距离终点小于该半径时，不再追踪名义直线路径投影点，
    % 而是直接追踪终点，避免绕过障碍物后又被拉回原始直线路径，
    % 从而在终点附近绕大圈。
    %
    % 修改说明：
    % 原值 8.0 m 偏小，导致无人机绕过最后一个障碍物后仍然继续
    % 追踪局部名义路径，而不是直接飞向终点。这里增大到 18.0 m，
    % 使无人机在终点附近更早切换到直接终点引导。
    params.goal_direct_radius = 18.0;

    % 当无人机距离终点小于该半径时，进一步降低参考速度，
    % 避免冲过终点或者在终点附近来回振荡。
    params.goal_slow_radius = 6.0;

    % 当无人机距离终点小于该半径时，main_simulation.m 会检查
    % 当前点到终点的直线路径是否基本无障碍；如果直线路径可行，
    % 则直接追踪终点，而不是继续沿名义路径绕行。
    params.goal_line_of_sight_radius = 28.0;

    % 终点直线路径可见性检测的采样分辨率，单位：m。
    % 数值越小，检测越密集但计算稍慢；1.0 m 对当前场景较合适。
    params.goal_line_check_resolution = 1.0;

    % 终点直线路径检测中的静态障碍物额外安全裕度，单位：m。
    % 原默认保护如果过大，容易把已经可以直飞终点的情况误判为不可行。
    params.goal_line_static_margin = 0.5;

    % 终点直线路径检测中的动态障碍物额外安全裕度，单位：m。
    % 适当降低该值可以避免已经远离的动态障碍物继续阻止终点直飞。
    params.goal_line_dynamic_margin = 0.5;

    % 当无人机进入该半径后，solve_nmpc_casadi.m 中的终端参考点
    % 直接使用真实终点 params.p_goal，而不是局部参考轨迹最后一点。
    params.terminal_goal_attract_radius = 25.0;


    %% ========================================================================
    % 4. 动态障碍物预测与卡尔曼滤波参数
    % ========================================================================

    % 过程噪声标准差，用于卡尔曼预测中的过程不确定性
    params.sigma_q = 0.1;

    % 观测噪声标准差，用于卡尔曼预测中的测量不确定性
    params.sigma_r = 0.05;

    % 三维高斯置信椭球对应的卡方分布阈值
    % 自由度为 3，置信度约为 95%
    params.chi2_alpha = 7.81;

    % 额外安全裕度，单位：m
    params.d_m = 0.2;

    % 默认动态障碍物半径，单位：m
    params.default_obs_radius = 1.5;

    % 原始单障碍物实验中使用的较大障碍物半径，单位：m
    params.large_obs_radius = 2.0;

    % 动态障碍物状态初始协方差矩阵
    % 状态向量为 [px, py, pz, vx, vy, vz]'
    params.obs_cov_default = diag([0.1, 0.1, 0.1, 0.05, 0.05, 0.05]);


    %% ========================================================================
    % 5. 新增审稿意见回应参数：多动态障碍物与复杂运动模式
    % ========================================================================

    % -------------------------------------------------------------------------
    % 5.1 动态障碍物数量参数
    % -------------------------------------------------------------------------

    % 默认复杂场景中的动态障碍物数量
    % 建议测试：
    % 1 个障碍物：对应原始简单场景
    % 3 个障碍物：中等复杂动态场景
    % 5 个障碍物：复杂动态场景
    % 8 个障碍物：可扩展性压力测试场景
    params.num_dynamic_obstacles = 5;

    % 可扩展性分析中的障碍物数量列表
    % 该参数用于回应 Reviewer 1 第 4 条：
    % 需要分析计算时间和触发率随障碍物数量变化的可扩展性
    params.scalability_obs_list = [1, 3, 5, 8];

    % -------------------------------------------------------------------------
    % 5.2 动态障碍物运动模式编号
    % -------------------------------------------------------------------------

    % 运动模式 1：CV，constant velocity，匀速直线运动
    params.motion_type_CV = 1;

    % 运动模式 2：CA，constant acceleration，带恒定加速度运动
    params.motion_type_CA = 2;

    % 运动模式 3：SIN，sinusoidal motion，带正弦横向扰动的非线性运动
    params.motion_type_SIN = 3;

    % 运动模式 4：RAND，random maneuver，随机机动或不可预测运动
    params.motion_type_RAND = 4;

    % 如果关闭异质运动模式，所有动态障碍物默认采用该运动模式
    params.default_motion_type = params.motion_type_CV;

    % 如果启用异质运动模式，动态障碍物会从以下运动模式中分配
    params.heterogeneous_motion_types = ...
        [params.motion_type_CV, ...
         params.motion_type_CA, ...
         params.motion_type_SIN, ...
         params.motion_type_RAND];

    % -------------------------------------------------------------------------
    % 5.3 随机运动与不可预测机动参数
    % -------------------------------------------------------------------------

    % 随机加速度最大幅值，单位：m/s^2
    % 用于模拟真实环境中动态障碍物的不规则运动
    params.obs_random_acc_max = 0.8;

    % 随机加速度标准差，单位：m/s^2
    params.obs_random_acc_std = 0.35;

    % 每个采样时刻发生突然机动的概率
    % 例如 0.05 表示每一步有 5% 的概率发生速度突变
    params.obs_maneuver_prob = 0.05;

    % 突然速度变化的最大幅值，单位：m/s
    params.obs_velocity_jump_max = 0.6;

    % 动态障碍物最大速度，单位：m/s
    params.obs_v_max = 2.0;

    % 正弦横向运动的幅值，单位：m
    params.obs_sin_amp = 2.0;

    % 正弦横向运动角频率，单位：rad/s
    params.obs_sin_omega = 0.8;

    % CA 运动模式下的恒定加速度幅值，单位：m/s^2
    params.obs_ca_acc = 0.25;

    % 三维非线性运动中的垂直振荡幅值，单位：m
    params.obs_vertical_amp = 0.8;

    % 三维非线性运动中的垂直振荡角频率，单位：rad/s
    params.obs_vertical_omega = 0.5;


    %% ========================================================================
    % 6. 传感器观测噪声参数
    % ========================================================================

    % 普通鲁棒性测试中的位置观测噪声标准差，单位：m
    params.noise_pos_std = 0.4;

    % 普通鲁棒性测试中的速度观测噪声标准差，单位：m/s
    params.noise_vel_std = 0.2;

    % 压力测试中的位置观测噪声标准差，单位：m
    params.noise_pos_std_hard = 0.6;

    % 压力测试中的速度观测噪声标准差，单位：m/s
    params.noise_vel_std_hard = 0.3;


    %% ========================================================================
    % 7. 事件触发与复合风险评估参数
    % ========================================================================

    % 事件触发激活阈值
    % 当复合风险 R_k 大于等于该值时，启动 NMPC 局部优化
    params.R_on = 0.6;

    % 事件触发关闭阈值
    % 当复合风险 R_k 小于等于该值时，关闭 NMPC 局部优化
    params.R_off = 0.2;

    % 触发冷却时间，单位：采样步
    % 用于避免频繁切换和 Zeno-like 抖振
    params.T_ref = 5;

    % TCPA/DCPA 会遇风险权重
    params.w1 = 0.4;

    % 预测占据风险权重
    params.w2 = 0.4;

    % 轨迹偏离风险权重
    params.w3 = 0.2;

    % TCPA 时间归一化尺度，单位：s
    params.T_c = 3.0;

    % DCPA 距离归一化尺度，单位：m
    params.D_c = 2.0;

    % 最大允许偏离尺度，单位：m
    params.e_max = 2.0;

    % 防止除零的小正数
    params.eps = 1e-4;


    %% ========================================================================
    % 8. NMPC 目标函数权重参数
    % ========================================================================

    % 路径跟踪或路径长度代价权重
    params.alpha = 1.0;

    % 轨迹平滑性代价权重
    params.beta = 0.5;

    % 能耗代价权重
    params.gamma = 0.8;

    % 障碍物风险惩罚代价权重
    params.delta = 5.0;

    % -------------------------------------------------------------------------
    % 8.1 终端代价权重参数
    % -------------------------------------------------------------------------
    % 终端位置代价权重：
    % 用于强化 NMPC 预测窗口末端靠近局部参考轨迹最后一点。
    % 在终点直接引导阶段，局部参考轨迹最后一点接近或等于目标点。
    %
    % 修改说明：
    % 原值 35.0 偏弱，终点附近容易被障碍物风险项和局部参考路径项
    % 牵引，出现绕过最后障碍物后仍绕行的问题。这里增强为 120.0。
    params.terminal_pos_weight = 120.0;

    % 终端速度代价权重：
    % 用于抑制无人机在终点附近冲过目标点或绕圈。
    params.terminal_vel_weight = 20.0;

    % 预测时域内的渐进终点吸引权重。
    % 该项需要在 solve_nmpc_casadi.m 中使用，用于让预测窗口内每一步
    % 都逐渐靠近终点，而不仅仅依赖最后一步终端代价。
    params.terminal_path_weight = 8.0;

    % 终点附近障碍物软风险代价衰减半径，单位：m。
    % 该参数不是关闭避障，而是在终点附近且路径已经基本可行时，
    % 避免风险软代价过强导致继续绕圈。
    params.terminal_risk_decay_radius = 12.0;

    % 终点附近风险代价最低保留比例。
    % 0.45 表示至少保留 45% 的避障软代价，防止安全性被完全削弱。
    params.terminal_risk_min_factor = 0.45;

    % CasADi 平滑正部函数与平滑爬升项的数值小量。
    params.smooth_eps = 1e-4;

    % 动态障碍物软风险主惩罚增益。
    params.dyn_risk_gain = 3000;

    % 静态建筑物软风险主惩罚增益。
    params.static_risk_gain = 12000;

    % 静态建筑物近边界软惩罚增益。
    params.static_near_gain = 2500;

    % 静态建筑物规划安全半径的额外保守裕度，单位：m。
    params.static_extra_margin = 0.55;


    %% ========================================================================
    % 9. 能耗代价子权重参数
    % ========================================================================

    % 基础平移距离能耗代理项权重
    params.c1 = 1.0;

    % 控制输入能耗权重
    params.c2 = 0.1;

    % 非对称爬升惩罚权重
    % 如果启用非对称能耗，则惩罚爬升动作；
    % 如果关闭，则该项为 0，用于消融实验
    if params.use_asymmetric_energy
        params.c3 = 3.0;
    else
        params.c3 = 0.0;
    end


    %% ========================================================================
    % 10. CasADi 与 IPOPT 求解器参数
    % ========================================================================

    % IPOPT 最大迭代次数
    params.ipopt_max_iter = 100;

    % IPOPT 可接受收敛容差
    params.ipopt_acceptable_tol = 1e-4;

    % IPOPT 打印等级
    % 0 表示不输出求解器详细信息
    params.ipopt_print_level = 0;

    % 是否展开 CasADi 计算图
    % true 通常可以加快求解速度，但会增加初始化开销
    params.casadi_expand = true;


    %% ========================================================================
    % 11. Monte Carlo 与可扩展性分析参数
    % ========================================================================

    % Monte Carlo 独立重复实验次数
    % 审稿意见中建议增强统计可靠性，因此推荐使用 100 次或更多
    params.num_mc_runs = 2;

    % 每个障碍物数量设置下的重复实验次数
    % 用于可扩展性分析
    params.num_scalability_repeats = 100;

    % 随机数种子
    % 固定随机种子可以保证每次实验结果可复现
    % 如果希望每次都完全随机，可以设置为 []
    params.random_seed = 2026;

    % 是否保存结果表格为 CSV 文件
    params.save_csv = true;

    % 结果输出文件夹
    params.output_folder = 'results_reviewer_response';

    % 可扩展性分析结果文件名
    params.scalability_result_file = 'scalability_multi_obstacle_results.csv';

    % 不同运动模式鲁棒性实验结果文件名
    params.motion_robustness_result_file = 'motion_robustness_results.csv';


    %% ========================================================================
    % 12. 碰撞检测与任务成功判定参数
    % ========================================================================

    % 碰撞检测额外缓冲距离，单位：m
    % 如果无人机与障碍物距离小于 r_u + r_i + collision_buffer，则判定碰撞
    params.collision_buffer = 0.0;

    % 结果统计中最小安全距离的报告阈值，单位：m
    params.report_min_dist_threshold = 0.0;

    % 是否检测静态建筑物碰撞
    params.check_static_collision = true;

    % 是否检测动态障碍物碰撞
    params.check_dynamic_collision = true;


    %% ========================================================================
    % 13. 可视化与图像保存参数
    % ========================================================================

    % 是否在 main_simulation.m 中显示动画
    params.enable_animation = true;

    % 是否保存仿真图片
    params.save_figures = true;

    % 图片保存分辨率
    params.figure_resolution = 300;

    % 图像显示中的 X 轴范围
    params.x_lim = [-5, 45];

    % 图像显示中的 Y 轴范围
    params.y_lim = [-5, 45];

    % 图像显示中的 Z 轴范围
    params.z_lim = [0, 20];


    %% ========================================================================
    % 14. APF 对比算法参数
    % ========================================================================

    % APF 引力增益
    params.apf_k_att = 1.2;

    % APF 斥力增益
    params.apf_k_rep = 30.0;

    % APF 斥力影响半径，单位：m
    params.apf_rho_0 = 6.0;

    % APF 局部极小值检测速度阈值
    params.apf_trap_speed_threshold = 0.05;

    % APF 局部极小值检测合力阈值
    params.apf_trap_force_threshold = 0.1;


    %% ========================================================================
    % 15. 参数合法性检查
    % ========================================================================

    % 检查事件触发激活阈值是否大于关闭阈值
    if params.R_on <= params.R_off
        error('参数错误：R_on 必须大于 R_off。');
    end

    % 检查终点直接引导半径是否大于减速半径
    if params.goal_direct_radius <= params.goal_slow_radius
        error('参数错误：goal_direct_radius 必须大于 goal_slow_radius。');
    end

    % 检查终点直线路径可见性半径是否大于直接引导半径
    if params.goal_line_of_sight_radius <= params.goal_direct_radius
        error('参数错误：goal_line_of_sight_radius 必须大于 goal_direct_radius。');
    end

    % 检查终点直线路径检测分辨率是否为正
    if params.goal_line_check_resolution <= 0
        error('参数错误：goal_line_check_resolution 必须大于 0。');
    end

    % 检查复合风险权重是否非负
    if params.w1 < 0 || params.w2 < 0 || params.w3 < 0
        error('参数错误：风险权重 w1、w2、w3 必须为非负数。');
    end

    % 检查预测时域长度是否为正
    if params.H <= 0
        error('参数错误：预测步长 H 必须大于 0。');
    end

    % 检查动态障碍物数量是否至少为 1
    if params.num_dynamic_obstacles < 1
        error('参数错误：num_dynamic_obstacles 必须至少为 1。');
    end

    % 检查终端代价权重是否非负
    if params.terminal_pos_weight < 0 || params.terminal_vel_weight < 0
        error('参数错误：terminal_pos_weight 和 terminal_vel_weight 必须为非负数。');
    end

    % 检查终点吸引和风险衰减参数是否合法
    if params.terminal_goal_attract_radius <= 0
        error('参数错误：terminal_goal_attract_radius 必须大于 0。');
    end

    if params.terminal_path_weight < 0
        error('参数错误：terminal_path_weight 必须为非负数。');
    end

    if params.terminal_risk_decay_radius <= 0
        error('参数错误：terminal_risk_decay_radius 必须大于 0。');
    end

    if params.terminal_risk_min_factor < 0 || params.terminal_risk_min_factor > 1
        error('参数错误：terminal_risk_min_factor 必须位于 [0, 1] 区间内。');
    end

    % 如果需要保存 CSV 或图片，则自动创建输出文件夹
    if params.save_csv || params.save_figures
        if ~exist(params.output_folder, 'dir')
            mkdir(params.output_folder);
        end
    end

end
