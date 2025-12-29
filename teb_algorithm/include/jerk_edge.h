#pragma once 
#include "base_teb_edges.h"
#include "vertexPoint.h"
#include "tools.h"
#include "vertexTImeDiff.h" 

namespace teb_local_planner
{

/**
 * @brief 多节点平滑Jerk约束边 - 中心差分法（精确版本）
 * 
 * 改进点：
 * 1. 使用中心差分法计算加速度（精度：一阶→二阶）
 * 2. 精确计算速度对应的时间中点
 * 3. 充分利用所有5个位姿点和4个时间段
 * 
 * 核心公式：
 * - 速度: v_k = dist_k / t_k（对应t_k段的中点时刻）
 * - 加速度（中心差分）: a_at_P1 = (v_k2 - v_k) / (0.5·t_k + t_k1 + 0.5·t_k2)
 * - Jerk: j = (a_at_P2 - a_at_P1) / smooth_dt
 * 
 * 连接顶点:
 * - 5个位姿顶点: x_k, x_{k+1}, x_{k+2}, x_{k+3}, x_{k+4}
 * - 4个时间差顶点: ΔT_k, ΔT_{k+1}, ΔT_{k+2}, ΔT_{k+3}
 */
class EdgeJerkConstraint : public BaseTebMultiEdge<2, Eigen::VectorXd>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeJerkConstraint()
    {
        resize(9);  // 连接9个顶点 (5个位姿 + 4个时间差)
    }

    /**
     * @brief 计算误差 - 使用精确的中心差分法
     * 
     * 时间对应关系详解：
     * 
     * 位姿点:  P0 -------- P1 -------- P2 -------- P3 -------- P4
     *             t_k        t_k1        t_k2        t_k3
     * 
     * 速度:       v_k         v_k1        v_k2        v_k3
     *          (t_k中点)   (t_k1中点)  (t_k2中点)  (t_k3中点)
     * 
     * 加速度计算（中心差分）:
     * a_at_P1 = (v_k2 - v_k) / (0.5·t_k + t_k1 + 0.5·t_k2)
     *           └─────────┘    └─────────────────────────┘
     *           跨越P1的速度    从v_k中点到v_k2中点的时间
     * 
     * a_at_P2 = (v_k3 - v_k1) / (0.5·t_k1 + t_k2 + 0.5·t_k3)
     *           └──────────┘    └──────────────────────────┘
     *           跨越P2的速度    从v_k1中点到v_k3中点的时间
     */
    virtual void computeError() override
    {
        if (!cfg_)
        {
            std::cerr << "Error: TebConfig not set for EdgeJerkConstraint!" << std::endl;
            _error.setZero();
            return;
        }
        
        // ============ 获取顶点 ============
        const VertexPoint2D* pose_k   = static_cast<const VertexPoint2D*>(_vertices[0]);
        const VertexPoint2D* pose_k1  = static_cast<const VertexPoint2D*>(_vertices[1]);
        const VertexPoint2D* pose_k2  = static_cast<const VertexPoint2D*>(_vertices[2]);
        const VertexPoint2D* pose_k3  = static_cast<const VertexPoint2D*>(_vertices[3]);
        const VertexPoint2D* pose_k4  = static_cast<const VertexPoint2D*>(_vertices[4]);
        
        const vertexTimeDiff* dt_k   = static_cast<const vertexTimeDiff*>(_vertices[5]);
        const vertexTimeDiff* dt_k1  = static_cast<const vertexTimeDiff*>(_vertices[6]);
        const vertexTimeDiff* dt_k2  = static_cast<const vertexTimeDiff*>(_vertices[7]);
        const vertexTimeDiff* dt_k3  = static_cast<const vertexTimeDiff*>(_vertices[8]);

        Eigen::Vector3d p_k   = pose_k->estimate();
        Eigen::Vector3d p_k1  = pose_k1->estimate();
        Eigen::Vector3d p_k2  = pose_k2->estimate();
        Eigen::Vector3d p_k3  = pose_k3->estimate();
        Eigen::Vector3d p_k4  = pose_k4->estimate();

        double t_k   = dt_k->estimate();
        double t_k1  = dt_k1->estimate();
        double t_k2  = dt_k2->estimate();
        double t_k3  = dt_k3->estimate();

        const double eps = 1e-6;  // 数值稳定性

        // ============ 线性运动的Jerk（精确中心差分法）============
        
        // 步骤1: 计算4段的线速度
        double dist_k   = sqrt(pow(p_k1[0] - p_k[0], 2) + pow(p_k1[1] - p_k[1], 2));
        double dist_k1  = sqrt(pow(p_k2[0] - p_k1[0], 2) + pow(p_k2[1] - p_k1[1], 2));
        double dist_k2  = sqrt(pow(p_k3[0] - p_k2[0], 2) + pow(p_k3[1] - p_k2[1], 2));
        double dist_k3  = sqrt(pow(p_k4[0] - p_k3[0], 2) + pow(p_k4[1] - p_k3[1], 2));

        double v_k   = dist_k / (t_k + eps);
        double v_k1  = dist_k1 / (t_k1 + eps);
        double v_k2  = dist_k2 / (t_k2 + eps);
        double v_k3  = dist_k3 / (t_k3 + eps);

        // 步骤2: 使用精确的中心差分计算加速度
        // 
        // 关键：v_k对应t_k段的中点，v_k2对应t_k2段的中点
        // 从v_k到v_k2的时间跨度 = 0.5·t_k + t_k1 + 0.5·t_k2
        //
        // 可视化：
        //   P0 ---t_k--- P1 ---t_k1--- P2 ---t_k2--- P3
        //      v_k(中)          v_k1(中)        v_k2(中)
        //      ↑                                ↑
        //      └────────────────────────────────┘
        //          0.5t_k + t_k1 + 0.5t_k2
        
        // 在P1位置的加速度（使用v_k和v_k2）
        double dt_for_a_at_P1 = 0.5 * t_k + t_k1 + 0.5 * t_k2;
        double a_at_P1 = (v_k2 - v_k) / (dt_for_a_at_P1 + eps);
        
        // 在P2位置的加速度（使用v_k1和v_k3）
        double dt_for_a_at_P2 = 0.5 * t_k1 + t_k2 + 0.5 * t_k3;
        double a_at_P2 = (v_k3 - v_k1) / (dt_for_a_at_P2 + eps);

        // 步骤3: 计算线性Jerk
        // 保持原来的加权平滑时间分母策略（给中间时间段更大权重）
        double smooth_dt = 0.25 * t_k + 0.75 * t_k1 + 0.75 * t_k2 + 0.25 * t_k3;
        double jerk_linear = (a_at_P2 - a_at_P1) / (smooth_dt + eps);

        // ============ 角运动的Jerk（精确中心差分法）============
        
        // 步骤1: 计算4段的角速度
        double dtheta_k   = tools::normalize_theta(p_k1[2] - p_k[2]);
        double dtheta_k1  = tools::normalize_theta(p_k2[2] - p_k1[2]);
        double dtheta_k2  = tools::normalize_theta(p_k3[2] - p_k2[2]);
        double dtheta_k3  = tools::normalize_theta(p_k4[2] - p_k3[2]);

        double w_k   = dtheta_k / (t_k + eps);
        double w_k1  = dtheta_k1 / (t_k1 + eps);
        double w_k2  = dtheta_k2 / (t_k2 + eps);
        double w_k3  = dtheta_k3 / (t_k3 + eps);

        // 步骤2: 使用精确的中心差分计算角加速度
        double alpha_at_P1 = (w_k2 - w_k) / (dt_for_a_at_P1 + eps);
        double alpha_at_P2 = (w_k3 - w_k1) / (dt_for_a_at_P2 + eps);

        // 步骤3: 计算角Jerk
        double jerk_angular = (alpha_at_P2 - alpha_at_P1) / (smooth_dt + eps);

        // ============ 应用惩罚函数 ============
        _error[0] = tools::penaltyBoundToInterval(jerk_linear, cfg_->max_jerk, cfg_->penalty_epsilon);
        _error[1] = tools::penaltyBoundToInterval(jerk_angular, cfg_->max_jerk_theta, cfg_->penalty_epsilon);
    }

    void setjerkcfg(const TebConfig* cfg)
    {
        cfg_ = cfg;
    }

    virtual bool read(std::istream& /*is*/) override
    {
        return true;
    }

    virtual bool write(std::ostream& /*os*/) const override
    {
        return true;
    }

protected:
    const TebConfig* cfg_;
};

}  // namespace teb_local_planner