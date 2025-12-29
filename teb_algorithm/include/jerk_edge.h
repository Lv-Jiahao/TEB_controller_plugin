
#pragma once 
#include "base_teb_edges.h"
#include "vertexPoint.h"
#include "tools.h"
#include "vertexTImeDiff.h" 

namespace teb_local_planner
{

/**
 * @brief 多节点平滑Jerk约束边
 * 
 * 公式: j_k = (a_{k+1} - a_k) / (0.25ΔT_k + 0.75ΔT_{k+1} + 0.75ΔT_{k+2} + 0.25ΔT_{k+3})
 * 
 * 连接顶点:
 * - 5个位姿顶点: x_k, x_{k+1}, x_{k+2}, x_{k+3}, x_{k+4}
 * - 4个时间差顶点: ΔT_k, ΔT_{k+1}, ΔT_{k+2}, ΔT_{k+3}
 * 
 * 总共9个顶点
 */
class EdgeJerkConstraint : public BaseTebMultiEdge<2, Eigen::VectorXd>  // 2维误差(线性jerk + 角度jerk)
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    EdgeJerkConstraint()
    {
        resize(9);  // 关键: 连接9个顶点 (5个位姿 + 4个时间差)
    }

    virtual void computeError() override
    {
        if (!cfg_)
        {
            std::cerr << "Error: TebConfig not set for EdgeJerkConstraint!" << std::endl;
            _error.setZero();
            return;
        }
        
        // 获取5个位姿顶点
        const VertexPoint2D* pose_k   = static_cast<const VertexPoint2D*>(_vertices[0]);
        const VertexPoint2D* pose_k1  = static_cast<const VertexPoint2D*>(_vertices[1]);
        const VertexPoint2D* pose_k2  = static_cast<const VertexPoint2D*>(_vertices[2]);
        const VertexPoint2D* pose_k3  = static_cast<const VertexPoint2D*>(_vertices[3]);
        const VertexPoint2D* pose_k4  = static_cast<const VertexPoint2D*>(_vertices[4]);
        
        // 获取4个时间差顶点
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

        // 计算速度 v_k, v_{k+1}, v_{k+2}, v_{k+3}
        double dist_k   = sqrt(pow(p_k1[0] - p_k[0], 2) + pow(p_k1[1] - p_k[1], 2));
        double dist_k1  = sqrt(pow(p_k2[0] - p_k1[0], 2) + pow(p_k2[1] - p_k1[1], 2));
        double dist_k2  = sqrt(pow(p_k3[0] - p_k2[0], 2) + pow(p_k3[1] - p_k2[1], 2));
        double dist_k3  = sqrt(pow(p_k4[0] - p_k3[0], 2) + pow(p_k4[1] - p_k3[1], 2));

        double v_k   = dist_k / (t_k + 1e-6);
        double v_k1  = dist_k1 / (t_k1 + 1e-6);
        double v_k2  = dist_k2 / (t_k2 + 1e-6);
        double v_k3  = dist_k3 / (t_k3 + 1e-6);

        // 计算加速度 a_k, a_{k+1} (论文公式11)
        // a_k = (v_{k+1} - v_k) / ((ΔT_k + ΔT_{k+1}) / 2)
        double a_k   = (v_k1 - v_k) / ((t_k + t_k1) / 2.0 + 1e-6);
        double a_k1  = (v_k2 - v_k1) / ((t_k1 + t_k2) / 2.0 + 1e-6);

        // 计算平滑时间分母 (论文公式15)
        double smooth_dt = 0.25 * t_k + 0.75 * t_k1 + 0.75 * t_k2 + 0.25 * t_k3;

        // 计算线性jerk
        double jerk_linear = (a_k1 - a_k) / (smooth_dt + 1e-6);

        // 计算角速度 w_k, w_{k+1}, w_{k+2}, w_{k+3}
        double dtheta_k   = tools::normalize_theta(p_k1[2] - p_k[2]);
        double dtheta_k1  = tools::normalize_theta(p_k2[2] - p_k1[2]);
        double dtheta_k2  = tools::normalize_theta(p_k3[2] - p_k2[2]);
        double dtheta_k3  = tools::normalize_theta(p_k4[2] - p_k3[2]);

        double w_k   = dtheta_k / (t_k + 1e-6);
        double w_k1  = dtheta_k1 / (t_k1 + 1e-6);
        double w_k2  = dtheta_k2 / (t_k2 + 1e-6);
        double w_k3  = dtheta_k3 / (t_k3 + 1e-6);

        // 计算角加速度
        double alpha_k   = (w_k1 - w_k) / ((t_k + t_k1) / 2.0 + 1e-6);
        double alpha_k1  = (w_k2 - w_k1) / ((t_k1 + t_k2) / 2.0 + 1e-6);

        // 计算角度jerk
        double jerk_angular = (alpha_k1 - alpha_k) / (smooth_dt + 1e-6);

        // 使用惩罚函数限制jerk在合理范围内
        _error[0] = tools::penaltyBoundToInterval(jerk_linear, cfg_->max_jerk, cfg_->penalty_epsilon);
        _error[1] = tools::penaltyBoundToInterval(jerk_angular, cfg_->max_jerk_theta, cfg_->penalty_epsilon);
    }

    void setjerkcfg(const TebConfig* cfg)
    {   // 这里实际优化的是 max_jerk，max_jerk_theta 只需要传一个  配置文件即可
        cfg_ = cfg;
    }
};

}  // namespace teb_local_planner