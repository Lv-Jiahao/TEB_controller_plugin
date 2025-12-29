#pragma once 
#include "base_teb_edges.h"
#include "vertexPoint.h"
#include "tools.h"

namespace teb_local_planner
{

/**
 * @brief 目标点约束边 - 强制路径终点到达目标位置和朝向
 * 连接: 1个位姿顶点
 * 测量: 目标点 (x, y, theta)
 */
class EdgeGoalPoseConstraint : public BaseTebUnaryEdge<3, tools::pathInfo, VertexPoint2D>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    virtual void computeError() override
    {
        if (!cfg_)
        {
            std::cerr << "Error: TebConfig not set for EdgeGoalPoseConstraint!" << std::endl;
            _error.setZero();
            return;
        }

        // 获取当前位姿估计
        const VertexPoint2D* pose = static_cast<const VertexPoint2D*>(_vertices[0]);
        Eigen::Vector3d current = pose->estimate();
        
        // 计算与目标点的误差
        // _measurement 存储目标点 (x, y, theta)
        _error[0] = current[0] - _measurement.x;  // x方向误差
        _error[1] = current[1] - _measurement.y;  // y方向误差
        
        // 角度误差(归一化到 [-π, π])
        double angle_diff = current[2] - _measurement.theta;
        _error[2] = tools::normalize_theta(angle_diff);
    }

    void setGoalPose(const tools::pathInfo& goal, const TebConfig* cfg)
    {
        _measurement = goal;  // 目标位姿
        cfg_ = cfg;
    }
};

} // namespace teb_local_planner