#pragma once 
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/factory.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <Eigen/Core>
#include <boost/thread/once.hpp>
#include <boost/shared_ptr.hpp>
#include <vector>
#include <iostream>
#include <iomanip>
#include <boost/make_shared.hpp>
#include <memory>

namespace teb_local_planner
{
// -----------------------------------------------------------------------------
// 全局配置
// -----------------------------------------------------------------------------
struct TebConfig
{
    // 障碍物约束
    double min_obstacle_dist = 2;
    double penalty_epsilon   = 0.05;
    double obstacle_weight   = 10;
    
    // 路径跟踪
    double weight_viapoint   = 1;
    
    // 速度约束
    double max_vel           = 1.0;
    double max_vel_theta     = 1.0;
    double max_vel_x_backwards = 0.3;
    double weight_max_vel_x    = 1;
    double weight_max_vel_theta = 1;
    
    // 运动学约束
    double weight_kinematics_nh  = 1;
    double weight_kinematics_forward_drive = 1;
    
    // ✅ 新增：转弯半径约束
    double min_turning_radius = 0.3;      // 最小转弯半径（米）
    double weight_curvature   = 10.0;     // 曲率约束权重
    
    // ✅ 新增：终点约束
    double weight_goal_position = 100.0;  // 终点位置权重
    double weight_goal_orientation = 10.0; // 终点朝向权重
    double goal_tolerance = 0.05;         // 终点容差（米）

    // 优化器配置
    bool optimization_verbose = true;
    int no_inner_iterations = 5;
    int no_outer_iterations = 4;

    // 局部优化距离
    double local_distance = 3.0;

    // Jerk约束参数
    double max_jerk           = 1.0;
    double max_jerk_theta     = 1.0;
    double weight_jerk        = 10.0;
};

// -----------------------------------------------------------------------------
// 1. 一元边基类
// -----------------------------------------------------------------------------
template <int D, typename E, typename VertexXi>
class BaseTebUnaryEdge : public g2o::BaseUnaryEdge<D, E, VertexXi>
{
public:
    using typename g2o::BaseUnaryEdge<D, E, VertexXi>::ErrorVector;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::computeError;

    BaseTebUnaryEdge() { this->_vertices[0] = nullptr; }
    virtual ~BaseTebUnaryEdge() = default;

    ErrorVector& getError()
    {
        computeError();
        return this->_error;
    }

    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

protected:
    const TebConfig* cfg_ = nullptr;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::_error;
    using g2o::BaseUnaryEdge<D, E, VertexXi>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// -----------------------------------------------------------------------------
// 2. 二元边基类
// -----------------------------------------------------------------------------
template <int D, typename E, typename VertexXi, typename VertexXj>
class BaseTebBinaryEdge : public g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>
{
public:
    using typename g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::ErrorVector;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::computeError;

    BaseTebBinaryEdge()
    {
        this->_vertices[0] = nullptr;
        this->_vertices[1] = nullptr;
    }

    virtual ~BaseTebBinaryEdge() = default;

    ErrorVector& getError()
    {
        computeError();
        return this->_error;
    }

    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

protected:
    const TebConfig* cfg_ = nullptr;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_error;
    using g2o::BaseBinaryEdge<D, E, VertexXi, VertexXj>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// -----------------------------------------------------------------------------
// 3. 多元边基类
// -----------------------------------------------------------------------------
template <int D, typename E>
class BaseTebMultiEdge : public g2o::BaseMultiEdge<D, E>
{
public:
    using typename g2o::BaseMultiEdge<D, E>::ErrorVector;
    using g2o::BaseMultiEdge<D, E>::computeError;

    BaseTebMultiEdge() {}
    virtual ~BaseTebMultiEdge() = default;

    ErrorVector& getError()
    {
        computeError();
        return this->_error;
    }

    virtual bool read(std::istream& is) override { return true; }
    virtual bool write(std::ostream& os) const override { return os.good(); }

    void setTebConfig(const TebConfig& cfg) { cfg_ = &cfg; }

protected:
    const TebConfig* cfg_ = nullptr;
    using g2o::BaseMultiEdge<D, E>::_error;
    using g2o::BaseMultiEdge<D, E>::_vertices;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace teb_local_planner