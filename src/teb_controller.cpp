// teb_controller_improved.cpp - 改进版
#include "teb_controller_plugin/teb_controller.hpp"
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/utils.h>

namespace teb_controller
{

void TebController::configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, 
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{ 
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock node!");
  }

  node_ = node;
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // 参数声明
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_vel_x", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_vel_x", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_vel_theta", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));
  
  // ✅ 新增参数
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".goal_tolerance", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_lookahead_dist", rclcpp::ParameterValue(0.3));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_lookahead_dist", rclcpp::ParameterValue(1.0));

  node->get_parameter(plugin_name_ + ".max_vel_x", max_vel_x_);
  node->get_parameter(plugin_name_ + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(plugin_name_ + ".min_vel_x", min_vel_x_);
  node->get_parameter(plugin_name_ + ".min_vel_theta", min_vel_theta_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);

  // TEB 参数配置
  teb_config_.no_inner_iterations = 5;  // ✅ 增加迭代次数
  teb_config_.no_outer_iterations = 2;
  teb_config_.min_obstacle_dist = 0.5;
  teb_config_.penalty_epsilon = 0.1;
  teb_config_.obstacle_weight = 10;
  teb_config_.optimization_verbose = false;
  
  // ✅ 路径跟踪 - 降低权重,避免过拟合
  teb_config_.weight_viapoint = 10.0;  // 从100降到10
  
  // ✅ 终点约束 - 添加强约束
  teb_config_.weight_goal_position = 50.0;
  teb_config_.weight_goal_orientation = 10.0;
  teb_config_.goal_tolerance = 0.05;
  
  // 速度约束
  teb_config_.max_vel = max_vel_x_;
  teb_config_.max_vel_theta = max_vel_theta_;
  teb_config_.max_vel_x_backwards = 0.0;
  teb_config_.weight_max_vel_x = 1.0;
  teb_config_.weight_max_vel_theta = 1.0;
  
  // 运动学约束
  teb_config_.weight_kinematics_nh = 100;
  teb_config_.weight_kinematics_forward_drive = 100;
  
  // Jerk 约束
  teb_config_.max_jerk = 2.0;
  teb_config_.max_jerk_theta = 2.0;
  teb_config_.weight_jerk = 5.0;
  
  // ✅ 动态局部距离
  teb_config_.local_distance = 5.0;

  teb_planner_ = nullptr;
  goal_reached_ = false;
  last_optimization_time_ = clock_->now();
  last_cmd_vel_.linear.x = 0.0;
  last_cmd_vel_.angular.z = 0.0;

  RCLCPP_INFO(logger_, "TEB Controller configured (improved version)");
}

void TebController::cleanup()
{
  RCLCPP_INFO(logger_, "TEB Controller cleanup");
  std::lock_guard<std::mutex> lock(planner_mutex_);
  teb_planner_.reset();
}

void TebController::activate()
{
  RCLCPP_INFO(logger_, "TEB Controller activated");
}

void TebController::deactivate()
{
  RCLCPP_INFO(logger_, "TEB Controller deactivated");
}

void TebController::setPlan(const nav_msgs::msg::Path & path)
{
  global_plan_ = path;
  goal_reached_ = false;
  
  std::lock_guard<std::mutex> lock(planner_mutex_);
  teb_planner_.reset();
  
  last_cmd_vel_.linear.x = 0.0;
  last_cmd_vel_.angular.z = 0.0;
}

geometry_msgs::msg::TwistStamped TebController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,
    nav2_core::GoalChecker *goal_checker)
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.linear.y = 0.0;
  cmd_vel.twist.angular.z = 0.0;

  // ✅ 检查是否到达目标
  if (goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity)) {
    goal_reached_ = true;
    RCLCPP_INFO(logger_, "Goal reached!");
    return cmd_vel;
  }

  // ✅ 计算到目标的距离,动态调整局部窗口
  double dist_to_goal = std::hypot(
    global_plan_.poses.back().pose.position.x - pose.pose.position.x,
    global_plan_.poses.back().pose.position.y - pose.pose.position.y
  );
  
  // 接近目标时缩小局部窗口
  if (dist_to_goal < 1.0) {
    teb_config_.local_distance = std::max(1.5, dist_to_goal + 0.5);
  } else {
    teb_config_.local_distance = 3.0;
  }

  std::vector<tools::pathInfo> local_plan = transformGlobalPlan(pose);
  
  if (local_plan.size() < 3) {
    RCLCPP_WARN(logger_, "Not enough waypoints: %zu", local_plan.size());
    return cmd_vel;
  }

  auto current_time = clock_->now();
  auto time_diff = (current_time - last_optimization_time_).seconds();
  
  const double MIN_OPTIMIZATION_INTERVAL = 0.1;
  bool should_optimize = (time_diff >= MIN_OPTIMIZATION_INTERVAL);
  
  std::vector<tools::pathInfo> optimized_plan;
  
  if (should_optimize)
  {
    try {
      std::lock_guard<std::mutex> lock(planner_mutex_);
      
      if (!teb_planner_) {
        teb_planner_ = std::make_shared<teb_local_planner::plannerManager>(teb_config_);
        RCLCPP_INFO(logger_, "TEB planner initialized");
      }
      
      teb_planner_->setpathInfo(local_plan);
      teb_planner_->runOptimization();
      teb_planner_->getPlannerResults(optimized_plan);

      if (optimized_plan.size() < 2) {
        optimized_plan = local_plan;
      } else {
        last_optimization_time_ = current_time;
      }
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(logger_, "TEB exception: %s", e.what());
      optimized_plan = local_plan;
    }
  } else {
    optimized_plan = local_plan;
  }

  // ✅ 改进的 Pure Pursuit 控制
  tools::pathInfo current_pose = optimized_plan[0];
  
  // 动态 lookahead 距离
  double current_vel = std::hypot(velocity.linear.x, velocity.linear.y);
  double adaptive_lookahead = std::max(0.3, std::min(1.0, current_vel * 0.8 + 0.3));
  
  tools::pathInfo lookahead_point = optimized_plan.back();  // 默认最后一个点
  double accumulated_dist = 0.0;
  
  for (size_t i = 1; i < optimized_plan.size(); ++i)
  {
    double dx = optimized_plan[i].x - optimized_plan[i-1].x;
    double dy = optimized_plan[i].y - optimized_plan[i-1].y;
    accumulated_dist += sqrt(dx*dx + dy*dy);
    
    if (accumulated_dist >= adaptive_lookahead) {
      lookahead_point = optimized_plan[i];
      break;
    }
  }
  
  double dx = lookahead_point.x - current_pose.x;
  double dy = lookahead_point.y - current_pose.y;
  double target_dist = sqrt(dx*dx + dy*dy);
  double target_angle = atan2(dy, dx);
  
  double angle_diff = target_angle - current_pose.theta;
  while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
  while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
  
  // ✅ 接近目标时降速
  double linear_vel;
  if (dist_to_goal < 0.5) {
    linear_vel = max_vel_x_ * 0.3;  // 目标附近慢速
  } else if (fabs(angle_diff) < 0.3) {
    linear_vel = max_vel_x_;
  } else if (fabs(angle_diff) < 0.8) {
    linear_vel = max_vel_x_ * 0.6;
  } else {
    linear_vel = max_vel_x_ * 0.3;
  }
  
  double curvature = 2.0 * sin(angle_diff) / (target_dist + 1e-6);
  double angular_vel = curvature * linear_vel;
  
  // 限幅
  linear_vel = std::max(min_vel_x_, std::min(max_vel_x_, linear_vel));
  angular_vel = std::max(-max_vel_theta_, std::min(max_vel_theta_, angular_vel));
  
  // 平滑滤波
  const double alpha = 0.3;
  linear_vel = alpha * linear_vel + (1 - alpha) * last_cmd_vel_.linear.x;
  angular_vel = alpha * angular_vel + (1 - alpha) * last_cmd_vel_.angular.z;
  
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;
  last_cmd_vel_ = cmd_vel.twist;
  
  return cmd_vel;
}

std::vector<tools::pathInfo> TebController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
    std::vector<tools::pathInfo> local_plan;
    
    if (global_plan_.poses.empty()) {
        return local_plan;
    }

    // 找到最近点
    size_t closest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
        double dx = global_plan_.poses[i].pose.position.x - pose.pose.position.x;
        double dy = global_plan_.poses[i].pose.position.y - pose.pose.position.y;
        double dist = std::sqrt(dx*dx + dy*dy);
        
        if (dist < min_dist) {
            min_dist = dist;
            closest_idx = i;
        }
    }

    // 计算到目标的距离
    double dist_to_goal = std::hypot(
        global_plan_.poses.back().pose.position.x - pose.pose.position.x,
        global_plan_.poses.back().pose.position.y - pose.pose.position.y
    );
    
    // ⭐ 关键改进:更精确的阈值
    const double GOAL_APPROACH_DISTANCE = 2.0;  // 2米内算接近目标
    bool near_goal = (dist_to_goal < GOAL_APPROACH_DISTANCE);
    
    // 添加当前位置
    tools::pathInfo current_point;
    current_point.x = pose.pose.position.x;
    current_point.y = pose.pose.position.y;
    current_point.theta = tf2::getYaw(pose.pose.orientation);
    local_plan.push_back(current_point);
    
    // 构建局部路径
    size_t start_idx = std::min(closest_idx + 1, global_plan_.poses.size() - 1);
    double accumulated_distance = 0.0;
    size_t prev_idx = closest_idx;
    
    const size_t MIN_POINTS = 15;
    const double MAX_DISTANCE = near_goal ? 100.0 : teb_config_.local_distance;  // ⭐ 接近时取大值
    
    for (size_t i = start_idx; i < global_plan_.poses.size(); ++i) {
        double dx = global_plan_.poses[i].pose.position.x - 
                    global_plan_.poses[prev_idx].pose.position.x;
        double dy = global_plan_.poses[i].pose.position.y - 
                    global_plan_.poses[prev_idx].pose.position.y;
        accumulated_distance += std::sqrt(dx*dx + dy*dy);
        prev_idx = i;
        
        tools::pathInfo path_point;
        path_point.x = global_plan_.poses[i].pose.position.x;
        path_point.y = global_plan_.poses[i].pose.position.y;
        
        if (i < global_plan_.poses.size() - 1) {
            double dx_next = global_plan_.poses[i+1].pose.position.x - 
                           global_plan_.poses[i].pose.position.x;
            double dy_next = global_plan_.poses[i+1].pose.position.y - 
                           global_plan_.poses[i].pose.position.y;
            path_point.theta = std::atan2(dy_next, dx_next);
        } else {
            // 最后一个点使用全局路径的朝向
            path_point.theta = tf2::getYaw(global_plan_.poses[i].pose.orientation);
        }
        
        local_plan.push_back(path_point);
        
        // ⭐ 接近目标时必须包含所有剩余点
        if (near_goal) {
            continue;  // 不检查距离,直到终点
        }
        
        // 否则按距离截断
        if (accumulated_distance >= MAX_DISTANCE && local_plan.size() >= MIN_POINTS) {
            break;
        }
    }
    
    // ⭐ 强制确保目标点在局部路径中
    if (near_goal) {
        tools::pathInfo& last_point = local_plan.back();
        tools::pathInfo goal_point;
        goal_point.x = global_plan_.poses.back().pose.position.x;
        goal_point.y = global_plan_.poses.back().pose.position.y;
        goal_point.theta = tf2::getYaw(global_plan_.poses.back().pose.orientation);
        
        // 检查最后一点是否是目标点
        double dist_last_to_goal = std::hypot(
            last_point.x - goal_point.x,
            last_point.y - goal_point.y
        );
        
        if (dist_last_to_goal > 0.01) {  // 不是目标点
            local_plan.push_back(goal_point);
            RCLCPP_INFO(logger_, "强制添加目标点到局部路径");
        }
    }
    
    RCLCPP_DEBUG(logger_, "局部路径点数: %zu, 到目标距离: %.2f", 
                 local_plan.size(), dist_to_goal);
    
    return local_plan;
}

std::vector<tools::obstacleInfo> TebController::extractObstacles()
{
  std::vector<tools::obstacleInfo> obstacles;
  
  auto costmap = costmap_ros_->getCostmap();
  unsigned int size_x = costmap->getSizeInCellsX();
  unsigned int size_y = costmap->getSizeInCellsY();
  double resolution = costmap->getResolution();
  double origin_x = costmap->getOriginX();
  double origin_y = costmap->getOriginY();
  
  int step = 5;
  for (unsigned int i = 0; i < size_x; i += step) {
    for (unsigned int j = 0; j < size_y; j += step) {
      unsigned char cost = costmap->getCost(i, j);
      
      if (cost > 200) {
        tools::obstacleInfo obs;
        obs.x = origin_x + i * resolution;
        obs.y = origin_y + j * resolution;
        obstacles.push_back(obs);
      }
    }
  }
  
  return obstacles;
}

void TebController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (percentage) {
    max_vel_x_ = max_vel_x_ * speed_limit / 100.0;
  } else {
    max_vel_x_ = speed_limit;
  }
  
  teb_config_.max_vel = max_vel_x_;
  
  RCLCPP_INFO(logger_, "Speed limit set to %.2f m/s", max_vel_x_);
}

tools::pathInfo TebController::poseToPathInfo(const geometry_msgs::msg::PoseStamped & pose)
{
  tools::pathInfo path_info;
  path_info.x = pose.pose.position.x;
  path_info.y = pose.pose.position.y;
  path_info.theta = tf2::getYaw(pose.pose.orientation);
  return path_info;
}

geometry_msgs::msg::PoseStamped TebController::pathInfoToPose(const tools::pathInfo & path_info)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = path_info.x;
  pose.pose.position.y = path_info.y;
  pose.pose.position.z = 0.0;
  
  tf2::Quaternion tf2_q;
  tf2_q.setRPY(0, 0, path_info.theta);
  pose.pose.orientation = tf2::toMsg(tf2_q);
  
  return pose;
}

} // namespace teb_controller

PLUGINLIB_EXPORT_CLASS(teb_controller::TebController, nav2_core::Controller)