// #include "teb_controller.hpp"
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
#include <tf2/utils.h> // 包含 getYaw 等实用工具

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

  node_ = node;  // 存储 WeakPtr，不是 SharedPtr
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  // 声明参数
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_vel_x", rclcpp::ParameterValue(0.5));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_vel_x", rclcpp::ParameterValue(0.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".min_vel_theta", rclcpp::ParameterValue(0.1));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".lookahead_dist", rclcpp::ParameterValue(2.0));
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(0.1));

  // 获取参数
  node->get_parameter(plugin_name_ + ".max_vel_x", max_vel_x_);
  node->get_parameter(plugin_name_ + ".max_vel_theta", max_vel_theta_);
  node->get_parameter(plugin_name_ + ".min_vel_x", min_vel_x_);
  node->get_parameter(plugin_name_ + ".min_vel_theta", min_vel_theta_);
  node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
  node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance_);

  // 配置TEB参数 头文件定义了
  teb_config_.no_inner_iterations = 5;
  teb_config_.no_outer_iterations = 4;
  teb_config_.min_obstacle_dist = 0.5;
  teb_config_.penalty_epsilon = 0.05;
  teb_config_.obstacle_weight = 50;
  teb_config_.optimization_verbose = false;
  teb_config_.weight_viapoint = 1;
  teb_config_.max_vel = max_vel_x_;
  teb_config_.max_vel_theta = max_vel_theta_;
  teb_config_.max_vel_x_backwards = 0.2;
  teb_config_.weight_max_vel_x = 2;
  teb_config_.weight_max_vel_theta = 1;
  teb_config_.weight_kinematics_nh = 1000;
  teb_config_.weight_kinematics_forward_drive = 1;
  // 局部优化距离
  teb_config_.local_distance =3.0;
  

  // 创建TEB规划器
  teb_planner_ = std::make_shared<teb_local_planner::plannerManager>(teb_config_);

  goal_reached_ = false;

  RCLCPP_INFO(logger_, "TEB Controller configured successfully");
}

void TebController::cleanup()
{
  RCLCPP_INFO(logger_, "TEB Controller cleanup");
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

void TebController:: setPlan(const nav_msgs::msg::Path & path)
{
  // 拿取nav2 规划的 轨迹
  global_plan_ = path;
  goal_reached_ = false;
  // RCLCPP_DEBUG(logger_, "TEB Controller received new plan with %zu poses", path.poses.size());
}

/* 
geometry_msgs::msg::PoseStamped:
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
Pose pose
	Point position
		float64 x
		float64 y
		float64 z
	Quaternion orientation  方向 (四元数)
		float64 x 0
		float64 y 0
		float64 z 0
		float64 w 1

  ----------------
geometry_msgs::msg::Twist 速度信息
std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
Twist twist
	Vector3  linear
		float64 x
		float64 y
		float64 z
	Vector3  angular
		float64 x
		float64 y
		float64 z

    */

// 速度计算
geometry_msgs::msg::TwistStamped TebController::computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped &pose,
    const geometry_msgs::msg::Twist &velocity,  // 需要命名
    nav2_core::GoalChecker *goal_checker)  
{
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header = pose.header;
  cmd_vel.twist.linear.x = 0.0;
  cmd_vel.twist.linear.y = 0.0;
  cmd_vel.twist.angular.z = 0.0;  // z：偏航 (Yaw，即左右转)，x：翻滚 (Roll)，y：俯仰 (Pitch)

  //检查是否已到达目标
  if (goal_checker->isGoalReached(pose.pose, global_plan_.poses.back().pose, velocity)) {
    goal_reached_ = true;
    RCLCPP_DEBUG(logger_, "Goal reached");
    return cmd_vel;
  }

  // 变换全局路径到局部坐标系  用于 teb优化
  std::vector<tools::pathInfo> local_plan = transformGlobalPlan(pose);
  
  if (local_plan.size() < 2) {
    RCLCPP_WARN(logger_, "Not enough waypoints in local plan");
    return cmd_vel;
  }

  // 从代价地图提取障碍物 可以不做
  // std::vector<tools::obstacleInfo> obstacles = extractObstacles();

  // 设置TEB规划器输入
  teb_planner_->setpathInfo(local_plan);
  // teb_planner_->setObstacleInfo(obstacles);

  // 运行TEB优化
  teb_planner_->runOptimization();

  // 获取优化结果  这里传的引用
  std::vector<tools::pathInfo> optimized_plan;
  teb_planner_->getPlannerResults(optimized_plan);

  if (optimized_plan.size() < 2) {
    RCLCPP_WARN(logger_, "TEB optimization failed");
    return cmd_vel;
  }

  // 计算控制指令（使用优化后的前两个点）
  tools::pathInfo current = optimized_plan[0];
  tools::pathInfo next = optimized_plan[1];
  
  double dt = 0.1; // 控制周期，应该从参数获取
  double dx = next.x - current.x;
  double dy = next.y - current. y;
  double dtheta = next.theta - current.theta;
  
  // 标准化角度
  while (dtheta > M_PI) dtheta -= 2.0 * M_PI;
  while (dtheta < -M_PI) dtheta += 2.0 * M_PI;
  
  // 计算线速度和角速度
  double linear_vel = sqrt(dx*dx + dy*dy) / dt;
  double angular_vel = dtheta / dt;
  
  // 限制速度
  linear_vel = std::max(min_vel_x_, std::min(max_vel_x_, linear_vel));
  angular_vel = std::max(-max_vel_theta_, std:: min(max_vel_theta_, angular_vel));
  
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  RCLCPP_DEBUG(logger_, "TEB command: linear=%.2f, angular=%.2f", 
               linear_vel, angular_vel);
  
  return cmd_vel;
}

/*
global_plan_： 上层规划的是点坐标，ros的标准格式

这里的操作将 ros的标准格式 --> pathInfo [x,y,theta](自定义)
并剔除 当前点离 最近离散轨迹点之前的 所有点

*/
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

  // 提取局部路径
  double accumulated_distance = 0.0;
  bool reached_goal = false;
  size_t last_idx = closest_idx;
  
  for (size_t i = closest_idx; i < global_plan_.poses.size(); ++i) {
    // 计算累积距离
    if (i > closest_idx) {
      double dx = global_plan_.poses[i].pose.position.x - 
                  global_plan_.poses[i-1].pose.position.x;
      double dy = global_plan_.poses[i].pose.position.y - 
                  global_plan_.poses[i-1].pose.position.y;
      accumulated_distance += std::sqrt(dx*dx + dy*dy);
    }
    
    // 添加路径点
    tools::pathInfo path_point;
    path_point.x = global_plan_.poses[i].pose.position.x;
    path_point.y = global_plan_.poses[i].pose.position.y;
    
    // 直接在这里计算方向
    if (i < global_plan_.poses.size() - 1) {
      // 不是最后一个点：使用当前点到下一个点
      double dx = global_plan_.poses[i+1].pose.position.x - 
                  global_plan_.poses[i].pose.position.x;
      double dy = global_plan_.poses[i+1].pose.position.y - 
                  global_plan_.poses[i].pose.position.y;
      path_point.theta = std::atan2(dy, dx);
    } else if (i > 0) {
      // 最后一个点：使用前一个点到当前点
      double dx = global_plan_.poses[i].pose.position.x - 
                  global_plan_.poses[i-1].pose.position.x;
      double dy = global_plan_.poses[i].pose.position.y - 
                  global_plan_.poses[i-1].pose.position.y;
      path_point.theta = std::atan2(dy, dx);
    } else {
      // 单点路径
      path_point.theta = tf2::getYaw(pose.pose.orientation);
    }
    
    last_idx = i;
    local_plan.push_back(path_point);
    
    // 检查停止条件
    if (i == global_plan_.poses.size() - 1) {
      reached_goal = true;
      break;
    }
    
    if (accumulated_distance >= teb_config_.local_distance) {
      break;
    }
  }
  
  if (local_plan.size() < 2 && !reached_goal) {
    RCLCPP_WARN(logger_, "Local plan has only %zu points", local_plan.size());
  }
  
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
  
  // 简化：每隔几个网格采样一次障碍物点
  int step = 5;
  for (unsigned int i = 0; i < size_x; i += step) {
    for (unsigned int j = 0; j < size_y; j += step) {
      unsigned char cost = costmap->getCost(i, j);
      
      // 如果是障碍物（代价值较高）
      if (cost > 200) { // nav2中通常使用253作为障碍物标记
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
  
  // 更新TEB配置
  teb_config_. max_vel = max_vel_x_;
  
  RCLCPP_INFO(logger_, "Speed limit set to %.2f m/s", max_vel_x_);
}

// 处理的 是点的转换
tools::pathInfo TebController::poseToPathInfo(const geometry_msgs::msg::PoseStamped & pose)
{
  tools::pathInfo path_info;
  path_info.x = pose.pose.position.x;
  path_info.y = pose.pose.position.y;
  
  // 从四元数转换为yaw角
  auto q_msg = pose.pose.orientation;
  path_info.theta = tf2::getYaw(q_msg);

  // tf2::Quaternion tf2_q(q.x, q.y, q.z, q.w);
  // tf2::Matrix3x3 m(tf2_q);
  // double roll, pitch, yaw;
  // m.getRPY(roll, pitch, yaw);
  // path_info.theta = yaw;

  // path_info.theta = atan2(2.0 * (q.w * q.z + q.x * q.y),
  //                         1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  
  return path_info;
}

geometry_msgs::msg::PoseStamped TebController::pathInfoToPose(const tools:: pathInfo & path_info)
{
  geometry_msgs::msg::PoseStamped pose;
  // 拿取3维坐标
  pose.pose.position. x = path_info.x;
  pose.pose.position. y = path_info.y;
  pose.pose.position.z = 0.0;
  
  // 从yaw角转换为四元数
  tf2::Quaternion tf2_q;
  tf2_q.setRPY(0, 0, path_info.theta); // 设置 Roll=0, Pitch=0, Yaw=theta
  // 将 tf2::Quaternion 转换为 geometry_msgs::msg::Quaternion
  pose.pose.orientation = tf2::toMsg(tf2_q);  
  // pose.pose.orientation.x = 0.0;
  // pose. pose.orientation.y = 0.0;
  // pose.pose.orientation.z = sin(path_info.theta / 2.0);
  // pose.pose.orientation.w = cos(path_info.theta / 2.0);
  
  return pose;
}



} // namespace teb_controller

// 注册插件
PLUGINLIB_EXPORT_CLASS(teb_controller::TebController, nav2_core::Controller)