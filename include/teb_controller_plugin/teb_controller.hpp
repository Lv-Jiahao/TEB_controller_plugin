#ifndef TEB_CONTROLLER_HPP_
#define TEB_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "tools.h"
#include "planner_manager.h"

#include <mutex>

namespace teb_controller
{

class TebController : public nav2_core::Controller
{
public:
  TebController() = default;
  ~TebController() override = default;

  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, 
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity,
                          nav2_core::GoalChecker * goal_checker) override;

  void setPlan(const nav_msgs::msg::Path &path) override;

  void setSpeedLimit(const double &speed_limit,
                     const bool &percentage) override;

protected:
  std::vector<tools::pathInfo> transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
  tools::pathInfo poseToPathInfo(const geometry_msgs::msg::PoseStamped & pose);
  geometry_msgs::msg::PoseStamped pathInfoToPose(const tools::pathInfo & path_info);
  std::vector<tools::obstacleInfo> extractObstacles();

  // ROS2 相关
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_util::LifecycleNode::SharedPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  nav_msgs::msg::Path global_plan_;
  
  rclcpp::Logger logger_ {rclcpp::get_logger("TebController")};
  rclcpp::Clock::SharedPtr clock_;

  // TEB算法相关
  std::shared_ptr<teb_local_planner::plannerManager> teb_planner_;
  teb_local_planner::TebConfig teb_config_;
  std::mutex planner_mutex_;  
  
  // 参数
  double max_vel_x_;
  double max_vel_theta_;
  double min_vel_x_;
  double min_vel_theta_;
  double lookahead_dist_;
  double transform_tolerance_;
  bool goal_reached_;
  
  // 频率控制
  rclcpp::Time last_optimization_time_;
  
  // ✅ 添加速度记录（用于平滑）
  geometry_msgs::msg::Twist last_cmd_vel_;
};

} // namespace teb_controller

#endif // TEB_CONTROLLER_HPP_