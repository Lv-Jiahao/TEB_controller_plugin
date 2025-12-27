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

// 包含TEB算法相关头文件
#include "tools.h"
#include "planner_manager.h"

namespace teb_controller
{

class TebController : public nav2_core::Controller
{
public:
  TebController() = default;
  ~TebController() override = default;

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, 
      std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup controller state machine
   */
  void cleanup() override;

  /**
   * @brief Activate controller state machine
   */
  void activate() override;

  /**
   * @brief Deactivate controller state machine
   */
  void deactivate() override;

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @param goal_checker Pointer to the goal checker for this task in case useful in computing commands
   * @return Best command for the robot to drive
   */
  geometry_msgs::msg::TwistStamped
  computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                          const geometry_msgs::msg::Twist &velocity,
                          nav2_core::GoalChecker * goal_checker) override;

  /**
   * @brief nav2_core setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_msgs::msg::Path &path) override;

  /**
   * @brief Limits the maximum linear speed of the robot. 
   * @param speed_limit expressed in absolute value (in m/s) 
   * or in percentage from maximum robot speed. 
   * @param percentage Setting speed limit in percentage if true
   * or in absolute values in false case.
   */
  void setSpeedLimit(const double &speed_limit,
                     const bool &percentage) override;
protected:
  /**
   * @brief Transform global plan to local coordinates
   */
  std::vector<tools::pathInfo> transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);


  /**
   * @brief Convert nav2 pose to TEB pathInfo   
   */
  tools::pathInfo poseToPathInfo(const geometry_msgs::msg::PoseStamped & pose);

  /**
   * @brief Convert TEB pathInfo to nav2 pose  
   */
  geometry_msgs::msg::PoseStamped pathInfoToPose(const tools::pathInfo & path_info);

  std::vector<tools::obstacleInfo> extractObstacles();

  // ROS2 节点相关
  // 存储插件名称
  std::string plugin_name_;
  // 存储坐标变换缓存指针，可用于查询坐标关系
  std::shared_ptr<tf2_ros::Buffer> tf_;
  // 存储代价地图
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  // 存储节点指针
  nav2_util::LifecycleNode::SharedPtr node_;
  // 存储全局代价地图
  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap_;
  // 全局路径
  nav_msgs::msg::Path global_plan_;
  
  rclcpp::Logger logger_ {rclcpp::get_logger("TebController")};
  rclcpp::Clock:: SharedPtr clock_;

  // TEB算法相关
  std::shared_ptr<teb_local_planner::plannerManager> teb_planner_;
  teb_local_planner::TebConfig teb_config_;
  
  // 参数
  double max_vel_x_;
  double max_vel_theta_;
  double min_vel_x_;
  double min_vel_theta_;
  double lookahead_dist_;
  double transform_tolerance_;
  bool goal_reached_;
};

} // namespace teb_controller

#endif // TEB_CONTROLLER_HPP_