#ifndef RTIOMS_RECOVERY_BEHAVIORS_RTIOMS_RECOVERY_BEHAVIORS_H
#define RTIOMS_RECOVERY_BEHAVIORS_RTIOMS_RECOVERY_BEHAVIORS_H
#include <nav_core/recovery_behavior.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <base_local_planner/costmap_model.h>
#include <string>

namespace rtioms_recovery_behaviors
{
/**
 * @class BackwardRecovery
 * @brief A backward behavior that move backwards the robot in 1-meter to clear out space
 */
class RtiomsRecoveryBehaviors : public nav_core::RecoveryBehavior
{
public:
  /**
   * @brief  Constructor, make sure to call initialize in addition to actually initialize the object
   */
  RtiomsRecoveryBehaviors();

  /**
   * @brief  Initialization function for the BackwardRecovery recovery behavior
   * @param name Namespace used in initialization
   * @param tf (unused)
   * @param global_costmap (unused)
   * @param local_costmap A pointer to the local_costmap used by the navigation stack
   */
  void initialize(std::string name, tf2_ros::Buffer*,
                  costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap);

  /**
   * @brief  Callback function for the subscriber.
   */
  //void amcl_poseCallback(const geometry_msgs::PoseStamped::ConstPtr & global_pose);

  //void odom_poseCallback(const nav_msgs::Odometry::ConstPtr & global_pose);



  /**
   * @brief  Run the BackwardRecovery recovery behavior.
   */
  void runBehavior();

  /**
   * @brief  Destructor for the backward recovery behavior.
   */
  ~RtiomsRecoveryBehaviors();

private:
  costmap_2d::Costmap2DROS* local_costmap_;
  bool initialized_;
  double sim_granularity_, min_rotational_vel_, max_rotational_vel_, acc_lim_th_, tolerance, frequency_, min_vel_x_, max_vel_x_, min_vel_theta_, max_vel_theta_, acc_lim_x_, xy_goal_tolerance_, yaw_goal_tolerance_, controller_frequency_;
  base_local_planner::CostmapModel* world_model_;
};
};  // namespace backward_recovery
#endif  // RTIOMS_RECOVERY_BEHAVIORS_RTIOMS_RECOVERY_BEHAVIORS_H