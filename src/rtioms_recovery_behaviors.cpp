/******************************************************************
*   Software License Agreement (BSD License)
*
*   Author : Kothandaraman Kannadasan
*   Maintainer : Dr.Chin-Sheng Chen
*   Department : Department of Electrical Engineering
*   Year : 2021
*   Labratory : Real-Time Intelligent Opto-Mechatronic System
*   University : National Taipei Univerity of Technology
*   
*   E-mail : kkraman02@gmail.com
*
*   Package name : backward_recovery
*
*   Description :
*                  This package main aims to prevent the robot before the die condition moreover, which gives the re-breath to the robot to move towards the goal.
*    we created a new recovery behavior to save the robot life-time. This recovery behavior will do recover the robot from the stucking situation also, it has the
*    efficiency to move backwards upto 0.2 meter to replanning the path again towards to the goal.
*
*******************************************************************/

#include <ros/ros.h>
#include <rtioms_recovery_behaviors/rtioms_recovery_behaviors.h>
#include <pluginlib/class_list_macros.h>
#include <nav_core/parameter_magic.h>
#include <tf2/utils.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <angles/angles.h>
#include <algorithm>
#include <string>
#include <math.h>
#include <vector>
#include <sstream>

PLUGINLIB_EXPORT_CLASS(rtioms_recovery_behaviors::RtiomsRecoveryBehaviors, nav_core::RecoveryBehavior)

namespace rtioms_recovery_behaviors
{

  geometry_msgs::PoseWithCovarianceStamped turtlebot3_pose;
  geometry_msgs::PoseWithCovarianceStamped goal_pose;

  // geometry_msgs::PoseWithCovarianceStamped initial_pose;

  double initial_position_x;    //which used for to metion the starting pose of the robot
  double initial_position_y;
  double initial_orientation_z;

  double a = 10;    //instance global variables just used for to mention the starting pose of the robot
  double b = 20;

 /* ros::Publisher vel_pub;
  ros::Subscriber amcl_pose_sub;
 
  geometry_msgs::PoseWithCovarianceStamped turtlebot3_pose;

   ros::NodeHandle n;

  vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  amcl_pose_sub = n.subscribe("/amcl_pose", 10, amcl_pose_CB);

  ros::Rate loop_rate(0.5);
  geometry_msgs::PoseWithCovarianceStamped goal_pose;
  goal_pose.pose.pose.position.x = 1;
  goal_pose.pose.pose.position.y = 1;
  runBehavior(goal_pose, 0.01);

  loop_rate.sleep();

  ros::spin(); */


  RtiomsRecoveryBehaviors::RtiomsRecoveryBehaviors(): local_costmap_(NULL), initialized_(false), world_model_(NULL)
  {
      //constructor for the backward recovery behavior
  }

  void RtiomsRecoveryBehaviors::initialize(std::string name, tf2_ros::Buffer*, costmap_2d::Costmap2DROS*, costmap_2d::Costmap2DROS* local_costmap)
  {
    if (!initialized_)
    {
      local_costmap_ = local_costmap;

      ros::NodeHandle private_nh("~/" + name);
      ros::NodeHandle blp_nh("~TrajectoryPlannerROS");      //Mentioning the local planner

      private_nh.param("sim_granularity", sim_granularity_, 0.017);   //Accessing the parameters of the base_local_planner
      private_nh.param("frequency", controller_frequency_, 20.0);

      blp_nh.param("min_vel_x", min_vel_x_, -0.10);
      blp_nh.param("max_vel_x", max_vel_x_, 0.10);
      blp_nh.param("min_vel_theta", min_vel_theta_, -0.10);
      blp_nh.param("max_vel_theta", max_vel_theta_, 0.10);
      blp_nh.param("acc_lim_x", acc_lim_x_, 2.5);
      blp_nh.param("xy_goal_tolerance", xy_goal_tolerance_, 0.10);
      blp_nh.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.10);

      acc_lim_th_ = nav_core::loadParameterWithDeprecation(blp_nh, "acc_lim_theta", "acc_lim_th", 3.2);
      max_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "max_vel_theta", "max_rotational_vel", 1.0);
      min_rotational_vel_ = nav_core::loadParameterWithDeprecation(blp_nh, "min_in_place_vel_theta", "min_in_place_rotational_vel", 0.4);

      world_model_ = new base_local_planner::CostmapModel(*local_costmap_->getCostmap());

      initialized_ = true;

    }
    else
    {
      ROS_ERROR("You should not call initialize twice on this object, doing nothing");
    }
    
  }

  RtiomsRecoveryBehaviors::~RtiomsRecoveryBehaviors()  //de-constructor for the backward recovery behavior
  {
    delete world_model_;
  }

 /* void initial_pose_CB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & initial_pose_message)
  {
    std::cout << "////////////////////////////" <<std::endl;
    std::cout << "tb3 Pose.X = " << initial_pose.pose.pose.position.x << std::endl << std::endl;
    std::cout << "////////////////////////////" <<std::endl;
    initial_pose.pose.pose.position.x = initial_pose_message->pose.pose.position.x;
    initial_pose.pose.pose.position.y = initial_pose_message->pose.pose.position.y;
    initial_pose.pose.pose.position.z = initial_pose_message->pose.pose.position.z;

    initial_pose.pose.pose.orientation.x = initial_pose_message->pose.pose.orientation.x;
    initial_pose.pose.pose.orientation.y = initial_pose_message->pose.pose.orientation.y;
    initial_pose.pose.pose.orientation.z = initial_pose_message->pose.pose.orientation.z;
    initial_pose.pose.pose.orientation.w = initial_pose_message->pose.pose.orientation.w;
  } */

  void tb3_amcl_pose_CB(const geometry_msgs::PoseWithCovarianceStamped& robot_pose)     //This callback function is used to mention the starting pose of the robot
  {
    initial_position_x = robot_pose.pose.pose.position.x;
    initial_position_y = robot_pose.pose.pose.position.y;
    initial_orientation_z = robot_pose.pose.pose.orientation.z;
  } 

  void amcl_pose_CB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & amcl_pose_message)
  {
    turtlebot3_pose.pose.pose.position.x = amcl_pose_message->pose.pose.position.x;
    turtlebot3_pose.pose.pose.position.y = amcl_pose_message->pose.pose.position.y;
    turtlebot3_pose.pose.pose.position.z = amcl_pose_message->pose.pose.position.z;

    turtlebot3_pose.pose.pose.orientation.x = amcl_pose_message->pose.pose.orientation.x;
    turtlebot3_pose.pose.pose.orientation.y = amcl_pose_message->pose.pose.orientation.y;
    turtlebot3_pose.pose.pose.orientation.z = amcl_pose_message->pose.pose.orientation.z;
    turtlebot3_pose.pose.pose.orientation.w = amcl_pose_message->pose.pose.orientation.w;
  }

  double getDistance(double x1, double y1, double x2, double y2)
  {
    return sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
  }

  void RtiomsRecoveryBehaviors::runBehavior()    //geometry_msgs::PoseWithCovarianceStamped goal_pose, double distance_tolerance
  {
    if (!initialized_)
    {
      ROS_ERROR("This object must be initialized before runbehaviour is called");
    }

    if (local_costmap_ == NULL)
    {
      ROS_ERROR("The costmap passed to the BackwardRecovery object cannot be NULL. Doing nothing.");
    }

    ROS_WARN("RTIOMS Backward recovery behaviour started.");
    
    geometry_msgs::Twist vel_msg;

    ros::Rate r(controller_frequency_);
    ros::NodeHandle n;
    ros::Publisher vel_pub;
    ros::Subscriber amcl_pose_sub;

    geometry_msgs::PoseStamped global_pose;
    local_costmap_->getRobotPose(global_pose);

    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Rate loop_rate(5);

    do
    {
      ros::Subscriber pose = n.subscribe("/amcl_pose", 10, tb3_amcl_pose_CB);
      //amcl_pose_sub = n.subscribe("/amcl_pose", 10, initial_pose_CB);
      loop_rate.sleep();
      ros::spinOnce();
      break;
    } while (a == b && ros::ok());
    
    

    while (ros::ok())
    {
      amcl_pose_sub = n.subscribe("/amcl_pose", 10, amcl_pose_CB);
      r.sleep();
      ros::spinOnce();
      break;
    }

    

    geometry_msgs::PoseWithCovarianceStamped goal_pose;

    double ang = initial_orientation_z;
    
    double th1 = -0.0;
    double th2 = -0.25;
    double th3 = -0.50;
    double th4 = -0.75;
    double th5 = -1.00;
    
    double th6 = 0.0;
    double th7 = 0.25;
    double th8 = 0.50;
    double th9 = 0.75;
    double th10 = 1.00;

    if (ang < th1 && ang > th2 || ang > th6 && ang < th7)         // -x direction
    {
      goal_pose.pose.pose.position.x = initial_position_x - 1;
      goal_pose.pose.pose.position.y = initial_position_y;
    }
    else if (ang < th4 && ang > th5 || ang > th9 && ang < th10)   // +x direction
    {
      goal_pose.pose.pose.position.x = initial_position_x + 1;
      goal_pose.pose.pose.position.y = initial_position_y;
    }
    else if (ang < th9 && ang > th8 || ang > th7 && ang < th8)    // -y direction
    {
      goal_pose.pose.pose.position.x = initial_position_x;
      goal_pose.pose.pose.position.y = initial_position_y - 1;
    }
    else if (ang < th2 && ang > th3 || ang > th5 && ang < th3)    // +y direction
    {
      goal_pose.pose.pose.position.x = initial_position_x;
      goal_pose.pose.pose.position.y = initial_position_y + 1;
    }
    else
    {
      ROS_ERROR("some problem is there to calculate the angles");
    }
    
    /*
     geometry_msgs::PoseWithCovarianceStamped goal_pose;

     goal_pose.pose.pose.position.x = initial_position_x - 1;
     goal_pose.pose.pose.position.y = initial_position_y;
     goal_pose.pose.pose.orientation.z = initial_orientation_z;
    */

    // goal_pose.pose.pose.position.x = -2.000000;
    // goal_pose.pose.pose.position.y = 1.500000;

     double footprint_cost;  

    do
    {

    /**** Proportional Controller ****/

    /*  local_costmap_->getRobotPose(global_pose);

      double x = global_pose.pose.position.x, y = global_pose.pose.position.y, theta = global_pose.pose.orientation.z;

      footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
      if (footprint_cost < 0.0)
      {
        ROS_ERROR("backward behavior not working there is collison. Cost: %.2f",
                  footprint_cost);
        return;
      }
    */
      double Kp = -0.20;
      double Kh = 0.40;

      double Distance = getDistance(goal_pose.pose.pose.position.x, goal_pose.pose.pose.position.y, turtlebot3_pose.pose.pose.position.x, turtlebot3_pose.pose.pose.position.y);
      double velocity_x = (Kp * Distance);

     double Angle = atan2(goal_pose.pose.pose.position.y - turtlebot3_pose.pose.pose.position.y, goal_pose.pose.pose.position.x - turtlebot3_pose.pose.pose.position.x);
      //double Angle = atan2(goal_pose.pose.pose.position.y - turtlebot3_pose.pose.pose.position.y, goal_pose.pose.pose.position.x - turtlebot3_pose.pose.pose.position.x);

     double Angle_Diff = Kh * ((Angle) - turtlebot3_pose.pose.pose.orientation.z);
      //double Angle_Diff = Kh * ( (-(Angle)) - turtlebot3_pose.pose.pose.orientation.z);

      //double velocity_z = - (Angle_Diff);
     double velocity_z = (Angle_Diff);



      std::cout <<"##########################################################" << std::endl;
      std::cout << "initial_position_x  = " << initial_position_x << std::endl <<std::endl;
      std::cout << "initial_position_y  = " << initial_position_y << std::endl << std::endl;
     // std::cout << "initial_orientation_z  = " << initial_orientation_z << std::endl << std::endl;
      std::cout <<"##########################################################" << std::endl;
      std::cout <<"goal_position_x  = " << goal_pose.pose.pose.position.x << std::endl << std::endl;
      std::cout <<"goal_position_y  = " << goal_pose.pose.pose.position.y << std::endl << std::endl;
      std::cout <<"##########################################################" << std::endl;
      std::cout <<"Distance   = " << Distance << std::endl << std::endl;
      //std::cout <<"Angle   = " << Angle << std::endl << std::endl;
      //std::cout <<"Angle_Diff   = " << Angle_Diff << std::endl << std::endl;
      std::cout <<"velocity_x   = " << velocity_x << std::endl << std::endl;
      //std::cout <<"velocity_z   = " << velocity_z << std::endl << std::endl;
      std::cout <<"##########################################################" << std::endl << std::endl;

      //linear velocity in the x-axis
      vel_msg.linear.x = velocity_x;
      vel_msg.linear.y = 0;
      vel_msg.linear.z = 0;

      //angular velocity in the z-axix
      vel_msg.angular.x = 0;
      vel_msg.angular.y = 0;
      vel_msg.angular.z = 0;
      //vel_msg.angular.z = velocity_z;
      
      //vel_msg.angular.z = max_vel_theta_*(atan2(goal_pose.pose.pose.position.y - turtlebot3_pose.pose.pose.position.y, goal_pose.pose.pose.position.x - turtlebot3_pose.pose.pose.position.x) - turtlebot3_pose.pose.pose.orientation.z);

      vel_pub.publish(vel_msg);

      ros::spinOnce();
      loop_rate.sleep();

    } while (getDistance(goal_pose.pose.pose.position.x, goal_pose.pose.pose.position.y, turtlebot3_pose.pose.pose.position.x, turtlebot3_pose.pose.pose.position.y)>0.80);
  //  } while (getDistance(goal_pose.pose.pose.position.x, goal_pose.pose.pose.position.y, turtlebot3_pose.pose.pose.position.x, turtlebot3_pose.pose.pose.position.y)>0.25);
    ROS_INFO("RTIOMS Backward recovery behaviour finished");
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    vel_pub.publish(vel_msg);

    ROS_WARN("RTIOMS Rotate recovery behavior started.");

    ros::Rate re(controller_frequency_);
    ros::NodeHandle ne;
    ros::Publisher vel_pube = ne.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    geometry_msgs::PoseStamped global_pose_;
    local_costmap_->getRobotPose(global_pose_);

    double current_angle = tf2::getYaw(global_pose_.pose.orientation);
    double start_angle = current_angle;

    bool got_180 = false;
      local_costmap_->getRobotPose(global_pose_);
      current_angle = tf2::getYaw(global_pose_.pose.orientation);

      // compute the distance left to rotate
      double dist_left;
      if (!got_180)
      {
        // If we haven't hit 180 yet, we need to rotate a half circle plus the distance to the 180 point
        double distance_to_180 = std::fabs(angles::shortest_angular_distance(current_angle, start_angle + M_PI));
        dist_left = M_PI + distance_to_180;

        if (distance_to_180 < yaw_goal_tolerance_)
        {
          got_180 = true;
        }
      }
      else
      {
        // If we have hit the 180, we just have the distance back to the start
        dist_left = std::fabs(angles::shortest_angular_distance(current_angle, start_angle));
      }

      double x = global_pose_.pose.position.x, y = global_pose_.pose.position.y;

      // check if that velocity is legal by forward simulating
      double sim_angle = 0.0;
      while (sim_angle < dist_left)
      {
        double theta = current_angle + sim_angle;

        // make sure that the point is legal, if it isn't... we'll abort
        double footprint_cost = world_model_->footprintCost(x, y, theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
        if (footprint_cost < 0.0)
        {
          ROS_ERROR("RTIOMS Rotate recovery can't rotate in place because there is a potential collision. Cost: %.2f",
                    footprint_cost);
          return;
        }

        sim_angle += sim_granularity_;
      }

      // compute the velocity that will let us stop by the time we reach the goal
      double vel = sqrt(2 * acc_lim_th_ * dist_left);   

      // make sure that this velocity falls within the specified limits
      vel = std::min(std::max(vel, min_rotational_vel_), max_rotational_vel_);

      geometry_msgs::Twist cmd_vel;
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = vel;

      vel_pube.publish(cmd_vel);

      re.sleep();
           
  }
};