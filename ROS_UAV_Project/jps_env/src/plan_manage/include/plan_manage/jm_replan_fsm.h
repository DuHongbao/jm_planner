

#ifndef _JM_REPLAN_FSM_H_
#define _JM_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <path_searching/kinodynamic_astar.h>
#include <plan_env/edt_environment.h>
#include <plan_env/obj_predictor.h>
#include <plan_env/sdf_map.h>
#include <plan_manage/Bspline.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;


namespace fast_planner {

class JMReplanFSM{
private:
  enum FSM_EXEC_STATE { INIT, WAIT_TARGET, GEN_NEW_TRAJ, REPLAN_TRAJ, EXEC_TRAJ, REPLAN_NEW };
  enum TARGET_TYPE { MANUAL_TARGET = 1, PRESET_TARGET = 2, REFENCE_PATH = 3 };

  FastPlannerManager::Ptr planner_manager_;
  PlanningVisualization::Ptr visualization_;

  /* parameters */
  int target_type_;  // 1 mannual select, 2 hard code
  double replan_distance_threshold_, replan_time_threshold_;
  double waypoints_[50][3];
  int waypoint_num_;
  bool act_map_;

  vector<Eigen::Vector3d>  jps_path;////////////////
  EDTEnvironment::Ptr edt_environment_;///////////
  Eigen::Vector3d pt;//////////////////



  /* planning data */
  bool trigger_, have_target_, have_odom_, collide_,have_path_;
  FSM_EXEC_STATE exec_state_;

  Eigen::Vector3d odom_pos_,odom_vel_; //odometry state
  Eigen::Quaterniond odom_orient_;

  Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_; //start state
  Eigen::Vector3d target_point_, end_vel_;                      //target state
  int current_wp_;
  pcl::PointCloud<pcl::PointXYZ>  pclCorner_input;

/*  ROS utils  */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_;
  ros::Subscriber waypoint_sub_,  odom_sub_, Jps_Points_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  /* helper function  */
  bool callSearchAndOptimization();
  bool callJmTraj(int step);

  void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
  void printFSMExecState();   

/* ROS function */

  void execFSMCallback(const ros::TimerEvent& e);
  void checkCollisionCallback(const ros::TimerEvent& e);
  void waypointCallback(const nav_msgs::PathConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void JpsPointsCallback(const sensor_msgs::PointCloud2& globalCorners);
  void  path_find(vector<Eigen::Vector3d> &jps_path);




public:
  JMReplanFSM(/* args */) {  pt <<0.0,0.0,0.0;}
  ~JMReplanFSM() {}
  void find_path(Eigen::Vector3d start, Eigen::Vector3d end);///////////////////
  void setEnvironment(const EDTEnvironment::Ptr& env);//////////////////
  void init(ros::NodeHandle& nh);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  //namespace fast_planner

#endif

