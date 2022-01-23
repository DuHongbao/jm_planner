#include <plan_manage/jm_replan_fsm.h>
#include <pcl/kdtree/kdtree_flann.h>
using namespace std;

namespace fast_planner {

void JMReplanFSM::init(ros::NodeHandle& nh) {
  current_wp_ = 0;
  exec_state_  =  FSM_EXEC_STATE::INIT;
  have_target_ = false;
  collide_ = false;
  have_path_ = 0;
/* fsm param */
  nh.param("fsm/flight_type", target_type_, -1);
  nh.param("fsm/thresh_replan",replan_time_threshold_,-1.0);
  nh.param("fsm/thresh_no_replan",replan_distance_threshold_,-1.0);
  nh.param("fsm/waypoint_num",waypoint_num_,-1);
  nh.param("fsm/act_map",act_map_,false);
  for (int i = 0; i< waypoint_num_; i++){
    nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
    nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
  }

 cout<<"********************************HELLO JMPLANNER****************************"<<endl;
  /* intialize main modules */
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);
  visualization_.reset(new PlanningVisualization(nh));

  /*callback*/
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &JMReplanFSM::execFSMCallback, this); 
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &JMReplanFSM::checkCollisionCallback, this);

  waypoint_sub_ =  nh.subscribe("/waypoint_generator/waypoints", 1, &JMReplanFSM::waypointCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &JMReplanFSM::odometryCallback, this);
  if (!have_path_){
      Jps_Points_sub_ = nh.subscribe("/global_corners/corner_points", 1, &JMReplanFSM::JpsPointsCallback, this);
  }
  
  
  replan_pub_  = nh.advertise<std_msgs::Empty>("/planning/replan", 20);
  new_pub_     = nh.advertise<std_msgs::Empty>("/planning/new", 20);
  bspline_pub_ = nh.advertise<plan_manage::Bspline>("/planning/bspline", 20);

}

////////////////
void JMReplanFSM::setEnvironment(const EDTEnvironment::Ptr& env) { this->edt_environment_ = env; }

////////////////
  void JMReplanFSM::find_path(Eigen::Vector3d start, Eigen::Vector3d end){
        double dist;
        dist = edt_environment_->evaluateCoarseEDT(pt, -1.0);
        //cout<<"Dist: "<<dist<<endl;
  }

bool JMReplanFSM::callJmTraj(int step){
  bool plan_success;

  if(step == 1){
    plan_success = planner_manager_ ->planGlobalTraj(start_pt_);//replace by JPS
  }else{
    cout<<"运行到这里啦2！"<<endl;
    plan_success = planner_manager_->jmJpsReplan(collide_);  //replace by JPS

  }
  if (plan_success) {

    planner_manager_->planYaw(start_yaw_);

    LocalTrajData* locdat = &planner_manager_->local_data_;

    /* publish newest trajectory to server */

    /* publish traj */
    plan_manage::Bspline bspline;
    bspline.order      = 3;
    bspline.start_time = locdat->start_time_;
    bspline.traj_id    = locdat->traj_id_;

    Eigen::MatrixXd pos_pts = locdat->position_traj_.getControlPoint();

    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = locdat->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }

    Eigen::MatrixXd yaw_pts = locdat->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = locdat->yaw_traj_.getInterval();

    bspline_pub_.publish(bspline);

    /* visualize new trajectories */

    MidPlanData* plan_data = &planner_manager_->plan_data_;
    visualization_->drawPolynomialTraj(planner_manager_->global_data_.global_traj_, 0.05,
                                       Eigen::Vector4d(0, 0, 0, 1), 0);
    visualization_->drawBspline(locdat->position_traj_, 0.08, Eigen::Vector4d(1.0, 0.0, 0.0, 1), false,
                                0.15, Eigen::Vector4d(1.0, 1.0, 1.0, 1), 99, 99);
    visualization_->drawBsplinesPhase2(plan_data->topo_traj_pos2_, 0.075);
    visualization_->drawYawTraj(locdat->position_traj_, locdat->yaw_traj_, plan_data->dt_yaw_);

    return true;
  } else {
    return false;
  }
}
/*JPS角点订阅函数*/
void JMReplanFSM::JpsPointsCallback(const sensor_msgs::PointCloud2& globalCorners){
	pcl::PointCloud<pcl::PointXYZ> corners_input;
	pcl::fromROSMsg(globalCorners, corners_input);
  pcl::fromROSMsg(globalCorners, this->pclCorner_input);
  vector<Eigen::Vector3d>  jpsPath;
  jps_path.clear();
 	//cout<<"corners_input before filter:"<<corners_input.size()<<endl;
   //path_find(jpsPath);
}

void  JMReplanFSM::path_find(vector<Eigen::Vector3d> &jps_path){
     /*************find the shortst path according to the corners points****************/
    pcl::KdTreeFLANN<pcl::PointXYZ>  kdtree;
   
    Eigen::Vector3d  delta_path = (target_point_ - start_pt_).normalized()*0.1;
    vector<Eigen::Vector3d>  colli_start,colli_end;
    bool safe(1),last_safe(1);
    for(int i  = 0; i< (target_point_ - start_pt_).norm()/delta_path.norm() ; i++){
      Eigen::Vector3d  current_point = start_pt_ + i*delta_path;
      auto edt_env = planner_manager_-> edt_environment_;
      safe = edt_env->evaluateCoarseEDT(current_point,-1.0) < 0.01 ? false : true;
      // if (dist <= 0.0){
      //     cout<<"the distance is:"<<dist<<endl;
      // }
    if (last_safe && !safe) {
      colli_start.push_back(current_point);
    } else if (!last_safe && safe) {
      colli_end.push_back(current_point);
    }
    last_safe = safe;
  }

    kdtree.setInputCloud(pclCorner_input.makeShared());
    cout<<"the size of collid is:"<<colli_start.size()<<endl;
  //1.   添加对碰撞点的中心点进行循环遍历
    if(colli_start.size()){
      pcl::PointXYZ searchPoint;
      searchPoint.x = (colli_start[0](0) + colli_end[0](0))/2.0;
      searchPoint.y = (colli_start[0](1) + colli_end[0](1))/2.0;
      searchPoint.z = (colli_start[0](2) + colli_end[0](2))/2.0;
  	  int K = 15;

	    std::vector<int> pointIdxNKNSearch(K);
	    std::vector<float>pointNKNSquaredDistance(K);

	    std::cerr << "K nearest neighbour search at ( " << searchPoint.x << "  " << searchPoint.y << "   " << searchPoint.z <<	")with K= "<<K<< std::endl;
	    if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) >0)  {
        //2.  将最近邻的K个角点构建kd树， 由起点开始扩展最近邻m个点，将每一个障碍中心作为一簇待扩展点，启发式搜索最短路径
		      for (std::size_t i = 0; i < pointIdxNKNSearch.size(); i++) {
       			    std::cerr	<< "    " << pclCorner_input[pointIdxNKNSearch[i]].x
						                      << "    " << pclCorner_input[pointIdxNKNSearch[i]].y
						                      << "   "  << pclCorner_input[pointIdxNKNSearch[i]].z 
						                      << " with a distance :"<<std::sqrt( pointNKNSquaredDistance[i] )<< std::endl;
           }
	    }
    }


     for(int i = 0; i<pclCorner_input.size(); i++){

        //cout<<"the start position is: "<<start_pt_<<endl;
        //cout<<"the delta_path is: "<<delta_path <<endl;
        //1. 起点出发步进向终点
        //2. 如果能够直达目标则直接作为路径
        //3. 如果不能直达目标则逐渐往前推进，搜索障碍碰撞段中心最近的角点
        //4. 连接起点到最近点作为这段路径
        //5. 继续循环1-4直到达到目标
     }
     Eigen::Vector3d waypoint = target_point_;
     jps_path.push_back(target_point_);
    have_path_ = 1;
}

void JMReplanFSM::printFSMExecState(){
  string state_str[6] = { "INIT",
                                  "WAIT_TARGET",
                                  "GEN_NEW_TRAJ",
                                  "REPLAN_TRAJ",
                                  "EXEC_TRAJ",
                                  "REPLAN_NEW" };
  cout << "state: " + state_str[int(exec_state_)] << endl;
}

void JMReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call){
  string state_str[6] = { "INIT",
                                  "WAIT_TARGET",
                                  "GEN_NEW_TRAJ",
                                  "REPLAN_TRAJ",
                                  "EXEC_TRAJ",
                                  "REPLAN_NEW" };
  exec_state_ = new_state;
}

void JMReplanFSM::execFSMCallback(const ros::TimerEvent& e){
    static int fsm_num = 0;
    fsm_num++;
    if(fsm_num == 100) {
      printFSMExecState();
      if (! have_odom_) cout <<"no odom." <<endl;
      if(!trigger_) cout << "no trigger_." << endl;
      fsm_num = 0;
    }
    switch (exec_state_)
    {
    case INIT:{
      if(!have_odom_){return;}
      if(!trigger_){return;}
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }
  case WAIT_TARGET:{
    if(!have_target_)
      return;
    else{
      changeFSMExecState(GEN_NEW_TRAJ,"FSM");
    }
    break;
  }
  case GEN_NEW_TRAJ:{
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0,0,3,1);
    start_yaw_(0) = atan2(rot_x(1),rot_x(0));
    start_yaw_(1) = start_yaw_(2) = 0.0;

    new_pub_.publish(std_msgs::Empty());

    bool success = callJmTraj(1);

    if(success){
      changeFSMExecState(EXEC_TRAJ,"FSM");
    }else{
      changeFSMExecState(GEN_NEW_TRAJ,"FSM");

    }
    break;
  }

  case EXEC_TRAJ:{
    GlobalTrajData* global_data =  &planner_manager_->global_data_;
    ros::Time time_now = ros::Time::now();
    double  t_cur = (time_now - global_data->global_start_time_).toSec();

    if(t_cur >global_data -> global_duration_ - 1e-2){
      have_target_ = false;
      changeFSMExecState(WAIT_TARGET, "FSM");
      return;
    }else{
      LocalTrajData* info = &planner_manager_ ->local_data_;
      Eigen::Vector3d start_pos = info->start_pos_;
      t_cur                              =  (time_now - info->start_time_).toSec();

      if(t_cur > replan_time_threshold_){

        if(!global_data-> localTrajReachTarget()){

          changeFSMExecState(REPLAN_TRAJ, "FSM");

        }else{

          Eigen::Vector3d cur_pos = info->position_traj_.evaluateDeBoorT(t_cur);
          Eigen::Vector3d end_pos = info->position_traj_.evaluateDeBoorT(info->duration_);

          if((cur_pos-end_pos).norm() > replan_distance_threshold_ )
            changeFSMExecState(REPLAN_TRAJ, "FSM");

        }

      }
    }
    break;
  }

  case REPLAN_TRAJ: {
    LocalTrajData* info  = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_cur)[0];
    start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_cur)[0];
    start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_cur)[0];
    cout<<"运行到这里啦1！"<<endl;
    bool success  = callJmTraj(2);
    cout<<"运行到这里啦！"<<endl;
    if(success){
      changeFSMExecState(EXEC_TRAJ, "FSM");
    }else{
      ROS_WARN("Replan fail, retrying...");
    }

    break;
  }

  case REPLAN_NEW:{
    LocalTrajData* info    = &planner_manager_->local_data_;
    ros::Time   time_now = ros::Time::now();
    double  t_cur = (time_now - info->start_time_).toSec();

    start_pt_ = info ->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    new_pub_.publish(std_msgs::Empty());

    bool success = callJmTraj(1);
    if(success){
      changeFSMExecState(EXEC_TRAJ,"FSM");
    }else{
      changeFSMExecState(GEN_NEW_TRAJ, "FSM");
    }

    break;
  }
  }
  }

void JMReplanFSM::checkCollisionCallback(const ros::TimerEvent& e) {
  LocalTrajData* info = &planner_manager_->local_data_;

  /* ---------- check goal safety ---------- */
  // if (have_target_)
  if (false) {
    auto edt_env = planner_manager_->edt_environment_;

    double dist = planner_manager_->pp_.dynamic_ ?
        edt_env->evaluateCoarseEDT(target_point_, /* time to program start */ info->duration_) :
        edt_env->evaluateCoarseEDT(target_point_, -1.0);

    if (dist <= 0.3) {
      /* try to find a max distance goal around */
      bool         new_goal = false;
      const double dr = 0.5, dtheta = 30, dz = 0.3;

      double          new_x, new_y, new_z, max_dist = -1.0;
      Eigen::Vector3d goal;

      for (double r = dr; r <= 5 * dr + 1e-3; r += dr) {
        for (double theta = -90; theta <= 270; theta += dtheta) {
          for (double nz = 1 * dz; nz >= -1 * dz; nz -= dz) {

            new_x = target_point_(0) + r * cos(theta / 57.3);
            new_y = target_point_(1) + r * sin(theta / 57.3);
            new_z = target_point_(2) + nz;
            Eigen::Vector3d new_pt(new_x, new_y, new_z);

            dist = planner_manager_->pp_.dynamic_ ?
                edt_env->evaluateCoarseEDT(new_pt, /* time to program start */ info->duration_) :
                edt_env->evaluateCoarseEDT(new_pt, -1.0);

            if (dist > max_dist) {
              /* reset target_point_ */
              goal(0)  = new_x;
              goal(1)  = new_y;
              goal(2)  = new_z;
              max_dist = dist;
            }
          }
        }
      }

      if (max_dist > 0.3) {
        cout << "change goal, replan." << endl;
        target_point_ = goal;
        have_target_  = true;
        end_vel_.setZero();

        if (exec_state_ == EXEC_TRAJ) {
          changeFSMExecState(REPLAN_NEW, "SAFETY");
        }

        visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));

      } else {
        // have_target_ = false;
        // cout << "Goal near collision, stop." << endl;
        // changeFSMExecState(WAIT_TARGET, "SAFETY");
        cout << "goal near collision, keep retry" << endl;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
    }
  }

  /* ---------- check trajectory ---------- */
  if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ) {
    double dist;
    bool   safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      if (dist > 0.5) {
        ROS_WARN("current traj %lf m to collision", dist);
        collide_ = true;
        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
      } else {
        ROS_ERROR("current traj %lf m to collision, emergency stop!", dist);
        replan_pub_.publish(std_msgs::Empty());
        have_target_ = false;
        changeFSMExecState(WAIT_TARGET, "SAFETY");
      }
    } else {
      collide_ = false;
    }
  }
}

void JMReplanFSM::waypointCallback(const nav_msgs::PathConstPtr& msg){
  if (msg->poses[0].pose.position.z < -0.1) return;
  cout << "Triggered!" << endl;

  vector<Eigen::Vector3d> global_wp;
  if (target_type_ == TARGET_TYPE::REFENCE_PATH) {
    for (int i = 0; i < waypoint_num_; ++i) {
      Eigen::Vector3d pt;
      pt(0) = waypoints_[i][0];
      pt(1) = waypoints_[i][1];
      pt(2) = waypoints_[i][2];
      global_wp.push_back(pt);
    }
  } else {
    if (target_type_ == TARGET_TYPE::MANUAL_TARGET) {
      target_point_(0) = msg->poses[0].pose.position.x;
      target_point_(1) = msg->poses[0].pose.position.y;
      target_point_(2) = 1.0;
      std::cout << "manual: " << target_point_.transpose() << std::endl;

    } else if (target_type_ == TARGET_TYPE::PRESET_TARGET) {
      target_point_(0) = waypoints_[current_wp_][0];
      target_point_(1) = waypoints_[current_wp_][1];
      target_point_(2) = waypoints_[current_wp_][2];

      current_wp_ = (current_wp_ + 1) % waypoint_num_;
      std::cout << "preset: " << target_point_.transpose() << std::endl;
    }

    global_wp.push_back(target_point_);
    visualization_->drawGoal(target_point_, 0.3, Eigen::Vector4d(1, 0, 0, 1.0));
  }

  planner_manager_->setGlobalWaypoints(global_wp);
  end_vel_.setZero();
  have_target_ = true;
  trigger_     = true;

  if (exec_state_ == WAIT_TARGET) {
    changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
  }
}

void JMReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg){
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
}


}  