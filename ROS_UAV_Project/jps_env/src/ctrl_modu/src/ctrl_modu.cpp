#include  "ctrl_modu/ctrl_modu.h"
//#include <plan_env/edt_environment.h>

//
ros::Subscriber obs_sub,odom_sub,goal_sub,imu_sub;
vector<ros::Subscriber> obs_subs(11);
ros::Subscriber env_msg_sub;
ros::Publisher pub1, cmd_pub,event_pub;

vector<Eigen::Vector3d> obs_pos,obs_directions;
nav_msgs::Odometry odom_,_odom_last;
Eigen::Vector3d envposition_, direction_,obs_direction_;
double distance_,kv(2.0),ky(2.0);
std_msgs::ColorRGBA  obs_color;
std_msgs::Bool  dynamic_event;
nav_msgs::Path goal_;
quadrotor_msgs::PositionCommand cmd;

Eigen::Vector3d goal,position, velocity,acceleration;
Eigen::Vector3d vel_d,vel_modu_d, acceleration_d;
double yaw_d, yaw, yawdot_d;


void rcvObsCallbck(const geometry_msgs::PoseStamped& obs ){
        //cout<<obs_pose.pose.position<<endl;
        ros::Time begin_ =ros::Time::now();

        Eigen::Vector3d obs_position(obs.pose.position.x,
                                                                      obs.pose.position.y,
                                                                      obs.pose.position.z);
        Eigen::Vector3d odom_position(odom_.pose.pose.position.x,
                                                                          odom_.pose.pose.position.y,
                                                                          odom_.pose.pose.position.z);  
        //obs_color = obs.color;
        //cout<<"The color is: "<<obs_color<<endl;
        //cout<<"The position is: "<<obs_position<<endl;
        obs_direction_ = odom_position - obs_position;
        if(obs_direction_.norm()<0.5){//5米圆周范围内动障碍
                //Dynamic_Obs dy_obs(obs.color, obs_position);
                obs_pos.push_back(obs_position);
                obs_directions.push_back(obs_direction_.normalized());
        }
        if(obs_pos.size())
        {
                dynamic_event.data=true;
                event_pub.publish(dynamic_event);
        }else{
                dynamic_event.data = false;
                event_pub.publish(dynamic_event);
        }
        obs_pos.push_back(envposition_);
        obs_directions.push_back(direction_.normalized());

        if(obs_pos.size()>10){
             obs_pos.clear();
             obs_directions.clear();
        }
         //cout<<"Obs continer: "<<obs_pos.size()<<endl;       
        ros::Time after_ = ros::Time::now();
        //cout<<"After:"<<(after_ - begin_).toSec()<<endl;

}
void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& msg){
        geometry_msgs::PoseStamped pt = *msg;
        goal_.poses.clear();
        goal_.poses.push_back(pt);
        goal_.header.frame_id = std::string("world");
        goal_.header.stamp = ros::Time::now();
        pub1.publish(goal_);
        goal  = Eigen::Vector3d(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
}

void rcvOdometryCallbck(const nav_msgs::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  if(odom_!= odom){
      odom_ = odom; } 
      position = Eigen::Vector3d(odom_.pose.pose.position.x,odom_.pose.pose.position.y,odom_.pose.pose.position.z); 
      velocity   =  Eigen::Vector3d(odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z);
      yaw = atan2(velocity(1),velocity(0));
      //cout<<"yaw:"<<yaw<<endl;
}

void rcvimuCallbck(const sensor_msgs::Imu&  imu){
         acceleration<<imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z;
}

void rcvEnvmsgCallbck(const modu_msgs::modu& env_msg){
        envposition_<<env_msg.position.x,env_msg.position.y,env_msg.position.z;
        direction_<<env_msg.direction.x,env_msg.direction.y, env_msg.direction.z;
        distance_= env_msg.distance;
        Eigen::Matrix3d matrixE, matrixM,matrixD;
        vector<double> w(obs_pos.size(),1);

        for(int i = 0; i<obs_pos.size(); i++){
                for(int j = 0; j< obs_pos.size(); j++){
                        if(j!=i) w[i]*= Tao(obs_pos[j])/( Tao(obs_pos[i]) + Tao(obs_pos[j]) );
                }
        }
        // double ws(0);
        // for(int i = 0;i <w.size();i++){
        //         ws += w[i]; 
        //         cout<<"w:"<<ws<<" ";
        // }
         for(int i = 0; i<obs_directions.size();i++){
                matrixE<<  obs_directions[i][0],obs_directions[i][1],obs_directions[i][2],
                                      -obs_directions[i][1],obs_directions[i][0],0,
                                        0,-obs_directions[i][2],obs_directions[i][1];
                matrixD << 1 - w[i]/(Tao(obs_pos[i]) + 1),  0,  0,
                                        0, 1 + w[i]/(Tao(obs_pos[i]) + 1), 0,
                                        0, 0, 1 + w[i]/(Tao(obs_pos[i]) + 1); 
                matrixM *= matrixE*matrixD*matrixE.inverse();                       
                //cout<<matrixE<<endl;
        }
        vel_d = 0.001*(goal - position).normalized();//max velocity 3;max acceleration 2.5
        //cout<<"vel_cmd:"<<vel_d<<endl;
        vel_modu_d = 3*(matrixM * vel_d).normalized();
        //cout<<"vel_modu_d:"<<vel_modu_d<<endl;
        yaw_d = atan2(goal(1), goal(0));
        yawdot_d = 0.001*ky*(yaw - yaw_d);
        acceleration_d = 0.001*kv*(velocity - vel_d);
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "world";

        cmd.position.x = goal(0);
        cmd.position.y = goal(1);
        cmd.position.z = goal(2);

        cmd.velocity.x = vel_d(0);
        cmd.velocity.y = vel_d(1);
        cmd.velocity.z = vel_d(2);

        cmd.acceleration.x = acceleration_d(0);
        cmd.acceleration.y = acceleration_d(1);
        cmd.acceleration.z = acceleration_d(2);
        cmd.yaw = yaw_d;
        cmd.yaw_dot = yawdot_d; 
        //double pos_gain[3] = { 5.7, 5.7, 6.2 };
        //double vel_gain[3] = { 3.4, 3.4, 4.0 };
        cmd.kx[0] = 5.7;
        cmd.kx[1] = 5.7;
        cmd.kx[2] = 6.2;
        cmd.kv[0] = 3.4;
        cmd.kv[1] = 3.4;
        cmd.kv[2] = 4.0;
  
        cmd_pub.publish(cmd);
}


double Tao(Eigen::Vector3d v3,double r ){
        double dis = pow(v3[0] - 2.0,2)+pow(v3[1] - 0.0,2)
        +pow(v3[2] - 2.0,2);
        //cout<<"dist:"<<dis/(R*R)-1<<endl;
        return dis/(r*r)-1;
}
int main(int argc,char** argv){
        ros::init(argc,argv,"ctrl_modu");
        ros::NodeHandle nh("~");
        odom_sub = nh.subscribe("/state_ukf/odom",1,rcvOdometryCallbck);
        for(int i =0; i<10; i++){
                obs_subs[i] = nh.subscribe("/dynamic/pose_" + to_string(i), 1, rcvObsCallbck);
        }

        goal_sub = nh.subscribe("/move_base_simple/goal",1,goal_callback);
        env_msg_sub = nh.subscribe("/fast_planner_node/env_msg", 1, rcvEnvmsgCallbck);
        //imu_sub = nh.subscribe();


        cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/modu_cmd", 50);
        event_pub = nh.advertise<std_msgs::Bool>("/Is_there_dynObs",50);

        ros::spin();
        return 0;
}