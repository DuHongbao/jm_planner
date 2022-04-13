#include "visualization_msgs/Marker.h"
#include<ros/ros.h>

#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <random>
#include <sensor_msgs/PointCloud2.h>
#include <string>

#include <plan_env/linear_obj_model.hpp>
using namespace std;

int obj_num;
double _xy_size,_h_size,_vel, _yaw_dot, _acc_r1, _acc_r2, _acc_z, _scale1, _scale2, _interval;

ros::Publisher obj_pub;            // visualize marker
vector<ros::Publisher> pose_pubs;  // obj pose (from optitrack)
vector<LinearObjModel> obj_models;



random_device rd;
default_random_engine eng(rd());
uniform_real_distribution<double> rand_acc_r;
uniform_real_distribution<double> rand_acc_t;
uniform_real_distribution<double> rand_acc_z;

ros::Time time_update, time_change;

void updateCallback(const ros::TimerEvent& e);
void visualizeObj(int id);

int main(int argc,char** argv){
        ros::init(argc,argv,"dynamic_obj");
        ros::NodeHandle node("~");

        node.param("obj_generator/obj_num", obj_num, 5);
        node.param("obj_generator/xy_size", _xy_size, 5.0);
        node.param("obj_generator/h_size", _h_size, 2.0);
        node.param("obj_generator/vel", _vel, 5.0);
        node.param("obj_generator/yaw_dot", _yaw_dot, 5.0);
        node.param("obj_generator/acc_r1", _acc_r1, 4.0);
        node.param("obj_generator/acc_r2", _acc_r2, 6.0);
        node.param("obj_generator/acc_z", _acc_z, 3.0);
        node.param("obj_generator/scale1", _scale1, 1.5);
        node.param("obj_generator/scale2", _scale2, 2.5);
        node.param("obj_generator/interval", _interval, 2.5);

        obj_pub = node.advertise<visualization_msgs::Marker>("/dynamic/obj", 10);
        for (int i = 0; i < obj_num; ++i) {
                ros::Publisher pose_pub =
                node.advertise<geometry_msgs::PoseStamped>("/dynamic/pose_" + to_string(i), 10);
                pose_pubs.push_back(pose_pub);
        }

        ros::Timer update_timer = node.createTimer(ros::Duration(1 / 30.0), updateCallback);
        cout << "[dynamic]: initialize with " + to_string(obj_num) << " moving obj." << endl;
        ros::Duration(1.0).sleep();

        for(int i = 0; i< obj_num;++i){
                LinearObjModel model;
                Eigen::Vector3d pos(-5+1.0*i, -5 + 1.0*i, 0.1*i );
                Eigen::Vector3d vel(1.0,1.0,0.0);
                Eigen::Vector3d color(1.0, 1.0, 1.0);
                Eigen::Vector3d scale(1.0, 1.0, 1.0);
                double yaw = 0.0;
                double yaw_dot = 0.1;
                double r, t, z;
                r = rand_acc_r(eng);
                t = rand_acc_t(eng);
                z = rand_acc_z(eng);
                Eigen::Vector3d acc(r * cos(t), r * sin(t), z);
                
                model.initialize(pos, vel, acc, yaw, yaw_dot, color, scale);
                model.setLimits(Eigen::Vector3d(_xy_size, _xy_size, _h_size), Eigen::Vector2d(0.0, _vel),
                                Eigen::Vector2d(0, 0));
                obj_models.push_back(model);

        }

        time_update = ros::Time::now();
        time_change = ros::Time::now();
        ros::spin();

        return 0;
}

void updateCallback(const ros::TimerEvent& e) {
  ros::Time time_now = ros::Time::now();

  /* ---------- change input ---------- */
  double dtc = (time_now - time_change).toSec();
  if (dtc > _interval) {
    for (int i = 0; i < obj_num; ++i) {
      /* ---------- use acc input ---------- */
      // double r, t, z;
      // r = rand_acc_r(eng);
      // t = rand_acc_t(eng);
      // z = rand_acc_z(eng);
      // Eigen::Vector3d acc(r * cos(t), r * sin(t), z);
      // obj_models[i].setInput(acc);

      /* ---------- use vel input ---------- */
      double vx, vy, vz, yd;
      vx = 0.0;
      vy = 0.0;
      vz = 0.0;
      yd = 0.1;

      obj_models[i].setInput(Eigen::Vector3d(vx, vy, vz));
      obj_models[i].setYawDot(yd);
    }
    time_change = time_now;
  }

  /* ---------- update obj state ---------- */
  double dt = (time_now - time_update).toSec();
  time_update = time_now;
  for (int i = 0; i < obj_num; ++i) {
    obj_models[i].update(dt);
    visualizeObj(i);
  }

  /* ---------- collision ---------- */
  for (int i = 0; i < obj_num; ++i)
    for (int j = i + 1; j < obj_num; ++j) {
      bool collision = LinearObjModel::collide(obj_models[i], obj_models[j]);
      if (collision) {
        double yd1 = 0.1;
        double yd2 = -0.1;
        obj_models[i].setYawDot(yd1);
        obj_models[j].setYawDot(yd2);
      }
    }
}




void visualizeObj(int id) {
  Eigen::Vector3d pos, color, scale;
  pos = obj_models[id].getPosition();
  color = obj_models[id].getColor();
  scale = obj_models[id].getScale();
  double yaw = obj_models[id].getYaw();

  Eigen::Matrix3d rot;
  rot << cos(yaw), -sin(yaw), 0.0, sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0;

  Eigen::Quaterniond qua;
  qua = rot;

  /* ---------- rviz ---------- */
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.type = visualization_msgs::Marker::SPHERE;
  mk.action = visualization_msgs::Marker::ADD;
  mk.id = id;

  mk.scale.x = scale(0), mk.scale.y = scale(1), mk.scale.z = scale(2);
  mk.color.a = 0.5, mk.color.r = color(0), mk.color.g = color(1), mk.color.b = color(2);

  mk.pose.orientation.w = qua.w();
  mk.pose.orientation.x = qua.x();
  mk.pose.orientation.y = qua.y();
  mk.pose.orientation.z = qua.z();

  mk.pose.position.x = pos(0), mk.pose.position.y = pos(1), mk.pose.position.z = pos(2);

  obj_pub.publish(mk);

  /* ---------- pose ---------- */
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.header.seq = id;
  pose.pose.position.x = pos(0), pose.pose.position.y = pos(1), pose.pose.position.z = pos(2);
  pose.pose.orientation.w = 1.0;
  pose_pubs[id].publish(pose);
}


