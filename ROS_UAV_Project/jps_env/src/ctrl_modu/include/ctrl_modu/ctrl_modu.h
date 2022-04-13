#ifndef _NON_CTRL_MODU_H_
#define _NON_CTRL_MODU_H_

        #include <iostream>
        #include <ros/ros.h>
        #include <Eigen/Eigen>
        #include <std_msgs/ColorRGBA.h>
        #include <std_msgs/Bool.h>
        #include <visualization_msgs/Marker.h>
        #include "quadrotor_msgs/PositionCommand.h"
        #include <geometry_msgs/PoseStamped.h>
        #include <nav_msgs/Odometry.h>
        #include <nav_msgs/Path.h>
        #include <sensor_msgs/Imu.h>
        #include <modu_msgs/modu.h>
        #include <vector>
        #include <string>
        #include <cmath>
        using namespace std;
        #define R 1.0
        double Tao(Eigen::Vector3d v3, double r=R  );
        class Dynamic_Obs{
                private:


                public:
                        std_msgs::ColorRGBA  color;
                        Eigen::Vector3d  position_;
                        Eigen::Vector3d  direction_;
                        ros::Time time_stemp_;
                        Dynamic_Obs(){}
                        Dynamic_Obs(std_msgs::ColorRGBA  c,Eigen::Vector3d p){
                                color = c;
                                position_ = p;
                                //direction_ = d;

                        }
                        ~Dynamic_Obs(){};
                
        };


#endif