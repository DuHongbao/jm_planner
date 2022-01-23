#include<iostream>
#include<ros/ros.h>
#include "jpsfinder.h"
#include <plan_env/edt_environment.h>
#include <thread>
using namespace std;
using namespace fast_planner;
fast_planner::EDTEnvironment::Ptr edt_environment_;
SDFMap::Ptr sdf_map_;


int main(int argc, char**argv){

        ros::init(argc, argv, "JM_planner");
        ros::NodeHandle n("~");
        Node node;
        Jps jps;
        //sdf_map_.reset(new SDFMap);
        //sdf_map_->initMap(n);
        //edt_environment_.reset(new EDTEnvironment);
        //edt_environment_->setMap(sdf_map_);
        //cout<<"Hello JM_Planner"<<endl;
        return 0;
}
