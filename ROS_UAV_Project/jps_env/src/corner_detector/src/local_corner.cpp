#include<iostream>
#include<ros/ros.h>
#include<pcl/search/impl/kdtree.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <Eigen/Core>
#include<Eigen/Dense>
#include<vector>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h> 
     

#include<pcl/point_cloud.h>
#include<sensor_msgs/PointCloud2.h>
using namespace std;

sensor_msgs::PointCloud2 local_map;
sensor_msgs::PointCloud2 local_corners, global_corners;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
pcl::PointCloud<pcl::PointXYZ> _jmp_points_map,_local_map, _global_corners;
pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;


//pcl::PointCloud<pcl::PointXYZ>localMap_GR;

bool has_local_map;

Eigen::Vector3i voxel_num_;      
vector<bool> occupancy_buff(4000000,0);
vector<Eigen::Vector3i> localPoints_idx;
vector<Eigen::Vector3i> _corners_map_idx;
Eigen::Matrix3d _M;
nav_msgs::Odometry _odom,_odom_last;


double sensing_horizon, sensing_rate, estimation_rate;
double _x_size, _y_size, _z_size;
Eigen::Vector3i _map_size;
double _gl_xl, _gl_yl, _gl_zl;
double _resolution, _inv_resolution;
int _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;
bool has_odom(0);

vector<Eigen::Matrix3d> sobel3d_Gx(3,Eigen::Matrix3d::Zero());
vector<Eigen::Matrix3d> sobel3d_Gy(3,Eigen::Matrix3d::Zero());
vector<Eigen::Matrix3d> sobel3d_Gz(3,Eigen::Matrix3d::Zero());

ros::Time last_odom_stamp = ros::TIME_MAX;
ros::Subscriber local_points_sub;
ros::Subscriber odom_sub;
ros::Publisher _local_corners_pub, _global_corners_pub;

inline Eigen::Vector3d gridIndex2coord(const Eigen::Vector3i& index) {
  Eigen::Vector3d pt;
  pt(0) = ((double)index(0) + 0.5) * _resolution + _gl_xl;
  pt(1) = ((double)index(1) + 0.5) * _resolution + _gl_yl;
  pt(2) = ((double)index(2) + 0.5) * _resolution + _gl_zl;
  return pt;
};

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0), _GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0),  _GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0),  _GLZ_SIZE - 1);

  return idx;
};

int toBuffAdress(Eigen::Vector3i idx){
    return idx(0)*voxel_num_(1)*voxel_num_(2) + idx(1)*voxel_num_(2) + idx(2);
}

void conv3D(vector<Eigen::Vector3i> &localMapIdx, string mode){
    //1.局部角点检测
    //2.对局部点云作巻积
    pcl::PointCloud<pcl::PointXYZI> localMap_Gx,localMap_Gy,localMap_Gz;
    pcl::PointCloud<pcl::PointXYZ> localMap_GR;
    pcl::PointXYZI pt_tmpX, pt_tmpY,pt_tmpZ;
    pcl::PointXYZ  pt_tmpR;
    bool local_update(0);
    if(mode=="same"){
        //cout<<"same mode!"<<endl;
        for(int i = 0; i<localMapIdx.size(); ++i){
            Eigen::Matrix3d matLocal;
            Eigen::Vector3i idx_tmp;
            double deri_x(0.0),deri_y(0.0),deri_z(0.0), _R;
            for(int z = 0; z < 3; ++z){
                for(int x = 0; x < 3; ++x){
                    for(int y = 0; y < 3; ++y){
                        idx_tmp<<localMapIdx[i](0)+x-1,localMapIdx[i](1)+y-1,localMapIdx[i](2)+z-1;
                        deri_x += sobel3d_Gx[z](x,y)*occupancy_buff[toBuffAdress(idx_tmp)];
                        deri_y += sobel3d_Gy[z](x,y)*occupancy_buff[toBuffAdress(idx_tmp)];
                        deri_z += sobel3d_Gz[z](x,y)*occupancy_buff[toBuffAdress(idx_tmp)];
                    }
                }
            }
            //cout<<"deri_x : "<<deri_x<<endl;
            //cout<<"deri_y : "<<deri_y<<endl;
            //cout<<"deri_z : "<<deri_z<<endl;
            pt_tmpX.x = localMapIdx[i](0);
            pt_tmpX.y = localMapIdx[i](1);
            pt_tmpX.z = localMapIdx[i](2);
            pt_tmpX.intensity = deri_x;
            pt_tmpY.x = localMapIdx[i](0);
            pt_tmpY.y = localMapIdx[i](1);
            pt_tmpY.z = localMapIdx[i](2);
            pt_tmpY.intensity = deri_y;
            pt_tmpZ.x = localMapIdx[i](0);
            pt_tmpZ.y = localMapIdx[i](1);
            pt_tmpZ.z = localMapIdx[i](2);
            pt_tmpZ.intensity = deri_z;

            localMap_Gx.points.push_back(pt_tmpX);
            localMap_Gy.points.push_back(pt_tmpY);
            localMap_Gz.points.push_back(pt_tmpZ);
    
    //计算响应函数
            Eigen::Vector3d _pos_maxR = gridIndex2coord(localMapIdx[i]);            
            pt_tmpR.x =  _pos_maxR(0);
            pt_tmpR.y =  _pos_maxR(1);
            pt_tmpR.z =  _pos_maxR(2);
            _M << pt_tmpX.intensity *pt_tmpX.intensity , pt_tmpX.intensity * pt_tmpY.intensity, pt_tmpX.intensity * pt_tmpZ.intensity,
                         pt_tmpX.intensity *pt_tmpY.intensity ,  pt_tmpY.intensity * pt_tmpY.intensity,   pt_tmpY.intensity *  pt_tmpZ.intensity,
                         pt_tmpX.intensity *pt_tmpZ.intensity,  pt_tmpY.intensity* pt_tmpZ.intensity,   pt_tmpZ.intensity * pt_tmpZ.intensity;
            _R = _M.determinant() - 0.04*_M.trace()*_M.trace();
            if(_R>=-250.0&&_R<=-200.0){
                pt_tmpR.x -= 0.2*pt_tmpY.intensity*0.1;
                pt_tmpR.y += 0.2*pt_tmpX.intensity*0.1;
                pt_tmpR.z -= 0.2*pt_tmpZ.intensity*0.1;
                localMap_GR.points.push_back(pt_tmpR);
                _corners_map_idx.push_back(localMapIdx[i]);
                _global_corners.points.push_back(pt_tmpR);
                //localMap_GR.points.push_back(_R);
                //cout<<"R : "<<_R<<endl;
           }
        }
    }
    if(mode=="full"){
        //full mode
    }
   localMap_GR.width = localMap_GR.points.size();
   localMap_GR.height = 1;
   localMap_GR.is_dense = true;
   pcl::toROSMsg(localMap_GR, local_corners);
   local_corners.header.frame_id = "world";
   _local_corners_pub.publish(local_corners);
  //cout<<"size of local corners:"<<local_corners.width <<endl;
    //3.非极大抑制    
    //4.将检测到的点映射到全局角点地图上

    //5. 对全局角点进行滤波

    _voxel_sampler.setLeafSize(0.01f, 0.01f, 0.01f);
    _voxel_sampler.setInputCloud(_global_corners.makeShared());
    _voxel_sampler.filter(_global_corners);

    _global_corners.width = _global_corners.points.size();
    _global_corners.height = 1;
    _global_corners.is_dense = true;
    pcl::toROSMsg(_global_corners, global_corners);
    global_corners.header.frame_id = "world";
    _global_corners_pub.publish(global_corners);
}

void setSobel(){
    sobel3d_Gx.clear();
    sobel3d_Gy.clear();
    sobel3d_Gz.clear();
    for(int i=0;i<3;i++){
        sobel3d_Gx[i]<<  1,0,-1,
                                            1,0,-1,
                                            1,0,-1;
        sobel3d_Gy[i]<< -1, -1, -1,
                                            0, 0, 0,
                                            1,1, 1;
        sobel3d_Gz[i]<<1,1,1,
                                         1,1,1,
                                         1,1, 1;
        sobel3d_Gz[i] *=(double)(i-1); 
        //cout<<sobel3d_Gz[i]<<endl;
    }
}

//test_sobel

void harris3d(){
    clock_t time_stt = clock();
    int count(0);
    for (int i = 0; i < 3; ++i) voxel_num_(i) = ceil(_map_size(i) / _resolution);
    setSobel();
    //
    localPoints_idx.clear();
    //cout<<"height :"<<_jmp_points_map.width<<endl;
    //cout<<"size :"<<_jmp_points_map.size()<<endl;
    for(int i = 0;i<_jmp_points_map.size();i++){
        Eigen::Vector3d pos_pt;
        Eigen::Vector3i idx_pt;
        pos_pt<<_jmp_points_map[i].x,_jmp_points_map[i].y,_jmp_points_map[i].z;
        idx_pt = coord2gridIndex(pos_pt);
        localPoints_idx.push_back(idx_pt);
        occupancy_buff[toBuffAdress(idx_pt)] = 1;
        //cout<<"pos_pt:"<<pos_pt<<endl;
        //cout<<"idx_pt:"<<idx_pt<<endl;
        count++;
    }
    conv3D(localPoints_idx, "same");
    //cout <<"time use in Modulation  is " <<1000*  (clock() - time_stt)/(double)CLOCKS_PER_SEC <<"ms" << endl;
}



void rcvOdometryCallbck(const nav_msgs::Odometry& odom) {
  /*if(!has_global_map)
    return;*/
  has_odom = true;

  if(_odom!= odom){
      _odom = odom; }  
}



void rcvLocalPointsCallbck(const sensor_msgs::PointCloud2& localPoints)
{
    if (has_local_map) ;//return;
    //ROS_WARN("Local Points received..");
    pcl::PointCloud<pcl::PointXYZ> points_input;

        pcl::fromROSMsg(localPoints, points_input);
        //cout<<"points_input:"<<points_input.size()<<endl;
        _voxel_sampler.setLeafSize(0.02f, 0.02f, 0.02f);
        _voxel_sampler.setInputCloud(points_input.makeShared());
        _voxel_sampler.filter(_jmp_points_map);
        _kdtreeLocalMap.setInputCloud(_jmp_points_map.makeShared());
        has_local_map = true;


}


int main(int argc,char** argv)
{
    ros::init(argc,argv,"local_corners");
    ros::NodeHandle nh("~");
    Eigen::Vector3d pos_pt;
    nh.getParam("resolution",     _resolution);
    nh.getParam("map/x_size",     _x_size);
    nh.getParam("map/y_size",     _y_size);
    nh.getParam("map/z_size",     _z_size);
    _map_size<<_x_size, _y_size, _z_size;

//subscribe local pointcloud2
    local_points_sub = nh.subscribe("/pcl_render_node/cloud",1,rcvLocalPointsCallbck);
    odom_sub = nh.subscribe("odom",1,rcvOdometryCallbck);
    _local_corners_pub = nh.advertise<sensor_msgs::PointCloud2>("/local_corners/corner_points", 1);
    _global_corners_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_corners/corner_points",1);

    _inv_resolution = 1.0 / _resolution;
    _gl_xl = -_x_size/2.0;
    _gl_yl = -_y_size/2.0;
    _gl_zl = -_z_size/2.0;
    _GLX_SIZE = (int)(_x_size * _inv_resolution);
    _GLY_SIZE = (int)(_y_size * _inv_resolution);
    _GLZ_SIZE = (int)(_z_size * _inv_resolution);



ros::Rate rate(10);
bool status = ros::ok();
while( ros::ok()){
    ros::spinOnce();

    harris3d();
    rate.sleep();
}

return 0;
}

