
#include <jpsfinder.h>
#include <sstream>
using namespace std;
using namespace Eigen;

ros::Subscriber global_corners_sub;
ros::Subscriber target_sub;
ros::Publisher _global_corners_pub;
pcl::VoxelGrid<pcl::PointXYZ> _voxel_sampler;
pcl::search::KdTree<pcl::PointXYZ> _kdtreeLocalMap;
pcl::PointCloud<pcl::PointXYZ> _jmp_corners_map,_local_map, _global_corners;
sensor_msgs::PointCloud2 global_corners_map;
bool has_corners(0);
Eigen::Vector3d _target(10.0, 10.0, 1.0),_start_pose(0.0, 0.0, 0.0);
fast_planner::EDTEnvironment::Ptr edt_environment_;
//1. 订阅角点地图索引
//2. 搜索JPS路径 
//3. 扩展角点
double _resolution, _inv_resolution;
double _gl_xl,_gl_yl,_gl_zl;
int  _GLX_SIZE, _GLY_SIZE, _GLZ_SIZE;
double _x_size, _y_size, _z_size;

inline Eigen::Vector3i coord2gridIndex(const Eigen::Vector3d& pt) {
  Eigen::Vector3i idx;
  idx(0) = std::min(std::max(int((pt(0) - _gl_xl) * _inv_resolution), 0), _GLX_SIZE - 1);
  idx(1) = std::min(std::max(int((pt(1) - _gl_yl) * _inv_resolution), 0),  _GLY_SIZE - 1);
  idx(2) = std::min(std::max(int((pt(2) - _gl_zl) * _inv_resolution), 0),  _GLZ_SIZE - 1);
  return idx;
};

void rcvglobalCornersCallbck(const sensor_msgs::PointCloud2& globalCorners){
	if (has_corners) ;//return;
	pcl::PointCloud<pcl::PointXYZ> corners_input;
	pcl::fromROSMsg(globalCorners, corners_input);

    	cout<<"corners_input before filter:"<<corners_input.size()<<endl;
    	// _voxel_sampler.setLeafSize(0.1f, 0.1f, 0.1f);
    	// _voxel_sampler.setInputCloud(corners_input.makeShared());
    	// _voxel_sampler.filter(_jmp_corners_map);
	// cout<<"_jmp_corners_map after filter:"<<_jmp_corners_map.size()<<endl;
    	// _kdtreeLocalMap.setInputCloud(_jmp_corners_map.makeShared());
    	// has_corners = true;
	// pcl::toROSMsg(_jmp_corners_map,global_corners_map);
	// global_corners_map.header.frame_id = "world";
   	// _global_corners_pub.publish(global_corners_map);
}

void rcvtargetCallbck(const nav_msgs::PathConstPtr& msg ){
	_target(0) = msg->poses[0].pose.position.x;
	_target(1) = msg->poses[1].pose.position.y;
	_target(2) = msg->poses[2].pose.position.z;
}


void path_finder( ){
	double _step_scale(0.1);
	Eigen::Vector3d current_pos(_start_pose(0),_start_pose(1),_start_pose(2));
	Eigen::Vector3d _direc(_target(0) - _start_pose(0),_target(1) - _start_pose(1),_target(2) - _start_pose(2));
	
	//_direc = _direc/_direc.norm();
	_direc = _direc.normalized();
	cout<<"drrection:"<<_direc<<endl;
	current_pos += _direc*_step_scale;
	coord2gridIndex(current_pos);
}


// int main(int argc,char** argv){
// 	ros::init(argc,argv,"JPSPlus_finder");
// 	ros::NodeHandle nh("~");
// 	nh.getParam("resolution",     _resolution);
//     	nh.getParam("map/x_size",     _x_size);
//     	nh.getParam("map/y_size",     _y_size);
//     	nh.getParam("map/z_size",     _z_size);

// 	_inv_resolution = 1.0 / _resolution;
//     	_gl_xl = -_x_size/2.0;
//     	_gl_yl = -_y_size/2.0;
//     	_gl_zl = -_z_size/2.0;
//     	_GLX_SIZE = (int)(_x_size * _inv_resolution);
//     	_GLY_SIZE = (int)(_y_size * _inv_resolution);
//     	_GLZ_SIZE = (int)(_z_size * _inv_resolution);

//     	global_corners_sub = nh.subscribe("/global_corners/corner_points",1,rcvglobalCornersCallbck);
// 	target_sub = nh.subscribe("/target",1,rcvtargetCallbck);
//         _global_corners_pub = nh.advertise<sensor_msgs::PointCloud2>("/global_corners/corner_points_after_filter",1);
//         vector<Eigen::Vector3d>  vec;
// 	Eigen::Vector3d  num_to_find(3.0,6.0,9.0);
// 	 for(int i = 0; i<11;  i++){
// 		Eigen::Vector3d v_point(i*1.0, i*2.0, i*3.0);
// 	 	vec.push_back(v_point);
// 		vec.push_back(v_point);
// 	 }
// 	//vector<Eigen::Vector3d>::iterator it = vec.begin();
// 	 vector<Eigen::Vector3d>::iterator it = find(vec.begin(),vec.end(), num_to_find );
// 	 cout<<"find it!" <<*it<<"located in:"<<it - vec.begin()<<endl;
// 	ros::Rate rate(1);
// 	bool status = ros::ok();
// 	while(ros::ok()){
//     		ros::spinOnce();
//     		cout<<"Hello JPSPlus!"<<endl;
// 		path_finder();
//     		rate.sleep();
// 	}
// return 0;
// }

Jps::~Jps(){
	for(int i = 0; i < allocate_num_; i++) {
		//delete path_node_pool_[i];
	}
}

int Jps::search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool dynamic, double time_start){

}

void Jps::setParam(ros::NodeHandle& nh){

}

void Jps::retrievePath(NodePtr end_node){

}

void Jps::setEnvironment(const fast_planner::EDTEnvironment::Ptr& env){
	  this->edt_environment_ = env;
}

 void Jps::findCollisionRange( vector<Eigen::Vector3d>& colli_start,
                                                      vector<Eigen::Vector3d>& colli_end,
                                                      Eigen::Vector3d& start_pts,
                                                      Eigen::Vector3d& end_pts) {
    bool     last_safe = true,safe;

    double dist = (end_pts - start_pts).norm();
    double delt_s(0.05);
    double d_num(dist/delt_s);

    
    for(double t = 0.0; t <1.0; t += 1/d_num){
	Eigen::Vector3d ptc = start_pts +  t *  (end_pts - start_pts);
		
	safe = edt_environment_->evaluateCoarseEDT(ptc, -1.0) < 0.1 ? false : true;
	if (last_safe && !safe) {
		colli_start.push_back(ptc);
	}else if(!last_safe && safe){
		colli_end.push_back(ptc);
	}
   }
  }
  
