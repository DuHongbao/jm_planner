#ifndef _JM_JPS_H
#define _JM_JPS_H

#include<plan_env/edt_environment.h>
#include<plan_env/raycast.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <path_searching/astar.h>
#include <visualization_msgs/Marker.h>

namespace fast_planner {
#define IN_CLOSE_SET 'a'
#define IN_OPEN_SET 'b'
#define NOT_EXPAND 'c'




class Graphnode{
private:

public:
        enum NODE_STATE{NEW = 1, CLOSE = 2, OPEN = 3};

        Graphnode(){}
        Graphnode(Eigen::Vector3d pos, NODE_STATE state, int id){
                pos_ = pos;
                state_ = state;
                id_  =id;
        }
        ~Graphnode(){}

        Eigen::Vector3d pos_;
        int  id_;
        NODE_STATE  state_;
        vector<shared_ptr<Graphnode>>  neighbors_;
        typedef shared_ptr<Graphnode>  Ptr;
};



class JmJps{
        private:
        
                NodeHashTable0 expanded_nodes_;
                std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
                vector<NodePtr> path_node_pool_;

                std::vector<NodePtr> path_nodes_;
                Eigen::Vector3d origin_, map_size_3d_;
                double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
                double lambda_heu_=1.0;
                int allocate_num_;
                vector<Graphnode::Ptr> graph_;
                int max_raw_path_;

                bool has_path_ = false;
                EDTEnvironment::Ptr edt_environment_;
                ros::Subscriber  odom_sub_, waypoint_sub_, Jps_points_sub_;
                int use_node_num_;
                ros::Publisher marker_pub_;

                Eigen::Vector3d start_pt_, start_vel_, start_acc_, start_yaw_;  // start state
                Eigen::Vector3d target_point_, end_vel_;    
                pcl::PointCloud<pcl::PointXYZ>  pclCorner_input;
                Eigen::Vector3d sample_inflate_;


        //raycasting 
                vector<RayCaster> casters_;
                Eigen::Vector3d offset_;
        public:
                JmJps();
                ~JmJps();

                double clearance_;
                enum { REACH_END = 1, NO_PATH = 2 };
                void init(ros::NodeHandle& nh);
                void JpsPointsCallback(const sensor_msgs::PointCloud2& globalCorners);
                void setEnvironment(const EDTEnvironment::Ptr& env);
                void pathFind(vector<Eigen::Vector3d> &jps_path);
                double pathLength(const vector<Eigen::Vector3d>& path) ;
                void findJpsPath(Eigen::Vector3d start,Eigen::Vector3d end,  vector<Eigen::Vector3d> & row_path);
                //void findJpsPath(Eigen::Vector3d start,Eigen::Vector3d end, vector<Eigen::Vector3d> & final_path);

                int Jsearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, vector<Graphnode::Ptr>& graph);
                Eigen::Vector3i posToIndex(Eigen::Vector3d pt);

                double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
                double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
                double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
                void retrievePath(NodePtr end_node);
                vector<Graphnode::Ptr> createGraph(Eigen::Vector3d start, Eigen::Vector3d end);
                //bool lineVis(Graphnode::Ptr nodeA, Graphnode::Ptr nodeB);
                bool lineVis(Graphnode::Ptr nodeA, Graphnode::Ptr nodeB, double thresh, Eigen::Vector3d& pc, int caster_id = 0);
                vector<Eigen::Vector3d> discretizePath(const vector<Eigen::Vector3d>& path, int pt_num);
                vector<Eigen::Vector3d> pathToGuidePts(vector<Eigen::Vector3d>& path, int pt_num);

};



}
#endif








