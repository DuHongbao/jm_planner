#include <path_searching/jm_jps.h>
using namespace std;
namespace fast_planner{
        JmJps::JmJps(){}
        
        JmJps::~JmJps(){}

        void JmJps::init(ros::NodeHandle& nh){
                //nh.param("jmjps/resolution", resolution_, -1.0);
                nh.param("jmjps/allocate_num", allocate_num_, -1);

                nh.param("topo_prm/sample_inflate_x", sample_inflate_(0), -1.0);
                nh.param("topo_prm/sample_inflate_y", sample_inflate_(1), -1.0);
                nh.param("topo_prm/sample_inflate_z", sample_inflate_(2), -1.0);
                nh.param("topo_prm/clearance", clearance_, -1.0);
                nh.param("topo_prm/max_raw_path", max_raw_path_, -1);
                resolution_ = edt_environment_->sdf_map_->getResolution();
                offset_ = Eigen::Vector3d(0.5, 0.5, 0.5) - edt_environment_->sdf_map_->getOrigin() / resolution_;


                /*------------map params--------------*/
                this->inv_resolution_ = 1.0 / resolution_;
                edt_environment_->getMapRegion(origin_, map_size_3d_);

                /*  ---------- pre-allocated node ---------- */
                path_node_pool_.resize(allocate_num_);
                for (int i = 0; i < allocate_num_; i++) {
                        path_node_pool_[i] = new Node;
                }
                use_node_num_ = 0;
                cout<<"________JMJPS__________"<<endl;
                Jps_points_sub_ = nh.subscribe("/local_corners/corner_points", 1, &JmJps::JpsPointsCallback, this);
                marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 1);

                for (int i = 0; i < max_raw_path_; ++i) {
                        casters_.push_back(RayCaster());
                }


        }

        void JmJps::setEnvironment(const EDTEnvironment::Ptr& env) { this->edt_environment_ = env; }


        vector<Graphnode::Ptr>  JmJps::createGraph(Eigen::Vector3d start, Eigen::Vector3d end){
                graph_.clear();
                Eigen::Vector3d pc;
                Graphnode::Ptr startnode = Graphnode::Ptr(new Graphnode(start,Graphnode::NEW,0));
                Graphnode::Ptr endnode  =  Graphnode::Ptr(new Graphnode(end,Graphnode::NEW,1));
                graph_.push_back(startnode);
                graph_.push_back(endnode);
                for(int i=0; i<pclCorner_input.size(); i++){
                        Eigen::Vector3d pos(pclCorner_input[i].x,pclCorner_input[i].y, pclCorner_input[i].z);
                        Graphnode::Ptr  Node = Graphnode::Ptr(new Graphnode(pos,Graphnode::NEW,i+2)); 
                        graph_.push_back(Node);
                }

                for(int i = 0; i< graph_.size(); i++){
                        for(int j = i+1; j< graph_.size(); j++){

                                if(lineVis(graph_[i], graph_[j], resolution_, pc)){
                                        graph_[i]->neighbors_.push_back(graph_[j]);
                                        graph_[j]->neighbors_.push_back(graph_[i]);
                                }
                        }
                }
                /*for(int i = 0; i < graph_.size(); i++){
                        cout<<"graph_:"<<i<<":"<<graph_[i]->pos_<<" neighbor:"<<graph_[i]->neighbors_.size()<<endl;
                }*/
                cout<<"size of graph:"<<graph_.size()<<endl;
                return graph_;
        }

        bool JmJps::lineVis(Graphnode::Ptr nodeA, Graphnode::Ptr nodeB, double thresh, Eigen::Vector3d& pc, int caster_id){
                Eigen::Vector3d ray_pt;
                Eigen::Vector3i pt_id;
                double dist;
                casters_[caster_id].setInput(nodeA->pos_/resolution_, nodeB->pos_/resolution_);
                while(casters_[caster_id].step(ray_pt)){
                        pt_id(0) = ray_pt(0) + offset_(0);
                        pt_id(1) = ray_pt(1) + offset_(1);
                        pt_id(2) = ray_pt(2) + offset_(2);
                        dist = edt_environment_->sdf_map_->getDistance(pt_id);
                        if (dist <= thresh) {
                                edt_environment_->sdf_map_->indexToPos(pt_id, pc);
                                return false;
                        }
                }
                return true;
        }

        void JmJps::JpsPointsCallback(const sensor_msgs::PointCloud2& globalCorners){

                pcl::PointCloud<pcl::PointXYZ> corners_input;
                pcl::fromROSMsg(globalCorners, pclCorner_input);
                //cout<<"________JMJPS-JpsPoints callbck__________"<< pclCorner_input.size()<<endl;

                visualization_msgs::Marker  marker;

                marker.header.frame_id = "world";
                marker.header.stamp = ros::Time::now();
                marker.type = visualization_msgs::Marker::CUBE;
                marker.pose.position.x = path_nodes_[0]->position(0);
                marker.pose.position.y = path_nodes_[0]->position(1);
                marker.pose.position.z = path_nodes_[0]->position(2);
                marker.pose.position.x = 0;
                marker.pose.position.y = 0;
                marker.pose.position.z = 1;

                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.color.r = 1.0f;
                marker.color.g = 1.0f;
                marker.color.b = 0.0f;
                marker.color.a = 1.0;
                marker.scale.x = 1.0;
                marker.scale.y = 1.0;
                marker.scale.z = 1.0;
                marker_pub_.publish(marker);
        }

        void JmJps::pathFind(vector<Eigen::Vector3d> &jps_path){
                     /*************find the shortst path according to the corners points****************/
                pcl::KdTreeFLANN<pcl::PointXYZ>  kdtree;
                Eigen::Vector3d  delta_path = (target_point_ - start_pt_).normalized()*0.1;
                vector<Eigen::Vector3d>  colli_start,colli_end;
                bool safe(1),last_safe(1);
                for(int i  = 0; i< (target_point_ - start_pt_).norm()/delta_path.norm() ; i++){
                        Eigen::Vector3d  current_point = start_pt_ + i*delta_path;
                        auto edt_env =  edt_environment_;
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
                has_path_ = 1;
        }



        void   queueClear(std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0>  &q){
                std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0>  empty;
                swap(empty,q);
        }
        //The new Jsearch method
        int JmJps::Jsearch(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, vector<Graphnode::Ptr>& graph){
                int graphnodeID = 0;
                Eigen::Vector3i  end_index = posToIndex(end_pt);
                graph = createGraph(start_pt, end_pt);
                expanded_nodes_.clear();
                queueClear(open_set_);
                
                Graphnode::Ptr curGraphnode = graph[0];
                NodePtr cur_node = path_node_pool_[0];
                cur_node->parent = NULL;
                cur_node->position = start_pt;
                cur_node->index = posToIndex(start_pt);
                cur_node->g_score = 0.0;
                cur_node->f_score = lambda_heu_*getEuclHeu(cur_node->position, end_pt);
                cur_node->origin_id = graphnodeID;
                cur_node->node_state = IN_OPEN_SET;
                open_set_.push(cur_node);
                use_node_num_ =1;
                expanded_nodes_.insert(cur_node->index, cur_node);
                NodePtr ternimate_node = NULL;

                while(!open_set_.empty()){
                        curGraphnode = graph[graphnodeID];
                        cur_node = open_set_.top();
                        graphnodeID = cur_node->origin_id;
                        bool  reach_end = abs(cur_node->index(0)-end_index(0))<=1&&abs(cur_node->index(1)-end_index(1))<=1&&abs(cur_node->index(2)-end_index(2))<=1;
                        if(reach_end){
                                ternimate_node = cur_node;
                                retrievePath(ternimate_node);
                                has_path_ = true;
                                cout<<"REACH END"<<endl;
                                return REACH_END;
                        }
                        /*----------------pop node and add to close set----------------*/
                        open_set_.pop();
                        cur_node->node_state = IN_CLOSE_SET;
                        graph[graphnodeID]->state_ = Graphnode::CLOSE;
                        cout<<"cur_node:"<<cur_node->position<<endl;
                        /*---------------------- neighbor expansion ---------------------- */
                        vector<Graphnode::Ptr> neighbors = curGraphnode->neighbors_;
                        cout<<"运行到这里啦Neighbors:"<<neighbors.size()<<endl;

                        for(int i = 0; i<neighbors.size(); i++){
                                if(neighbors[i]->state_ ==Graphnode::CLOSE) continue;
                                Eigen::Vector3d pro_pos = neighbors[i]->pos_;
                                Eigen::Vector3i  pro_idx = posToIndex(pro_pos);
                                double tmp_g_score,tmp_f_score;
                                tmp_g_score = (pro_pos-cur_node->position).norm() + cur_node->g_score;
                                tmp_f_score = tmp_g_score + lambda_heu_*getEuclHeu(pro_pos,end_pt);
                                NodePtr  pro_node = expanded_nodes_.find(pro_idx);
                                if(pro_node == NULL){
                                        cout<<" noE ";
                                        pro_node = path_node_pool_[use_node_num_];
                                        pro_node->index = pro_idx;
                                        pro_node->origin_id = neighbors[i]->id_;
                                        pro_node->position = pro_pos;
                                        pro_node->parent = cur_node;
                                        pro_node->g_score = tmp_g_score;
                                        pro_node->f_score = tmp_f_score;
                                        pro_node->node_state = IN_OPEN_SET;
                                        expanded_nodes_.insert(pro_node->index, pro_node);
                                        use_node_num_ += 1;
                                        open_set_.push(pro_node);
                                }else if(pro_node->node_state == IN_OPEN_SET){
                                        cout<<" O ";
                                        if(tmp_g_score < pro_node->g_score){
                                                //pro_node->index = pro_idx;
                                                pro_node->g_score = tmp_g_score;
                                                pro_node->f_score = tmp_f_score;
                                                pro_node->position = pro_pos;
                                                pro_node->parent = cur_node;
                                        }
                                }else{
                                        cout << "error type in searching: " << pro_node->node_state << endl;
                                        
                                }
                        }
                }
                cout<<"NO_PATH"<<endl;
                return NO_PATH;
        }

        Eigen::Vector3i JmJps::posToIndex(Eigen::Vector3d pt){
                Eigen::Vector3i index = ((pt - origin_) * inv_resolution_).array().floor().cast<int>();
                return index;
        }

        double JmJps::getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2){
                return (x1 - x2).norm();
        }

        void  JmJps::retrievePath(NodePtr end_node){
                path_nodes_.clear();
                NodePtr cur_node = end_node;
                path_nodes_.push_back(cur_node);

                while (cur_node->parent != NULL) {
                        cur_node = cur_node->parent;
                        path_nodes_.push_back(cur_node);
                }

                reverse(path_nodes_.begin(), path_nodes_.end());

                cout<<"The Path Node size is:"<<path_nodes_.size()<<endl;
                for(int i = 0; i<path_nodes_.size(); i++){
                        cout<<path_nodes_[i]->position<<endl;
                }
        }

        double JmJps::pathLength(const vector<Eigen::Vector3d>& path) {
                double length = 0.0;
                if (path.size() < 2) return length;
                for (int i = 0; i < path.size() - 1; ++i) {
                        length += (path[i + 1] - path[i]).norm();
                }
                return length;
        }

        vector<Eigen::Vector3d> JmJps::discretizePath(const vector<Eigen::Vector3d>& path, int pt_num) {
                vector<double> len_list;
                len_list.push_back(0.0);

                for (int i = 0; i < path.size() - 1; ++i) {
                        double inc_l = (path[i + 1] - path[i]).norm();
                        len_list.push_back(inc_l + len_list[i]);
                }
                // calc pt_num points along the path
                double len_total = len_list.back();
                double dl = len_total / double(pt_num - 1);
                double cur_l;

                vector<Eigen::Vector3d> dis_path;
                for (int i = 0; i < pt_num; ++i) {
                        cur_l = double(i) * dl;
                        // find the range cur_l in
                        int idx = -1;
                        for (int j = 0; j < len_list.size() - 1; ++j) {
                                if (cur_l >= len_list[j] - 1e-4 && cur_l <= len_list[j + 1] + 1e-4) {
                                        idx = j;
                                        break;
                                }
                        }
                        // find lambda and interpolate
                        double lambda = (cur_l - len_list[idx]) / (len_list[idx + 1] - len_list[idx]);
                        Eigen::Vector3d inter_pt = (1 - lambda) * path[idx] + lambda * path[idx + 1];
                        dis_path.push_back(inter_pt);
                }
                        return dis_path;
        }

        vector<Eigen::Vector3d> JmJps::pathToGuidePts(vector<Eigen::Vector3d>& path, int pt_num) {
                return discretizePath(path, pt_num);
        }

       

        void JmJps::findJpsPath(Eigen::Vector3d start,Eigen::Vector3d end,  vector<Eigen::Vector3d> & row_path){
                ros::Time t1, t2;
                double search_time;
                row_path.clear();
                t1 = ros::Time::now();
                cout<<"运行到这里啦8！"<<endl;

               int p = Jsearch(start, end,  graph_);
               for(int i = 0;i<path_nodes_.size();i++){
                       row_path.push_back(path_nodes_[i]->position);
               }
               cout<<"The Path Node size is:"<<row_path.size();
        }
}



