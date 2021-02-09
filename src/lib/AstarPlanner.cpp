#include "AstarPlanner.h"
namespace astar{
    AStarPlanner::AStarPlanner(const Eigen::MatrixXi map){
        if(map.size()!=0){
            ok_to_run_ = true;
            map_cols = static_cast<int>(map.cols());
            map_rows = static_cast<int>(map.rows());
            map_ = map;
            std::cout<<"The map size in x directions is: "<<map_rows<<", size in y is: "<< map_cols <<"\n";
        }else{
            ok_to_run_ = false;
            std::cout<<"Invalid map! \n";
        }
        
    }
    AStarPlanner::~AStarPlanner(){

    }
    double AStarPlanner::ManhattDist(int current_x, int current_y, int target_x, int target_y){
        return static_cast<double>(abs(current_x-target_x) + abs(current_y-target_y));
    }
    double AStarPlanner::EulerDist(int current_x, int current_y, int target_x, int target_y){
        double temp_x = static_cast<double>(current_x - target_x);
        double temp_y = static_cast<double>(current_y - target_y);
        return sqrt(temp_x*temp_x+temp_y*temp_y);
    }
    double AStarPlanner::GetHeuristicCost(int current_x, int current_y, int target_x, int target_y){
        if(allow_diagonal_){
            return EulerDist(current_x, current_y, target_x, target_y);
        }else{
            return ManhattDist(current_x, current_y, target_x, target_y);
        }
    }
    void AStarPlanner::GetLowestFcostEntry(){
        if(open_list_.empty()){
            // if open list is empty, the iterator is set to the past-end position
            open_list_it_ = open_list_.end();
        }else{
            // other wish, find the element with the lowst f cost
            open_list_it_ = std::min_element(open_list_.begin(),open_list_.end(),
            [](const node& a, const node& b){return a.f_cost_ < b.f_cost_; });

        }
    }
    bool AStarPlanner::CompareLocation(int location1_x, int location1_y, int location2_x, int location2_y){
        if(static_cast<double>(abs(location1_x-location2_x)+abs(location1_y-location2_y))<0.1){
            return true;
        }else{
            return false;
        }
    }
    bool AStarPlanner::CompareLocation(const location& p1, const location& p2){
        if(static_cast<double>(abs(p1.x_-p2.x_)+abs(p1.y_-p2.y_))<0.1){
            return true;
        }else{
            return false;
        }
    }

    std::vector<node>::iterator AStarPlanner::IsPointInOpenList(const location& point){
        // add this in the capture list to use the member function
        return std::find_if(open_list_.begin(),open_list_.end(), [&point,this](const node& t){
                if( CompareLocation(point.x_, point.y_, t.node_location_.x_, t.node_location_.y_) ){
                    return true;
                }else{
                    return false;
                }
            }
        );
    }
    //
    std::unordered_map<int, node>::iterator AStarPlanner::IsPointInCloseList(int point){
        return std::find_if(close_list_.begin(),close_list_.end(),[&point](std::pair<const int, node>& t){
            if(t.first==point){
                return true;
            }else{
                return false;
            }
        });
    }

    void AStarPlanner::GetNeighbourCost(double parent_g_cost, const location& current_position){
        // step 1, generate valid neighbour locations (check bound)
        std::vector<location> neigbour;
        location p;
        p.x_ = current_position.x_ + 1;
        p.y_ = current_position.y_;
        neigbour.push_back(p);
         p.x_ = current_position.x_ - 1;
        p.y_ = current_position.y_;
        neigbour.push_back(p);
        p.x_ = current_position.x_;
        p.y_ = current_position.y_+1;
        neigbour.push_back(p);
        p.x_ = current_position.x_;
        p.y_ = current_position.y_-1;
        neigbour.push_back(p);

        if(allow_diagonal_){
            p.x_ = current_position.x_ + 1;
            p.y_ = current_position.y_ + 1;
            neigbour.push_back(p);
            p.x_ = current_position.x_ + 1;
            p.y_ = current_position.y_ - 1;
            neigbour.push_back(p);
            p.x_ = current_position.x_ - 1;
            p.y_ = current_position.y_ + 1;
            neigbour.push_back(p);
            p.x_ = current_position.x_ - 1;
            p.y_ = current_position.y_ - 1;
            neigbour.push_back(p);
        }

        neighbour_g_cost_list_.clear();
        // step 2. calculate g cost and 
        
        std::for_each(neigbour.begin(),neigbour.end(),[&parent_g_cost,&current_position,this](const location& t){
            // firstly check boundary
            if((t.x_>=0)&&(t.x_<map_cols)&&(t.y_>=0)&&(t.y_<map_rows)) {
                // then check barrier   
                if(map_(t.y_,t.x_)==0){// 0 means the map is accessable
                    // check whether the neighbour is in the closed list
                    if(IsPointInCloseList( MapSignature(map_cols, t ))== close_list_.end()){// equals end() means this point is not in the closed list 
                        // then calulate the g cost for this point
                        neighbour_list neigh_cost;
                        neigh_cost.node_location_ = t;
                        double temp_x = static_cast<double>(t.x_-current_position.x_);
                        double temp_y = static_cast<double>(t.y_-current_position.y_);
                        neigh_cost.g_cost_ = parent_g_cost + sqrt(temp_x*temp_x+temp_y*temp_y);
                        neighbour_g_cost_list_.push_back(neigh_cost);
                    }
                }
            }
        });
    }

    void AStarPlanner::ReconstructPath(path_list& path, const location& start_location, const location& end_location){
        path.clear();
        path.push_back(end_location);
        location previous_location;
        while(!CompareLocation(path.back(), start_location)) {// if the back of the list is not the starting point, the continue
            // find the parent signature of the current point
            int parent_signature = close_list_.at(MapSignature(map_cols, path.back())).parent_signature_;
            // use the parent signature to update the location 
            previous_location = close_list_.at(parent_signature).node_location_;
            // push this into the back of the path ist.
            path.push_back(previous_location);
        }
        // update map
        for_each(path.begin(),path.end(),[this](const location& t){
            map_(t.y_,t.x_)=-1;
        });
    }

    bool AStarPlanner::GetPath(path_list& path, 
                 const location& start_location, 
                 const location& target_location, 
                 bool allow_diagonal) {

        bool solution_found = false;
        allow_diagonal_ = allow_diagonal;
        // put the starting point into the openlist
        node start_node;
        start_node.node_location_ = start_location;
        start_node.parent_location_.x_ = -1;
        start_node.parent_location_.y_ = -1;
        start_node.UpdateSignature(map_cols);
        start_node.g_cost_ = 0.0;
        start_node.h_cost_ = GetHeuristicCost(start_node.node_location_.x_, 
                                              start_node.node_location_.y_, 
                                              target_location.x_, target_location.y_);
        start_node.f_cost_ = start_node.h_cost_;
        // push this into the openlist
        open_list_.push_back(start_node);
        close_list_.clear();
        while(!open_list_.empty()){
            // find the entry with lowest f cost in the openlist
            GetLowestFcostEntry();
            if(open_list_it_==open_list_.end()){// if nothing then return false flag
                solution_found = false;
                break;
            }
            double parent_g_cost = open_list_it_->g_cost_;
            location current_position = open_list_it_->node_location_;
            int current_signature = open_list_it_->node_signature_;
            
            close_list_.insert({open_list_it_->node_signature_,*open_list_it_});
            // remove this point from the openlist.
            open_list_.erase(open_list_it_);

            // check whether this currenty location is the target point
            if( CompareLocation(current_position,target_location)){
                solution_found = true;
                // if this is the target point, end the loop and reconstruct the path
                ReconstructPath(path, start_location, target_location);
                break;
            }

            // then calculate the g_cost for all the neighouring points
            GetNeighbourCost(parent_g_cost, current_position);
            if(!neighbour_g_cost_list_.empty()){
                // if the neigbour g cost is not empty, then update gcost for the openlist
                std::for_each(neighbour_g_cost_list_.begin(),neighbour_g_cost_list_.end(),
                    [&current_position,&current_signature,&target_location,this](const neighbour_list& t){
                    auto it = IsPointInOpenList(t.node_location_);
                    if(it!=open_list_.end()){// if this is in the open list
                        if(t.g_cost_<it->g_cost_){// if the neighour cost is lower than the recorded cost
                            // update cost
                            it->g_cost_ = t.g_cost_;
                            it->h_cost_ = GetHeuristicCost(t.node_location_.x_, t.node_location_.y_, target_location.x_, target_location.y_);
                            it->f_cost_ = it->g_cost_ + it->h_cost_;
                            it->parent_location_ = current_position;
                            it->parent_signature_ = current_signature;
                        }
                    }else{
                        // if this is not in the open list, add this point inf the openlist
                        node new_node;
                        new_node.node_location_ = t.node_location_;
                        new_node.g_cost_ = t.g_cost_;
                        new_node.h_cost_ = GetHeuristicCost(t.node_location_.x_, t.node_location_.y_, target_location.x_, target_location.y_);
                        new_node.f_cost_ = new_node.g_cost_ + new_node.h_cost_;
                        new_node.node_signature_ = MapSignature(map_cols, t.node_location_);
                        new_node.parent_location_ = current_position;
                        new_node.parent_signature_ = current_signature;
                        open_list_.push_back(new_node);
                    }
                });
            } 
        }

        return solution_found;
    }

    void  AStarPlanner::DispOpenlist(){
        std::cout<<"------------------------ openlist --------------------------- \n";
        std::for_each(open_list_.begin(),open_list_.end(),[](const node& t){
            std::cout<<"position X: "<<t.node_location_.x_<<", Y: "<<t.node_location_.y_<<", location sig: "<<t.node_signature_<<", parent X:"
            << t.parent_location_.x_<< ", Y: "<<t.parent_location_.y_<<", parent sig: "<<t.parent_signature_<<", g cost: "<<t.g_cost_
            <<", h cost: "<<t.h_cost_<<", f cost: "<<t.f_cost_<<"\n";
        });
    }
    
    void AStarPlanner::DispCloselist(){
        std::cout<<"------------------------ closelist --------------------------- \n";
        std::for_each(close_list_.begin(),close_list_.end(),[](const std::pair<int,node>& t){
            std::cout<<"position X: "<<t.second.node_location_.x_<<", Y: "<<t.second.node_location_.y_<<", location sig: "<<t.second.node_signature_<<", parent X:"
            << t.second.parent_location_.x_<< ", Y: "<<t.second.parent_location_.y_<<", parent sig: "<<t.second.parent_signature_<<", g cost: "<<t.second.g_cost_
            <<", h cost: "<<t.second.h_cost_<<", f cost: "<<t.second.f_cost_<<"\n";
        });
    }

    void AStarPlanner::DispNeighbourCost(){
        std::cout<<"------------------------ neighour list --------------------------- \n";
        std::for_each( neighbour_g_cost_list_.begin(), neighbour_g_cost_list_.end(),[](const neighbour_list& t){
            std::cout<<"position X: "<<t.node_location_.x_<<", Y: "<<t.node_location_.y_<<", g cost: "<<t.g_cost_<<"\n";
        });
    }

    void AStarPlanner::DisplayMap(){
        std::cout<<"------------------------ map --------------------------- \n";
        for(int i = 0;i<map_rows;i++){
            for(int j= 0;j<map_cols;j++){
                if(map_(i,j)==-1){
                    std::cout<<"+";
                }else if (map_(i,j)==0){
                    std::cout<<"O";
                }else{
                    std::cout<<"=";
                }
            }
            std::cout<<"\n";
        }
    }

}