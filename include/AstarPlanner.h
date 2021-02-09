/*
A start planner
Author: Longhao Qian


*/ 
#pragma once
#include <Eigen/Eigen>
#include <vector>
#include <unordered_map>
#include <memory>
#include <algorithm>
#include <iostream>
#include <math.h>
namespace astar{
    // map: A x B matirx wth 0 and 1
    // feasible locations are represented by 0 , obstacles are represented by 1

    struct location{
        /*location(int x, int y){
            x_ = x;
            y_ = y;
        }*/
        int x_{0};
        int y_{0};
    };

    int MapSignature(int map_size_x, const location& point ){
        return  map_size_x * point.y_ + point.x_;
    }

    struct node {
        location node_location_;
        location parent_location_;
        int node_signature_{0};
        int parent_signature_{0};
        double h_cost_{0.0};
        double g_cost_{0.0};
        double f_cost_{0.0};
        void UpdateSignature(int map_size_x){
            node_signature_ = MapSignature(map_size_x, node_location_);
            parent_signature_ = MapSignature(map_size_x, parent_location_);
        }
    };

    struct neighbour_list{
        double g_cost_{0.0};
        location node_location_;
    };


    typedef std::vector<node> node_list;
    typedef std::vector<location> path_list;
    typedef std::vector<neighbour_list> neigh_list;
    
    class AStarPlanner{
        public:
        // constructor to load the map
        AStarPlanner(const Eigen::MatrixXi map);
        bool GetPath(path_list& path, const location& start_location, const location& target_location, bool allow_diagonal);
        void DispOpenlist();
        void DispCloselist();
        void DispNeighbourCost();
        void DisplayMap();
        ~AStarPlanner();
        private:
        // 
        void GetLowestFcostEntry();
        // get the Heuristic
        double GetHeuristicCost(int current_x, int current_y, int target_x, int target_y);
        // compare whether the location is the same
        bool CompareLocation(int location1_x, int location1_y, int location2_x, int location2_y);
        bool CompareLocation(const location& p1, const location& p2);
        // compute the cost for all neighbour costs of a given point and put them into neighbour_g_cost_list_
        void GetNeighbourCost(double parent_g_cost, const location& current_position);
        // determine whether a location is in the open list
        std::vector<node>::iterator IsPointInOpenList(const location& point);
        // determine whether a location is in the closed list
        std::unordered_map<int, node>::iterator IsPointInCloseList(int point);
        // reconstruct the path if a path is found
        void ReconstructPath(path_list& path, const location& start_location, const location& end_location);
        // Manhatten distance
        double ManhattDist(int current_x, int current_y, int target_x, int target_y);
        // Euler distance
        double EulerDist(int current_x, int current_y, int target_x, int target_y);
        // the map:
        Eigen::MatrixXi map_;
        // the computation option:
        bool allow_diagonal_{false};
        bool ok_to_run_{false};
        int map_cols{0};
        int map_rows{0};
        // the open list and close list of the algorithm
        node_list open_list_;// use vector as the open list for insert and remove.
        std::unordered_map<int, node>  close_list_;// use unordered_map for efficient search and insertion
        node_list::iterator open_list_it_;
        std::unordered_map<int, node>::iterator close_list_it_;
        // same temp variables
        neigh_list neighbour_g_cost_list_;
    };

}