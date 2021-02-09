/* 
---------------------------------
The planner node 
----------------------------------
Author: Longhao Qian 
Data Dec 16 2020
----------------------------------
The a ros service server is added to provide the path: topic: "/get_plan"


----------------------------------
*/
// C++ libs
#include <iostream>
#include <string>
#include <functional>
#include <vector>
#include <algorithm> 
// ros libs
#include <ros/ros.h>
#include <Eigen/Eigen>
// package libs
#include <multi_agent_planner/agentpos.h>
#include <multi_agent_planner/pathinfo.h>
#include <AstarPlanner.h>

enum TransitionDirection{
    TRANSITION_UP = 0,
    TRANSITION_DOWN,
    TRANSITION_LEFT,
    TRANSITION_RIGHT,
};

enum VectorIndex{
    Vector_X = 0,
    Vector_Y,
};

struct nodeinfo {
    float NodeCost{0.0};
    Eigen::Vector4f TransitionCost;
    Eigen::Matrix<bool, 4, 1> Accessible;
};

struct agentmove {
    int x{0};
    int y{0};
};

// subscriber function for getting the agent position.

void GetAgentPosition(const multi_agent_planner::agentpos::ConstPtr& msg, 
                      int ID, Eigen::Matrix<int,Eigen::Dynamic, 2>* PosPtr){
   (*PosPtr)(ID,0) =  msg->pos_x;
   (*PosPtr)(ID,1) =  msg->pos_y;
}

int HeuristicPlanning (int agentID,
                        int x_target, 
                        int y_target, 
                        int x_pos,
                        int y_pos,
                        Eigen::Matrix<agentmove, 20, Eigen::Dynamic>* path,
                        const Eigen::Matrix<nodeinfo,11,11>* grid){

    /*
    Since there is no obstacle in the grid and the cost is homegenous across the entire map, a heuristic method is used 
    */
    int Xdiff = x_target - x_pos;
    int Ydiff = y_target - y_pos;

    int Nx = abs(Xdiff);
    int Ny = abs(Ydiff);

    int step_x = x_pos;
    int step_y = y_pos;

    for(int i = 0;i<20;i++){
        if(i<Nx){
            if(Xdiff>0){// positve direction
                step_x += 1;
            }
            else{//negative direction
                step_x += - 1; 
            }
        }else if(Nx<=i<Ny+Nx){
            if(Ydiff>0){// positve direction
                step_y += 1;
            }
            else{//negative direction
                step_y += -1; 
            }
        }
        (*path)(i,agentID).x = step_x;
        (*path)(i,agentID).y = step_y;
    }

    int NumberOfSteps  = Nx + Ny;

    if(NumberOfSteps== 0 ){
        std::cout << " No movements required. \n";
    }

    for (int i = 0; i< NumberOfSteps ; i++){
        std::cout << " (" << (*path)(i,agentID).x <<", " << (*path)(i,agentID).y <<") -> ";
    }
    
    std::cout<<"Target \n";

    return NumberOfSteps;

    /*------------ ------------------------- */
}


bool ResponseToGetplanCall(multi_agent_planner::pathinfo::Request& req, multi_agent_planner::pathinfo::Response& res, 
                           const Eigen::Matrix<nodeinfo,11,11>* grid,
                           Eigen::Matrix<agentmove, 20, Eigen::Dynamic>* path, 
                           const Eigen::Matrix<int,Eigen::Dynamic, 2>* agentpos){

    // get the target position
    ROS_INFO(" Recieved path request for agent: %d", req.agentID);
    std::cout <<"The initial position is X: " << (*agentpos)(req.agentID, Vector_X) << " Y: " << (*agentpos)(req.agentID,Vector_Y) << "\n";
    std::cout <<"The target  position is X : " << req.x_target << " Y: " << req.y_target << "\n";
    // calculate path. (Here since the map is homegenous, heuristic planning method is used. For more complicated scenarios, methods such as A star can be used.)
    res.NumSteps = HeuristicPlanning (req.agentID, req.x_target,  req.y_target, (*agentpos)(req.agentID, Vector_X),(*agentpos)(req.agentID,Vector_Y), path, grid);
    for(int i = 0; i< res.NumSteps; i++){
       res.x_indexlist[i] =  (*path)(i,req.agentID).x;
       res.y_indexlist[i] =  (*path)(i,req.agentID).y;
    }
    res.isListComplete = true;

    ROS_INFO("Response sent to agent%d", req.agentID);
    return true;  

}

int main(int argc, char **argv){

    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);
    namespace arg = std::placeholders;
    // define the grid
    Eigen::Matrix<nodeinfo,11,11> Grid;
    // initailize the corners of the grid:

    Grid(0,0).NodeCost = 0.0;
    Grid(0,0).TransitionCost.setZero();
    Grid(0,0).TransitionCost(TRANSITION_UP) = 1000.0;
    Grid(0,0).TransitionCost(TRANSITION_DOWN) = 10.0;
    Grid(0,0).TransitionCost(TRANSITION_LEFT) = 1000.0;
    Grid(0,0).TransitionCost(TRANSITION_RIGHT) = 10.0; 

    Grid(0,10).NodeCost = 0.0;
    Grid(0,10).TransitionCost.setZero();
    Grid(0,10).TransitionCost(TRANSITION_UP) = 1000.0;
    Grid(0,10).TransitionCost(TRANSITION_DOWN) = 10.0;
    Grid(0,10).TransitionCost(TRANSITION_LEFT) = 10.0;
    Grid(0,10).TransitionCost(TRANSITION_RIGHT) = 1000.0; 


    Grid(10,0).NodeCost = 0.0;
    Grid(10,0).TransitionCost.setZero();
    Grid(10,0).TransitionCost(TRANSITION_UP) = 10.0;
    Grid(10,0).TransitionCost(TRANSITION_DOWN) = 1000.0;
    Grid(10,0).TransitionCost(TRANSITION_LEFT) = 1000.0;
    Grid(10,0).TransitionCost(TRANSITION_RIGHT) = 10.0; 

    Grid(10,10).NodeCost = 0.0;
    Grid(10,10).TransitionCost.setZero();
    Grid(10,10).TransitionCost(TRANSITION_UP) = 10.0;
    Grid(10,10).TransitionCost(TRANSITION_DOWN) = 1000.0;
    Grid(10,10).TransitionCost(TRANSITION_LEFT) = 10.0;
    Grid(10,10).TransitionCost(TRANSITION_RIGHT) = 1000.0; 

    // initialize the boundary of the grid:


    for (int i = 1; i < 10; i++){
        Grid(0,i).NodeCost = 0.0;
        Grid(0,i).TransitionCost.setZero();
        Grid(0,i).TransitionCost(TRANSITION_UP) = 1000.0;
        Grid(0,i).TransitionCost(TRANSITION_DOWN) = 10.0;
        Grid(0,i).TransitionCost(TRANSITION_LEFT) = 10.0;
        Grid(0,i).TransitionCost(TRANSITION_RIGHT) = 10.0;

        Grid(i,0).NodeCost = 0.0;
        Grid(i,0).TransitionCost.setZero();
        Grid(i,0).TransitionCost(TRANSITION_UP) = 10.0;
        Grid(i,0).TransitionCost(TRANSITION_DOWN) = 10.0;
        Grid(i,0).TransitionCost(TRANSITION_LEFT) = 1000.0;
        Grid(i,0).TransitionCost(TRANSITION_RIGHT) = 10.0;


        Grid(10,i).NodeCost = 0.0;
        Grid(10,i).TransitionCost.setZero();
        Grid(10,i).TransitionCost(TRANSITION_UP) = 10.0;
        Grid(10,i).TransitionCost(TRANSITION_DOWN) = 1000.0;
        Grid(10,i).TransitionCost(TRANSITION_LEFT) = 10.0;
        Grid(10,i).TransitionCost(TRANSITION_RIGHT) = 10.0;

        Grid(i,10).NodeCost = 0.0;
        Grid(i,10).TransitionCost.setZero();
        Grid(i,10).TransitionCost(TRANSITION_UP) = 10.0;
        Grid(i,10).TransitionCost(TRANSITION_DOWN) = 10.0;
        Grid(i,10).TransitionCost(TRANSITION_LEFT) = 10.0;
        Grid(i,10).TransitionCost(TRANSITION_RIGHT) = 1000.0;


        for(int j = 1; j < 10 ; j++){
            Grid(i,j).NodeCost = 0.0;
            Grid(i,j).TransitionCost.setZero();
            Grid(i,j).TransitionCost(TRANSITION_UP)    = 10.0;
            Grid(i,j).TransitionCost(TRANSITION_DOWN)  = 10.0;
            Grid(i,j).TransitionCost(TRANSITION_LEFT)  = 10.0;
            Grid(i,j).TransitionCost(TRANSITION_RIGHT) = 10.0;
        }

    }

    // get the number of agents
    int NumberOfAgents = 0;
    nh.param<int>("NumberofAgents", NumberOfAgents, 0);
    // initialize the storage for agent positions
    Eigen::Matrix<int,Eigen::Dynamic, 2> AgentPosition;
    Eigen::Matrix<agentmove, 20, Eigen::Dynamic> AgentPath;
    AgentPosition.resize(NumberOfAgents,2);
    AgentPath.resize(20, NumberOfAgents);
    // initialize the position 
    AgentPosition.setZero();
    // get the address of the agent position
    Eigen::Matrix<int,Eigen::Dynamic, 2>* PosPtr = & AgentPosition; 
    // 
    ros::ServiceServer PlannerAction =  
    nh.advertiseService<multi_agent_planner::pathinfo::Request, multi_agent_planner::pathinfo::Response>("/get_plan",                                                     
        std::bind(&ResponseToGetplanCall, arg::_1, arg::_2,&Grid,&AgentPath,&AgentPosition));// bind the function to the server
    std::vector<std::unique_ptr<ros::Subscriber>> SubAgentList;

    for (int i = 0; i< NumberOfAgents; i++){
        SubAgentList.emplace_back(new ros::Subscriber);
        std::string agentID = std::to_string(i);
        (*SubAgentList[i]) = nh.subscribe<multi_agent_planner::agentpos>("/agent" + agentID + "/agent_feedback", 50, 
                                                                         std::bind(&GetAgentPosition, arg::_1, i, &AgentPosition));
    }

    Eigen::MatrixXi map(10,10);
    // load map
    map.setZero();
    for(int i = 2;i<10;i++){
        map(1,i)=1;
    }
    astar::AStarPlanner planner0(map);
    astar::path_list path;
    astar::location start;
    astar::location target;

    start.x_ = 1;
    start.y_ = 0;

    target.x_ = 7;
    target.y_ = 5;

    if(planner0.GetPath(path, start, target, false))
    {
        planner0.DispOpenlist();
        planner0.DispCloselist();
        planner0.DispNeighbourCost();
        std::cout<<"the path (end to start) is: \n";
        for(auto& t: path){
            std::cout<<"("<<t.x_<<", "<<t.y_<<"), \n";
        }
        planner0.DisplayMap();                
    }else{
        std::cout<<"path not found! \n";
    }



    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}