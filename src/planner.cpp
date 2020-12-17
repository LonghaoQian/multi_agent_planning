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
// ros libs
#include <ros/ros.h>
#include <Eigen/Eigen>
// package libs
#include <multi_agent_planner/agentpos.h>
#include <multi_agent_planner/pathinfo.h>

enum TransitionDirection{
    TRANSITION_UP = 0,
    TRANSITION_DOWN,
    TRANSITION_LEFT,
    TRANSITION_RIGHT,
};

struct nodeinfo {
    float NodeCost{0.0};
    Eigen::Vector4f TransitionCost;
    Eigen::Matrix<bool, 4, 1> Accessible;
};

// subscriber function for getting the agent position.

void GetAgentPosition(const multi_agent_planner::agentpos::ConstPtr& msg, int drone_ID, Eigen::Matrix<int,Eigen::Dynamic, 2>* PosPtr){
   (*PosPtr)(drone_ID,0) =  msg->pos_x;
   (*PosPtr)(drone_ID,1) =  msg->pos_y;
}



bool ResponseToGetplanCall(multi_agent_planner::pathinfo::Request& req, multi_agent_planner::pathinfo::Response& res, const Eigen::Matrix<nodeinfo,11,11>* grid){

    // get the target position

    // get the ID
    //isperformAction = req.perform_action;



    // calculate the index list
   /* *if(DroneGeoFenceCheck()&&req.perform_action){
        isperformAction = req.perform_action;
    }else{
        isperformAction = false;
    }
    res.status_ok = isperformAction;
    res.trajectory_type = type;
    return true;    
    
    */

    // for any remaining 

    return true;  

}

 /*void DisplayResults(const Eigen::Vector3f& payloadposition, 
                          const Eigen::Vector3f& payloadvelocity, 
                          const Eigen::Vector3f& payloadpositionbody,
                          const Eigen::Vector3f& payloadvelocitybody,
                          const float& sampletime){

    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);
    cout<<setprecision(2);// number of decimals 
   
    std::cout<< "----------------------- Payload Info ------------------ \n";
    std::cout<< "Payload inertial position X: " << payloadposition(math_utils::Vector_X) << " (m), Y: " << payloadposition(math_utils::Vector_Y)
    <<" (m), Z: "<< payloadposition(math_utils::Vector_Z) << " (m). \n";
     std::cout<< "Payload inertial velocity X: " << payloadvelocity(math_utils::Vector_X) * 100.0 << " (cm/s), Y: " << payloadvelocity(math_utils::Vector_Y)* 100.0
    <<" (cm/s), Z: "<< payloadvelocity(math_utils::Vector_Z)* 100.0 << " (cm/s). \n";   
    std::cout<< "The sample time is: " << sampletime << " (s) \n";
    std::cout<< "Payload body position X: " << payloadpositionbody(math_utils::Vector_X) << " (m), Y: " << payloadpositionbody(math_utils::Vector_Y)
    <<" (m), Z: "<< payloadpositionbody(math_utils::Vector_Z) << " (m). \n";
     std::cout<< "Payload body velocity X: " << payloadvelocitybody(math_utils::Vector_X)* 100.0<< " (cm/s), Y: " << payloadvelocitybody(math_utils::Vector_Y)* 100.0
    <<" (cm/s), Z: "<< payloadvelocitybody(math_utils::Vector_Z)* 100.0 << " (cm/s). \n";   
    std::cout<< "The sample time is: " << sampletime << " (s) \n";
    std::cout<< "------------------End of Payload Info ------------------ \n";
}*/

int main(int argc, char **argv){

    ros::init(argc, argv, "planner");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);
    namespace arg = std::placeholders;
    // define the grid
    Eigen::Matrix<nodeinfo,11,11> Grid;

    // initialize the grid
    //for ()


    // get the number of agents
    int NumberOfAgents = 0;
    nh.param<int>("NumberofAgents", NumberOfAgents, 0);
    // initialize the storage for agent positions
    Eigen::Matrix<int,Eigen::Dynamic, 2> AgentPosition;
    AgentPosition.resize(NumberOfAgents,2);
    // initialize the position 
    AgentPosition.setZero();
    // get the address of the agent position
    Eigen::Matrix<int,Eigen::Dynamic, 2>* PosPtr = & AgentPosition; 
    // 
    ros::ServiceServer serverAction =  
    nh.advertiseService<multi_agent_planner::pathinfo::Request, multi_agent_planner::pathinfo::Response>("/get_plan",                                                     
        std::bind(&ResponseToGetplanCall, arg::_1, arg::_2,&Grid));// bind the function to the server

    while(ros::ok()) {
        // display results
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}