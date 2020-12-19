/* 
---------------------------------
The agent node 
----------------------------------
Author: Longhao Qian 
Data Dec 16 2020
----------------------------------
An agent node that requires 


----------------------------------
*/
// C++ headers
#include <iostream>
#include <string>
#include <functional>
// ros headers
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <visualization_msgs/Marker.h>

// package libs
#include <multi_agent_planner/agentpos.h>
#include <multi_agent_planner/agentgoal.h>
#include <multi_agent_planner/pathinfo.h>


bool ResponseToGetplanCall(multi_agent_planner::agentgoal::Request& req,
                           multi_agent_planner::agentgoal::Response& res,
                           int agentID,
                           const multi_agent_planner::agentpos* posPtr,
                           ros::ServiceClient* PlanClientPtr,
                           multi_agent_planner::pathinfo* RequestedPlan,
                           bool* isUpdateGraphics,
                           visualization_msgs::Marker* pointsPtr,
                           visualization_msgs::Marker* linestripPtr){
    // get the goal location from the 
    RequestedPlan->request.x_target = req.x_target;
    RequestedPlan->request.y_target = req.y_target;
    RequestedPlan->request.agentID  = agentID;
    res.isPlanRecieved = PlanClientPtr->call(*RequestedPlan);

    if( res.isPlanRecieved) {// if service recieved, update the graphics
        *isUpdateGraphics = true;
        geometry_msgs::Point p;
        linestripPtr->points.clear();// clear the line buffer
        p.x = (float) posPtr->pos_x -5.0;
        p.y = (float) posPtr->pos_y -5.0;
        p.z = 0.0;
        linestripPtr->points.push_back(p);         
        for (int i = 0; i < RequestedPlan->response.NumSteps; i++){
            p.x = (float)RequestedPlan->response.x_indexlist[i]-5.0;
            p.y = (float)RequestedPlan->response.y_indexlist[i]-5.0;
            p.z = 0.0;
            linestripPtr->points.push_back(p);        
        }
        ROS_INFO("Plan recieved, showing results:");
        if(RequestedPlan->response.NumSteps == 0){
            std::cout << " No movements required. \n";
        }

        for (int i = 0; i< RequestedPlan->response.NumSteps ; i++){
            std::cout << " (" << RequestedPlan->response.x_indexlist[i] <<", " << RequestedPlan->response.y_indexlist[i] <<") -> ";
        }
    
        std::cout<<"Target \n";


    }else{
        ROS_WARN("No reponse from the planner server...");
    }

    return true;
}


int main(int argc, char **argv){

    ros::init(argc, argv, "agent");
    ros::NodeHandle nh("~");
    ros::Rate rate(10.0);
    namespace arg = std::placeholders;
    // the position of the agent
    multi_agent_planner::agentpos Position;
    multi_agent_planner::pathinfo RequestedPlan;
    bool isUpdateGraphics = false;

    visualization_msgs::Marker points, line_strip;
    // define the topics
    std::string TopicFeedback = "/agent";
    std::string TopicUpdateGoal = "/agent";
    //std::string TopicGetPlan = "/agent";

    char* agentID = NULL;
    int agentIDi = 0;
    char agentDefaultID = '0';
    if ( argc > 1) {    // if ID is specified as the second argument 
        ROS_INFO("Agent ID specified as: agent%s", argv[1]);
        agentID = argv[1];
    } else {
        agentID = &agentDefaultID;
        ROS_WARN("NO Agent ID is specified, set the ID to 0!");
    }
    agentIDi = (*agentID) - '0';

    // add uav prefixes to topic strings 
    TopicFeedback.push_back(*agentID);
    TopicUpdateGoal.push_back(*agentID);

    // add topic names to topic strings
    TopicFeedback         += "/agent_feedback";
    TopicUpdateGoal       += "/update_goal";
    //TopicGetPlan          += "/get_plan"//
    std::string TopicGetPlan  = "/get_plan";

    ros::Publisher PubAgentPos  = nh.advertise<multi_agent_planner::agentpos>(TopicFeedback, 50);

    ros::ServiceClient ClientCallPlanner =  nh.serviceClient<multi_agent_planner::pathinfo>(TopicGetPlan);

    ros::ServiceServer AgentSetGoal =  
    nh.advertiseService<multi_agent_planner::agentgoal::Request, multi_agent_planner::agentgoal::Response>(TopicUpdateGoal,                                                     
        std::bind(&ResponseToGetplanCall, arg::_1, arg::_2, 
        agentIDi, 
        &Position,&ClientCallPlanner,
        &RequestedPlan,
        &isUpdateGraphics,
        &points,
        &line_strip));// bind the function to the server

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);

    // load the initial position of the agent

    nh.param<int>("AgentPos_x", Position.pos_x, 0);
    nh.param<int>("AgentPos_y", Position.pos_y, 0);

    std::cout<< "The initial position for agent "<< agentIDi << " is: x = " <<Position.pos_x << ", y = " << Position.pos_y << "\n";

    // initialize the graphics information
    points.header.frame_id = line_strip.header.frame_id  = "/grid_frame";
    points.ns = line_strip.ns = "points_and_lines";
  
    points.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    points.id = agentIDi*2;
    line_strip.id = agentIDi*2+1;

    points.type = visualization_msgs::Marker::POINTS;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.1;

    // assign line color based on the agent ID
    switch (agentIDi){
        case 0: {
            // Points are green
            points.color.g = 1.0f;
            points.color.a = 1.0;

            // Line strip is red
            line_strip.color.r = 1.0;
            line_strip.color.a = 1.0;            
            break;
        }
        case 1: {

            points.color.g = 0.5f;
            points.color.r = 0.5f;
            points.color.a = 1.0;

            line_strip.color.g = 0.5f;
            line_strip.color.b = 0.5f;
            line_strip.color.a = 1.0;                 
            break;
        }
        default: {
            // Points are green
            points.color.g = 1.0f;
            points.color.a = 1.0;

            // Line strip is red
            line_strip.color.r = 1.0;
            line_strip.color.a = 1.0;      
            break;
        }
    }

    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    
    geometry_msgs::Point p;    
    int SendDeleteCounter = 0;
    int SendDrawCounter = 0;

    while(ros::ok()) {
        // display results
        ros::spinOnce();
        // publish the agent location:
        PubAgentPos.publish(Position);
        // update the graphics
        if (isUpdateGraphics) {
            if (SendDeleteCounter<10){
                points.header.stamp = line_strip.header.stamp  = ros::Time::now();
                points.action = line_strip.action  = visualization_msgs::Marker::DELETE;
                marker_pub.publish(points);
                marker_pub.publish(line_strip);
                SendDeleteCounter++;               
            }else{// load path information
                if (SendDrawCounter<10){
                    points.header.stamp = line_strip.header.stamp  = ros::Time::now();
                    points.action = line_strip.action = visualization_msgs::Marker::ADD; // set the flag back to ADD 
                    marker_pub.publish(points);
                    marker_pub.publish(line_strip);
                    ROS_INFO("points published...");
                    SendDrawCounter++;            
                }else{
                    isUpdateGraphics = false;
                }
            }       
        }else{ // if update in off, reset the counter
            SendDeleteCounter = 0;
            SendDrawCounter   = 0;
        }

        rate.sleep();
    }
    return 0;
}