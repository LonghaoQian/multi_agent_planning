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



#include <ros/ros.h>
#include <Eigen/Eigen>
#include <iostream>
#include <string>


int main(int argc, char **argv){

    ros::init(argc, argv, "agent");
    ros::NodeHandle nh("~");
    ros::Rate rate(50.0);
    /*
    ros::ServiceServer serverAction =  
    main_handle.advertiseService<qt_ground_station::GeneralInfo::Request, qt_ground_station::GeneralInfo::Response>
    ("/get_plan", &pos_controller_TIE::ResponseToActionCall);
    */
    while(ros::ok()) {
        // display results
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}