#include <iostream>
#include <ros/ros.h>
// #include "trajectory_publisher/trajectory_publisher.h"
#include "trajectory_publisher.h"


int main (int argc, char** argv) 
{        
    ros::init (argc, argv, "trajectory_publisher");
    ros::NodeHandle nh("~");

    trajectory_publisher::trajectory_publisher trajectory_publisher = trajectory_publisher::trajectory_publisher(nh);

    // ros::MultiThreadedSpinner spinner(2);
    // spinner.spin();
    ros::spin();
    return 0;

}