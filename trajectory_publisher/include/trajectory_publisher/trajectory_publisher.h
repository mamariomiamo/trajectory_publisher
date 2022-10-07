#pragma once

#include <Eigen/Dense>
#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include "quadrotor_msgs/TrajectoryPoint.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Byte.h>
#include <iostream>

namespace trajectory_publisher
{
    class trajectory_publisher
    {
    private:
        ros::NodeHandle nh;
        // ros::Subscriber
        ros::Publisher trajectory_nwu_pub_, trajectory_point_nwu_pub_, ref_pose_pub_;
        ros::Subscriber user_cmd_sub_;
        ros::Timer trajectory_pub_timer_;

        double speed_, radius_, omega_;
        double sampling_frequency_;
        std::vector<trajectory_msgs::JointTrajectoryPoint> trajectory_list_;
        std::vector<quadrotor_msgs::TrajectoryPoint> trajectory_point_list_;
        int counter_;
        

        Eigen::Vector3d centre_ = Eigen::Vector3d(0.0, 0.0, 2.0);

        bool trajectory_pub_timer_auto_start_;
        bool publish_pva;

    public:
    
        trajectory_publisher(ros::NodeHandle &nh);
        ~trajectory_publisher() = default;

        void trajectory_pub_timer_cb(const ros::TimerEvent &);
        void cmd_cb(const std_msgs::Byte::ConstPtr &msg);
        // void initialize_trajectory_list(std::vector<trajectory_msgs::JointTrajectoryPoint> & trajectory_list);
    };
} // namespace trajectory_publisher