// #include "trajectory_publisher/trajectory_publisher.h"
#include "trajectory_publisher.h"

namespace trajectory_publisher
{
    trajectory_publisher::trajectory_publisher(ros::NodeHandle &nh) : nh(nh)
    {
        nh.param<double>("speed", speed_, 1.0);
        nh.param<double>("sampling_frequency", sampling_frequency_, 20.0);
        nh.param<double>("radius", radius_, 1.0);

        omega_ = speed_ * radius_;

        // initialize_trajectory_list(trajectory_list_);

        // Publisher
        trajectory_nwu_pub_ = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/uav/trajectory/nwu", 1);
        trajectory_point_nwu_pub_ = nh.advertise<quadrotor_msgs::TrajectoryPoint>("/uav/trajectory_point/nwu", 1);
        ref_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/uav/ref_pose/nwu", 1);

        // Timer
        trajectory_pub_timer_ = nh.createTimer(ros::Duration(1.0/sampling_frequency_), &trajectory_publisher::trajectory_pub_timer_cb, this, false, false);

        double angle_increment = omega_/sampling_frequency_;
        for (double angle = 0; angle < 2*3.14; angle+= angle_increment)
        {
            std::cout << "adding " << angle << std::endl;
            Eigen::Vector3d position, velocity, acceleration;
            const double cos_phi = cos(angle);
            const double sin_phi = sin(angle);
            position = radius_ * Eigen::Vector3d(sin_phi, -cos_phi, 0.0) + centre_;
            velocity = radius_ * omega_ * Eigen::Vector3d(cos_phi, sin_phi, 0.0) + centre_;
            acceleration = radius_ * pow(omega_, 2.0) * Eigen::Vector3d(cos_phi, sin_phi, 0.0) + centre_;

            quadrotor_msgs::TrajectoryPoint point;
            tf::vectorEigenToMsg(position, point.position);
            tf::vectorEigenToMsg(velocity, point.velocity);
            tf::vectorEigenToMsg(acceleration, point.acceleration);

            trajectory_point_list_.push_back(point);
        }

        std::cout << "initialized" << std::endl;
        // std::cout << trajectory_point_list_.size() << std::endl;

        counter_ = 0;

        trajectory_pub_timer_.start();
        
    }

    void trajectory_publisher::trajectory_pub_timer_cb(const ros::TimerEvent &)
    {
        // std::cout << "hahahaha" << std::endl;
        if(counter_ == trajectory_point_list_.size()) counter_ = 0;

        quadrotor_msgs::TrajectoryPoint msg;
        msg = trajectory_point_list_[counter_];

        geometry_msgs::PoseStamped ref_pose;
        ref_pose.header.stamp = ros::Time::now();
        ref_pose.header.frame_id = "map";
        ref_pose.pose.position.x = msg.position.x;
        ref_pose.pose.position.y = msg.position.y;
        ref_pose.pose.position.z = msg.position.z;
        ref_pose.pose.orientation.w = 1.0;
        ref_pose.pose.orientation.x = 0.0;
        ref_pose.pose.orientation.y = 0.0;
        ref_pose.pose.orientation.z = 0.0;

        // tf::vectorEigenToMsg(Eigen::Vector3d(1.1, 2.2, 3.3), msg.velocity);
        trajectory_point_nwu_pub_.publish(msg);
        ref_pose_pub_.publish(ref_pose);

        // std::cout << trajectory_point_list_.size() << std::endl;

        counter_++;
    }

    // void initialize_trajectory_list(std::vector<trajectory_msgs::JointTrajectoryPoint> & trajectory_list)
    // {
    // }

} // namespace trajectory_publisher