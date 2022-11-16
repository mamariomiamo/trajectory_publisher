// #include "trajectory_publisher/trajectory_publisher.h"
#include "trajectory_publisher.h"

namespace trajectory_publisher
{
    trajectory_publisher::trajectory_publisher(ros::NodeHandle &nh) : nh(nh)
    {
        nh.param<double>("speed", speed_, 1.0);
        nh.param<double>("sampling_frequency", sampling_frequency_, 20.0);
        nh.param<double>("radius", radius_, 1.0);
        nh.param<bool>("trajectory_pub_timer_auto_start", trajectory_pub_timer_auto_start_, true);
        nh.param<bool>("publish_pva", publish_pva, true);

        omega_ = speed_ * radius_;

        // initialize_trajectory_list(trajectory_list_);

        // Publisher
        trajectory_nwu_pub_ = nh.advertise<trajectory_msgs::JointTrajectoryPoint>("/uav/trajectory/nwu", 1);
        trajectory_point_nwu_pub_ = nh.advertise<quadrotor_msgs::TrajectoryPoint>("/uav/trajectory_point/nwu", 1);
        ref_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/uav/ref_pose/nwu", 1);

        // Subscriber
        if (!trajectory_pub_timer_auto_start_)
        {
            user_cmd_sub_ = nh.subscribe<std_msgs::Byte>("/user_cmd", 1, &trajectory_publisher::cmd_cb, this);
        }

        // Timer
        trajectory_pub_timer_ = nh.createTimer(ros::Duration(1.0 / sampling_frequency_), &trajectory_publisher::trajectory_pub_timer_cb, this, false, false);

        double angle_increment = omega_ / sampling_frequency_;
        for (double angle = 0; angle < 2 * 3.14; angle += angle_increment)
        {
            std::cout << "adding " << angle << std::endl;
            Eigen::Vector3d position, velocity, acceleration;
            const double cos_phi = cos(angle);
            const double sin_phi = sin(angle);
            position = radius_ * Eigen::Vector3d(cos_phi, sin_phi, 0.0) + centre_;
            velocity = radius_ * omega_ * Eigen::Vector3d(-sin_phi, cos_phi, 0.0);
            acceleration = radius_ * pow(omega_, 2.0) * Eigen::Vector3d(-cos_phi, -sin_phi, 0.0);

            quadrotor_msgs::TrajectoryPoint point;
            tf::vectorEigenToMsg(position, point.position);
            tf::vectorEigenToMsg(velocity, point.velocity);
            tf::vectorEigenToMsg(acceleration, point.acceleration);

            trajectory_point_list_.push_back(point);
        }

        std::cout << "initialized" << std::endl;
        // std::cout << trajectory_point_list_.size() << std::endl;

        counter_ = 0;
        if (trajectory_pub_timer_auto_start_)
        {
            trajectory_pub_timer_.start();
        }
    }

    void trajectory_publisher::trajectory_pub_timer_cb(const ros::TimerEvent &)
    {
        // std::cout << "hahahaha" << std::endl;
        if (counter_ == trajectory_point_list_.size())
            counter_ = 0;

        quadrotor_msgs::TrajectoryPoint msg;
        msg = trajectory_point_list_[counter_];
        msg.heading = 1.57;

        Eigen::Vector3d vector_acc;
        vector_acc << msg.acceleration.x, msg.acceleration.y, msg.acceleration.z;

        Eigen::Vector4d quat;
        Eigen::Vector3d zb_des, yb_des, xb_des, proj_xb_des;
        Eigen::Matrix3d rotmat;
        double yaw = msg.heading;

        proj_xb_des << std::cos(yaw), std::sin(yaw), 0.0;

        zb_des = vector_acc / vector_acc.norm();
        yb_des = zb_des.cross(proj_xb_des) / (zb_des.cross(proj_xb_des)).norm();
        xb_des = yb_des.cross(zb_des) / (yb_des.cross(zb_des)).norm();

        rotmat << xb_des(0), yb_des(0), zb_des(0), xb_des(1), yb_des(1), zb_des(1), xb_des(2), yb_des(2), zb_des(2);
        //   quat = rotmat;
        quat = rot2Quaternion(rotmat);

        geometry_msgs::PoseStamped ref_pose;
        ref_pose.header.stamp = ros::Time::now();
        ref_pose.header.frame_id = "map";
        ref_pose.pose.position.x = msg.position.x;
        ref_pose.pose.position.y = msg.position.y;
        ref_pose.pose.position.z = msg.position.z;
        ref_pose.pose.orientation.w = quat(0);
        ref_pose.pose.orientation.x = quat(1);
        ref_pose.pose.orientation.y = quat(2);
        ref_pose.pose.orientation.z = quat(3);

        // tf::vectorEigenToMsg(Eigen::Vector3d(1.1, 2.2, 3.3), msg.velocity);
        trajectory_point_nwu_pub_.publish(msg);
        ref_pose_pub_.publish(ref_pose);

        // std::cout << trajectory_point_list_.size() << std::endl;

        counter_++;
    }

    void trajectory_publisher::cmd_cb(const std_msgs::Byte::ConstPtr &msg)
    {
        int cmd = msg->data;
        if (cmd == 2)
        {
            trajectory_pub_timer_.start();
        }
    }

    Eigen::Vector4d trajectory_publisher::rot2Quaternion(const Eigen::Matrix3d &R)
    {
        Eigen::Vector4d quat;
        double tr = R.trace();
        if (tr > 0.0)
        {
            double S = sqrt(tr + 1.0) * 2.0; // S=4*qw
            quat(0) = 0.25 * S;
            quat(1) = (R(2, 1) - R(1, 2)) / S;
            quat(2) = (R(0, 2) - R(2, 0)) / S;
            quat(3) = (R(1, 0) - R(0, 1)) / S;
        }
        else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2)))
        {
            double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0; // S=4*qx
            quat(0) = (R(2, 1) - R(1, 2)) / S;
            quat(1) = 0.25 * S;
            quat(2) = (R(0, 1) + R(1, 0)) / S;
            quat(3) = (R(0, 2) + R(2, 0)) / S;
        }
        else if (R(1, 1) > R(2, 2))
        {
            double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0; // S=4*qy
            quat(0) = (R(0, 2) - R(2, 0)) / S;
            quat(1) = (R(0, 1) + R(1, 0)) / S;
            quat(2) = 0.25 * S;
            quat(3) = (R(1, 2) + R(2, 1)) / S;
        }
        else
        {
            double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0; // S=4*qz
            quat(0) = (R(1, 0) - R(0, 1)) / S;
            quat(1) = (R(0, 2) + R(2, 0)) / S;
            quat(2) = (R(1, 2) + R(2, 1)) / S;
            quat(3) = 0.25 * S;
        }
        return quat;
    }

    // void initialize_trajectory_list(std::vector<trajectory_msgs::JointTrajectoryPoint> & trajectory_list)
    // {
    // }

} // namespace trajectory_publisher