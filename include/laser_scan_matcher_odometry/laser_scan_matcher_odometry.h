//
// Created by christian on 14.12.18.
//

#ifndef LASER_SCAN_MATCHER_ODOMETRY_LASER_SCAN_MATCHER_ODOMETRY_H
#define LASER_SCAN_MATCHER_ODOMETRY_LASER_SCAN_MATCHER_ODOMETRY_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>

#include <csm/csm_all.h>  // csm defines min and max, but Eigen complains
#undef min
#undef max

class LaserScanMatcherOdometry {

private:

    typedef pcl::PointXYZ           PointT;
    typedef pcl::PointCloud<PointT> PointCloudT;

    ros::NodeHandle nh_, nh_private_;

    bool initialized_, received_imu_, received_odom_;

    tf::Transform w2b_;

    double v_x_, v_y_, v_a_; // velocity estimated by the alpha-beta tracker

    double vel_x_, vel_y_, vel_a_;  // simple velocity estimates
    double last_vel_x_, last_vel_y_, last_vel_a_;


    bool use_cloud_input_, use_imu_, use_odom_, publish_pose_;

    // subscriber
    ros::Subscriber scan_subscriber_, cloud_subscriber_, odom_subscriber_, imu_subscriber_, vel_subscriber_;

    // publisher
    ros::Publisher pose_publisher_;

    // Tranformations
    tf::TransformBroadcaster tf_broadcaster_;
    tf::TransformListener    tf_listener_;
    tf::Transform base_to_laser_; // static, cached
    tf::Transform laser_to_base_; // static, cached, calculated from base_to_laser_

    // lsm odom
    bool publish_lsm_odom;
    ros::Publisher lsm_odom_publisher_;

    std::string base_frame_, fixed_frame_;
    ros::Time last_icp_time_;
    double cloud_range_min_, cloud_range_max_;

    bool use_alpha_beta_;
    bool publish_tf_;

    double alpha_, beta_;

    double latest_imu_yaw_, last_imu_yaw_;

    nav_msgs::Odometry latest_odom_, last_odom_;


    nav_msgs::Odometry odom;

    std::vector<double> a_cos_;
    std::vector<double> a_sin_;

    // csm
    sm_params input_;
    sm_result output_;
    LDP prev_ldp_scan_;


    void initParams();

    void cloudCallback (const PointCloudT::ConstPtr& cloud);
    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    void imuCallback (const sensor_msgs::ImuPtr& imu_msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg);

    void processScan(LDP& curr_ldp_scan, const ros::Time& time);
    void laserScanToLDP(const sensor_msgs::LaserScan::ConstPtr& scan_msg, LDP& ldp);
    void PointCloudToLDP(const PointCloudT::ConstPtr& cloud, LDP& ldp);

    void createCache (const sensor_msgs::LaserScan::ConstPtr& scan_msg);
    bool getBaseToLaserTf (const std::string& frame_id);

    void getPrediction(double& pr_ch_x, double& pr_ch_y, double& pr_ch_a, double dt);

    double getYawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
    double getYawFromQuaternion(const tf::Quaternion& quaternion);
    void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);

public:
    LaserScanMatcherOdometry(ros::NodeHandle nh, ros::NodeHandle nh_private);
    ~LaserScanMatcherOdometry();

};

#endif //LASER_SCAN_MATCHER_ODOMETRY_LASER_SCAN_MATCHER_ODOMETRY_H
