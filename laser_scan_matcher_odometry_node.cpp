//
// Created by christian on 14.12.18.
//

#include "laser_scan_matcher_odometry.h"


int main(int argc, char** argv)
{
    ros::init(argc, argv, "LaserScanMatcherOdometry");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    LaserScanMatcherOdometry laser_scan_matcher_odometry(nh, nh_private);
    ros::spin();
    return 0;
}
