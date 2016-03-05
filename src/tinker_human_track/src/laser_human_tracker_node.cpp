#include <ros/ros.h>
#include <tinker_human_track/laser_human_tracker.h>
#include <string>

using namespace tinker::navigation;
using std::string;


int main(int argc, char * argv[]) {
    ros::init(argc, argv, "tinker_laser_human_tracker");
    ros::NodeHandle private_nh("~/");
    ros::NodeHandle nh("/");
    string laser_topic_name;
    private_nh.param("laser_topic_name", laser_topic_name, string("scan"));
    LaserHumanTracker laser_human_tracker;
    ros::Subscriber sub = nh.subscribe(laser_topic_name, 10, &LaserHumanTracker::LaserDataHandler, &laser_human_tracker);
    ROS_INFO("laser tracker initialized");
    ros::spin();
    return 0;
}

