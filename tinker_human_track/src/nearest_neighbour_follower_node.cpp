#include "tinker_human_track/nearest_neighbour_follower.h"
#include <ros/ros.h>

using std::string;
using namespace tinker::navigation;

int main(int argc, char * argv[]) {
    ros::init(argc, argv, "tinker_nearest_neighbour_follower");
    ros::NodeHandle private_nh("~/");
    ros::NodeHandle nh("/");
    string people_topic_name;
    private_nh.param("people_topic_name", people_topic_name, string("tinker_laser_human_tracker/laser_found_people"));
    double follow_rate;
    private_nh.param("follow_rate", follow_rate, 1.);
    NearestNeighbourFollower follower;
    ros::Subscriber sub = nh.subscribe(people_topic_name, 10,
            &NearestNeighbourFollower::FoundPeopleHandler,
            &follower);
    ros::Timer timer = private_nh.createTimer(ros::Duration(1/follow_rate), 
            &NearestNeighbourFollower::TimerHandler,
            &follower);
    ros::spin();
    return 0;
}
