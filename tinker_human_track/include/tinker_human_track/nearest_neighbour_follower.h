#ifndef __TINKER_NEAREST_NEIGHBOUR_FOLLOWER_H__
#define __TINKER_NEAREST_NEIGHBOUR_FOLLOWER_H__

#include <tf/transform_listener.h>
#include <people_msgs/People.h>
#include <ros/ros.h>
#include "tinker_human_track/utilities.h"
#include <string>

namespace tinker {
namespace navigation {

typedef people_msgs::People::ConstPtr CPeoplePtr;

enum TrackState {
    IDLE, TRACKING, LOST
};

class NearestNeighbourFollower {
public:
    NearestNeighbourFollower();
    void FoundPeopleHandler(const CPeoplePtr & people);
    void TimerHandler(const ros::TimerEvent & event);
private:
    void HandlePeopleOnIdle(const CPeoplePtr & people);
    void HandlePeopleOnTracking(const CPeoplePtr & people);
    void HandlePeopleOnLost(const CPeoplePtr & people);
    void HandleTimerOnIdle();
    void HandleTimerOnTracking();
    void HandleTimerOnLost();

    TrackState track_state_;
    Point track_point_;
    ros::Time track_point_time_;
    int pose_seq_;
    int lost_count_;
    int max_lost_count_;
    double max_move_distance_;
    double track_distance_;
    std::string frame_id_;
    tf::TransformListener tf_listener_;
    ros::Publisher target_pub_;
};

}
}

#endif
