#include "tinker_human_track/nearest_neighbour_follower.h"
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <climits>
#include <boost/foreach.hpp>
#include <math.h>

using std::string;

namespace tinker {
namespace navigation {

NearestNeighbourFollower::NearestNeighbourFollower()
    :track_state_(IDLE), pose_seq_(0), lost_count_(0) {
    ros::NodeHandle private_nh("~/");
    ros::NodeHandle nh;
    private_nh.param("max_lost_count", max_lost_count_, 120);
    private_nh.param("max_move_distance", max_move_distance_, 0.3);
    private_nh.param("track_distance", track_distance_, 2.0);
    string target_topic_name;
    private_nh.param("target_topic_name", target_topic_name, 
            string("move_base_simple/goal"));
    target_pub_ = nh.advertise<geometry_msgs::PoseStamped>(target_topic_name, 1);
} 

void NearestNeighbourFollower::FoundPeopleHandler(const CPeoplePtr & people) {
    switch (track_state_){
    case IDLE:
        HandlePeopleOnIdle(people);
        break;
    case TRACKING:
        HandlePeopleOnTracking(people);
        break;
    case LOST:
        HandlePeopleOnLost(people);
        break;
    default:
        break;
    }
}

void NearestNeighbourFollower::TimerHandler(const ros::TimerEvent & event) {
    switch (track_state_){
    case IDLE:
        HandleTimerOnIdle();
        break;
    case TRACKING:
        HandleTimerOnTracking();
        break;
    case LOST:
        HandleTimerOnLost();
        break;
    default:
        break;
    }

}

void NearestNeighbourFollower::HandlePeopleOnIdle(const CPeoplePtr & people) {
    if(people->people.size() == 0) return;
    track_point_.x = people->people[0].position.x;
    track_point_.y = people->people[0].position.y;
    track_point_time_ = people->header.stamp;
    frame_id_ = people->header.frame_id;
    lost_count_ = 0;
    track_state_ = TRACKING;
}

void NearestNeighbourFollower::HandlePeopleOnTracking(const CPeoplePtr & people) {
    Point new_tracking_point;
    double min_distance = INT_MAX;
    BOOST_FOREACH(const people_msgs::Person & person, people->people) {
        Point person_point = {
            .x = person.position.x,
            .y = person.position.y
        };
        double person_distance = Distance(person_point, track_point_);
        if(person_distance < min_distance) {
            min_distance = person_distance;
            new_tracking_point = person_point;
        }
    }
    if(min_distance < max_move_distance_) {
        track_point_ = new_tracking_point;
        track_point_time_ = people->header.stamp;
        lost_count_ = 0;
    }
    else {
        lost_count_++;
        if(lost_count_ > max_lost_count_) {
            track_state_ = LOST;
        }
    }
}

void NearestNeighbourFollower::HandlePeopleOnLost(const CPeoplePtr & people) {
}

void NearestNeighbourFollower::HandleTimerOnIdle() {
}

void NearestNeighbourFollower::HandleTimerOnTracking() {
    tf::StampedTransform transform;
    try {
        tf_listener_.lookupTransform(frame_id_, frame_id_, track_point_time_, transform);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s", ex.what());
        return;
    }
    tf::Vector3 old_track_target(track_point_.x, track_point_.y, 0);
    tf::Vector3 now_track_target = transform * old_track_target;
    double x = now_track_target.getX();
    double y = now_track_target.getY();
    double distance = sqrt(x*x + y*y);
    if (distance < track_distance_) {
        return;
    }
    double theta = acos(x / distance);
    theta = x < 0 ? -theta : theta;
    double move_distance = distance - track_distance_;
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = move_distance * cos(theta);
    target_pose.pose.position.y = move_distance * sin(theta);
    tf::Quaternion q;
    q.setRPY(0, 0, theta);
    target_pose.pose.orientation.x = q.getX();
    target_pose.pose.orientation.y = q.getY();
    target_pose.pose.orientation.z = q.getZ();
    target_pose.pose.orientation.w = q.getW();
    target_pub_.publish(target_pose);

}

void NearestNeighbourFollower::HandleTimerOnLost() {
}


}
}
