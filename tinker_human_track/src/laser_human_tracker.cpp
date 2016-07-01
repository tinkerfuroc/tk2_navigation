#include <tinker_human_track/laser_human_tracker.h>
#include <ros/ros.h>
#include <people_msgs/Person.h>
#include <sensor_msgs/PointCloud.h>
#include <boost/foreach.hpp>
#include <math.h>

using std::vector;
using std::string;

namespace tinker{
namespace navigation {

LaserHumanTracker::LaserHumanTracker()
    :seq_(0), debug_seq_(0) {
    ros::NodeHandle private_nh("~/");
    private_nh.param("frame_id", frame_id_, string("base_laser_link"));
    private_nh.param("same_segment_distance", same_segment_distance_, 0.05);
    private_nh.param("min_segment_size", min_segment_size_, 8.0);
    private_nh.param("max_segment_size", max_segment_size_, 80.0);
    private_nh.param("min_segment_distance", min_segment_distance_, 0.05);
    private_nh.param("max_segment_distance", max_segment_distance_, 0.25);
    private_nh.param("min_toward_weight", min_toward_weight_, 0.6);
    private_nh.param("min_inscribe_angle", min_inscribe_angle_, 0.2);
    private_nh.param("max_inscribe_angle", max_inscribe_angle_, 3.0);
    private_nh.param("max_leg_distance", max_leg_distance_, 0.7);
    people_pub_ = private_nh.advertise<people_msgs::People>("laser_found_people", 1);
    debug_pub_ = private_nh.advertise<sensor_msgs::PointCloud>("debug_point_cloud", 1);
}

void LaserHumanTracker::LaserDataHandler(const sensor_msgs::LaserScan::ConstPtr & laser_scan) {
    vector<Point> laser_points;
    double now_angle = laser_scan->angle_min;
    double angle_step = laser_scan->angle_increment;
    BOOST_FOREACH(float radius, laser_scan->ranges) {
        //just to get rid of inf values
        if(radius > -1000 && radius < 1000) {
            Point point = {
                .x = radius * cos(now_angle),
                .y = radius * sin(now_angle)
            };
            laser_points.push_back(point);
        }
        now_angle += angle_step;
    }
    vector<vector<Point> > segments = GetSegments(laser_points);
    vector<Point> leg_points;
    BOOST_FOREACH(vector<Point> & segment, segments) {
        leg_points.push_back(GetCenter(segment));
        //ROS_INFO("found leg: center %f %f", leg_points.back().x, leg_points.back().y);
    }
    vector<Point> human_points = GetHumanPoints(leg_points);
    PublishSegment(human_points);
    PublishHumanPoints(human_points);
    //PublishHumanPoints(leg_points);
    //ROS_INFO("==========================");
}

vector<vector<Point> > LaserHumanTracker::GetSegments(const std::vector<Point> & laser_points) {
    Point last_point;
    bool is_in_segment = false;
    vector<Point> now_segment;
    vector<vector<Point> > segments;
    BOOST_FOREACH(Point point, laser_points) {
        if (!is_in_segment) {
            now_segment.push_back(point);
            is_in_segment = true;
        }
        else {
            if(Distance(last_point, point) < same_segment_distance_) {
                now_segment.push_back(point);
            }
            else{
                if(IsValidSegment(now_segment)) {
                    segments.push_back(now_segment);
                }
                now_segment.clear();
                now_segment.push_back(point);
            }
        }
        last_point = point;
    }
    return segments;
}

vector<Point> LaserHumanTracker::GetHumanPoints(const vector<Point> & leg_points) {
    vector<Point> human_points;
    for(int i=0; i<leg_points.size(); i++)
        for(int j=i+1; j<leg_points.size(); j++) {
            if(Distance(leg_points[i], leg_points[j]) < max_leg_distance_) {
                Point mid_point = {
                    .x = (leg_points[i].x + leg_points[j].x) / 2,
                    .y = (leg_points[i].y + leg_points[j].y) / 2
                };
                human_points.push_back(mid_point);
            }
        }
    return human_points;
}


bool LaserHumanTracker::IsValidSegment(const vector<Point> & segment) {
    if(segment.size() < min_segment_size_ || segment.size() > max_segment_size_) 
        return false;
    double segment_distance = Distance(segment[0], segment.back());
    if(segment_distance < min_segment_distance_ || segment_distance > max_segment_distance_) 
        return false;
    double inscribe_angle = AvgInscribeAngle(segment);
    if (inscribe_angle < min_inscribe_angle_ || inscribe_angle > max_inscribe_angle_)
        return false;
    if (TowardWeight(segment) < min_toward_weight_)
        return false;
    return true;
}

double LaserHumanTracker::TowardWeight(const vector<Point> & segment) {
    Point start_point = segment[0];
    Point end_point = segment.back();
    int count_towards = 0;
    int count_backwards = 0;
    for(int i=1; i < segment.size() - 1; i++) {
        if (OriginIsSameSide(start_point, end_point, segment[i]))
            count_towards++;
        else
            count_backwards++;
    }
    return double(count_towards) / double(count_towards + count_backwards);
}

double LaserHumanTracker::AvgInscribeAngle(const vector<Point> & segment) {
    Point start_point = segment[0];
    Point end_point = segment.back();
    double sum_angle = 0;
    for(int i=1; i<segment.size() - 1; i++) {
        sum_angle += GetInscribeAngle(start_point, end_point, segment[i]);
    }
    return sum_angle / segment.size();
}

void LaserHumanTracker::PublishHumanPoints(const vector<Point> & human_points) {
    people_msgs::People people;
    people.header.seq = seq_++;
    people.header.stamp = ros::Time::now();
    people.header.frame_id = frame_id_;
    BOOST_FOREACH(const Point & point, human_points) {
        people_msgs::Person person;
        person.position.x = point.x;
        person.position.y = point.y;
        person.position.z = 0;
        people.people.push_back(person);
    }
    people_pub_.publish(people);
}

void LaserHumanTracker::PublishSegment(const vector<Point> & segment) {
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header.seq = debug_seq_++;
    point_cloud.header.stamp = ros::Time::now();
    point_cloud.header.frame_id = frame_id_;
    BOOST_FOREACH(const Point & point, segment) {
        geometry_msgs::Point32 point32;
        point32.x = point.x;
        point32.y = point.y;
        point32.z = 0;
        point_cloud.points.push_back(point32);
    }
    debug_pub_.publish(point_cloud);
}

Point LaserHumanTracker::GetCenter(const vector<Point> & segment) {
    double sum_x = 0;
    double sum_y = 0;
    BOOST_FOREACH(const Point & point, segment) {
        sum_x += point.x;
        sum_y += point.y;
    }
    Point center = {
        .x = sum_x / segment.size(),
        .y = sum_y / segment.size()
    };
    return center;
}

}
}
