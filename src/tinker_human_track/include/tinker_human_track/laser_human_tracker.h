#ifndef __LASER_HUMAN_TRACKER_H__
#define __LASER_HUMAN_TRACKER_H__

#include <tinker_human_track/utilities.h>
#include <sensor_msgs/LaserScan.h>
#include <people_msgs/People.h>
#include <vector>
#include <string>
#include <ros/ros.h>

namespace tinker {
namespace navigation {
    
class LaserHumanTracker {
public:
    LaserHumanTracker();
    void LaserDataHandler(const sensor_msgs::LaserScan::ConstPtr & laser_scan);
private:
    std::vector<std::vector<Point> > GetSegments(const std::vector<Point> & laser_points);
    std::vector<Point> GetHumanPoints(const std::vector<Point> & leg_points);
    bool IsValidSegment(const std::vector<Point> & segment);
    double TowardWeight(const std::vector<Point> & segment);
    double AvgInscribeAngle(const std::vector<Point> & segment);
    void PublishHumanPoints(const std::vector<Point> & human_points);
    Point GetCenter(const std::vector<Point> & segment);

    int seq_;
    std::string frame_id_;
    double same_segment_distance_;
    double min_segment_size_;
    double max_segment_size_;
    double min_segment_distance_;
    double max_segment_distance_;
    double min_toward_weight_;
    double min_inscribe_angle_;
    double max_inscribe_angle_;
    double max_leg_distance_;
    ros::Publisher people_pub_;    
};

}
}

#endif
