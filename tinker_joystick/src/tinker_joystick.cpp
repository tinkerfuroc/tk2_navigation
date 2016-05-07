#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>


class TinkerTeleop {
public:
    TinkerTeleop();

private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr &joy);

    double scale_x_, scale_y_, scale_omega_;
    ros::NodeHandle private_nh_;
    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
};


TinkerTeleop::TinkerTeleop()
        : private_nh_("~") {
    private_nh_.param("scale_x", scale_x_, 1.0);
    private_nh_.param("scale_y", scale_y_, 1.0);
    private_nh_.param("scale_omega", scale_omega_, 1.0);

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TinkerTeleop::joyCallback, this);
}

void TinkerTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr &joy) {
    geometry_msgs::Twist vel;
    vel.linear.x = scale_x_ * joy->axes[1];
    vel.linear.y = scale_y_ * joy->axes[0];
    vel.angular.z = scale_omega_ * joy->axes[2];
    vel_pub_.publish(vel);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "teleop_turtle");
    TinkerTeleop teleop_turtle;
    ros::spin();
}