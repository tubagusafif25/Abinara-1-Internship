#include "ros/ros.h"
#include "student_msgs/StudentMsg.h"

void studentCallback(const student_msgs::StudentMsg::ConstPtr& msg) {
    ROS_INFO("[Subscriber] Update received -> ID: %d, Name: %s, Grade: %.2f",
             msg->id, msg->name.c_str(), msg->grade);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "student_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("student_updates", 10, studentCallback);
    ROS_INFO("Student subscriber started, listening for updates...");
    ros::spin();
    return 0;
}
