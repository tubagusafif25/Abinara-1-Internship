#include "ros/ros.h"
#include "student_msgs/StudentMsg.h"
#include <iostream>
using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "student_publisher");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<student_msgs::StudentMsg>("student_updates", 10);
    ros::Rate loop_rate(1);

    while (ros::ok()) {
        student_msgs::StudentMsg msg;
        cout << "Enter ID: ";
        cin >> msg.id;
        cout << "Enter name: ";
        cin >> msg.name;
        cout << "Enter grade: ";
        cin >> msg.grade;

        pub.publish(msg);
        ROS_INFO("Published: ID=%d, Name=%s, Grade=%.2f",
                 msg.id, msg.name.c_str(), msg.grade);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
