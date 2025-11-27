#include "ros/ros.h"
#include "student_msgs/ManageStudent.h"
#include "student_msgs/StudentMsg.h"
#include <vector>
#include <algorithm>

using namespace std;

vector<student_msgs::StudentMsg> studentList;
ros::Publisher student_pub;

bool manageCallback(student_msgs::ManageStudent::Request &req,
                    student_msgs::ManageStudent::Response &res) {
    if (req.cmd == 0) {
        student_msgs::StudentMsg s;
        s.id = req.id;
        s.name = req.name;
        s.grade = req.grade;
        studentList.push_back(s);
        res.success = true;
        res.info = "Student added.";
        ROS_INFO("Added student: %d %s %.2f", s.id, s.name.c_str(), s.grade);

        student_pub.publish(s);
    } 
    else if (req.cmd == 1) {
        bool found = false;
        for (auto &s : studentList) {
            if (s.id == req.id) {
                s.name = req.name;
                s.grade = req.grade;
                found = true;
                ROS_INFO("Updated student %d", s.id);
                student_pub.publish(s);
                break;
            }
        }
        res.success = found;
        res.info = found ? "Student updated." : "Student not found.";
    } 
    else if (req.cmd == 2) {
        auto it = remove_if(studentList.begin(), studentList.end(),
                            [&](const student_msgs::StudentMsg &s) { return s.id == req.id; });
        if (it != studentList.end()) {
            studentList.erase(it, studentList.end());
            res.success = true;
            res.info = "Student deleted.";
            ROS_INFO("Deleted student %d", req.id);
        } else {
            res.success = false;
            res.info = "Student not found.";
        }
    } 
    else if (req.cmd == 3) {
        for (auto &s : studentList) {
            if (s.id == req.id) {
                res.all_students.push_back(s);
                res.success = true;
                res.info = "Student found.";
                break;
            }
        }
        if (res.all_students.empty()) {
            res.success = false;
            res.info = "Student not found.";
        }
    } 
    else if (req.cmd == 4) {
        res.all_students = studentList;
        res.success = true;
        res.info = "Listing all students.";
    } 
    else {
        res.success = false;
        res.info = "Invalid command.";
    }

    res.all_students = studentList;
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "student_node");
    ros::NodeHandle nh;

    student_pub = nh.advertise<student_msgs::StudentMsg>("student_updates", 10);

    ros::ServiceServer service = nh.advertiseService("manage_student", manageCallback);
    ROS_INFO("Student service node is ready.");

    ros::spin();
    return 0;
}
