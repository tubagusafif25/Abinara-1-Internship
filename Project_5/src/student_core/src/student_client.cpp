#include "ros/ros.h"
#include "student_msgs/ManageStudent.h"
#include <iostream>
using namespace std;

void printMenu() {
    cout << "\n===== Student Management Client =====" << endl;
    cout << "0. Add student" << endl;
    cout << "1. Update student" << endl;
    cout << "2. Delete student" << endl;
    cout << "3. Get student" << endl;
    cout << "4. List all students" << endl;
    cout << "9. Exit" << endl;
    cout << "Choose command: ";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "student_client");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<student_msgs::ManageStudent>("manage_student");
    student_msgs::ManageStudent srv;

    while (ros::ok()) {
        printMenu();
        cin >> srv.request.cmd;

        if (srv.request.cmd == 9) {
            cout << "Exiting client..." << endl;
            break;
        }

        if (srv.request.cmd == 0 || srv.request.cmd == 1) { // Add or Update
            cout << "Enter ID: ";
            cin >> srv.request.id;
            cout << "Enter name: ";
            cin.ignore();
            getline(cin, srv.request.name);
            cout << "Enter grade: ";
            cin >> srv.request.grade;
        } else if (srv.request.cmd == 2 || srv.request.cmd == 3) { // Delete or Get
            cout << "Enter ID: ";
            cin >> srv.request.id;
            srv.request.name = "";
            srv.request.grade = 0;
        } else {
            srv.request.id = 0;
            srv.request.name = "";
            srv.request.grade = 0;
        }

        if (client.call(srv)) {
            cout << "\nResponse: " << srv.response.info << endl;
            if (!srv.response.all_students.empty()) {
                cout << "\nCurrent Students:" << endl;
                for (const auto& s : srv.response.all_students) {
                    cout << "  ID: " << s.id << ", Name: " << s.name << ", Grade: " << s.grade << endl;
                }
            }
        } else {
            ROS_ERROR("Failed to call service /manage_student");
        }
    }

    return 0;
}
