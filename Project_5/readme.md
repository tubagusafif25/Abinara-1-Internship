# 🎓 ROS Student Management System

![ROS](https://img.shields.io/badge/ROS-Noetic-blue) ![C++](https://img.shields.io/badge/C++-11-green) ![Catkin](https://img.shields.io/badge/Build-Catkin-orange)

A distributed student database system built on **ROS (Robot Operating System)**. This project demonstrates the implementation of a full **CRUD (Create, Read, Update, Delete)** application using ROS Services for transactional data management and ROS Topics for real-time broadcasting.

## 📋 Project Overview

This package consists of four nodes interacting to simulate a backend database system:

1.  **Service Server (`student_node`):** The central brain. It stores the student list in memory, handles requests, and broadcasts updates.
2.  **Service Client (`student_client`):** A CLI interface allowing users to Add, Update, Delete, or List students.
3.  **Subscriber (`student_subscriber`):** A monitoring node that listens for changes and logs real-time updates.
4.  **Manual Publisher (`student_publisher`):** A standalone tool to manually inject data into the system.

## 🛠️ System Architecture

| Node Name | Type | Role | Communication Method |
| :--- | :--- | :--- | :--- |
| `student_node` | Server | Data Storage & Logic | Service Host & Publisher |
| `student_client` | Client | User Interface | Service Caller |
| `student_subscriber` | Listener | Data Logger | Topic Subscriber |
| `student_publisher` | Publisher | Data Injector | Topic Publisher |

## 📦 Dependencies

* **ROS Noetic** (or Melodic)
* `roscpp`
* `std_msgs`
* Custom Package: `student_msgs` (must contain the Msg and Srv definitions below)

### Custom Definitions
**StudentMsg.msg**
```text
int32 id
string name
float32 grade
````

**ManageStudent.srv**

```text
int32 cmd      # 0=Add, 1=Update, 2=Delete, 3=Get, 4=List
int32 id
string name
float32 grade
---
bool success
string info
StudentMsg[] all_students
```

## 🚀 Installation & Build

1.  **Clone the repository** into your catkin workspace:

    ```bash
    cd ~/catkin_ws/src
    git clone [https://github.com/YourUsername/ros_student_manager.git](https://github.com/YourUsername/ros_student_manager.git)
    ```

2.  **Build the package:**

    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## 💻 Usage Instructions

Open **four separate terminals** to simulate the distributed network.

### Terminal 1: ROS Master

Start the core system.

```bash
roscore
```

### Terminal 2: The Server (Database)

This node must run first to handle requests.

```bash
rosrun student_pkg student_node
```

### Terminal 3: The Monitor (Subscriber)

This will stay silent until a change occurs in the database.

```bash
rosrun student_pkg student_subscriber
```

### Terminal 4: The Client (UI)

Run this to interact with the system.

```bash
rosrun student_pkg student_client
```

*Follow the on-screen menu to Add (0), Update (1), Delete (2), or List (4) students.*

## 📡 ROS Topics & Services

| Name | Type | Description |
| :--- | :--- | :--- |
| `/manage_student` | `student_msgs/ManageStudent` | **Service**: Handles database transactions (Request/Response). |
| `/student_updates` | `student_msgs/StudentMsg` | **Topic**: Broadcasts real-time changes to subscribers. |

## 📸 Example Workflow

1.  **Client** selects "0. Add Student" -\> Enters ID: `101`, Name: `Zack`, Grade: `3.8`.
2.  **Server** processes request, saves data, and responds "Student Added".
3.  **Server** publishes the new student object to `/student_updates`.
4.  **Subscriber** wakes up and prints: `[Subscriber] Update received -> ID: 101, Name: Zack, Grade: 3.8`.

## 🔮 Future Improvements

  * Implement persistent storage (save to `.csv` or SQL) so data isn't lost on shutdown.
  * Add a GUI (using Qt or Python Tkinter) instead of a terminal menu.
  * Implement mutex locking to handle multiple clients simultaneously.

-----

*Project created for ROS Middleware Architecture Course.*
