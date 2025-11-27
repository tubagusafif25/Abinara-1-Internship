````markdown
# 🤖 ROS 3-DOF Arm Inverse Kinematics Solver

![ROS](https://img.shields.io/badge/ROS-Noetic-blue) ![C++](https://img.shields.io/badge/C++-11-green) ![Rviz](https://img.shields.io/badge/Visualization-Rviz-orange)

A ROS package implementing an **Analytic Inverse Kinematics (IK) Solver** for a custom 3-DOF robotic manipulator. This project features a custom C++ solver node that calculates joint angles from a target Cartesian point (X, Y, Z) and visualizes the movement on a URDF robot model.

## 📋 Project Overview

This project bridges the gap between mathematical theory and robot simulation. It consists of:
1.  **The Brain (C++ Node):** A geometric IK solver that computes the necessary angles for the Hip, Shoulder, and Elbow joints to reach a target coordinate.
2.  **The Body (URDF):** A physical description of a robotic arm with a Yaw-Pitch-Pitch configuration.

### Key Features
* **Analytic Solver:** Uses geometric triangulation (Law of Cosines) rather than iterative numerical methods for faster computation.
* **Closed-Loop Verification:** Includes a Forward Kinematics (FK) check to calculate the "Delta Error" (difference between desired target and actual achieved position).
* **Real-time Visualization:** Publishes `JointState` messages to drive the robot model in Rviz.
* **Safety Checks:** Detects and reports unreachable workspace targets.

## 🛠️ System Architecture

### 1. Robot Physical Parameters (URDF)
Based on the `arm.urdf` definition:
* **Base Configuration:** Fixed cylinder base.
* **Joint 1 (Hip):** Continuous rotation (Z-axis / Yaw).
* **Joint 2 (Shoulder):** Revolute limit -1.57 to 0.69 rad (Y-axis / Pitch).
* **Joint 3 (Elbow):** Revolute limit ±1.57 rad (Y-axis / Pitch).
* **Link Lengths:** * $L_1$ (Upper Arm) = 0.3m
    * $L_2$ (Forearm) = 0.3m

### 2. The IK Algorithm
The solver breaks the 3D problem into two 2D planes:
1.  **Yaw ($q_1$):** Calculated using `atan2(y, x)`.
2.  **Pitch ($q_2, q_3$):** Calculated by reducing the problem to a planar 2-link arm using the geometric approach:
    * $r = \sqrt{x^2 + y^2}$
    * $D^2 = r^2 + z^2$
    * $q_3$ derived via Law of Cosines.
    * $q_2$ derived via geometric difference.

## 🚀 Usage

### Prerequisites
* ROS 1 (Noetic or Melodic)
* `roscpp`
* `sensor_msgs`
* `geometry_msgs`

### 1. Build the Package
```bash
cd ~/catkin_ws/src
# Clone or create your package here
cd ~/catkin_ws
catkin_make
source devel/setup.bash
````

### 2\. Launch the Node

*Note: Ensure you have a launch file that loads the URDF and starts the `robot_state_publisher`.*

```bash
rosrun your_package_name ik_solver_node
```

### 3\. Send a Target Command

Open a new terminal and publish a target point (X, Y, Z) to the solver:

```bash
# Example: Move to X=0.3, Y=0.0, Z=0.4
rostopic pub -1 /ik_target geometry_msgs/Point "x: 0.3 y: 0.0 z: 0.4"
```

## 📡 ROS Topics

| Topic | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `/ik_target` | `geometry_msgs/Point` | Subscriber | The desired Cartesian coordinates (X, Y, Z) relative to the base. |
| `/joint_states` | `sensor_msgs/JointState` | Publisher | The calculated joint angles sent to the robot description. |

## 📊 Performance & Verification

The node automatically verifies its own math. Every time a solution is found, it calculates:
$$ \text{Error} = \sqrt{(x_{target} - x_{fk})^2 + (y_{target} - y_{fk})^2 + (z_{target} - z_{fk})^2} $$

**Console Output Example:**

```text
[INFO]: IK Solution (q1, q2, q3): [0.000, 0.523, -0.785]
[INFO]: Target (relative to hip): (0.300, 0.000, 0.200)
[INFO]: Actual (relative to hip): (0.300, 0.000, 0.200)
[INFO]: Delta Error: 0.000000
```

## 📂 File Structure

```text
.
├── src/
│   └── ik_solver_node.cpp    # The C++ source code for calculations
├── urdf/
│   └── arm.urdf              # The robot physical description
├── launch/
│   └── display.launch        # (Optional) Launch file for Rviz
├── CMakeLists.txt
└── package.xml
```

## 🔮 Future Improvements

  * Implement `kdl_parser` to read link lengths dynamically from URDF instead of hardcoding.
  * Add a GUI slider to control the target point.
  * Implement path planning to avoid self-collisions.

-----

*Project created for Robotics System Course.*

```
```
